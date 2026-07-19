/**
 * ldpc_decode.c -- Min-sum LDPC belief propagation decoder.
 *
 * Uses circulant sub-block structure directly for message passing
 * (avoids expanding the full H matrix). Early syndrome termination.
 *
 * Optimized for ARM Cortex-A9:
 * - Maintains per-variable total LLR for O(1) Q computation
 * - Processes Z rows per block-row together (cache-friendly)
 * - Min-sum with 2-min tracking (avoids per-edge min recomputation)
 *
 * Convention: positive LLR = bit 0 more likely.
 */

#include "lib80211/ldpc.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ========================================================================
 * H_SUB circulant definitions (from IEEE 802.11-2020 Annex F)
 * ======================================================================== */

#define N_BLOCKS 24  /* Number of block columns */
#define MAX_M    12  /* Maximum number of block rows */
#define MAX_DEG  24  /* Maximum check node degree (N_BLOCKS) */

typedef struct {
    int m;
    int n;
    int z;
    const uint8_t *hsub;
} ldpc_h_desc;

static const ldpc_h_desc *get_h_desc(int cw_len, int rate_n, int rate_d);

/* ========================================================================
 * Min-sum decoder core (optimized)
 *
 * Key optimization: maintain L[var] = channel_llr[var] + sum(R[check→var])
 * for all checks connected to var. Then Q = L[var] - R[this_check→var].
 * This eliminates the O(var_degree) inner loop from the naive approach.
 * ======================================================================== */

#define LDPC_MAX_ITERS_DEFAULT 30
#define LDPC_SCALE 0.75f

int lib80211_ldpc_decode(const float *llr_in, uint8_t *decoded_bits,
                         int cw_len, int rate_n, int rate_d,
                         int max_iterations)
{
    if (max_iterations <= 0) max_iterations = LDPC_MAX_ITERS_DEFAULT;

    const ldpc_h_desc *hd = get_h_desc(cw_len, rate_n, rate_d);
    if (!hd) return 0;

    int M = hd->m;
    int N = hd->n;
    int Z = hd->z;
    int n_vars = N * Z;

    if (n_vars != cw_len) return 0;

    /* Count non-zero entries */
    int nnz = 0;
    for (int i = 0; i < M; i++)
        for (int j = 0; j < N; j++)
            if (hd->hsub[i * N + j] != 0) nnz++;

    int total_edges = nnz * Z;

    /* Allocate working memory */
    float *R = (float *)calloc((size_t)total_edges, sizeof(float));
    float *L = (float *)malloc((size_t)n_vars * sizeof(float));
    /* Pre-computed variable index for each edge (eliminates modulo from hot path) */
    int16_t *var_map = (int16_t *)malloc((size_t)total_edges * sizeof(int16_t));

    if (!R || !L || !var_map) {
        free(R); free(L); free(var_map);
        return 0;
    }

    /* Initialize L = channel LLR (no R messages yet, all zero) */
    memcpy(L, llr_in, (size_t)n_vars * sizeof(float));

    /* Build block-row structure */
    int degree[MAX_M];
    int col_list[MAX_M][MAX_DEG];
    int shift_list[MAX_M][MAX_DEG];
    int edge_off[MAX_M][MAX_DEG];

    int edge_idx = 0;
    for (int i = 0; i < M; i++) {
        degree[i] = 0;
        for (int j = 0; j < N; j++) {
            uint8_t val = hd->hsub[i * N + j];
            if (val != 0) {
                int k = degree[i];
                col_list[i][k] = j;
                shift_list[i][k] = val % Z;
                edge_off[i][k] = edge_idx;
                edge_idx += Z;
                degree[i]++;
            }
        }
    }

    /* Pre-compute variable indices for all edges (eliminates modulo in hot loop) */
    for (int i = 0; i < M; i++) {
        for (int k = 0; k < degree[i]; k++) {
            int bj = col_list[i][k];
            int shift = shift_list[i][k];
            int base = edge_off[i][k];
            int bj_Z = bj * Z;
            for (int r = 0; r < Z; r++) {
                int c = r + shift;
                if (c >= Z) c -= Z;  /* branchless-friendly modulo for 0 <= r,shift < Z */
                var_map[base + r] = (int16_t)(bj_Z + c);
            }
        }
    }

    /* Main iteration loop */
    int converged_iter = 0;

    for (int iter = 0; iter < max_iterations; iter++) {
        /* === Check-to-Variable (R) update === */
        for (int i = 0; i < M; i++) {
            int deg = degree[i];

            /* Process all Z rows in this block-row */
            for (int r = 0; r < Z; r++) {
                /* Compute Q[k] = L[var] - R_old[this_check→var]
                 * This is O(deg) instead of O(deg * var_degree) */
                float Q[MAX_DEG];

                for (int k = 0; k < deg; k++) {
                    int edge = edge_off[i][k] + r;
                    Q[k] = L[var_map[edge]] - R[edge];
                }

                /* Min-sum: find min1, min2, sign product */
                float min1 = 1e30f, min2 = 1e30f;
                int min1_idx = 0;
                int sign_prod = 0;  /* XOR of sign bits */

                for (int k = 0; k < deg; k++) {
                    float absq = fabsf(Q[k]);
                    if (Q[k] < 0) sign_prod ^= 1;
                    if (absq < min1) {
                        min2 = min1;
                        min1 = absq;
                        min1_idx = k;
                    } else if (absq < min2) {
                        min2 = absq;
                    }
                }

                /* Write new R messages and update L incrementally */
                for (int k = 0; k < deg; k++) {
                    int edge = edge_off[i][k] + r;
                    int var_idx = var_map[edge];

                    /* Sign excluding k */
                    int s = sign_prod ^ (Q[k] < 0 ? 1 : 0);
                    /* Min excluding k */
                    float m_val = (k == min1_idx) ? min2 : min1;
                    float new_R = LDPC_SCALE * (s ? -m_val : m_val);

                    /* Update L: remove old R, add new R */
                    L[var_idx] += new_R - R[edge];
                    R[edge] = new_R;
                }
            }
        }

        /* === Hard decision + syndrome check === */
        int syndrome_ok = 1;

        for (int i = 0; i < M && syndrome_ok; i++) {
            int deg = degree[i];
            for (int r = 0; r < Z && syndrome_ok; r++) {
                uint8_t s = 0;
                for (int k = 0; k < deg; k++) {
                    int edge = edge_off[i][k] + r;
                    s ^= (L[var_map[edge]] < 0) ? 1 : 0;
                }
                if (s != 0) syndrome_ok = 0;
            }
        }

        if (syndrome_ok) {
            converged_iter = iter + 1;
            break;
        }
    }

    /* Hard decision output */
    for (int j = 0; j < n_vars; j++)
        decoded_bits[j] = (L[j] < 0) ? 1 : 0;

    free(R);
    free(L);
    free(var_map);

    return converged_iter > 0 ? converged_iter : 0;
}

/* ========================================================================
 * H_SUB tables for all 12 codes (IEEE 802.11-2020 Annex F / Table 20-13..20-24)
 * ======================================================================== */

#include "ldpc_h_tables.h"

static const ldpc_h_desc h_descs[] = {
    { 12, 24, 27, ldpc_hsub_648_12 },
    {  8, 24, 27, ldpc_hsub_648_23 },
    {  6, 24, 27, ldpc_hsub_648_34 },
    {  4, 24, 27, ldpc_hsub_648_56 },
    { 12, 24, 54, ldpc_hsub_1296_12 },
    {  8, 24, 54, ldpc_hsub_1296_23 },
    {  6, 24, 54, ldpc_hsub_1296_34 },
    {  4, 24, 54, ldpc_hsub_1296_56 },
    { 12, 24, 81, ldpc_hsub_1944_12 },
    {  8, 24, 81, ldpc_hsub_1944_23 },
    {  6, 24, 81, ldpc_hsub_1944_34 },
    {  4, 24, 81, ldpc_hsub_1944_56 },
};

static const ldpc_h_desc *get_h_desc(int cw_len, int rate_n, int rate_d)
{
    int cw_idx;
    switch (cw_len) {
        case 648:  cw_idx = 0; break;
        case 1296: cw_idx = 4; break;
        case 1944: cw_idx = 8; break;
        default: return NULL;
    }

    int rate_idx;
    if (rate_n == 1 && rate_d == 2)      rate_idx = 0;
    else if (rate_n == 2 && rate_d == 3) rate_idx = 1;
    else if (rate_n == 3 && rate_d == 4) rate_idx = 2;
    else if (rate_n == 5 && rate_d == 6) rate_idx = 3;
    else return NULL;

    return &h_descs[cw_idx + rate_idx];
}

/* ========================================================================
 * Multi-codeword LDPC decode (mirrors lib80211_ldpc_encode_data)
 * ======================================================================== */

int lib80211_ldpc_decode_data(const float *llr_in, size_t total_soft,
                              uint8_t *decoded_out,
                              int n_symbols, int n_dbps, int n_cbps,
                              int cr_n, int cr_d)
{
    int n_pld = n_symbols * n_dbps;
    int n_avail = n_symbols * n_cbps;

    /* Codeword length selection */
    int l_cw = 1944, n_cw = 1;
    int k_cap;
    for (int cw_try = 0; cw_try < 3; cw_try++) {
        static const int cw_sizes[] = { 648, 1296, 1944 };
        int cw = cw_sizes[cw_try];
        k_cap = cw * cr_n / cr_d;
        if (k_cap >= n_pld) {
            l_cw = cw;
            break;
        }
    }
    k_cap = l_cw * cr_n / cr_d;
    if (k_cap < n_pld) {
        n_cw = (n_pld + k_cap - 1) / k_cap;
        l_cw = 1944;
        k_cap = l_cw * cr_n / cr_d;
    }

    int k_per_cw = l_cw * cr_n / cr_d;
    int k_total = k_per_cw * n_cw;
    int n_parity_per_cw = l_cw - k_per_cw;
    int n_shrt = k_total - n_pld;
    if (n_shrt < 0) n_shrt = 0;
    int n_parity = n_parity_per_cw * n_cw;
    int n_punc = n_pld + n_parity - n_avail;
    if (n_punc < 0) n_punc = 0;

    /* Distribute shortening/puncturing evenly */
    int shrt_per_cw = n_shrt / n_cw;
    int shrt_extra = n_shrt % n_cw;
    int punc_per_cw = n_punc / n_cw;
    int punc_extra = n_punc % n_cw;

    /* Workspace — stack allocated (max l_cw = 1944) */
    float full_llr[LIB80211_LDPC_MAX_CW];
    uint8_t cw_decoded[LIB80211_LDPC_MAX_CW];

    int bit_offset = 0;
    int out_offset = 0;

    for (int j = 0; j < n_cw; j++) {
        int shrt_j = shrt_per_cw + (j < shrt_extra ? 1 : 0);
        int punc_j = punc_per_cw + (j < punc_extra ? 1 : 0);
        int k_j = k_per_cw - shrt_j;
        int parity_j = n_parity_per_cw - punc_j;
        int n_llr_j = k_j + parity_j;

        /* Info part: k_j received LLRs + shrt_j shortened (high confidence 0) */
        for (int i = 0; i < k_j && (bit_offset + i) < (int)total_soft; i++)
            full_llr[i] = llr_in[bit_offset + i];
        for (int i = k_j; i < k_per_cw; i++)
            full_llr[i] = 100.0f;  /* shortened = known zero */

        /* Parity part: parity_j received LLRs + punc_j erasures */
        for (int i = 0; i < parity_j && (bit_offset + k_j + i) < (int)total_soft; i++)
            full_llr[k_per_cw + i] = llr_in[bit_offset + k_j + i];
        for (int i = parity_j; i < n_parity_per_cw; i++)
            full_llr[k_per_cw + i] = 0.0f;  /* punctured = erasure */

        bit_offset += n_llr_j;

        /* Decode */
        lib80211_ldpc_decode(full_llr, cw_decoded, l_cw, cr_n, cr_d, 30);

        /* Extract info bits (first k_j) */
        int copy_len = k_j;
        if (out_offset + copy_len > n_pld) copy_len = n_pld - out_offset;
        if (copy_len > 0) {
            memcpy(decoded_out + out_offset, cw_decoded, (size_t)copy_len);
            out_offset += copy_len;
        }
    }

    return 0;
}
