/**
 * ldpc_encode.c — LDPC systematic encoder and DATA framing.
 *
 * Single-codeword encoding: codeword = [info_bits | parity_bits]
 * where parity = P_GEN @ info_bits (mod 2), computed via sparse CSR.
 *
 * DATA framing (§19.3.11.7): codeword selection, shortening, puncturing,
 * LDPC extra symbol, multi-codeword support.
 */

#include "lib80211/ldpc.h"
#include <string.h>

int lib80211_ldpc_encode(const uint8_t *info_bits, uint8_t *codeword,
                         int cw_len, int rate_n, int rate_d) {
    const lib80211_ldpc_pgen *pgen = lib80211_ldpc_get_pgen(cw_len, rate_n, rate_d);
    if (!pgen) return -1;

    int K = pgen->n_cols;
    int n_parity = pgen->n_rows;

    /* Systematic part: copy info bits */
    memcpy(codeword, info_bits, (size_t)K);

    /* Parity part: P_GEN @ info_bits mod 2 (sparse row-scan) */
    uint8_t *parity = &codeword[K];

    for (int row = 0; row < n_parity; row++) {
        uint32_t start = pgen->row_ptr[row];
        uint32_t end = pgen->row_ptr[row + 1];
        uint8_t acc = 0;
        for (uint32_t idx = start; idx < end; idx++) {
            acc ^= info_bits[pgen->col_idx[idx]];
        }
        parity[row] = acc;
    }

    return 0;
}

/* ========================================================================
 * LDPC DATA encoding (§19.3.11.7 / §21.3.12.5)
 * ======================================================================== */

/**
 * Select codeword length per IEEE 802.11-2020 Table 19-14.
 * Returns (l_cw, n_cw) via output parameters.
 */
static void select_codeword_params(int n_pld, int cr_n, int cr_d,
                                   int *l_cw_out, int *n_cw_out) {
    /* R = cr_n / cr_d; info capacity of codeword = l_cw * cr_n / cr_d */
    int l_cw = 1944;
    int n_cw = 1;

    if (n_pld <= 648 * cr_n / cr_d) {
        l_cw = 648;
    } else if (n_pld <= 1296 * cr_n / cr_d) {
        l_cw = 1296;
    } else if (n_pld <= 1944 * cr_n / cr_d) {
        l_cw = 1944;
    } else {
        /* Multi-codeword: use 1944 and multiple codewords */
        l_cw = 1944;
        int k_per_cw = 1944 * cr_n / cr_d;
        n_cw = (n_pld + k_per_cw - 1) / k_per_cw;
    }

    *l_cw_out = l_cw;
    *n_cw_out = n_cw;
}

int lib80211_ldpc_encode_data(const uint8_t *payload_bits, int n_payload,
                              int n_dbps, int n_cbps, int cr_n, int cr_d,
                              uint8_t *coded_out, int *n_sym_out,
                              int *ldpc_extra_out) {
    /* Step 1: Compute N_SYM (no tail bits for LDPC) */
    int n_symbols = (n_payload + n_dbps - 1) / n_dbps;
    int n_pld = n_symbols * n_dbps;

    /* Step 2: Check for LDPC extra symbol (§19.3.11.7.5) */
    int ldpc_extra = 0;
    int l_cw, n_cw;
    select_codeword_params(n_pld, cr_n, cr_d, &l_cw, &n_cw);

    int k_per_cw = l_cw * cr_n / cr_d;
    int n_avail = n_symbols * n_cbps;
    int n_shrt_test = n_cw * k_per_cw - n_pld;
    if (n_shrt_test < 0) n_shrt_test = 0;
    int n_punc_test = n_cw * l_cw - n_avail - n_shrt_test;
    if (n_punc_test < 0) n_punc_test = 0;
    int n_parity_total = n_cw * (l_cw - k_per_cw);

    if (n_parity_total > 0 && n_punc_test * 10 > n_parity_total) {
        ldpc_extra = 1;
        n_symbols += 1;
        n_pld = n_symbols * n_dbps;
    }

    /* Re-select codeword params with updated n_pld */
    if (ldpc_extra) {
        select_codeword_params(n_pld, cr_n, cr_d, &l_cw, &n_cw);
        k_per_cw = l_cw * cr_n / cr_d;
    }

    /* Step 3: Total available coded bits */
    n_avail = n_symbols * n_cbps;

    /* Step 4: Compute shortening and puncturing */
    int k_total = k_per_cw * n_cw;
    int n_shrt = k_total - n_pld;
    if (n_shrt < 0) n_shrt = 0;
    int n_punc = n_cw * l_cw - n_avail - n_shrt;
    if (n_punc < 0) n_punc = 0;

    /* Distribute shortening/puncturing evenly across codewords */
    int shrt_per_cw = n_shrt / n_cw;
    int shrt_extra = n_shrt % n_cw;
    int punc_per_cw = n_punc / n_cw;
    int punc_extra = n_punc % n_cw;

    /* Step 5: Encode each codeword */
    /* Temporary buffers for per-codeword encoding */
    uint8_t info_padded[LIB80211_LDPC_MAX_CW];  /* max K = 1620 (rate 5/6, 1944) */
    uint8_t codeword[LIB80211_LDPC_MAX_CW];

    int bit_offset = 0;
    int out_offset = 0;

    for (int j = 0; j < n_cw; j++) {
        int shrt_j = shrt_per_cw + (j < shrt_extra ? 1 : 0);
        int punc_j = punc_per_cw + (j < punc_extra ? 1 : 0);

        /* Info bits for this codeword */
        int k_j = k_per_cw - shrt_j;

        /* Copy info bits, pad with shortening zeros */
        memset(info_padded, 0, (size_t)k_per_cw);
        int copy_len = k_j;
        if (bit_offset + copy_len > n_pld) {
            /* Remaining payload may be shorter */
            copy_len = n_pld - bit_offset;
            if (copy_len < 0) copy_len = 0;
        }
        if (copy_len > 0) {
            /* Copy from payload_bits if available, else zeros (pad bits) */
            int from_payload = copy_len;
            if (bit_offset + from_payload > n_payload) {
                from_payload = n_payload - bit_offset;
                if (from_payload < 0) from_payload = 0;
            }
            if (from_payload > 0) {
                memcpy(info_padded, &payload_bits[bit_offset], (size_t)from_payload);
            }
            /* Remaining (copy_len - from_payload) are already zero (pad) */
        }
        bit_offset += k_j;

        /* Encode full codeword (k_per_cw info bits -> l_cw coded bits) */
        lib80211_ldpc_encode(info_padded, codeword, l_cw, cr_n, cr_d);

        /* Output: systematic (without shortening zeros) + parity (without punctured bits) */
        /* Systematic part: first k_j bits (excluding shortening zeros at the end) */
        memcpy(&coded_out[out_offset], codeword, (size_t)k_j);
        out_offset += k_j;

        /* Parity part: l_cw - k_per_cw bits, minus punc_j from the end */
        int parity_len = (l_cw - k_per_cw) - punc_j;
        if (parity_len > 0) {
            memcpy(&coded_out[out_offset], &codeword[k_per_cw], (size_t)parity_len);
            out_offset += parity_len;
        }
    }

    /* Pad to n_avail if needed (rounding in multi-codeword case) */
    while (out_offset < n_avail) {
        coded_out[out_offset++] = 0;
    }

    *n_sym_out = n_symbols;
    *ldpc_extra_out = ldpc_extra;
    return 0;
}
