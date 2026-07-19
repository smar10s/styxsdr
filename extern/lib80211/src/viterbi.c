/**
 * viterbi.c — Soft-decision Viterbi decoder (K=7, rate-1/2)
 *
 * IEEE 802.11-2020 Section 17.3.5.5.
 * Generator polynomials: G0 = 133 (octal), G1 = 171 (octal).
 *
 * 64-state trellis, ACS (Add-Compare-Select), traceback from state 0.
 *
 * Uses int16_t path metrics and packed 64-bit decision traceback for
 * performance on integer NEON (Cortex-A9) and general cache efficiency.
 */

#include "lib80211/fec.h"
#include "parity_internal.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <pthread.h>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#define K        7
#define N_STATES 64  /* 2^(K-1) */
#define G0       0133
#define G1       0171

/* Quantization scale: float soft bits -> int8_t */
#define QUANT_SCALE 32

/* Renormalize every this many steps to prevent int16 overflow */
#define RENORM_INTERVAL 16

/* Stack-allocate decisions for frames up to this many steps */
#define STACK_DECISIONS 4096

/**
 * Precomputed trellis structure.
 * For each destination state ns and predecessor bit (0 or 1):
 *   pred[ns][0], pred[ns][1] = the two predecessor states
 *   bm_idx[ns][0], bm_idx[ns][1] = 2-bit index into per-step branch metric table
 */
static struct {
    uint8_t next_state[N_STATES][2];
    uint8_t output[N_STATES][2][2];  /* [state][input_bit][poly] */
    uint8_t pred[N_STATES][2];       /* [dest_state][pred_bit] = predecessor state */
    uint8_t bm_idx[N_STATES][2];    /* [dest_state][pred_bit] = branch metric index */
#ifdef __ARM_NEON
    /* Pre-packed branch metric indices for NEON: 8 groups × 8 lanes */
    uint8_t bm_idx_grp[8][2][8];    /* [group][pred_bit][lane] = bm_idx value */
    /* Pre-computed byte-level TBL indices for vqtbl1q_u8 gather.
     * Each entry has 16 bytes selecting from an 8-byte BM table (4 × int16_t). */
    uint8_t bm_tbl_idx[8][2][16];   /* [group][pred_bit][byte_lane] */
    /* Sign vectors for BM construction: bm_vec = sign0*qs0 + sign1*qs1
     * sign is +1 or -1, stored as int16_t */
    int16_t bm_sign0[8][2][8];      /* [group][pred_bit][lane] */
    int16_t bm_sign1[8][2][8];      /* [group][pred_bit][lane] */
#endif
} g_trellis;

static pthread_once_t g_trellis_once = PTHREAD_ONCE_INIT;

static void init_trellis(void) {
    for (unsigned s = 0; s < N_STATES; s++) {
        for (unsigned b = 0; b < 2; b++) {
            unsigned reg = (b << 6) | s;
            g_trellis.next_state[s][b] = (uint8_t)((reg >> 1) & 0x3F);
            g_trellis.output[s][b][0] = lib80211_parity(reg & G0);
            g_trellis.output[s][b][1] = lib80211_parity(reg & G1);
        }
    }

    /* Precompute predecessor table and branch metric indices.
     *
     * For destination state ns:
     *   input bit b = ns >> 5
     *   predecessors: s0 = ((ns & 0x1F) << 1) | 0
     *                 s1 = ((ns & 0x1F) << 1) | 1
     *
     * Branch metric index encodes which of 4 possible BM values to use:
     *   index = (output_g0 << 1) | output_g1
     *   where output_g0, output_g1 are the expected output bits for that transition.
     */
    for (unsigned ns = 0; ns < N_STATES; ns++) {
        unsigned b = ns >> 5;
        unsigned s0 = ((ns & 0x1F) << 1) | 0;
        unsigned s1 = ((ns & 0x1F) << 1) | 1;
        g_trellis.pred[ns][0] = (uint8_t)s0;
        g_trellis.pred[ns][1] = (uint8_t)s1;
        g_trellis.bm_idx[ns][0] = (uint8_t)((g_trellis.output[s0][b][0] << 1) |
                                              g_trellis.output[s0][b][1]);
        g_trellis.bm_idx[ns][1] = (uint8_t)((g_trellis.output[s1][b][0] << 1) |
                                              g_trellis.output[s1][b][1]);
    }

#ifdef __ARM_NEON
    /* Pack bm_idx into groups of 8 for vectorized ACS */
    for (unsigned g = 0; g < 8; g++) {
        unsigned base = g * 8;
        for (unsigned lane = 0; lane < 8; lane++) {
            g_trellis.bm_idx_grp[g][0][lane] = g_trellis.bm_idx[base + lane][0];
            g_trellis.bm_idx_grp[g][1][lane] = g_trellis.bm_idx[base + lane][1];
        }
        /* Pre-compute byte-level TBL indices for vqtbl1q_u8.
         * BM table is 4 × int16_t = 8 bytes. For lane i, we need bytes at
         * offset idx*2 (low byte) and idx*2+1 (high byte). */
        for (unsigned pb = 0; pb < 2; pb++) {
            for (unsigned lane = 0; lane < 8; lane++) {
                uint8_t idx = g_trellis.bm_idx_grp[g][pb][lane];
                g_trellis.bm_tbl_idx[g][pb][2*lane]     = idx * 2;
                g_trellis.bm_tbl_idx[g][pb][2*lane + 1] = idx * 2 + 1;
            }
        }

        /* Pre-compute sign vectors for BM construction without gather.
         * bm[idx] = (-1)^((idx>>1)&1) * qs0 + (-1)^(idx&1) * qs1
         * i.e., sign0 = (idx & 2) ? -1 : +1
         *       sign1 = (idx & 1) ? -1 : +1 */
        for (unsigned pb = 0; pb < 2; pb++) {
            for (unsigned lane = 0; lane < 8; lane++) {
                uint8_t idx = g_trellis.bm_idx_grp[g][pb][lane];
                g_trellis.bm_sign0[g][pb][lane] = (idx & 2) ? -1 : 1;
                g_trellis.bm_sign1[g][pb][lane] = (idx & 1) ? -1 : 1;
            }
        }
    }
#endif
}

/**
 * Quantize a float soft bit to int8_t range [-127, +127].
 * Positive = bit 1 likely, negative = bit 0 likely.
 */
static inline int8_t quantize_soft(float f) {
    int v = (int)(f * QUANT_SCALE);
    if (v > 127) v = 127;
    if (v < -127) v = -127;
    return (int8_t)v;
}

int lib80211_viterbi_decode(const float *soft_bits, uint8_t *out_bits,
                            size_t n_coded_bits, size_t n_data_bits) {
    pthread_once(&g_trellis_once, init_trellis);

    size_t n_steps = n_data_bits + (K - 1);  /* data + tail */

    /* Sanity check */
    if (n_coded_bits < 2 * n_steps) return -1;

    /* Decision traceback: 1 bit per state per step, packed into uint64_t.
     * Bit s of decisions[t] = which predecessor of state s won (0 or 1). */
    uint64_t decisions_stack[STACK_DECISIONS];
    uint64_t *decisions;
    if (n_steps <= STACK_DECISIONS) {
        decisions = decisions_stack;
    } else {
        decisions = malloc(n_steps * sizeof(uint64_t));
        if (!decisions) return -1;
    }

    /* Path metrics: double-buffered int16_t */
    int16_t metrics[2][N_STATES];
    int cur = 0;

    /* Initialize: state 0 starts at 0, all others at high value.
     * Use INT16_MAX/2 to leave headroom for branch metric additions. */
    for (int s = 0; s < N_STATES; s++)
        metrics[cur][s] = INT16_MAX / 2;
    metrics[cur][0] = 0;

    /* ACS loop — iterate by destination state (butterfly structure) */
    for (size_t t = 0; t < n_steps; t++) {
        int nxt = 1 - cur;

        /* Quantize received soft bits for this step */
        int16_t qs0 = quantize_soft(soft_bits[2 * t]);
        int16_t qs1 = quantize_soft(soft_bits[2 * t + 1]);

        uint64_t d = 0;

#ifdef __ARM_NEON
        /* BM table for vqtbl1q (AArch64) or sign-vector construction (ARMv7) */
#ifdef __aarch64__
        int16_t bm[4];
        bm[3] = (int16_t)(-qs0 - qs1);
        bm[2] = (int16_t)(-qs0 + qs1);
        bm[1] = (int16_t)( qs0 - qs1);
        bm[0] = (int16_t)( qs0 + qs1);
        int16_t bm_pad[8] = {bm[0], bm[1], bm[2], bm[3], 0, 0, 0, 0};
        uint8x16_t bm_tbl = vld1q_u8((const uint8_t *)bm_pad);
#else
        int16x8_t qs0_vec = vdupq_n_s16(qs0);
        int16x8_t qs1_vec = vdupq_n_s16(qs1);
#endif

        for (unsigned g = 0; g < 8; g++) {
            unsigned base = g * 8;
            unsigned pred_off = ((base & 0x1F) << 1);

            /* De-interleave load: val[0] = even (s0 metrics), val[1] = odd (s1 metrics) */
            int16x8x2_t pred_m = vld2q_s16(&metrics[cur][pred_off]);

            /* Gather branch metrics via precomputed byte-level table lookup */
#ifdef __aarch64__
            uint8x16_t tidx0 = vld1q_u8(g_trellis.bm_tbl_idx[g][0]);
            uint8x16_t tidx1 = vld1q_u8(g_trellis.bm_tbl_idx[g][1]);
            int16x8_t bm0_vec = vreinterpretq_s16_u8(vqtbl1q_u8(bm_tbl, tidx0));
            int16x8_t bm1_vec = vreinterpretq_s16_u8(vqtbl1q_u8(bm_tbl, tidx1));
#else
            /* ARMv7 NEON: construct BM vectors using precomputed sign patterns.
             * bm_vec[lane] = sign0[lane]*qs0 + sign1[lane]*qs1 */
            int16x8_t s0_0 = vld1q_s16(g_trellis.bm_sign0[g][0]);
            int16x8_t s1_0 = vld1q_s16(g_trellis.bm_sign1[g][0]);
            int16x8_t s0_1 = vld1q_s16(g_trellis.bm_sign0[g][1]);
            int16x8_t s1_1 = vld1q_s16(g_trellis.bm_sign1[g][1]);
            int16x8_t bm0_vec = vaddq_s16(vmulq_s16(s0_0, qs0_vec), vmulq_s16(s1_0, qs1_vec));
            int16x8_t bm1_vec = vaddq_s16(vmulq_s16(s0_1, qs0_vec), vmulq_s16(s1_1, qs1_vec));
#endif

            /* Add: candidate metrics */
            int16x8_t m0_vec = vaddq_s16(pred_m.val[0], bm0_vec);
            int16x8_t m1_vec = vaddq_s16(pred_m.val[1], bm1_vec);

            /* Select: lower metric wins */
            vst1q_s16(&metrics[nxt][base], vminq_s16(m0_vec, m1_vec));

            /* Decision bits: 1 where m1 < m0 */
            uint16x8_t cmp = vcltq_s16(m1_vec, m0_vec);

            /* Pack 8 comparison results into a single byte */
            static const uint8_t bit_tbl[8] = {1, 2, 4, 8, 16, 32, 64, 128};
            uint8x8_t narrow = vmovn_u16(cmp);
            uint8x8_t bits = vand_u8(narrow, vld1_u8(bit_tbl));
            uint8x8_t p1 = vpadd_u8(bits, bits);
            uint8x8_t p2 = vpadd_u8(p1, p1);
            uint8x8_t p3 = vpadd_u8(p2, p2);
            uint8_t decision_byte = vget_lane_u8(p3, 0);

            d |= ((uint64_t)decision_byte << (g * 8));
        }
#else
        int16_t bm[4];
        bm[3] = (int16_t)(-qs0 - qs1);
        bm[2] = (int16_t)(-qs0 + qs1);
        bm[1] = (int16_t)( qs0 - qs1);
        bm[0] = (int16_t)( qs0 + qs1);
        for (unsigned ns = 0; ns < N_STATES; ns++) {
            unsigned s0 = g_trellis.pred[ns][0];
            unsigned s1 = g_trellis.pred[ns][1];

            int16_t m0 = (int16_t)(metrics[cur][s0] + bm[g_trellis.bm_idx[ns][0]]);
            int16_t m1 = (int16_t)(metrics[cur][s1] + bm[g_trellis.bm_idx[ns][1]]);

            /* Select: lower metric wins (minimize cost) */
            if (m1 < m0) {
                metrics[nxt][ns] = m1;
                d |= ((uint64_t)1 << ns);
            } else {
                metrics[nxt][ns] = m0;
                /* bit stays 0 */
            }
        }
#endif

        decisions[t] = d;
        cur = nxt;

        /* Renormalize metrics periodically to prevent int16 overflow */
        if ((t & (RENORM_INTERVAL - 1)) == (RENORM_INTERVAL - 1)) {
#ifdef __ARM_NEON
            /* Vectorized min reduction */
            int16x8_t vmin = vld1q_s16(&metrics[cur][0]);
            for (unsigned s = 8; s < N_STATES; s += 8) {
                vmin = vminq_s16(vmin, vld1q_s16(&metrics[cur][s]));
            }
            /* Reduce 8 lanes to scalar min */
            int16x4_t vmin4 = vmin_s16(vget_low_s16(vmin), vget_high_s16(vmin));
            int16x4_t vmin2 = vpmin_s16(vmin4, vmin4);
            int16x4_t vmin1 = vpmin_s16(vmin2, vmin2);
            int16_t min_m = vget_lane_s16(vmin1, 0);

            /* Broadcast subtract */
            int16x8_t min_v = vdupq_n_s16(min_m);
            for (unsigned s = 0; s < N_STATES; s += 8) {
                int16x8_t v = vld1q_s16(&metrics[cur][s]);
                vst1q_s16(&metrics[cur][s], vsubq_s16(v, min_v));
            }
#else
            int16_t min_m = metrics[cur][0];
            for (unsigned s = 1; s < N_STATES; s++) {
                if (metrics[cur][s] < min_m)
                    min_m = metrics[cur][s];
            }
            for (unsigned s = 0; s < N_STATES; s++) {
                metrics[cur][s] = (int16_t)(metrics[cur][s] - min_m);
            }
#endif
        }
    }

    /* Traceback from state 0 (encoder flushed with tail bits).
     *
     * At each step, the decoded output bit = state >> 5 (top bit of current state).
     * Predecessor = ((state & 0x1F) << 1) | decision_bit.
     */
    unsigned state = 0;
    for (size_t t = n_steps; t > 0; t--) {
        uint64_t d = decisions[t - 1];
        unsigned pred_lsb = (unsigned)((d >> state) & 1);

        /* Output decoded bit for data steps */
        if (t - 1 < n_data_bits)
            out_bits[t - 1] = (uint8_t)((state >> 5) & 1);

        /* Move to predecessor state */
        state = ((state & 0x1F) << 1) | pred_lsb;
    }

    if (n_steps > STACK_DECISIONS)
        free(decisions);

    return 0;
}
