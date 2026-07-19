/**
 * demap.c — Approximate max-log LLR soft demapper
 *
 * IEEE 802.11 constellation demapping: given received complex symbols,
 * produce per-bit LLR (log-likelihood ratio) soft decisions.
 *
 * Convention: positive LLR = bit 1 more likely.
 *
 * Uses the max-log approximation without noise variance scaling,
 * which is sufficient for the Viterbi decoder.
 */

#include "lib80211/modulation.h"
#include <assert.h>
#include <math.h>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

void lib80211_soft_demap(const float *rx_real, const float *rx_imag,
                         float *soft_bits, size_t n_symbols, int n_bpsc) {
    switch (n_bpsc) {
    case 1: /* BPSK: LLR = 2 * Re */
    {
#ifdef __ARM_NEON
        const float32x4_t v2 = vdupq_n_f32(2.0f);
        size_t i = 0;
        for (; i + 4 <= n_symbols; i += 4) {
            float32x4_t r = vld1q_f32(rx_real + i);
            vst1q_f32(soft_bits + i, vmulq_f32(r, v2));
        }
        for (; i < n_symbols; i++)
            soft_bits[i] = 2.0f * rx_real[i];
#else
        for (size_t i = 0; i < n_symbols; i++)
            soft_bits[i] = 2.0f * rx_real[i];
#endif
        break;
    }

    case 2: /* QPSK: LLR = 2*sqrt(2) * {Re, Im} */
    {
        const float scale = 2.0f * 1.4142135623730951f; /* 2*sqrt(2) */
#ifdef __ARM_NEON
        const float32x4_t scale_v = vdupq_n_f32(scale);
        size_t i = 0;
        for (; i + 4 <= n_symbols; i += 4) {
            float32x4_t r = vmulq_f32(vld1q_f32(rx_real + i), scale_v);
            float32x4_t m = vmulq_f32(vld1q_f32(rx_imag + i), scale_v);
            float32x4x2_t out = { { r, m } };
            vst2q_f32(soft_bits + i * 2, out);
        }
        for (; i < n_symbols; i++) {
            soft_bits[i * 2]     = scale * rx_real[i];
            soft_bits[i * 2 + 1] = scale * rx_imag[i];
        }
#else
        for (size_t i = 0; i < n_symbols; i++) {
            soft_bits[i * 2]     = scale * rx_real[i];
            soft_bits[i * 2 + 1] = scale * rx_imag[i];
        }
#endif
        break;
    }

    case 4: /* 16-QAM: undo normalization by sqrt(10) */
    {
        const float scale = 3.1622776601683795f; /* sqrt(10) */
#ifdef __ARM_NEON
        const float32x4_t scale_v = vdupq_n_f32(scale);
        const float32x4_t v2 = vdupq_n_f32(2.0f);
        size_t i = 0;
        for (; i + 4 <= n_symbols; i += 4) {
            float32x4_t x = vmulq_f32(vld1q_f32(rx_real + i), scale_v);
            float32x4_t y = vmulq_f32(vld1q_f32(rx_imag + i), scale_v);
            float32x4_t ax = vabsq_f32(x);
            float32x4_t ay = vabsq_f32(y);
            float32x4_t bit1 = vsubq_f32(v2, ax);
            float32x4_t bit3 = vsubq_f32(v2, ay);
            float32x4x4_t out = { { x, bit1, y, bit3 } };
            vst4q_f32(soft_bits + i * 4, out);
        }
        for (; i < n_symbols; i++) {
            float x = rx_real[i] * scale;
            float y = rx_imag[i] * scale;
            soft_bits[i * 4]     = x;
            soft_bits[i * 4 + 1] = 2.0f - fabsf(x);
            soft_bits[i * 4 + 2] = y;
            soft_bits[i * 4 + 3] = 2.0f - fabsf(y);
        }
#else
        for (size_t i = 0; i < n_symbols; i++) {
            float x = rx_real[i] * scale;
            float y = rx_imag[i] * scale;
            soft_bits[i * 4]     = x;
            soft_bits[i * 4 + 1] = 2.0f - fabsf(x);
            soft_bits[i * 4 + 2] = y;
            soft_bits[i * 4 + 3] = 2.0f - fabsf(y);
        }
#endif
        break;
    }

    case 6: /* 64-QAM: undo normalization by sqrt(42) */
    {
        const float scale = 6.4807406984078604f; /* sqrt(42) */
        for (size_t i = 0; i < n_symbols; i++) {
            float x = rx_real[i] * scale;
            float y = rx_imag[i] * scale;
            float ax = fabsf(x);
            float ay = fabsf(y);
            soft_bits[i * 6]     = x;
            soft_bits[i * 6 + 1] = 4.0f - ax;
            soft_bits[i * 6 + 2] = 2.0f - fabsf(ax - 4.0f);
            soft_bits[i * 6 + 3] = y;
            soft_bits[i * 6 + 4] = 4.0f - ay;
            soft_bits[i * 6 + 5] = 2.0f - fabsf(ay - 4.0f);
        }
        break;
    }

    case 8: /* 256-QAM: undo normalization by sqrt(170) */
    {
        const float scale = 13.038404810405298f; /* sqrt(170) */
        for (size_t i = 0; i < n_symbols; i++) {
            float x = rx_real[i] * scale;
            float y = rx_imag[i] * scale;
            float ax = fabsf(x);
            float ay = fabsf(y);
            float ax1 = fabsf(ax - 8.0f);
            float ay1 = fabsf(ay - 8.0f);
            soft_bits[i * 8]     = x;
            soft_bits[i * 8 + 1] = 8.0f - ax;
            soft_bits[i * 8 + 2] = 4.0f - ax1;
            soft_bits[i * 8 + 3] = 2.0f - fabsf(ax1 - 4.0f);
            soft_bits[i * 8 + 4] = y;
            soft_bits[i * 8 + 5] = 8.0f - ay;
            soft_bits[i * 8 + 6] = 4.0f - ay1;
            soft_bits[i * 8 + 7] = 2.0f - fabsf(ay1 - 4.0f);
        }
        break;
    }

    default:
        /* n_bpsc must be 1, 2, 4, 6, or 8 — caller error */
        assert(0 && "lib80211_soft_demap: unsupported n_bpsc");
        break;
    }
}
