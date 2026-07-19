/**
 * modulation.c — Gray-coded QAM modulator
 *
 * IEEE 802.11-2020 Section 17.3.5.8 (BPSK, QPSK, 16-QAM, 64-QAM)
 * and Section 21.3.10.10 (256-QAM for VHT).
 *
 * Each modulation maps N bits to a complex point with average power = 1.
 * Gray coding ensures adjacent constellation points differ by 1 bit.
 */

#include "lib80211/modulation.h"
#include <math.h>

/* Normalization factors: 1/sqrt(2*(2^n_bpsc - 1)/3) for square QAM
 * BPSK:    1.0
 * QPSK:    1/sqrt(2)
 * 16-QAM:  1/sqrt(10)
 * 64-QAM:  1/sqrt(42)
 * 256-QAM: 1/sqrt(170)
 */
static const float NORM_BPSK   = 1.0f;
static const float NORM_QPSK   = 0.7071067811865475f;  /* 1/sqrt(2) */
static const float NORM_16QAM  = 0.3162277660168380f;  /* 1/sqrt(10) */
static const float NORM_64QAM  = 0.1543033499620919f;  /* 1/sqrt(42) */
static const float NORM_256QAM = 0.0766964989121553f;  /* 1/sqrt(170) */

/**
 * Convert Gray-coded bits to PAM level.
 * For M-QAM, each axis uses M^(1/2) levels: {-M+1, -M+3, ..., M-1}
 * Input: n bits in Gray code. Output: integer level.
 */
static inline int gray_to_pam(const uint8_t *bits, int n_bits) {
    /* Convert bits to Gray code integer */
    int gray = 0;
    for (int i = 0; i < n_bits; i++)
        gray = (gray << 1) | bits[i];

    /* Gray to binary */
    int bin = gray;
    for (int shift = 1; shift < n_bits; shift <<= 1)
        bin ^= (bin >> shift);

    /* Map binary to PAM level: {0,1,...,2^n-1} -> {-(2^n-1), -(2^n-3), ..., (2^n-1)} */
    int M = 1 << n_bits;
    return 2 * bin - (M - 1);
}

void lib80211_modulate(const uint8_t *bits, float *out_real, float *out_imag,
                       size_t n_symbols, int n_bpsc) {
    switch (n_bpsc) {
    case 1: /* BPSK: 1 -> +1, 0 -> -1 */
        for (size_t i = 0; i < n_symbols; i++) {
            out_real[i] = bits[i] ? NORM_BPSK : -NORM_BPSK;
            out_imag[i] = 0.0f;
        }
        break;

    case 2: /* QPSK: 2 bits -> I, Q. bit=1 -> +1, bit=0 -> -1 */
        for (size_t i = 0; i < n_symbols; i++) {
            float i_val = bits[i * 2]     ? NORM_QPSK : -NORM_QPSK;
            float q_val = bits[i * 2 + 1] ? NORM_QPSK : -NORM_QPSK;
            out_real[i] = i_val;
            out_imag[i] = q_val;
        }
        break;

    case 4: /* 16-QAM: 4 bits -> 2 bits I, 2 bits Q */
        for (size_t i = 0; i < n_symbols; i++) {
            int i_level = gray_to_pam(&bits[i * 4], 2);
            int q_level = gray_to_pam(&bits[i * 4 + 2], 2);
            out_real[i] = (float)i_level * NORM_16QAM;
            out_imag[i] = (float)q_level * NORM_16QAM;
        }
        break;

    case 6: /* 64-QAM: 6 bits -> 3 bits I, 3 bits Q */
        for (size_t i = 0; i < n_symbols; i++) {
            int i_level = gray_to_pam(&bits[i * 6], 3);
            int q_level = gray_to_pam(&bits[i * 6 + 3], 3);
            out_real[i] = (float)i_level * NORM_64QAM;
            out_imag[i] = (float)q_level * NORM_64QAM;
        }
        break;

    case 8: /* 256-QAM: 8 bits -> 4 bits I, 4 bits Q */
        for (size_t i = 0; i < n_symbols; i++) {
            int i_level = gray_to_pam(&bits[i * 8], 4);
            int q_level = gray_to_pam(&bits[i * 8 + 4], 4);
            out_real[i] = (float)i_level * NORM_256QAM;
            out_imag[i] = (float)q_level * NORM_256QAM;
        }
        break;

    default:
        /* Unsupported n_bpsc: zero output to avoid undefined data */
        for (size_t i = 0; i < n_symbols; i++) {
            out_real[i] = 0.0f;
            out_imag[i] = 0.0f;
        }
        break;
    }
}
