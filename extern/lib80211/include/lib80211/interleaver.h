#ifndef LIB80211_INTERLEAVER_H
#define LIB80211_INTERLEAVER_H

#include <stdint.h>
#include <stddef.h>

/* Forward declaration for soft-bit deinterleaver */

/**
 * Legacy interleaver (IEEE 802.11-2020, Equation 17-17).
 * Two permutations applied per OFDM symbol.
 *
 * @param in_bits   Input coded bits (one bit per byte)
 * @param out_bits  Output interleaved bits
 * @param n_cbps    Coded bits per OFDM symbol
 * @param n_bpsc    Bits per subcarrier
 */
void lib80211_interleave(const uint8_t *in_bits, uint8_t *out_bits,
                         int n_cbps, int n_bpsc);

/**
 * HT interleaver (IEEE 802.11-2020, Section 19.3.11.8.2).
 * Uses N_col=13 instead of 16.
 *
 * @param in_bits   Input coded bits
 * @param out_bits  Output interleaved bits
 * @param n_cbps    Coded bits per OFDM symbol (52 * n_bpsc for HT 20MHz)
 * @param n_bpsc    Bits per subcarrier
 */
void lib80211_interleave_ht(const uint8_t *in_bits, uint8_t *out_bits,
                            int n_cbps, int n_bpsc);

/**
 * Legacy soft-bit deinterleaver (inverse of lib80211_interleave).
 * Applies inverse permutation on float soft-bit arrays.
 *
 * @param in_soft   Input interleaved soft bits (float per bit)
 * @param out_soft  Output deinterleaved soft bits
 * @param n_cbps    Coded bits per OFDM symbol
 * @param n_bpsc    Bits per subcarrier
 */
void lib80211_deinterleave(const float *in_soft, float *out_soft,
                           int n_cbps, int n_bpsc);

/**
 * HT soft-bit deinterleaver (inverse of lib80211_interleave_ht).
 * Uses N_col=13 instead of 16.
 *
 * @param in_soft   Input interleaved soft bits (float per bit)
 * @param out_soft  Output deinterleaved soft bits
 * @param n_cbps    Coded bits per OFDM symbol (52 * n_bpsc for HT 20MHz)
 * @param n_bpsc    Bits per subcarrier
 */
void lib80211_deinterleave_ht(const float *in_soft, float *out_soft,
                              int n_cbps, int n_bpsc);

#endif /* LIB80211_INTERLEAVER_H */
