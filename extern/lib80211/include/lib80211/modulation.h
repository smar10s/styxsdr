#ifndef LIB80211_MODULATION_H
#define LIB80211_MODULATION_H

#include <stdint.h>
#include <stddef.h>

/**
 * Gray-coded QAM modulator.
 *
 * Maps groups of n_bpsc bits to complex constellation points.
 * Supported: BPSK(1), QPSK(2), 16-QAM(4), 64-QAM(6), 256-QAM(8).
 * Normalization: average power = 1.
 *
 * @param bits       Input bit array (n_symbols * n_bpsc bits)
 * @param out_real   Output I component
 * @param out_imag   Output Q component
 * @param n_symbols  Number of QAM symbols to produce
 * @param n_bpsc     Bits per subcarrier (1, 2, 4, 6, or 8)
 */
void lib80211_modulate(const uint8_t *bits, float *out_real, float *out_imag,
                       size_t n_symbols, int n_bpsc);

/**
 * Approximate max-log LLR soft demapper.
 *
 * Given received constellation points, compute soft-bit LLRs.
 * Convention: positive LLR = bit 1 more likely.
 * Supported: BPSK(1), QPSK(2), 16-QAM(4), 64-QAM(6), 256-QAM(8).
 *
 * @param rx_real    Received I component (n_symbols)
 * @param rx_imag    Received Q component (n_symbols)
 * @param soft_bits  Output LLR array (n_symbols * n_bpsc values)
 * @param n_symbols  Number of received symbols
 * @param n_bpsc     Bits per subcarrier (1, 2, 4, 6, or 8)
 */
void lib80211_soft_demap(const float *rx_real, const float *rx_imag,
                         float *soft_bits, size_t n_symbols, int n_bpsc);

#endif /* LIB80211_MODULATION_H */
