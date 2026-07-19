#ifndef LIB80211_OFDM_H
#define LIB80211_OFDM_H

#include "lib80211/fft.h"
#include <stdint.h>
#include <stddef.h>

/**
 * Construct one legacy OFDM symbol: map 48 data values to subcarriers,
 * insert pilots with polarity, perform 64-pt IFFT, prepend cyclic prefix.
 *
 * @param plan          FFT plan
 * @param data_real     48 data subcarrier I values
 * @param data_imag     48 data subcarrier Q values
 * @param symbol_index  OFDM symbol index (for pilot polarity, 0-based)
 * @param out_real      Output: 80 time-domain samples (CP + 64)
 * @param out_imag      Output: 80 time-domain samples (CP + 64)
 */
void lib80211_make_ofdm_symbol(lib80211_fft_plan *plan,
                               const float *data_real, const float *data_imag,
                               int symbol_index,
                               float *out_real, float *out_imag);

/**
 * Construct one HT OFDM symbol: map 52 data values to subcarriers,
 * insert HT pilots with per-subcarrier cyclic pattern, IFFT + CP.
 *
 * @param plan          FFT plan
 * @param data_real     52 data subcarrier I values
 * @param data_imag     52 data subcarrier Q values
 * @param symbol_index  DATA symbol index (0-based, for pilot formula)
 * @param cp_len        Cyclic prefix length (16 for normal GI, 8 for SGI)
 * @param out_real      Output: (cp_len + 64) time-domain samples
 * @param out_imag      Output: (cp_len + 64) time-domain samples
 */
void lib80211_make_ht_ofdm_symbol(lib80211_fft_plan *plan,
                                  const float *data_real, const float *data_imag,
                                  int symbol_index, int cp_len,
                                  float *out_real, float *out_imag);

/**
 * Generate STF (Short Training Field): 160 time-domain samples.
 */
void lib80211_generate_stf(lib80211_fft_plan *plan,
                           float *out_real, float *out_imag);

/**
 * Generate LTF (Long Training Field): 160 time-domain samples.
 * Structure: GI2 (32 samples) + T1 (64 samples) + T2 (64 samples).
 */
void lib80211_generate_ltf(lib80211_fft_plan *plan,
                           float *out_real, float *out_imag);

/**
 * Generate HT-STF: 80 time-domain samples (CP16 + 64).
 * IEEE 802.11-2020 Section 19.3.9.4.5.
 */
void lib80211_generate_ht_stf(lib80211_fft_plan *plan,
                              float *out_real, float *out_imag);

/**
 * Generate HT-LTF: 80 time-domain samples (CP16 + 64).
 * For 1 spatial stream. Uses extended subcarriers ±28.
 */
void lib80211_generate_ht_ltf(lib80211_fft_plan *plan,
                              float *out_real, float *out_imag);

#endif /* LIB80211_OFDM_H */
