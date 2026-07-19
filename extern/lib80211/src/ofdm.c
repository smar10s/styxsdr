/**
 * ofdm.c — OFDM symbol construction, STF and LTF generation.
 *
 * Handles subcarrier mapping, pilot insertion, IFFT, and cyclic prefix.
 */

#include "lib80211/ofdm.h"
#include "lib80211/constants.h"

#include <string.h>

void lib80211_make_ofdm_symbol(lib80211_fft_plan *plan,
                               const float *data_real, const float *data_imag,
                               int symbol_index,
                               float *out_real, float *out_imag) {
    /* Build frequency-domain symbol (64 bins) */
    float freq_real[64] = {0};
    float freq_imag[64] = {0};

    /* Map 48 data values to their subcarrier bins */
    for (int i = 0; i < 48; i++) {
        int bin = LIB80211_DATA_BINS[i];
        freq_real[bin] = data_real[i];
        freq_imag[bin] = data_imag[i];
    }

    /* Insert 4 pilots with polarity */
    int pol_idx = symbol_index % 127;
    float polarity = (float)LIB80211_PILOT_POLARITY[pol_idx];

    for (int i = 0; i < 4; i++) {
        int bin = LIB80211_PILOT_BINS[i];
        freq_real[bin] = polarity * LIB80211_PILOT_BASE[i];
        freq_imag[bin] = 0.0f;
    }

    /* 64-point IFFT -> time domain */
    float time_real[64], time_imag[64];
    lib80211_fft_inverse(plan, freq_real, freq_imag, time_real, time_imag);

    /* Prepend cyclic prefix (last 16 samples) */
    memcpy(out_real, &time_real[48], 16 * sizeof(float));
    memcpy(out_imag, &time_imag[48], 16 * sizeof(float));
    memcpy(&out_real[16], time_real, 64 * sizeof(float));
    memcpy(&out_imag[16], time_imag, 64 * sizeof(float));
}

void lib80211_generate_stf(lib80211_fft_plan *plan,
                           float *out_real, float *out_imag) {
    /* IFFT of STF frequency domain */
    float time_real[64], time_imag[64];
    lib80211_fft_inverse(plan, LIB80211_STF_FREQ_REAL, LIB80211_STF_FREQ_IMAG,
                         time_real, time_imag);

    /* STF is 10 repetitions of the first 16 samples (160 total) */
    for (int rep = 0; rep < 10; rep++) {
        memcpy(&out_real[rep * 16], time_real, 16 * sizeof(float));
        memcpy(&out_imag[rep * 16], time_imag, 16 * sizeof(float));
    }
}

void lib80211_generate_ltf(lib80211_fft_plan *plan,
                           float *out_real, float *out_imag) {
    /* LTF freq is real-only */
    float zero_imag[64] = {0};
    float time_real[64], time_imag[64];
    lib80211_fft_inverse(plan, LIB80211_LTF_FREQ_REAL, zero_imag,
                         time_real, time_imag);

    /* GI2: last 32 samples of the 64-sample period */
    memcpy(out_real, &time_real[32], 32 * sizeof(float));
    memcpy(out_imag, &time_imag[32], 32 * sizeof(float));

    /* T1: full 64-sample period */
    memcpy(&out_real[32], time_real, 64 * sizeof(float));
    memcpy(&out_imag[32], time_imag, 64 * sizeof(float));

    /* T2: full 64-sample period (same as T1) */
    memcpy(&out_real[96], time_real, 64 * sizeof(float));
    memcpy(&out_imag[96], time_imag, 64 * sizeof(float));
}

void lib80211_make_ht_ofdm_symbol(lib80211_fft_plan *plan,
                                  const float *data_real, const float *data_imag,
                                  int symbol_index, int cp_len,
                                  float *out_real, float *out_imag) {
    /* Build frequency-domain symbol (64 bins) */
    float freq_real[64] = {0};
    float freq_imag[64] = {0};

    /* Map 52 data values to HT subcarrier bins */
    for (int i = 0; i < 52; i++) {
        int bin = LIB80211_HT_DATA_BINS[i];
        freq_real[bin] = data_real[i];
        freq_imag[bin] = data_imag[i];
    }

    /* Insert 4 HT pilots with per-subcarrier cyclic pattern:
     * pilot(n, k) = PILOT_POLARITY[(z_start + n) % 127] * HT_PILOT_PATTERN[(n + k) % 4]
     * where z_start=3 for HT-mixed DATA, n=symbol_index, k=pilot index */
    float polarity = (float)LIB80211_PILOT_POLARITY[
        (LIB80211_HT_PILOT_Z_START + symbol_index) % 127];

    for (int k = 0; k < 4; k++) {
        int bin = LIB80211_HT_PILOT_BINS[k];
        float pattern = LIB80211_HT_PILOT_PATTERN[(symbol_index + k) % 4];
        freq_real[bin] = polarity * pattern;
        freq_imag[bin] = 0.0f;
    }

    /* 64-point IFFT -> time domain */
    float time_real[64], time_imag[64];
    lib80211_fft_inverse(plan, freq_real, freq_imag, time_real, time_imag);

    /* Prepend cyclic prefix (last cp_len samples) */
    memcpy(out_real, &time_real[64 - cp_len], (size_t)cp_len * sizeof(float));
    memcpy(out_imag, &time_imag[64 - cp_len], (size_t)cp_len * sizeof(float));
    memcpy(&out_real[cp_len], time_real, 64 * sizeof(float));
    memcpy(&out_imag[cp_len], time_imag, 64 * sizeof(float));
}

void lib80211_generate_ht_stf(lib80211_fft_plan *plan,
                              float *out_real, float *out_imag) {
    /* HT-STF reuses the L-STF waveform but as a single symbol (CP16 + 64) */
    float time_real[64], time_imag[64];
    lib80211_fft_inverse(plan, LIB80211_STF_FREQ_REAL, LIB80211_STF_FREQ_IMAG,
                         time_real, time_imag);

    /* CP: last 16 samples */
    memcpy(out_real, &time_real[48], 16 * sizeof(float));
    memcpy(out_imag, &time_imag[48], 16 * sizeof(float));
    /* Data: 64 samples */
    memcpy(&out_real[16], time_real, 64 * sizeof(float));
    memcpy(&out_imag[16], time_imag, 64 * sizeof(float));
}

void lib80211_generate_ht_ltf(lib80211_fft_plan *plan,
                              float *out_real, float *out_imag) {
    /* HT-LTF uses extended subcarriers (±28), real-only */
    float zero_imag[64] = {0};
    float time_real[64], time_imag[64];
    lib80211_fft_inverse(plan, LIB80211_HT_LTF_FREQ_REAL, zero_imag,
                         time_real, time_imag);

    /* CP: last 16 samples */
    memcpy(out_real, &time_real[48], 16 * sizeof(float));
    memcpy(out_imag, &time_imag[48], 16 * sizeof(float));
    /* Data: 64 samples */
    memcpy(&out_real[16], time_real, 64 * sizeof(float));
    memcpy(&out_imag[16], time_imag, 64 * sizeof(float));
}
