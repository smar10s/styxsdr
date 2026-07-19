/**
 * htsig.c — HT-SIG field generation (802.11n, Section 19.3.9.4.3).
 *
 * 48-bit HT-SIG: 34 data + 8 CRC-8 + 6 tail.
 * Encoded at rate-1/2, legacy interleaved, Q-BPSK modulated (2 OFDM symbols).
 */

#include "lib80211/signal.h"
#include "lib80211/constants.h"
#include "lib80211/fec.h"
#include "lib80211/interleaver.h"
#include "lib80211/modulation.h"
#include "lib80211/ofdm.h"

#include <string.h>

uint8_t lib80211_htsig_crc8(const uint8_t *bits, size_t n_bits) {
    uint8_t state = 0xFF;
    for (size_t i = 0; i < n_bits; i++) {
        uint8_t feedback = ((state >> 7) ^ bits[i]) & 1;
        state = (uint8_t)((state << 1) & 0xFF);
        if (feedback)
            state ^= 0x07;
    }
    return state;
}

void lib80211_make_htsig_bits(int mcs, int length_bytes,
                              bool short_gi, bool coding_ldpc,
                              uint8_t *out_bits) {
    /* 34 data bits */
    uint8_t data_bits[34];
    memset(data_bits, 0, sizeof(data_bits));

    /* MCS: bits [0:6] (7 bits, LSB first) */
    for (int i = 0; i < 7; i++)
        data_bits[i] = (uint8_t)((mcs >> i) & 1);

    /* BW: bit 7 (0 = 20 MHz) */
    data_bits[7] = 0;

    /* Length: bits [8:23] (16 bits LSB first) */
    for (int i = 0; i < 16; i++)
        data_bits[8 + i] = (uint8_t)((length_bytes >> i) & 1);

    /* Smoothing: bit 24 */
    data_bits[24] = 1;

    /* Not sounding: bit 25 */
    data_bits[25] = 1;

    /* Reserved: bit 26 (set to 1 per IEEE TGn reference) */
    data_bits[26] = 1;

    /* Aggregation: bit 27 */
    data_bits[27] = 0;

    /* STBC: bits [28:29] */
    data_bits[28] = 0;
    data_bits[29] = 0;

    /* FEC coding: bit 30 (0=BCC, 1=LDPC) */
    data_bits[30] = coding_ldpc ? 1 : 0;

    /* Short GI: bit 31 */
    data_bits[31] = short_gi ? 1 : 0;

    /* N_ESS: bits [32:33] */
    data_bits[32] = 0;
    data_bits[33] = 0;

    /* Copy data bits to output */
    memcpy(out_bits, data_bits, 34);

    /* CRC-8: ones-complement, MSB-first in bits [34:41] */
    uint8_t crc = lib80211_htsig_crc8(data_bits, 34);
    uint8_t crc_inv = (uint8_t)(~crc & 0xFF);
    for (int i = 0; i < 8; i++)
        out_bits[34 + i] = (uint8_t)((crc_inv >> (7 - i)) & 1);

    /* Tail: 6 zeros at bits [42:47] */
    memset(&out_bits[42], 0, 6);
}

void lib80211_make_htsig_symbols(lib80211_fft_plan *plan,
                                 int mcs, int length_bytes,
                                 bool short_gi, bool coding_ldpc,
                                 float *out_real, float *out_imag) {
    /* Generate 48 HT-SIG bits */
    uint8_t htsig_bits[48];
    lib80211_make_htsig_bits(mcs, length_bytes, short_gi, coding_ldpc, htsig_bits);

    /* BCC encode: 48 bits -> 96 coded bits (rate-1/2, no extra tail) */
    uint8_t coded[96];
    lib80211_conv_encode(htsig_bits, coded, 48, false);

    /* Process 2 OFDM symbols */
    for (int sym = 0; sym < 2; sym++) {
        uint8_t *sym_coded = &coded[sym * 48];

        /* Legacy interleave (N_col=16, n_cbps=48, n_bpsc=1 for BPSK) */
        uint8_t interleaved[48];
        lib80211_interleave(sym_coded, interleaved, 48, 1);

        /* BPSK modulate then rotate to Q-axis (Q-BPSK):
         * BPSK maps bit -> {-1, +1} on real axis.
         * Q-BPSK rotates 90°: real -> imag, imag stays 0 -> real stays 0.
         * So: bit 0 -> (0, -1), bit 1 -> (0, +1) */
        float mod_real[48], mod_imag[48];
        lib80211_modulate(interleaved, mod_real, mod_imag, 48, 1);

        /* Rotate to Q-axis: Q-BPSK means data goes on imag */
        float qbpsk_real[48], qbpsk_imag[48];
        for (int i = 0; i < 48; i++) {
            qbpsk_real[i] = 0.0f;        /* -imag (imag is 0 for BPSK) */
            qbpsk_imag[i] = mod_real[i];  /* real becomes imag */
        }

        /* Build frequency-domain symbol with legacy subcarrier mapping */
        float freq_real[64] = {0};
        float freq_imag[64] = {0};

        for (int i = 0; i < 48; i++) {
            int bin = LIB80211_DATA_BINS[i];
            freq_real[bin] = qbpsk_real[i];
            freq_imag[bin] = qbpsk_imag[i];
        }

        /* Pilots: legacy polarity at symbol index (sym + 1)
         * (L-SIG is symbol 0, HT-SIG1 is symbol 1, HT-SIG2 is symbol 2) */
        int pol_idx = (sym + 1) % 127;
        float polarity = (float)LIB80211_PILOT_POLARITY[pol_idx];
        for (int i = 0; i < 4; i++) {
            int bin = LIB80211_PILOT_BINS[i];
            freq_real[bin] = polarity * LIB80211_PILOT_BASE[i];
            freq_imag[bin] = 0.0f;
        }

        /* 64-pt IFFT -> time domain */
        float time_real[64], time_imag[64];
        lib80211_fft_inverse(plan, freq_real, freq_imag, time_real, time_imag);

        /* Prepend cyclic prefix (normal GI: 16 samples) */
        float *dst_real = &out_real[sym * 80];
        float *dst_imag = &out_imag[sym * 80];
        memcpy(dst_real, &time_real[48], 16 * sizeof(float));
        memcpy(dst_imag, &time_imag[48], 16 * sizeof(float));
        memcpy(&dst_real[16], time_real, 64 * sizeof(float));
        memcpy(&dst_imag[16], time_imag, 64 * sizeof(float));
    }
}
