/**
 * vhtsig.c — VHT-SIG-A and VHT-SIG-B field generation (802.11ac).
 *
 * VHT-SIG-A: 48 bits (34 data + 8 CRC-8 + 6 tail), 2 OFDM symbols.
 *   Symbol 1: BPSK (I-axis), Symbol 2: Q-BPSK (Q-axis).
 *   Uses legacy interleaver (n_cbps=48, n_bpsc=1) and legacy subcarrier map.
 *
 * VHT-SIG-B: 26 bits (17 LENGTH + 3 reserved + 6 tail), 1 OFDM symbol.
 *   BPSK, HT interleaver (n_cbps=52, n_bpsc=1), HT subcarrier map.
 */

#include "lib80211/signal.h"
#include "lib80211/constants.h"
#include "lib80211/fec.h"
#include "lib80211/interleaver.h"
#include "lib80211/modulation.h"
#include "lib80211/ofdm.h"

#include <string.h>

void lib80211_make_vhtsiga_bits(int mcs, int length_bytes, bool short_gi,
                                bool coding_ldpc, int ldpc_extra,
                                uint8_t *out_bits) {
    /* 34 data bits */
    uint8_t data_bits[34];
    memset(data_bits, 0, sizeof(data_bits));

    /* Symbol 1 [0:23] */
    /* BW: bits[0:1] = 0 (20 MHz) */
    data_bits[0] = 0;
    data_bits[1] = 0;

    /* Reserved: bit[2] = 1 */
    data_bits[2] = 1;

    /* STBC: bit[3] = 0 */
    data_bits[3] = 0;

    /* Group ID: bits[4:9] = 0 */
    /* already zero */

    /* NSTS-1: bits[10:12] = 0 (1 spatial stream) */
    /* already zero */

    /* Partial AID: bits[13:21] = 0x15D (9 bits LSB first) */
    int paid = 0x15D;
    for (int i = 0; i < 9; i++)
        data_bits[13 + i] = (uint8_t)((paid >> i) & 1);

    /* TXOP_PS_NOT_ALLOWED: bit[22] = 0 */
    data_bits[22] = 0;

    /* Reserved: bit[23] = 1 */
    data_bits[23] = 1;

    /* Symbol 2 [24:33] */
    /* Short GI: bit[24] */
    data_bits[24] = short_gi ? 1 : 0;

    /* SGI disambig: bit[25] = 0 */
    data_bits[25] = 0;

    /* Coding: bit[26] (0=BCC, 1=LDPC) */
    data_bits[26] = coding_ldpc ? 1 : 0;

    /* LDPC extra: bit[27] */
    data_bits[27] = (uint8_t)(ldpc_extra & 1);

    /* MCS: bits[28:31] (4 bits LSB first) */
    for (int i = 0; i < 4; i++)
        data_bits[28 + i] = (uint8_t)((mcs >> i) & 1);

    /* Beamformed: bit[32] = 0 */
    data_bits[32] = 0;

    /* Reserved: bit[33] = 1 */
    data_bits[33] = 1;

    /* Copy data bits to output */
    memcpy(out_bits, data_bits, 34);

    /* CRC-8: same polynomial as HT-SIG, ones-complement, MSB-first */
    uint8_t crc = lib80211_htsig_crc8(data_bits, 34);
    uint8_t crc_inv = (uint8_t)(~crc & 0xFF);
    for (int i = 0; i < 8; i++)
        out_bits[34 + i] = (uint8_t)((crc_inv >> (7 - i)) & 1);

    /* Tail: 6 zeros at bits [42:47] */
    memset(&out_bits[42], 0, 6);

    (void)length_bytes;  /* length_bytes not used in VHT-SIG-A */
}

void lib80211_make_vhtsiga_symbols(lib80211_fft_plan *plan,
                                   int mcs, int length_bytes, bool short_gi,
                                   bool coding_ldpc, int ldpc_extra,
                                   float *out_real, float *out_imag) {
    /* Generate 48 VHT-SIG-A bits */
    uint8_t vhtsiga_bits[48];
    lib80211_make_vhtsiga_bits(mcs, length_bytes, short_gi,
                               coding_ldpc, ldpc_extra, vhtsiga_bits);

    /* BCC encode: 48 bits -> 96 coded bits (rate-1/2) */
    uint8_t coded[96];
    lib80211_conv_encode(vhtsiga_bits, coded, 48, false);

    /* Process 2 OFDM symbols */
    for (int sym = 0; sym < 2; sym++) {
        uint8_t *sym_coded = &coded[sym * 48];

        /* Legacy interleave (N_col=16, n_cbps=48, n_bpsc=1) */
        uint8_t interleaved[48];
        lib80211_interleave(sym_coded, interleaved, 48, 1);

        /* BPSK modulate */
        float mod_real[48], mod_imag[48];
        lib80211_modulate(interleaved, mod_real, mod_imag, 48, 1);

        /* Symbol 1: BPSK (data on I-axis, already correct from modulate)
         * Symbol 2: Q-BPSK (rotate to Q-axis) */
        float final_real[48], final_imag[48];
        if (sym == 0) {
            /* Symbol 1: BPSK on I-axis (no rotation) */
            memcpy(final_real, mod_real, 48 * sizeof(float));
            memcpy(final_imag, mod_imag, 48 * sizeof(float));
        } else {
            /* Symbol 2: Q-BPSK (rotate 90°: real -> imag) */
            for (int i = 0; i < 48; i++) {
                final_real[i] = 0.0f;
                final_imag[i] = mod_real[i];
            }
        }

        /* Build frequency-domain symbol with legacy subcarrier mapping */
        float freq_real[64] = {0};
        float freq_imag[64] = {0};

        for (int i = 0; i < 48; i++) {
            int bin = LIB80211_DATA_BINS[i];
            freq_real[bin] = final_real[i];
            freq_imag[bin] = final_imag[i];
        }

        /* Pilots: legacy polarity at symbol indices 1 and 2
         * (L-SIG is symbol 0, VHT-SIG-A1 is symbol 1, VHT-SIG-A2 is symbol 2) */
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

void lib80211_make_vhtsigb_bits(int psdu_length, uint8_t *out_bits) {
    memset(out_bits, 0, 26);

    /* LENGTH: ceil(psdu_length / 4), 17 bits LSB first */
    int length_field = (psdu_length + 3) / 4;
    for (int i = 0; i < 17; i++)
        out_bits[i] = (uint8_t)((length_field >> i) & 1);

    /* Reserved: 3 bits = 1,1,1 */
    out_bits[17] = 1;
    out_bits[18] = 1;
    out_bits[19] = 1;

    /* Tail: 6 zeros [20:25] — already zero from memset */
}

void lib80211_make_vhtsigb_symbol(lib80211_fft_plan *plan,
                                  int psdu_length,
                                  float *out_real, float *out_imag) {
    /* Generate 26 VHT-SIG-B bits */
    uint8_t sigb_bits[26];
    lib80211_make_vhtsigb_bits(psdu_length, sigb_bits);

    /* BCC encode: 26 bits -> 52 coded bits (rate-1/2) */
    uint8_t coded[52];
    lib80211_conv_encode(sigb_bits, coded, 26, false);

    /* HT interleave (N_col=13, n_cbps=52, n_bpsc=1) */
    uint8_t interleaved[52];
    lib80211_interleave_ht(coded, interleaved, 52, 1);

    /* BPSK modulate (52 symbols) */
    float mod_real[52], mod_imag[52];
    lib80211_modulate(interleaved, mod_real, mod_imag, 52, 1);

    /* Build frequency-domain symbol with HT subcarrier mapping (52 data bins) */
    float freq_real[64] = {0};
    float freq_imag[64] = {0};

    for (int i = 0; i < 52; i++) {
        int bin = LIB80211_HT_DATA_BINS[i];
        freq_real[bin] = mod_real[i];
        freq_imag[bin] = mod_imag[i];
    }

    /* Pilots: VHT-SIG-B uses PILOT_POLARITY[3] (the 4th symbol in the
     * frame: L-STF, L-LTF, L-SIG, VHT-SIG-A1, VHT-SIG-A2, VHT-SIG-B = index 3
     * in the z=3 pilot polarity sequence). Ref: IEEE 802.11ac-2013 §22.3.10.10 */
    float polarity = (float)LIB80211_PILOT_POLARITY[3];
    for (int k = 0; k < 4; k++) {
        int bin = LIB80211_HT_PILOT_BINS[k];
        freq_real[bin] = polarity * LIB80211_HT_PILOT_PATTERN[k];
        freq_imag[bin] = 0.0f;
    }

    /* 64-pt IFFT -> time domain */
    float time_real[64], time_imag[64];
    lib80211_fft_inverse(plan, freq_real, freq_imag, time_real, time_imag);

    /* Prepend cyclic prefix (normal GI: 16 samples) */
    memcpy(out_real, &time_real[48], 16 * sizeof(float));
    memcpy(out_imag, &time_imag[48], 16 * sizeof(float));
    memcpy(&out_real[16], time_real, 64 * sizeof(float));
    memcpy(&out_imag[16], time_imag, 64 * sizeof(float));
}
