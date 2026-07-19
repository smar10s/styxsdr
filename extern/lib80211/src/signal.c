/**
 * signal.c — L-SIG field generation
 *
 * IEEE 802.11-2020 Section 17.3.4.
 * The SIGNAL field is always BPSK, rate 1/2, on 48 legacy subcarriers.
 */

#include "lib80211/signal.h"
#include "lib80211/constants.h"
#include "lib80211/fec.h"
#include "lib80211/interleaver.h"
#include "lib80211/modulation.h"
#include "lib80211/ofdm.h"

int lib80211_make_signal_bits(int rate_mbps, int length_bytes, uint8_t *out_bits) {
    const lib80211_rate_info *rate = lib80211_rate_lookup(rate_mbps);
    if (!rate) return -1;

    /* Bits 0-3: rate (4 bits, R1 at bit 0 = LSB) */
    out_bits[0] = (rate->rate_bits >> 0) & 1;
    out_bits[1] = (rate->rate_bits >> 1) & 1;
    out_bits[2] = (rate->rate_bits >> 2) & 1;
    out_bits[3] = (rate->rate_bits >> 3) & 1;

    /* Bit 4: reserved (0) */
    out_bits[4] = 0;

    /* Bits 5-16: length in bytes (12 bits, LSB first) */
    for (int i = 0; i < 12; i++)
        out_bits[5 + i] = (length_bytes >> i) & 1;

    /* Bit 17: even parity over bits 0-16 */
    uint8_t parity = 0;
    for (int i = 0; i < 17; i++)
        parity ^= out_bits[i];
    out_bits[17] = parity;

    /* Bits 18-23: tail (6 zeros) */
    for (int i = 18; i < 24; i++)
        out_bits[i] = 0;

    return 0;
}

void lib80211_make_lsig_symbol(lib80211_fft_plan *plan,
                               int rate_mbps, int length_bytes,
                               float *out_real, float *out_imag) {
    uint8_t sig_bits[24];
    lib80211_make_signal_bits(rate_mbps, length_bytes, sig_bits);

    /* Rate-1/2 convolutional encode (24 bits -> 48 coded bits, no extra tail) */
    uint8_t coded[48];
    lib80211_conv_encode(sig_bits, coded, 24, false);

    /* Interleave (n_cbps=48, n_bpsc=1 for BPSK) */
    uint8_t interleaved[48];
    lib80211_interleave(coded, interleaved, 48, 1);

    /* BPSK modulate -> 48 subcarrier values */
    float data_real[48], data_imag[48];
    lib80211_modulate(interleaved, data_real, data_imag, 48, 1);

    /* Build OFDM symbol (symbol_index = 0 for SIGNAL field polarity) */
    lib80211_make_ofdm_symbol(plan, data_real, data_imag, 0, out_real, out_imag);
}
