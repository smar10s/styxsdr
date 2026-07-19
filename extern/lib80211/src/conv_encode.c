/**
 * conv_encode.c — Rate-1/2 K=7 convolutional encoder
 *
 * IEEE 802.11-2020 Section 17.3.5.5.
 * Generator polynomials: G0 = 133 (octal), G1 = 171 (octal).
 */

#include "lib80211/fec.h"
#include "parity_internal.h"

#define G0 0133  /* 0b1011011 — output A */
#define G1 0171  /* 0b1111001 — output B */

void lib80211_conv_encode(const uint8_t *in_bits, uint8_t *out_bits,
                          size_t n_input_bits, bool add_tail) {
    size_t n_total = n_input_bits + (add_tail ? 6 : 0);
    unsigned state = 0;  /* 6-bit shift register */

    for (size_t i = 0; i < n_total; i++) {
        uint8_t bit = (i < n_input_bits) ? in_bits[i] : 0;
        unsigned reg = ((unsigned)bit << 6) | state;

        out_bits[2 * i]     = lib80211_parity(reg & G0);
        out_bits[2 * i + 1] = lib80211_parity(reg & G1);

        state = (reg >> 1) & 0x3F;
    }
}
