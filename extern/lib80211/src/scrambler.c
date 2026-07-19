/**
 * scrambler.c — 802.11 data scrambler (x^7 + x^4 + 1 LFSR)
 *
 * IEEE 802.11-2020 Section 17.3.5.4.
 * The LFSR generates a pseudo-random sequence XORed with the data.
 * Self-inverse: same operation for scramble and descramble.
 */

#include "lib80211/scrambler.h"

void lib80211_scramble(const uint8_t *in_bits, uint8_t *out_bits,
                       size_t n_bits, uint8_t seed) {
    uint8_t state = seed & 0x7F;  /* 7-bit LFSR state */

    for (size_t i = 0; i < n_bits; i++) {
        /* Feedback: bit6 XOR bit3 (x^7 + x^4 + 1) */
        uint8_t feedback = ((state >> 6) ^ (state >> 3)) & 1;

        /* XOR input with feedback */
        out_bits[i] = in_bits[i] ^ feedback;

        /* Shift register: new bit enters at position 0 */
        state = ((state << 1) | feedback) & 0x7F;
    }
}
