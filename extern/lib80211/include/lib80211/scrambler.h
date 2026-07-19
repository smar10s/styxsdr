#ifndef LIB80211_SCRAMBLER_H
#define LIB80211_SCRAMBLER_H

#include <stdint.h>
#include <stddef.h>

/**
 * Scramble/descramble bits using the 802.11 LFSR (x^7 + x^4 + 1).
 * The scrambler is self-inverse: scramble(scramble(x)) = x.
 *
 * @param in_bits   Input bit array (one bit per byte, 0 or 1)
 * @param out_bits  Output bit array (may alias in_bits for in-place)
 * @param n_bits    Number of bits to process
 * @param seed      7-bit scrambler seed (bits 6..0, bit 6 is MSB of LFSR)
 */
void lib80211_scramble(const uint8_t *in_bits, uint8_t *out_bits,
                       size_t n_bits, uint8_t seed);

#endif /* LIB80211_SCRAMBLER_H */
