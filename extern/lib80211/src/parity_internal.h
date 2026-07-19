#ifndef LIB80211_PARITY_INTERNAL_H
#define LIB80211_PARITY_INTERNAL_H

/**
 * parity_internal.h — Shared bit parity helper.
 *
 * Used by conv_encode.c and viterbi.c.
 */

#include <stdint.h>

static inline uint8_t lib80211_parity(unsigned x) {
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (uint8_t)(x & 1);
}

#endif /* LIB80211_PARITY_INTERNAL_H */
