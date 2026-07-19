#ifndef LIB80211_TX_INTERNAL_H
#define LIB80211_TX_INTERNAL_H

/**
 * tx_internal.h — Shared TX helper functions.
 *
 * Used by tx_ht.c and tx_vht.c.
 */

#include <stddef.h>

/**
 * Apply a scalar gain to an IQ buffer in-place.
 */
static inline void lib80211_scale_iq(float *real, float *imag, size_t n, float gain) {
    for (size_t i = 0; i < n; i++) {
        real[i] *= gain;
        imag[i] *= gain;
    }
}

#endif /* LIB80211_TX_INTERNAL_H */
