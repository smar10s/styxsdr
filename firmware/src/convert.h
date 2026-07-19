// SPDX-License-Identifier: MIT
#ifndef STYX_CONVERT_H
#define STYX_CONVERT_H

#include <stdint.h>
#include <stddef.h>

/*
 * IQ format conversion between DDR packed samples and lib80211 split-float.
 *
 * DDR format: {8'b0, imag[11:0], real[11:0]} packed in 32-bit words.
 * lib80211:   separate float *real, float *imag arrays, scaled as int12 / 2048.0f.
 */

/**
 * Unpack DDR RX samples to split-complex float for lib80211.
 * Scales 12-bit signed integers to [-1.0, +1.0) range (divide by 2048).
 */
void convert_rx_to_float(const volatile uint32_t *ddr_buf, size_t n_samples,
                         float *out_real, float *out_imag);

/**
 * Pack split-complex float into DDR TX format.
 * peak_scale: multiply float values by this before clamping to [-2047, +2047].
 */
void convert_float_to_tx(const float *in_real, const float *in_imag,
                         size_t n_samples, float peak_scale,
                         volatile uint32_t *ddr_buf);

/**
 * Auto-scaling variant: finds peak magnitude across both I and Q,
 * computes scale factor to map peak to 2047, then packs.
 * Returns the scale factor used (useful for reporting backoff).
 */
float convert_float_to_tx_auto(const float *in_real, const float *in_imag,
                               size_t n_samples,
                               volatile uint32_t *ddr_buf);

#endif /* STYX_CONVERT_H */
