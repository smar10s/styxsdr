// SPDX-License-Identifier: MIT
#ifndef STYX_DMA_TX_H
#define STYX_DMA_TX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Maximum TX samples (4 MB / 4 bytes = 1,048,576) */
#define DMA_TX_MAX_SAMPLES  (0x00400000 / 4)

/**
 * Load waveform into DDR and start TX DMA.
 *
 * Stops any existing TX, converts float samples to DDR format (auto-scaled),
 * configures DDR_BASE + TX_COUNT, enables and triggers playback.
 *
 * @param in_real    Real (I) component, lib80211 float format
 * @param in_imag    Imaginary (Q) component, lib80211 float format
 * @param n_samples  Number of samples (must be <= DMA_TX_MAX_SAMPLES)
 * @param cyclic     If true, loop playback continuously
 * @return           0 on success, -1 on error
 */
int dma_tx_start(const float *in_real, const float *in_imag,
                 size_t n_samples, bool cyclic);

/**
 * Two-phase TX: load waveform and configure DMA without triggering.
 *
 * Use with dma_tx_trigger() when precise timing control is needed
 * (e.g., recording RX position between load and trigger).
 *
 * @param in_real    Real (I) component, lib80211 float format
 * @param in_imag    Imaginary (Q) component, lib80211 float format
 * @param n_samples  Number of samples (must be <= DMA_TX_MAX_SAMPLES)
 * @param cyclic     If true, loop playback continuously when triggered
 * @return           0 on success, -1 on error
 */
int dma_tx_load(const float *in_real, const float *in_imag,
                size_t n_samples, bool cyclic);

/**
 * Trigger a pre-loaded TX waveform.  Must call dma_tx_load() first.
 * This is a single register write — returns in microseconds.
 *
 * @return  0 on success, -1 if TX failed to start
 */
int dma_tx_trigger(void);

/**
 * Stop TX DMA engine.
 */
void dma_tx_stop(void);

/**
 * Check if one-shot TX playback is complete.
 * Returns true if STATUS bit 1 indicates done.
 */
bool dma_tx_done(void);

#endif /* STYX_DMA_TX_H */
