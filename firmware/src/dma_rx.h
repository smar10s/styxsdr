// SPDX-License-Identifier: MIT
#ifndef STYX_DMA_RX_H
#define STYX_DMA_RX_H

#include <stdint.h>
#include <stddef.h>

/* Ring buffer capacity in samples (128 MB / 4 bytes = 33,554,432) */
#define DMA_RX_BUF_SAMPLES  (0x08000000 / 4)

/**
 * Start the RX DMA engine.
 * Writes DDR_BASE register, enables capture.
 * Returns 0 on success, -1 if DMA fails to become active.
 */
int dma_rx_start(void);

/**
 * Stop the RX DMA engine.
 */
void dma_rx_stop(void);

/**
 * Read current write pointer (sample index into ring buffer).
 */
uint32_t dma_rx_wr_ptr(void);

/**
 * Capture n_samples of fresh IQ data from the DMA ring buffer.
 *
 * Waits for sufficient new data (polls WR_PTR with 50us sleep intervals).
 * Handles ring buffer wrap-around.
 * Converts captured DDR samples to split-complex float.
 *
 * @param n_samples   Number of samples to capture (must be <= DMA_RX_BUF_SAMPLES)
 * @param out_real    Output buffer for real (I) component
 * @param out_imag    Output buffer for imaginary (Q) component
 * @param timeout_ms  Timeout in milliseconds (0 = no timeout)
 * @return            Number of samples actually captured, or -1 on timeout/error
 */
int dma_rx_capture(size_t n_samples, float *out_real, float *out_imag,
                   uint32_t timeout_ms);

#endif /* STYX_DMA_RX_H */
