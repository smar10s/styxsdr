// SPDX-License-Identifier: MIT
#ifndef STYX_DMA_TX_H
#define STYX_DMA_TX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Maximum TX samples (32 MB / 4 bytes = 8,388,608) */
#define DMA_TX_MAX_SAMPLES  (0x02000000 / 4)

/*
 * Recommended feed chunk size for streaming.  Chosen to balance DDR
 * burst efficiency (larger chunks amortise AXI transaction overhead)
 * against ARM-side buffering latency.  The fill FSM issues 32-sample
 * bursts, so any multiple of 32 works.  4096 samples = 128 bursts
 * per feed, giving ~100 µs of headroom at 20 MSPS between ARM feeds.
 */
#define DMA_TX_STREAM_CHUNK  4096

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

// ---- Continuous streaming API ----

/**
 * Start continuous TX streaming.
 *
 * Configures the TX DMA for ring-buffer streaming: writes DDR_BASE,
 * sets TX_COUNT to the buffer size (in samples), enables cyclic+stream
 * mode, and triggers.  The caller must then call dma_tx_stream_feed()
 * to supply data continuously.
 *
 * The ring buffer is DMA_TX_MAX_SAMPLES samples (32 MB).  The fill FSM
 * reads DDR data automatically and wraps at the buffer boundary.  The
 * drain FSM outputs samples to the DAC with zero bubbles.
 *
 * @return  0 on success, -1 on error
 */
int dma_tx_stream_start(void);

/**
 * Feed a chunk of samples into the streaming TX ring buffer.
 *
 * Converts float IQ samples to DDR format (auto-scaled), writes them
 * into the ring buffer at the current write cursor, and updates the
 * FPGA's WR_PTR register so the fill FSM knows new data is available.
 * Issues a memory barrier before updating WR_PTR.
 *
 * The caller should call dma_tx_stream_available() first to ensure
 * there is room.  Calling feed() when the buffer is full will over-
 * write data the fill FSM hasn't consumed yet, producing torn output.
 *
 * @param in_real    Real (I) component
 * @param in_imag    Imaginary (Q) component
 * @param n_samples  Number of samples to feed (should be > 0)
 * @return           0 on success, -1 if buffer would overflow
 */
int dma_tx_stream_feed(const float *in_real, const float *in_imag,
                       size_t n_samples);

/**
 * Return the FPGA's current read position (RD_PTR register).
 *
 * This is a Gray-code-synchronized, conservative value: the actual
 * FPGA read position may be slightly ahead.  Use this to determine
 * which regions of the ring buffer are safe to overwrite.
 *
 * @return  Sample index (0 .. DMA_TX_MAX_SAMPLES-1)
 */
uint32_t dma_tx_stream_rd_ptr(void);

/**
 * Return how many samples can safely be written without overrunning
 * the FPGA's read position.
 *
 * Computed as (rd_ptr - wr_cursor - 1) modulo buffer size.  Always
 * leaves at least 1 sample of headroom to avoid fill_ptr == rd_ptr
 * ambiguity (fill FSM stops at fill_ptr >= wr_ptr, so wr_ptr must
 * never catch up to rd_ptr from the other side).
 *
 * @return  number of samples available for feeding
 */
int dma_tx_stream_available(void);

/**
 * Stop the streaming TX engine gracefully.
 *
 * Writes enable=0 + trigger=1 to latch the disable into the l_clk
 * domain.  The current buffer iteration completes and then the FSM
 * halts (tx_done asserts).
 */
void dma_tx_stream_stop(void);

#endif /* STYX_DMA_TX_H */
