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
 * Arm the streaming engine without starting it.
 *
 * Resets the TX path, zeroes the DDR ring, and configures DDR_BASE,
 * TX_COUNT and WR_PTR — but leaves ENABLE and TRIGGER clear so the drain
 * FSM stays idle.  Feed one or more chunks, then call
 * dma_tx_stream_trigger().
 *
 * Triggering an empty ring is a trap: the drain FSM immediately runs
 * ahead of the write cursor, and every subsequent feed is chasing a read
 * pointer that is already in front of it.  Pre-filling first is the same
 * two-phase discipline dma_tx_load()/dma_tx_trigger() uses for one-shot
 * TX.
 *
 * @return  0 on success
 */
int dma_tx_stream_arm(void);

/**
 * Start the armed streaming engine.  Single register write.
 *
 * @return  0 on success, -1 if the engine failed to start
 */
int dma_tx_stream_trigger(void);

/**
 * Signed buffer depth, in samples.
 *
 * Positive: the ARM is that many samples ahead of the FPGA — the healthy
 * case, and the value to compare against a latency budget.
 * Negative: the drain FSM has overtaken the write cursor and is emitting
 * stale ring contents.  That is an underrun, and for FM it is audible as
 * time-stretched ("dragging") audio rather than a click, because the
 * modulation timeline lives in the phase trajectory.
 *
 * Prefer this over dma_tx_stream_available() for flow control: a plain
 * unsigned "space free" figure reports a small value both when the ring
 * is full and when it has underrun, so a feeder that waits on it will
 * throttle itself into a permanent underrun it can never exit.
 *
 * @return  samples queued (positive) or overrun distance (negative)
 */
int32_t dma_tx_stream_depth(void);

/**
 * Total samples the FPGA has emitted since dma_tx_stream_trigger().
 *
 * Monotonic across ring wraps.  Divide by elapsed time to get the true
 * output sample rate: the drain FSM only advances RD_PTR when it actually
 * emits, so a deficit against the nominal DAC rate is exactly the
 * time-stretch a receiver hears.
 *
 * @return  samples emitted
 */
uint64_t dma_tx_stream_emitted(void);

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
 * Feed a chunk using a caller-supplied fixed scale instead of auto-scaling.
 *
 * Prefer this whenever |IQ| is known by construction, as it is for FM and
 * any other constant-envelope modulation.  Two reasons:
 *
 *  - Speed.  convert_float_to_tx_auto() makes a peak-finding pass over the
 *    chunk before packing it, and that pass costs more than an entire FM
 *    modulation step.  Measured on a Cortex-A9 at 2.1 MSPS it is the
 *    difference between 1.69x and 1.82x headroom.
 *
 *  - Correctness.  Auto-scale normalises whatever chunk it is handed, so a
 *    short or quiet chunk gets boosted to full scale and the transmitted
 *    amplitude steps at every chunk boundary.
 *
 * @param in_real     Real (I) component
 * @param in_imag     Imaginary (Q) component
 * @param n_samples   Number of samples to feed
 * @param peak_scale  Multiplier applied before clamping to [-2047, 2047];
 *                    use 2047.0f for unit-magnitude input
 * @return            0 on success, -1 if the buffer would overflow
 */
int dma_tx_stream_feed_fixed(const float *in_real, const float *in_imag,
                             size_t n_samples, float peak_scale);

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
 * Derived from dma_tx_stream_depth(), so an underrun correctly reports
 * the whole ring as writable rather than "nearly full".  Always leaves 1
 * sample of headroom to avoid fill_ptr == rd_ptr ambiguity.
 *
 * @return  number of samples available for feeding
 */
int dma_tx_stream_available(void);

/**
 * Block until the FPGA has consumed all data written to the DDR
 * streaming buffer at the given sample rate.  Call this after the
 * last dma_tx_stream_feed() and before dma_tx_stream_stop() to
 * avoid cutting off unread data.
 *
 * Uses a time-based drain: computes (wr_cursor - rd_ptr) samples
 * remaining and sleeps for (samples / sample_rate) + 50 ms margin.
 * The rd_ptr is a conservative Gray-code-synchronized value from the
 * FPGA, so the pending count can only overestimate — never truncate
 * unread data.
 *
 * @param sample_rate_hz  DAC sample rate (Hz)
 */
void dma_tx_stream_drain(uint32_t sample_rate_hz);

/**
 * Stop the streaming TX engine gracefully.
 *
 * Writes enable=0 + trigger=1 to latch the disable into the l_clk
 * domain.  The current buffer iteration completes and then the FSM
 * halts (tx_done asserts).
 */
void dma_tx_stream_stop(void);

#endif /* STYX_DMA_TX_H */
