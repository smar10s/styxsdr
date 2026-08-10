// SPDX-License-Identifier: MIT
#include "dma_tx.h"
#include "hal.h"
#include "convert.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

/* Ensure all prior stores (DDR waveform data) are visible to the
 * interconnect before triggering DMA.  On Cortex-A9, volatile alone
 * prevents compiler reordering but not store buffer reordering. */
static inline void memory_barrier(void) {
    __sync_synchronize();
}

/* Saved control word from dma_tx_load for dma_tx_trigger */
static uint32_t pending_ctrl = 0;

/* --------------------------------------------------------------------------
 * Internal: reset TX DMA to clean idle state
 * -------------------------------------------------------------------------- */
static void tx_reset(void)
{
    /* Write enable=0 + trigger to get FSMs back to idle */
    hal_reg_write(REG_IQ_DMA_TX_CONTROL, TX_CTRL_TRIGGER);
    usleep(1000);

    /* If tx_done is still asserted, toggle again to clear it */
    uint32_t status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    if (status & 0x02) {
        hal_reg_write(REG_IQ_DMA_TX_CONTROL, TX_CTRL_TRIGGER);
        usleep(1000);
    }
}

/* --------------------------------------------------------------------------
 * Two-phase TX API: load + trigger
 * -------------------------------------------------------------------------- */

int dma_tx_load(const float *in_real, const float *in_imag,
                size_t n_samples, bool cyclic)
{
    if (n_samples == 0) {
        fprintf(stderr, "dma_tx: zero samples requested\n");
        return -1;
    }

    if (n_samples > DMA_TX_MAX_SAMPLES) {
        fprintf(stderr, "dma_tx: %zu samples exceeds max (%u)\n",
                n_samples, DMA_TX_MAX_SAMPLES);
        return -1;
    }

    /* Reset to clean state */
    tx_reset();

    /* Convert float samples to DDR format (auto-scaled) */
    volatile uint32_t *tx_buf = hal_ddr_tx_buf();
    float scale = convert_float_to_tx_auto(in_real, in_imag, n_samples, tx_buf);
    (void)scale;

    /* All waveform data must be committed to DDR before the FPGA
     * starts reading. */
    memory_barrier();

    /* Configure DMA registers (but don't trigger yet) */
    hal_reg_write(REG_IQ_DMA_TX_DDR_BASE, DDR_TX_BASE);
    hal_reg_write(REG_IQ_DMA_TX_COUNT, (uint32_t)n_samples);

    /* Save control word for trigger phase */
    pending_ctrl = TX_CTRL_ENABLE | TX_CTRL_TRIGGER;
    if (cyclic)
        pending_ctrl |= TX_CTRL_CYCLIC;

    return 0;
}

int dma_tx_trigger(void)
{
    /* Single register write — fires immediately */
    hal_reg_write(REG_IQ_DMA_TX_CONTROL, pending_ctrl);

    /* Brief delay then verify TX started */
    usleep(100);
    uint32_t status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    if (!(status & 0x03)) {  /* neither active nor done */
        fprintf(stderr, "dma_tx: failed to start (status=0x%08x)\n", status);
        return -1;
    }

    return 0;
}

/* --------------------------------------------------------------------------
 * Combined API (convenience wrapper)
 * -------------------------------------------------------------------------- */

int dma_tx_start(const float *in_real, const float *in_imag,
                 size_t n_samples, bool cyclic)
{
    int rc = dma_tx_load(in_real, in_imag, n_samples, cyclic);
    if (rc != 0)
        return rc;
    return dma_tx_trigger();
}

/* --------------------------------------------------------------------------
 * Stop / status
 * -------------------------------------------------------------------------- */

void dma_tx_stop(void)
{
    /* Write enable=0 + trigger=1 to latch the disable into l_clk domain.
     * The trigger toggle crosses the CDC and causes lcl_enable=0 to be
     * captured by the fill/drain FSMs.  For one-shot TX that already
     * completed, this is a harmless no-op.  For cyclic TX, this causes
     * the current waveform iteration to finish and then stop. */
    hal_reg_write(REG_IQ_DMA_TX_CONTROL, TX_CTRL_TRIGGER);  /* enable=0, trigger=1 */

    /* Poll for completion — drain FSM sets tx_done when it stops.
     * Timeout after 100ms (well above any waveform duration). */
    for (int i = 0; i < 1000; i++) {
        uint32_t status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
        if ((status & 0x03) != 0x01)  /* not (active && !done) */
            return;
        usleep(100);
    }
    fprintf(stderr, "dma_tx: stop timeout (STATUS=0x%08x)\n",
            hal_reg_read(REG_IQ_DMA_TX_STATUS));
}

bool dma_tx_done(void)
{
    uint32_t status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    return (status & 0x02) != 0;  /* bit 1 = done */
}

/* --------------------------------------------------------------------------
 * Continuous streaming API
 * -------------------------------------------------------------------------- */

/* Local write cursor — advanced by dma_tx_stream_feed().  Tracks the
 * ARM's position within the ring buffer.  The FPGA reads from RD_PTR
 * to this cursor. */
static uint32_t stream_wr_cursor = 0;

/* Set once the streaming engine has been triggered.  Before that the
 * drain FSM is idle, so feeds must not consult RD_PTR (which still holds
 * a stale value from the previous run). */
static bool stream_running = false;

/* Monotonic sample counters.  See dma_tx_stream_depth() for why ring
 * positions alone are insufficient. */
static uint64_t stream_wr_total = 0;   /* samples handed to feed()    */
static uint64_t stream_rd_total = 0;   /* samples emitted by the FPGA  */
static uint32_t stream_rd_last  = 0;   /* previous RD_PTR reading      */

int dma_tx_stream_arm(void)
{
    /* Reset to clean state */
    tx_reset();

    /* Zero DDR buffer so the FPGA doesn't play stale IQ data from a
     * previous TX session before the first feed arrives. */
    memset((void *)hal_ddr_tx_buf(), 0,
           DMA_TX_MAX_SAMPLES * sizeof(uint32_t));
    memory_barrier();

    /* Reset cursors and counters */
    stream_wr_cursor = 0;
    stream_wr_total  = 0;
    stream_rd_total  = 0;
    stream_running   = false;

    /* Configure DMA for streaming: buffer-size TX_COUNT, cyclic+stream.
     * Deliberately does NOT set ENABLE or TRIGGER — the caller pre-fills
     * first, then calls dma_tx_stream_trigger().  Triggering with an
     * empty ring lets the drain FSM run ahead of the write cursor
     * immediately, and because dma_tx_stream_available() reports a small
     * value whenever RD_PTR is ahead of the cursor, the feeder then
     * throttles itself and can never catch up. */
    hal_reg_write(REG_IQ_DMA_TX_DDR_BASE, DDR_TX_BASE);
    hal_reg_write(REG_IQ_DMA_TX_COUNT, DMA_TX_MAX_SAMPLES);
    hal_reg_write(REG_IQ_DMA_TX_WR_PTR, 0);  /* no data yet */

    return 0;
}

int dma_tx_stream_trigger(void)
{
    uint32_t ctrl = TX_CTRL_ENABLE | TX_CTRL_TRIGGER
                  | TX_CTRL_CYCLIC | TX_CTRL_STREAM;
    hal_reg_write(REG_IQ_DMA_TX_CONTROL, ctrl);

    /* Brief delay then verify TX started */
    usleep(100);
    uint32_t status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    if (!(status & 0x03)) {
        fprintf(stderr, "dma_tx_stream: failed to start (status=0x%08x)\n",
                status);
        return -1;
    }

    /* RD_PTR is reset by the drain FSM when it leaves D_IDLE.  Latch the
     * origin now so the monotonic accumulator starts from a known point. */
    stream_rd_last = hal_reg_read(REG_IQ_DMA_TX_RD_PTR);
    stream_rd_total = 0;
    stream_running = true;
    return 0;
}

int dma_tx_stream_start(void)
{
    if (dma_tx_stream_arm() != 0)
        return -1;
    return dma_tx_stream_trigger();
}

/*
 * Shared body for both feed variants.  Everything except the pack call is
 * identical, and the ring-wrap split plus the monotonic write accounting
 * are exactly the parts that must not be duplicated.
 *
 * auto_scale selects convert_float_to_tx_auto() (peak-finding pass, then
 * pack); otherwise peak_scale is applied directly.
 */
static int stream_feed(const float *in_real, const float *in_imag,
                       size_t n_samples, bool auto_scale, float peak_scale)
{
    if (n_samples == 0)
        return 0;

    if (n_samples > (size_t)DMA_TX_MAX_SAMPLES) {
        fprintf(stderr, "dma_tx_stream: %zu samples exceeds buffer\n", n_samples);
        return -1;
    }

    /* Check available room before writing.  Skipped before trigger: the
     * drain FSM is idle, so the entire ring is writable and RD_PTR is
     * stale. */
    if (stream_running) {
        int avail = dma_tx_stream_available();
        if (avail < 0 || (size_t)avail < n_samples) {
            fprintf(stderr, "dma_tx_stream: buffer full (need %zu, have %d)\n",
                    n_samples, avail < 0 ? 0 : avail);
            return -1;
        }
    }

    /* Convert float samples to DDR format at current write cursor */
    volatile uint32_t *tx_buf = hal_ddr_tx_buf();
    uint32_t cursor = stream_wr_cursor;

    /* Handle ring buffer wrap: if the write spans the buffer boundary,
     * split into two contiguous writes.
     *
     * The tx_buf pointer is volatile-qualified; casting to uint32_t* for
     * the convert calls is safe because the memory barrier below commits
     * all pending stores before the FPGA sees the WR_PTR update. */
    size_t first = n_samples;
    if (cursor + (uint32_t)n_samples > DMA_TX_MAX_SAMPLES)
        first = DMA_TX_MAX_SAMPLES - cursor;
    size_t second = n_samples - first;

    if (auto_scale) {
        /* Find peak over the ENTIRE chunk so both halves of a wrapping
         * write use the same scale factor.  Without this, each half gets
         * an independent peak search and normalization, producing a 20 dB
         * step at the wrap boundary (issue #4).
         *
         * NEON path: vabs + vmax, 4 samples/iteration (~4x scalar). */
        float peak = 0.0f;
#ifdef __ARM_NEON
        float32x4_t vpeak = vdupq_n_f32(0.0f);
        size_t i = 0;
        size_t n4 = n_samples & ~(size_t)3;
        for (; i < n4; i += 4) {
            float32x4_t vr = vld1q_f32(&in_real[i]);
            float32x4_t vi = vld1q_f32(&in_imag[i]);
            float32x4_t ar = vabsq_f32(vr);
            float32x4_t ai = vabsq_f32(vi);
            vpeak = vmaxq_f32(vpeak, vmaxq_f32(ar, ai));
        }
        /* Horizontal max of the 4 lanes */
        float32x2_t vmax2 = vpmax_f32(vget_low_f32(vpeak), vget_high_f32(vpeak));
        vmax2 = vpmax_f32(vmax2, vmax2);
        peak = vget_lane_f32(vmax2, 0);
        /* Scalar tail */
        for (; i < n_samples; i++) {
            float ar = fabsf(in_real[i]);
            float ai = fabsf(in_imag[i]);
            if (ar > peak) peak = ar;
            if (ai > peak) peak = ai;
        }
#else
        for (size_t i = 0; i < n_samples; i++) {
            float ar = fabsf(in_real[i]);
            float ai = fabsf(in_imag[i]);
            if (ar > peak) peak = ar;
            if (ai > peak) peak = ai;
        }
#endif
        float scale = (peak > 0.0f) ? 2047.0f / peak : 2047.0f;

        convert_float_to_tx(in_real, in_imag, first, scale,
                            (volatile uint32_t *)&tx_buf[cursor]);
        if (second)
            convert_float_to_tx(&in_real[first], &in_imag[first], second,
                                scale, (volatile uint32_t *)&tx_buf[0]);
    } else {
        convert_float_to_tx(in_real, in_imag, first, peak_scale,
                            (volatile uint32_t *)&tx_buf[cursor]);
        if (second)
            convert_float_to_tx(&in_real[first], &in_imag[first], second,
                                peak_scale, (volatile uint32_t *)&tx_buf[0]);
    }

    /* Advance cursor with wrap */
    cursor += (uint32_t)n_samples;
    if (cursor >= DMA_TX_MAX_SAMPLES)
        cursor -= DMA_TX_MAX_SAMPLES;
    stream_wr_cursor = cursor;
    stream_wr_total += n_samples;

    /* Committing stores to DDR before updating WR_PTR.  Without this
     * barrier, the FPGA could read stale data through its AXI port. */
    memory_barrier();

    /* Tell the fill FSM new data is available */
    hal_reg_write(REG_IQ_DMA_TX_WR_PTR, cursor);

    return 0;
}

int dma_tx_stream_feed(const float *in_real, const float *in_imag,
                       size_t n_samples)
{
    return stream_feed(in_real, in_imag, n_samples, true, 0.0f);
}

int dma_tx_stream_feed_fixed(const float *in_real, const float *in_imag,
                             size_t n_samples, float peak_scale)
{
    return stream_feed(in_real, in_imag, n_samples, false, peak_scale);
}

uint32_t dma_tx_stream_rd_ptr(void)
{
    return hal_reg_read(REG_IQ_DMA_TX_RD_PTR);
}

int32_t dma_tx_stream_depth(void)
{
    /* Not yet triggered: the drain FSM is idle and RD_PTR holds a stale
     * value, so everything written so far is genuinely buffered. */
    if (!stream_running)
        return (int32_t)stream_wr_total;

    /*
     * Accumulate RD_PTR into a monotonic total and subtract from the
     * monotonic write total.
     *
     * A single-shot comparison of the two ring positions cannot work: a
     * position pair is ambiguous between "ARM ahead by D" and "FPGA ahead
     * by BUF-D", and both occur in practice — the ring legitimately fills
     * past half capacity, and an underrun legitimately puts RD_PTR in
     * front.  Monotonic counters remove the ambiguity entirely.
     *
     * Requires that this be called at least once per ring wrap (4 s at
     * 2.1 MSPS).  Any feeder polls orders of magnitude more often.
     */
    uint32_t rd = hal_reg_read(REG_IQ_DMA_TX_RD_PTR);
    uint32_t delta = (rd - stream_rd_last) & (DMA_TX_MAX_SAMPLES - 1);
    stream_rd_total += delta;
    stream_rd_last = rd;

    int64_t depth = (int64_t)stream_wr_total - (int64_t)stream_rd_total;
    if (depth > (int64_t)DMA_TX_MAX_SAMPLES)
        depth = (int64_t)DMA_TX_MAX_SAMPLES;
    if (depth < -(int64_t)DMA_TX_MAX_SAMPLES)
        depth = -(int64_t)DMA_TX_MAX_SAMPLES;
    return (int32_t)depth;
}

uint64_t dma_tx_stream_emitted(void)
{
    (void)dma_tx_stream_depth();   /* refresh the accumulator */
    return stream_rd_total;
}

int dma_tx_stream_available(void)
{
    int32_t depth = dma_tx_stream_depth();

    /* Underrun: the whole ring is free to overwrite.  Report it as such
     * so the caller races to refill instead of waiting for space that is
     * already there. */
    if (depth < 0)
        return (int)DMA_TX_MAX_SAMPLES - 1;

    return (int)(DMA_TX_MAX_SAMPLES - (uint32_t)depth - 1);
}

void dma_tx_stream_drain(uint32_t sample_rate_hz)
{
    /* Signed depth: if the drain FSM has already overtaken the write
     * cursor there is nothing left to drain, and the old unsigned form
     * would have computed a near-full-ring wait of several seconds. */
    int32_t depth = dma_tx_stream_depth();
    if (depth <= 0)
        return;

    uint32_t to_drain = (uint32_t)depth;

    unsigned int drain_us = (unsigned int)(
        (uint64_t)to_drain * 1000000ULL / sample_rate_hz) + 50000;
    usleep(drain_us);
}

void dma_tx_stream_stop(void)
{
    /* Same as dma_tx_stop — writes enable=0 + trigger=1 */
    dma_tx_stop();
    stream_running = false;
}
