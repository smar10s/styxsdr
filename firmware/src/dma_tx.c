// SPDX-License-Identifier: MIT
#include "dma_tx.h"
#include "hal.h"
#include "convert.h"

#include <stdio.h>
#include <unistd.h>

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

int dma_tx_stream_start(void)
{
    /* Reset to clean state */
    tx_reset();

    /* Reset write cursor */
    stream_wr_cursor = 0;

    /* Configure DMA for streaming: buffer-size TX_COUNT, cyclic+stream */
    hal_reg_write(REG_IQ_DMA_TX_DDR_BASE, DDR_TX_BASE);
    hal_reg_write(REG_IQ_DMA_TX_COUNT, DMA_TX_MAX_SAMPLES);
    hal_reg_write(REG_IQ_DMA_TX_WR_PTR, 0);  /* no data yet */

    /* Trigger: enable + cyclic + stream + trigger */
    uint32_t ctrl = TX_CTRL_ENABLE | TX_CTRL_TRIGGER
                  | TX_CTRL_CYCLIC | TX_CTRL_STREAM;
    hal_reg_write(REG_IQ_DMA_TX_CONTROL, ctrl);

    /* Brief delay then verify TX started */
    usleep(100);
    uint32_t status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    if (!(status & 0x03)) {
        fprintf(stderr, "dma_tx_stream: failed to start (status=0x%08x)\n", status);
        return -1;
    }

    return 0;
}

int dma_tx_stream_feed(const float *in_real, const float *in_imag,
                       size_t n_samples)
{
    if (n_samples == 0)
        return 0;

    if (n_samples > (size_t)DMA_TX_MAX_SAMPLES) {
        fprintf(stderr, "dma_tx_stream: %zu samples exceeds buffer\n", n_samples);
        return -1;
    }

    /* Check available room before writing */
    int avail = dma_tx_stream_available();
    if (avail < 0 || (size_t)avail < n_samples) {
        fprintf(stderr, "dma_tx_stream: buffer full (need %zu, have %d)\n",
                n_samples, avail < 0 ? 0 : avail);
        return -1;
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
    if (cursor + (uint32_t)n_samples <= DMA_TX_MAX_SAMPLES) {
        convert_float_to_tx_auto(in_real, in_imag, n_samples,
                                 (volatile uint32_t *)&tx_buf[cursor]);
    } else {
        size_t first = DMA_TX_MAX_SAMPLES - cursor;
        size_t second = n_samples - first;
        convert_float_to_tx_auto(in_real, in_imag, first,
                                 (volatile uint32_t *)&tx_buf[cursor]);
        convert_float_to_tx_auto(&in_real[first], &in_imag[first], second,
                                 (volatile uint32_t *)&tx_buf[0]);
    }

    /* Advance cursor with wrap */
    cursor += (uint32_t)n_samples;
    if (cursor >= DMA_TX_MAX_SAMPLES)
        cursor -= DMA_TX_MAX_SAMPLES;
    stream_wr_cursor = cursor;

    /* Committing stores to DDR before updating WR_PTR.  Without this
     * barrier, the FPGA could read stale data through its AXI port. */
    memory_barrier();

    /* Tell the fill FSM new data is available */
    hal_reg_write(REG_IQ_DMA_TX_WR_PTR, cursor);

    return 0;
}

uint32_t dma_tx_stream_rd_ptr(void)
{
    return hal_reg_read(REG_IQ_DMA_TX_RD_PTR);
}

int dma_tx_stream_available(void)
{
    uint32_t rd = dma_tx_stream_rd_ptr();
    uint32_t wr = stream_wr_cursor;

    /* Ring distance: wr may be ahead or behind rd due to wrap.
     * "Ahead" means wr >= rd → the ARM is past the FPGA.
     * Available space = (rd - wr - 1) mod BUF_SIZE.
     * The -1 ensures we never let wr catch up to rd (fill_ptr
     * would see wr_ptr == rd_ptr and might misbehave). */
    uint32_t space;
    if (rd > wr)
        space = rd - wr - 1;
    else
        space = (DMA_TX_MAX_SAMPLES - wr) + rd - 1;

    return (int)space;
}

void dma_tx_stream_stop(void)
{
    /* Same as dma_tx_stop — writes enable=0 + trigger=1 */
    dma_tx_stop();
}
