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
