// SPDX-License-Identifier: MIT
#include "dma_rx.h"
#include "hal.h"
#include "convert.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

static uint64_t time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

/* --------------------------------------------------------------------------
 * DMA RX control
 * -------------------------------------------------------------------------- */

int dma_rx_start(void)
{
    /* Set DDR base address for RX DMA engine */
    hal_reg_write(REG_IQ_DMA_RX_DDR_BASE, DDR_RX_BASE);

    /* Enable capture */
    hal_reg_write(REG_IQ_DMA_RX_CONTROL, RX_CTRL_ENABLE);

    /* Verify DMA is actually running by checking SAMPLE_COUNT advances.
     * STATUS bit 0 (active) only reflects burst FSM state, which returns
     * to IDLE between bursts — unreliable for a single-read check. */
    usleep(500);
    uint32_t count1 = hal_reg_read(REG_IQ_DMA_RX_SAMPLE_COUNT);
    usleep(500);
    uint32_t count2 = hal_reg_read(REG_IQ_DMA_RX_SAMPLE_COUNT);

    if (count2 == count1) {
        fprintf(stderr, "dma_rx: failed to start (sample_count stuck at %u)\n", count1);
        return -1;
    }

    return 0;
}

void dma_rx_stop(void)
{
    hal_reg_write(REG_IQ_DMA_RX_CONTROL, 0);
}

uint32_t dma_rx_wr_ptr(void)
{
    return hal_reg_read(REG_IQ_DMA_RX_WR_PTR);
}

int dma_rx_capture(size_t n_samples, float *out_real, float *out_imag,
                   uint32_t timeout_ms)
{
    if (n_samples == 0)
        return 0;

    if (n_samples > DMA_RX_BUF_SAMPLES) {
        fprintf(stderr, "dma_rx: requested %zu samples exceeds buffer (%u)\n",
                n_samples, DMA_RX_BUF_SAMPLES);
        return -1;
    }

    /* Record starting write pointer — we want n_samples of FRESH data */
    uint32_t start_ptr = dma_rx_wr_ptr();
    uint64_t t_start = time_ms();

    /* Wait until enough fresh samples have been written */
    while (1) {
        uint32_t cur_ptr = dma_rx_wr_ptr();

        /* Calculate how many fresh samples are available since start_ptr */
        uint32_t available;
        if (cur_ptr >= start_ptr) {
            available = cur_ptr - start_ptr;
        } else {
            /* Wrapped around */
            available = (DMA_RX_BUF_SAMPLES - start_ptr) + cur_ptr;
        }

        if (available >= (uint32_t)n_samples)
            break;

        /* Check timeout */
        if (timeout_ms > 0) {
            uint64_t elapsed = time_ms() - t_start;
            if (elapsed >= timeout_ms) {
                fprintf(stderr, "dma_rx: capture timeout (%u ms, got %u/%zu samples)\n",
                        timeout_ms, available, n_samples);
                return -1;
            }
        }

        usleep(50);  /* 50 us poll interval */
    }

    /* Read fresh data starting from start_ptr */
    volatile uint32_t *rx_buf = hal_ddr_rx_buf();
    uint32_t read_ptr = start_ptr;

    /* Check if read wraps around the ring buffer */
    if (read_ptr + (uint32_t)n_samples <= DMA_RX_BUF_SAMPLES) {
        /* Contiguous read */
        convert_rx_to_float(&rx_buf[read_ptr], n_samples, out_real, out_imag);
    } else {
        /* Split read: end of buffer + start of buffer */
        size_t first_chunk = DMA_RX_BUF_SAMPLES - read_ptr;
        size_t second_chunk = n_samples - first_chunk;

        convert_rx_to_float(&rx_buf[read_ptr], first_chunk,
                            out_real, out_imag);
        convert_rx_to_float(&rx_buf[0], second_chunk,
                            &out_real[first_chunk], &out_imag[first_chunk]);
    }

    return (int)n_samples;
}
