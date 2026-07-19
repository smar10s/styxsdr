// SPDX-License-Identifier: MIT
/*
 * pluto_dma_test — DMA start/stop/restart validation
 *
 * Exercises the IQ DMA engines without RF/PHY involvement.
 * Validates register semantics, re-arm sequences, and status transitions.
 *
 * Tests:
 *   1. RX DMA start → verify active + wr_ptr advancing → stop → verify inactive
 *   2. RX DMA stop → restart → verify wr_ptr advances from new position
 *   3. TX DMA one-shot → verify tx_done → re-trigger → verify tx_done again
 *   4. TX DMA cyclic → verify active → stop → verify tx_done
 *
 * All tests are register-level only: they confirm the DMA engines respond
 * correctly to control sequences.  No waveform or decode validation.
 *
 * Exit code: 0 = all pass, 1 = failure (details on stderr).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "hal.h"
#include "dma_tx.h"
#include "dma_rx.h"

/* -------------------------------------------------------------------------- */

#define TX_TEST_SAMPLES  1024  /* small waveform for TX tests */

static int tests_run = 0;
static int tests_passed = 0;

static uint64_t time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

#define TEST_START(name) \
    do { \
        tests_run++; \
        fprintf(stderr, "  [%d] %s: ", tests_run, name); \
    } while (0)

#define TEST_PASS() \
    do { \
        tests_passed++; \
        fprintf(stderr, "PASS\n"); \
        return 0; \
    } while (0)

#define TEST_FAIL(fmt, ...) \
    do { \
        fprintf(stderr, "FAIL — " fmt "\n", ##__VA_ARGS__); \
        return -1; \
    } while (0)

/* --------------------------------------------------------------------------
 * Test 1: RX DMA start/stop basic
 * -------------------------------------------------------------------------- */
static int test_rx_start_stop(void)
{
    TEST_START("RX start/stop");

    /* Start RX DMA */
    if (dma_rx_start() != 0)
        TEST_FAIL("dma_rx_start failed");

    /* Wait briefly, then check wr_ptr is advancing */
    usleep(5000);  /* 5 ms = ~100K samples at 20 MSPS */
    uint32_t ptr1 = dma_rx_wr_ptr();

    usleep(5000);
    uint32_t ptr2 = dma_rx_wr_ptr();

    if (ptr2 <= ptr1 && ptr2 != 0)  /* ptr2==0 means wrap, still ok */
        TEST_FAIL("wr_ptr not advancing: %u -> %u", ptr1, ptr2);

    /* Stop */
    dma_rx_stop();
    usleep(1000);

    /* Verify wr_ptr is no longer advancing */
    uint32_t ptr3 = dma_rx_wr_ptr();
    usleep(5000);
    uint32_t ptr4 = dma_rx_wr_ptr();

    if (ptr4 != ptr3)
        TEST_FAIL("wr_ptr still advancing after stop: %u -> %u", ptr3, ptr4);

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Test 2: RX DMA restart
 * -------------------------------------------------------------------------- */
static int test_rx_restart(void)
{
    TEST_START("RX restart");

    /* Start, let it run briefly, stop */
    if (dma_rx_start() != 0)
        TEST_FAIL("first dma_rx_start failed");

    usleep(10000);
    uint32_t ptr_before_stop = dma_rx_wr_ptr();
    dma_rx_stop();
    usleep(1000);

    /* Restart */
    if (dma_rx_start() != 0)
        TEST_FAIL("second dma_rx_start failed");

    /* Verify wr_ptr continues advancing (from where it left off) */
    usleep(5000);
    uint32_t ptr_after_restart = dma_rx_wr_ptr();

    dma_rx_stop();

    /* wr_ptr should have moved from the stop position */
    if (ptr_after_restart == ptr_before_stop)
        TEST_FAIL("wr_ptr did not advance after restart: stuck at %u", ptr_before_stop);

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Test 3: TX DMA one-shot + re-trigger
 * -------------------------------------------------------------------------- */
static int test_tx_oneshot_retrigger(void)
{
    TEST_START("TX one-shot retrigger");

    /* Write a small constant waveform to TX DDR buffer */
    volatile uint32_t *tx_buf = hal_ddr_tx_buf();
    for (uint32_t i = 0; i < TX_TEST_SAMPLES; i++)
        tx_buf[i] = IQ_PACK(0x100, 0x200);

    /* First one-shot TX (bypass dma_tx_start to avoid float conversion) */
    hal_reg_write(REG_IQ_DMA_TX_DDR_BASE, DDR_TX_BASE);
    hal_reg_write(REG_IQ_DMA_TX_COUNT, TX_TEST_SAMPLES);
    hal_reg_write(REG_IQ_DMA_TX_CONTROL, TX_CTRL_ENABLE | TX_CTRL_TRIGGER);

    /* Wait for tx_done */
    uint64_t t0 = time_ms();
    while (!dma_tx_done()) {
        if (time_ms() - t0 > 500)
            TEST_FAIL("first TX did not complete within 500 ms");
        usleep(100);
    }

    /* Verify STATUS: active=0, done=1 */
    uint32_t status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    if (!(status & 0x02))
        TEST_FAIL("tx_done not set after first TX (STATUS=0x%08x)", status);

    /* Re-trigger: same parameters */
    hal_reg_write(REG_IQ_DMA_TX_CONTROL, TX_CTRL_ENABLE | TX_CTRL_TRIGGER);

    /* Wait for second tx_done */
    t0 = time_ms();
    while (1) {
        status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
        if (status & 0x02)
            break;
        if (time_ms() - t0 > 500)
            TEST_FAIL("second TX did not complete within 500 ms (STATUS=0x%08x)", status);
        usleep(100);
    }

    /* Verify TX_PTR = TX_TEST_SAMPLES */
    uint32_t tx_ptr = hal_reg_read(REG_IQ_DMA_TX_PTR);
    if (tx_ptr != TX_TEST_SAMPLES)
        TEST_FAIL("TX_PTR=%u, expected %u after second TX", tx_ptr, TX_TEST_SAMPLES);

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Test 4: TX DMA cyclic start/stop
 * -------------------------------------------------------------------------- */
static int test_tx_cyclic_stop(void)
{
    TEST_START("TX cyclic stop");

    /* Write waveform */
    volatile uint32_t *tx_buf = hal_ddr_tx_buf();
    for (uint32_t i = 0; i < TX_TEST_SAMPLES; i++)
        tx_buf[i] = IQ_PACK(0x100, 0x200);

    /* Start cyclic TX */
    hal_reg_write(REG_IQ_DMA_TX_DDR_BASE, DDR_TX_BASE);
    hal_reg_write(REG_IQ_DMA_TX_COUNT, TX_TEST_SAMPLES);
    hal_reg_write(REG_IQ_DMA_TX_CONTROL,
                  TX_CTRL_ENABLE | TX_CTRL_TRIGGER | TX_CTRL_CYCLIC);

    /* Verify it becomes active */
    usleep(1000);
    uint32_t status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    if (!(status & 0x01))
        TEST_FAIL("cyclic TX not active (STATUS=0x%08x)", status);

    /* Let it run for a few iterations */
    usleep(50000);  /* 50 ms */

    /* Verify still active, NOT done */
    status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    if (status & 0x02)
        TEST_FAIL("cyclic TX set tx_done unexpectedly (STATUS=0x%08x)", status);

    /* Stop: enable=0 + trigger to latch disable */
    dma_tx_stop();

    /* Verify tx_done is now set */
    status = hal_reg_read(REG_IQ_DMA_TX_STATUS);
    if (!(status & 0x02))
        TEST_FAIL("tx_done not set after cyclic stop (STATUS=0x%08x)", status);

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */
int main(void)
{
    fprintf(stderr, "pluto_dma_test — DMA start/stop/restart validation\n");

    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    /* Run all tests */
    test_rx_start_stop();
    test_rx_restart();
    test_tx_oneshot_retrigger();
    test_tx_cyclic_stop();

    hal_cleanup();

    /* Summary */
    fprintf(stderr, "\nResult: %d/%d tests passed\n", tests_passed, tests_run);

    /* JSON output */
    printf("{\"passed\":%d,\"failed\":%d,\"total\":%d}\n",
           tests_passed, tests_run - tests_passed, tests_run);

    return (tests_passed == tests_run) ? 0 : 1;
}
