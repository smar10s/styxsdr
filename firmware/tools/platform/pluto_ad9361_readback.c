// SPDX-License-Identifier: MIT
/*
 * pluto_ad9361_readback — AD9361 readback getter validation
 *
 * Verifies the HAL readback getters against the live AD9361 sysfs.
 * The getters exist because the AD9361 quantizes LO, bandwidth, and
 * sample rate: the hardware value differs from the value requested.
 *
 * Tests:
 *   1. Getters match the raw sysfs attribute (RX LO, RX BW, sample rate)
 *   2. RX LO getter tracks a real frequency change, then restores
 *   3. RX BW getter tracks a real bandwidth change, then restores
 *
 * Sample rate is read-only here: changing it re-clocks the AD9361 and
 * would disrupt a running capture.
 *
 * Exit code: 0 = all pass, 1 = failure (details on stderr).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include "hal.h"
#include "styx_tool.h"

#define AD9361_IIO_PATH "/sys/bus/iio/devices/iio:device1/"

static int tests_run = 0;
static int tests_passed = 0;

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

/* Read a sysfs attribute directly, bypassing the HAL, to cross-check. */
static long long read_raw(const char *attr)
{
    char path[128];
    snprintf(path, sizeof(path), AD9361_IIO_PATH "%s", attr);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    long long v = 0;
    if (fscanf(f, "%lld", &v) != 1) v = -1;
    fclose(f);
    return v;
}

/* --------------------------------------------------------------------------
 * Test 1: getter output equals the raw sysfs value
 * -------------------------------------------------------------------------- */
static int test_getters_match_sysfs(void)
{
    TEST_START("getter matches raw sysfs");

    struct { const char *name; long long got; const char *attr; } t[] = {
        { "get_rx_lo",        hal_ad9361_get_rx_lo(),        "out_altvoltage0_RX_LO_frequency" },
        { "get_rx_bandwidth", hal_ad9361_get_rx_bandwidth(), "in_voltage_rf_bandwidth" },
        { "get_sample_rate",  hal_ad9361_get_sample_rate(),  "in_voltage_sampling_frequency" },
    };

    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
        long long raw = read_raw(t[i].attr);
        if (t[i].got < 0)
            TEST_FAIL("%s returned error (%lld)", t[i].name, t[i].got);
        if (t[i].got != raw)
            TEST_FAIL("%s: getter=%lld raw=%lld", t[i].name, t[i].got, raw);
    }

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Test 2: RX LO getter tracks a change
 * -------------------------------------------------------------------------- */
static int test_rx_lo_tracks_change(void)
{
    TEST_START("RX LO tracks change");

    long long orig = hal_ad9361_get_rx_lo();
    if (orig < 0)
        TEST_FAIL("initial read failed (%lld)", orig);

    long long target = orig + 2000000;  /* +2 MHz */
    if (hal_ad9361_set_rx_lo((uint64_t)target) != 0)
        TEST_FAIL("set_rx_lo(%lld) failed", target);

    long long got = hal_ad9361_get_rx_lo();
    long long raw = read_raw("out_altvoltage0_RX_LO_frequency");
    if (got != raw)
        TEST_FAIL("getter=%lld raw=%lld after set", got, raw);
    if (llabs(got - target) > 1000)
        TEST_FAIL("did not reach target: got=%lld target=%lld", got, target);

    if (hal_ad9361_set_rx_lo((uint64_t)orig) != 0)
        TEST_FAIL("restore to %lld failed", orig);
    long long back = hal_ad9361_get_rx_lo();
    if (llabs(back - orig) > 1000)
        TEST_FAIL("restore mismatch: got=%lld expected=%lld", back, orig);

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Test 3: RX bandwidth getter tracks a change
 * -------------------------------------------------------------------------- */
static int test_rx_bw_tracks_change(void)
{
    TEST_START("RX BW tracks change");

    long long orig = hal_ad9361_get_rx_bandwidth();
    if (orig < 0)
        TEST_FAIL("initial read failed (%lld)", orig);

    uint64_t target = (orig > 10000000) ? 10000000 : 20000000;
    if (hal_ad9361_set_rx_bandwidth(target) != 0)
        TEST_FAIL("set_rx_bandwidth(%llu) failed", (unsigned long long)target);

    long long got = hal_ad9361_get_rx_bandwidth();
    long long raw = read_raw("in_voltage_rf_bandwidth");
    if (got != raw)
        TEST_FAIL("getter=%lld raw=%lld after set", got, raw);
    if (got == orig)
        TEST_FAIL("bandwidth unchanged (stuck at %lld)", got);

    if (hal_ad9361_set_rx_bandwidth((uint64_t)orig) != 0)
        TEST_FAIL("restore to %lld failed", orig);
    long long back = hal_ad9361_get_rx_bandwidth();
    if (llabs(back - orig) > 1000)
        TEST_FAIL("restore mismatch: got=%lld expected=%lld", back, orig);

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */
int main(void)
{
    fprintf(stderr, "pluto_ad9361_readback — AD9361 readback validation\n");

    styx_install_shutdown_handler();

    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    test_getters_match_sysfs();
    test_rx_lo_tracks_change();
    test_rx_bw_tracks_change();

    hal_cleanup();

    fprintf(stderr, "\nResult: %d/%d tests passed\n", tests_passed, tests_run);
    printf("{\"passed\":%d,\"failed\":%d,\"total\":%d}\n",
           tests_passed, tests_run - tests_passed, tests_run);

    return (tests_passed == tests_run) ? 0 : 1;
}
