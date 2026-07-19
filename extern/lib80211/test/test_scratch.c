/**
 * test_scratch.c -- Unit tests for lib80211_scratch bump allocator.
 */

#include "test_util.h"
#include "lib80211/scratch.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

static void test_basic_alloc(void)
{
    TEST_BEGIN("scratch_basic_alloc");

    uint8_t mem[1024];
    lib80211_scratch s;
    lib80211_scratch_init(&s, mem, sizeof(mem));

    void *p1 = lib80211_scratch_alloc(&s, 100, 0);
    void *p2 = lib80211_scratch_alloc(&s, 200, 0);
    void *p3 = lib80211_scratch_alloc(&s, 50, 0);

    if (!p1 || !p2 || !p3) {
        TEST_FAIL("allocation returned NULL");
        return;
    }

    /* Non-overlapping */
    if ((uint8_t *)p2 < (uint8_t *)p1 + 100) {
        TEST_FAIL("p2 overlaps p1");
        return;
    }
    if ((uint8_t *)p3 < (uint8_t *)p2 + 200) {
        TEST_FAIL("p3 overlaps p2");
        return;
    }

    TEST_PASS();
}

static void test_alignment(void)
{
    TEST_BEGIN("scratch_alignment");

    uint8_t mem[4096];
    lib80211_scratch s;
    lib80211_scratch_init(&s, mem, sizeof(mem));

    /* Default alignment (16) */
    void *p1 = lib80211_scratch_alloc(&s, 1, 0);
    void *p2 = lib80211_scratch_alloc(&s, 1, 0);

    if ((uintptr_t)p1 % 16 != 0 && (uintptr_t)p1 - (uintptr_t)mem < 16) {
        /* p1 might not be 16-aligned if mem isn't, but p2 must be */
    }
    if ((uintptr_t)p2 % 16 != 0) {
        TEST_FAIL("p2 not 16-byte aligned: %p", p2);
        return;
    }

    /* Explicit 64-byte alignment */
    void *p3 = lib80211_scratch_alloc(&s, 32, 64);
    if ((uintptr_t)p3 % 64 != 0) {
        TEST_FAIL("p3 not 64-byte aligned: %p", p3);
        return;
    }

    TEST_PASS();
}

static void test_exhaustion(void)
{
    TEST_BEGIN("scratch_exhaustion");

    uint8_t mem[256];
    lib80211_scratch s;
    lib80211_scratch_init(&s, mem, sizeof(mem));

    /* Allocate most of the buffer */
    void *p1 = lib80211_scratch_alloc(&s, 200, 0);
    if (!p1) {
        TEST_FAIL("first alloc failed");
        return;
    }

    /* This should fail (200 + 16 alignment + 200 > 256) */
    void *p2 = lib80211_scratch_alloc(&s, 200, 0);
    if (p2 != NULL) {
        TEST_FAIL("expected NULL on exhaustion, got %p", p2);
        return;
    }

    TEST_PASS();
}

static void test_reset(void)
{
    TEST_BEGIN("scratch_reset");

    uint8_t mem[256];
    lib80211_scratch s;
    lib80211_scratch_init(&s, mem, sizeof(mem));

    void *p1 = lib80211_scratch_alloc(&s, 200, 0);
    if (!p1) {
        TEST_FAIL("first alloc failed");
        return;
    }

    /* Exhausted */
    void *p2 = lib80211_scratch_alloc(&s, 200, 0);
    if (p2 != NULL) {
        TEST_FAIL("should be exhausted");
        return;
    }

    /* Reset and try again */
    lib80211_scratch_reset(&s);
    void *p3 = lib80211_scratch_alloc(&s, 200, 0);
    if (!p3) {
        TEST_FAIL("alloc after reset failed");
        return;
    }

    TEST_PASS();
}

static void test_null_safety(void)
{
    TEST_BEGIN("scratch_null_safety");

    /* NULL scratch */
    void *p = lib80211_scratch_alloc(NULL, 100, 0);
    if (p != NULL) {
        TEST_FAIL("expected NULL for NULL scratch");
        return;
    }

    /* NULL memory */
    lib80211_scratch s;
    lib80211_scratch_init(&s, NULL, 0);
    p = lib80211_scratch_alloc(&s, 100, 0);
    if (p != NULL) {
        TEST_FAIL("expected NULL for NULL base");
        return;
    }

    /* Zero size */
    uint8_t mem[128];
    lib80211_scratch_init(&s, mem, sizeof(mem));
    p = lib80211_scratch_alloc(&s, 0, 0);
    if (p != NULL) {
        TEST_FAIL("expected NULL for zero size");
        return;
    }

    /* NULL init */
    lib80211_scratch_init(NULL, mem, sizeof(mem));  /* should not crash */
    lib80211_scratch_reset(NULL);                   /* should not crash */

    TEST_PASS();
}

static void test_sizing_macros(void)
{
    TEST_BEGIN("scratch_sizing_macros");

    /* Verify macros produce reasonable values */
    size_t rx_size = LIB80211_SCRATCH_RX_SIZE(20000);
    size_t tx_size = LIB80211_SCRATCH_TX_SIZE;
    size_t max_size = LIB80211_SCRATCH_MAX;

    /* RX for 20k samples should be > 160 KB (IQ alone is 160 KB) */
    if (rx_size < 160 * 1024) {
        TEST_FAIL("RX size too small: %zu", rx_size);
        return;
    }

    /* TX size should be > 260 KB (coded buffers ~530 KB total) */
    if (tx_size < 260 * 1024) {
        TEST_FAIL("TX size too small: %zu", tx_size);
        return;
    }

    /* MAX should be >= both */
    if (max_size < rx_size || max_size < tx_size) {
        TEST_FAIL("MAX (%zu) smaller than RX (%zu) or TX (%zu)",
                  max_size, rx_size, tx_size);
        return;
    }

    TEST_PASS();
}

int main(void)
{
    printf("test_scratch\n");

    test_basic_alloc();
    test_alignment();
    test_exhaustion();
    test_reset();
    test_null_safety();
    test_sizing_macros();

    TEST_SUMMARY();
    return TEST_EXIT();
}
