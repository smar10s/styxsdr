/**
 * test_rx_legacy.c — Validate full legacy 802.11a frame decode pipeline.
 *
 * Decodes all 8 legacy waveform vectors (6-54 Mbps) and verifies:
 * 1. Frame detected (return 0)
 * 2. Correct rate decoded from L-SIG
 * 3. FCS valid
 */

#include "test_util.h"
#include "lib80211/rx.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>

static const int RATES[] = { 6, 9, 12, 18, 24, 36, 48, 54 };
static const int N_RATES = 8;

/* Expected PSDU length: 100 bytes (96 payload + 4 FCS) */
#define EXPECTED_LENGTH 100

static void test_decode_rate(lib80211_fft_plan *plan, int rate_mbps)
{
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "rx_legacy_%dmbps", rate_mbps);
    TEST_BEGIN(test_name);

    char vec_name[64];
    snprintf(vec_name, sizeof(vec_name), "legacy_%dmbps_waveform", rate_mbps);

    test_vector *vec = vector_load(vec_name);
    if (!vec || !vec->real || !vec->imag || vec->n_complex == 0) {
        TEST_FAIL("cannot load %s", vec_name);
        if (vec) vector_free(vec);
        return;
    }

    lib80211_rx_result result;
    int rc = lib80211_rx_decode(plan, vec->real, vec->imag, vec->n_complex, &result);

    if (rc != 0) {
        TEST_FAIL("rx_decode returned -1 (decode failed)");
        vector_free(vec);
        return;
    }

    int ok = 1;

    /* Check rate */
    if (result.rate_mbps != rate_mbps) {
        printf("    rate: expected %d, got %d\n", rate_mbps, result.rate_mbps);
        ok = 0;
    }

    /* Check PSDU length */
    if (result.psdu_len != EXPECTED_LENGTH) {
        printf("    psdu_len: expected %d, got %zu\n", EXPECTED_LENGTH, result.psdu_len);
        ok = 0;
    }

    /* Check FCS */
    if (!result.fcs_valid) {
        printf("    FCS check FAILED\n");
        /* Print first few PSDU bytes for debugging */
        printf("    PSDU[0:10]:");
        for (int i = 0; i < 10 && i < (int)result.psdu_len; i++)
            printf(" %02x", result.psdu[i]);
        printf("\n");
        ok = 0;
    }

    if (ok) {
        printf("    rate=%d, len=%zu, n_sym=%d, fcs=OK\n",
               result.rate_mbps, result.psdu_len, result.n_symbols);
        TEST_PASS();
    } else {
        TEST_FAIL("decode results incorrect");
    }

    vector_free(vec);
}

int main(void)
{
    printf("test_rx_legacy: Full legacy frame decode (8 rates)\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("  [ FAIL ] could not create FFT plan\n");
        return 1;
    }

    for (int i = 0; i < N_RATES; i++) {
        test_decode_rate(plan, RATES[i]);
    }

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
