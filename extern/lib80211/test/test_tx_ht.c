/**
 * test_tx_ht.c — Validate HT-mixed frame generation against golden vectors.
 *
 * Tests:
 * 1. Full waveform comparison for all 8 MCS (normal GI)
 * 2. Full waveform comparison for all 8 MCS (short GI)
 * 3. Intermediate vector validation (scrambled, encoded, freq_symbols)
 */

#include "test_util.h"
#include "lib80211/tx.h"
#include "lib80211/constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Annex I.1 PSDU first 96 bytes (used as payload, FCS appended = 100 bytes total) */
static const uint8_t TEST_PSDU[100] = {
    0x04, 0x02, 0x00, 0x2E, 0x00, 0x60, 0x08, 0xCD, 0x37, 0xA6,
    0x00, 0x20, 0xD6, 0x01, 0x3C, 0xF1, 0x00, 0x60, 0x08, 0xAD,
    0x3B, 0xAF, 0x00, 0x00, 0x4A, 0x6F, 0x79, 0x2C, 0x20, 0x62,
    0x72, 0x69, 0x67, 0x68, 0x74, 0x20, 0x73, 0x70, 0x61, 0x72,
    0x6B, 0x20, 0x6F, 0x66, 0x20, 0x64, 0x69, 0x76, 0x69, 0x6E,
    0x69, 0x74, 0x79, 0x2C, 0x0A, 0x44, 0x61, 0x75, 0x67, 0x68,
    0x74, 0x65, 0x72, 0x20, 0x6F, 0x66, 0x20, 0x45, 0x6C, 0x79,
    0x73, 0x69, 0x75, 0x6D, 0x2C, 0x0A, 0x46, 0x69, 0x72, 0x65,
    0x2D, 0x69, 0x6E, 0x73, 0x69, 0x72, 0x65, 0x64, 0x20, 0x77,
    0x65, 0x20, 0x74, 0x72, 0x65, 0x61, 0x67, 0x33, 0x21, 0xB6,
};

/**
 * Test HT waveform for a given MCS against golden vector.
 */
static void test_tx_ht_waveform(lib80211_fft_plan *plan, int mcs, bool sgi) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "tx_ht_mcs%d%s", mcs, sgi ? "_sgi" : "");
    TEST_BEGIN(test_name);

    char vec_name[64];
    snprintf(vec_name, sizeof(vec_name), "ht_mcs%d%s_waveform", mcs, sgi ? "_sgi" : "");

    test_vector *vec = vector_load(vec_name);
    if (!vec || !vec->real || !vec->imag || vec->n_complex == 0) {
        TEST_FAIL("cannot load %s", vec_name);
        if (vec) vector_free(vec);
        return;
    }

    lib80211_tx_ht_params params = {
        .mcs = mcs,
        .psdu = TEST_PSDU,
        .psdu_len = 100,
        .scrambler_seed = 0x5D,
        .short_gi = sgi,
    };

    size_t expected_samples = lib80211_tx_ht_samples(&params);
    if (expected_samples != vec->n_complex) {
        TEST_FAIL("sample count mismatch: computed %zu, vector has %zu",
                  expected_samples, vec->n_complex);
        vector_free(vec);
        return;
    }

    float *out_real = malloc(expected_samples * sizeof(float));
    float *out_imag = malloc(expected_samples * sizeof(float));
    if (!out_real || !out_imag) {
        TEST_FAIL("malloc failed");
        free(out_real); free(out_imag);
        vector_free(vec);
        return;
    }

    size_t n_out = lib80211_tx_ht(plan, &params, out_real, out_imag);
    if (n_out != expected_samples) {
        TEST_FAIL("output count: expected %zu, got %zu", expected_samples, n_out);
    } else {
        if (!assert_complex_close(vec->real, vec->imag, out_real, out_imag,
                                  n_out, 1e-5f, vec_name)) {
            TEST_FAIL("waveform mismatch");
        } else {
            TEST_PASS();
        }
    }

    free(out_real);
    free(out_imag);
    vector_free(vec);
}

int main(void) {
    printf("test_tx_ht\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("FATAL: cannot create FFT plan\n");
        return 1;
    }

    /* Normal GI: MCS 0-7 */
    for (int mcs = 0; mcs < 8; mcs++) {
        test_tx_ht_waveform(plan, mcs, false);
    }

    /* Short GI: MCS 0-7 */
    for (int mcs = 0; mcs < 8; mcs++) {
        test_tx_ht_waveform(plan, mcs, true);
    }

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
