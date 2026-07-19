/**
 * test_tx_legacy.c — Validate legacy 802.11a frame generation against golden vectors.
 *
 * Tests:
 * 1. Full waveform comparison for all 8 rates against Python oracle
 *
 * NOTE: Unlike the HT/VHT tests which validate against IEEE reference waveform
 * generators (TGn 11-06/1715r0 and TGac 11-14/0571r10), these legacy vectors
 * come from our own Python implementation. This is a port-correctness test
 * (C matches Python) but NOT an independent spec-compliance test.
 *
 * Legacy correctness is established indirectly via Annex I.1 intermediate
 * vectors (scrambled, encoded, interleaved, QAM, freq domain, L-SIG, STF, LTF)
 * which ARE spec-sourced. But no full-frame reference waveform exists here.
 *
 * TODO: Generate authoritative legacy waveforms upstream in Python using the
 * IEEE 802.11a reference implementation (11-03/0940r4, "PHY Abstraction for
 * 802.11a" or the Annex G waveform generator if available). Fix Python first,
 * then regenerate these vectors.
 */

#include "test_util.h"
#include "lib80211/tx.h"
#include "lib80211/constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Annex I.1 PSDU (100 bytes including FCS) */
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
 * Test legacy waveform for a given rate against golden vector.
 */
static void test_tx_legacy_waveform(lib80211_fft_plan *plan, int rate_mbps) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "tx_legacy_%dmbps", rate_mbps);
    TEST_BEGIN(test_name);

    char vec_name[64];
    snprintf(vec_name, sizeof(vec_name), "legacy_%dmbps_waveform", rate_mbps);

    test_vector *vec = vector_load(vec_name);
    if (!vec || !vec->real || !vec->imag || vec->n_complex == 0) {
        TEST_FAIL("cannot load %s", vec_name);
        if (vec) vector_free(vec);
        return;
    }

    lib80211_tx_legacy_params params = {
        .rate_mbps = rate_mbps,
        .psdu = TEST_PSDU,
        .psdu_len = 100,
        .scrambler_seed = 0x5D,
    };

    size_t expected_samples = lib80211_tx_legacy_samples(&params);
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

    size_t n_out = lib80211_tx_legacy(plan, &params, out_real, out_imag);
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
    printf("test_tx_legacy\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("FATAL: cannot create FFT plan\n");
        return 1;
    }

    /* Full waveform golden vector comparison for all 8 rates */
    static const int rates[] = {6, 9, 12, 18, 24, 36, 48, 54};
    for (int r = 0; r < 8; r++) {
        test_tx_legacy_waveform(plan, rates[r]);
    }

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
