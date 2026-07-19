/**
 * test_sync.c — Validate STF detection, LTF timing, and CFO estimation.
 *
 * Tests:
 * 1. STF detection on clean waveform (legacy_6mbps_waveform.json)
 * 2. LTF timing precision (synthetic STF+LTF)
 * 3. CFO estimation accuracy with known offset
 */

#include "test_util.h"
#include "lib80211/sync.h"
#include "lib80211/ofdm.h"
#include "lib80211/constants.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ========================================================================
 * Test 1: STF detection on clean waveform
 * ======================================================================== */

static void test_sync_clean_waveform(lib80211_fft_plan *plan) {
    TEST_BEGIN("sync_clean_waveform");

    test_vector *vec = vector_load("legacy_6mbps_waveform");
    if (!vec || !vec->real || !vec->imag || vec->n_complex == 0) {
        TEST_FAIL("cannot load legacy_6mbps_waveform");
        if (vec) vector_free(vec);
        return;
    }

    lib80211_sync_result result;
    int rc = lib80211_sync_detect(plan, vec->real, vec->imag, vec->n_complex, &result);

    if (rc != 0) {
        TEST_FAIL("sync_detect returned -1 (no frame detected)");
        vector_free(vec);
        return;
    }

    int ok = 1;

    /* frame_start should be 0 (STF is at the beginning) */
    if (result.frame_start > 16) {
        printf("    frame_start=%zu (expected 0, tolerance <=16)\n", result.frame_start);
        ok = 0;
    }

    /* ltf_start should be 192 (STF=160 + GI2=32) */
    int ltf_err = abs((int)result.ltf_start - 192);
    if (ltf_err > 2) {
        printf("    ltf_start=%zu (expected 192, error=%d)\n", result.ltf_start, ltf_err);
        ok = 0;
    }

    /* CFO should be ~0 for clean waveform */
    if (fabsf(result.cfo_rad) > 0.01f) {
        printf("    cfo_rad=%.6f (expected ~0)\n", result.cfo_rad);
        ok = 0;
    }

    if (ok) {
        printf("    frame_start=%zu, ltf_start=%zu, cfo_rad=%.6f\n",
               result.frame_start, result.ltf_start, result.cfo_rad);
        TEST_PASS();
    } else {
        TEST_FAIL("sync results out of tolerance");
    }

    vector_free(vec);
}

/* ========================================================================
 * Test 2: LTF timing precision (synthetic preamble)
 * ======================================================================== */

static void test_sync_ltf_timing(lib80211_fft_plan *plan) {
    TEST_BEGIN("sync_ltf_timing");

    /* Generate STF + LTF = 320 samples, prepend 100 samples of zeros */
    size_t prefix = 100;
    size_t total = prefix + LIB80211_STF_SAMPLES + LIB80211_LTF_SAMPLES + 200;
    float *re = (float *)calloc(total, sizeof(float));
    float *im = (float *)calloc(total, sizeof(float));
    if (!re || !im) {
        TEST_FAIL("malloc failed");
        free(re); free(im);
        return;
    }

    lib80211_generate_stf(plan, re + prefix, im + prefix);
    lib80211_generate_ltf(plan, re + prefix + LIB80211_STF_SAMPLES,
                          im + prefix + LIB80211_STF_SAMPLES);

    lib80211_sync_result result;
    int rc = lib80211_sync_detect(plan, re, im, total, &result);

    if (rc != 0) {
        TEST_FAIL("sync_detect returned -1");
        free(re); free(im);
        return;
    }

    int ok = 1;

    /* frame_start should be at prefix (100) */
    int frame_err = abs((int)result.frame_start - (int)prefix);
    if (frame_err > 16) {
        printf("    frame_start=%zu (expected %zu, error=%d)\n",
               result.frame_start, prefix, frame_err);
        ok = 0;
    }

    /* ltf_start should be at prefix + 160 + 32 = 292 */
    size_t expected_ltf = prefix + LIB80211_STF_SAMPLES + 32;
    int ltf_err = abs((int)result.ltf_start - (int)expected_ltf);
    if (ltf_err > 2) {
        printf("    ltf_start=%zu (expected %zu, error=%d)\n",
               result.ltf_start, expected_ltf, ltf_err);
        ok = 0;
    }

    /* CFO should be ~0 */
    if (fabsf(result.cfo_rad) > 0.01f) {
        printf("    cfo_rad=%.6f (expected ~0)\n", result.cfo_rad);
        ok = 0;
    }

    if (ok) {
        printf("    frame_start=%zu, ltf_start=%zu, cfo_rad=%.6f\n",
               result.frame_start, result.ltf_start, result.cfo_rad);
        TEST_PASS();
    } else {
        TEST_FAIL("timing results out of tolerance");
    }

    free(re);
    free(im);
}

/* ========================================================================
 * Test 3: CFO estimation with known offset
 * ======================================================================== */

static void test_sync_cfo_estimation(lib80211_fft_plan *plan) {
    TEST_BEGIN("sync_cfo_estimation");

    /* Generate STF + LTF, apply known CFO */
    size_t total = LIB80211_STF_SAMPLES + LIB80211_LTF_SAMPLES + 200;
    float *re = (float *)calloc(total, sizeof(float));
    float *im = (float *)calloc(total, sizeof(float));
    if (!re || !im) {
        TEST_FAIL("malloc failed");
        free(re); free(im);
        return;
    }

    lib80211_generate_stf(plan, re, im);
    lib80211_generate_ltf(plan, re + LIB80211_STF_SAMPLES,
                          im + LIB80211_STF_SAMPLES);

    /* Apply CFO: 5000 Hz at 20 MSPS = 5000 / 20e6 * 2*pi rad/sample */
    float cfo_hz = 5000.0f;
    float cfo_applied = cfo_hz / (float)LIB80211_SAMPLE_RATE * 2.0f * (float)M_PI;

    for (size_t n = 0; n < total; n++) {
        float phase = (float)n * cfo_applied;
        float c = cosf(phase);
        float s = sinf(phase);
        float r = re[n];
        float i = im[n];
        re[n] = r * c - i * s;
        im[n] = r * s + i * c;
    }

    lib80211_sync_result result;
    int rc = lib80211_sync_detect(plan, re, im, total, &result);

    if (rc != 0) {
        TEST_FAIL("sync_detect returned -1");
        free(re); free(im);
        return;
    }

    /* Check estimated CFO matches applied CFO within 10% */
    float cfo_error = fabsf(result.cfo_rad - cfo_applied);
    float relative_error = cfo_error / fabsf(cfo_applied);

    printf("    applied_cfo=%.6f rad/s, estimated_cfo=%.6f rad/s, rel_error=%.2f%%\n",
           cfo_applied, result.cfo_rad, relative_error * 100.0f);

    if (relative_error < 0.10f) {
        TEST_PASS();
    } else {
        TEST_FAIL("CFO estimation error %.1f%% exceeds 10%% tolerance", relative_error * 100.0f);
    }

    free(re);
    free(im);
}

/* ========================================================================
 * Test 4: CFO correction
 * ======================================================================== */

static void test_cfo_correction(lib80211_fft_plan *plan) {
    TEST_BEGIN("cfo_correction");
    (void)plan;

    /* Create a pure tone, apply CFO, correct it, verify residual is small */
    size_t n = 256;
    float *re = (float *)malloc(n * sizeof(float));
    float *im = (float *)malloc(n * sizeof(float));
    if (!re || !im) {
        TEST_FAIL("malloc failed");
        free(re); free(im);
        return;
    }

    /* Pure DC signal (all 1+0j) */
    for (size_t k = 0; k < n; k++) {
        re[k] = 1.0f;
        im[k] = 0.0f;
    }

    /* Apply CFO offset */
    float cfo = 0.05f;  /* rad/sample */
    for (size_t k = 0; k < n; k++) {
        float phase = (float)k * cfo;
        float c = cosf(phase);
        float s = sinf(phase);
        float r = re[k];
        float i = im[k];
        re[k] = r * c - i * s;
        im[k] = r * s + i * c;
    }

    /* Correct */
    lib80211_cfo_correct(re, im, n, cfo);

    /* After correction, signal should be back to DC (1+0j) */
    float max_err = 0.0f;
    for (size_t k = 0; k < n; k++) {
        float err_r = fabsf(re[k] - 1.0f);
        float err_i = fabsf(im[k]);
        float err = (err_r > err_i) ? err_r : err_i;
        if (err > max_err) max_err = err;
    }

    if (max_err < 1e-5f) {
        printf("    max_error=%.2e (excellent)\n", max_err);
        TEST_PASS();
    } else {
        TEST_FAIL("max residual error %.2e exceeds tolerance", max_err);
    }

    free(re);
    free(im);
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(void) {
    printf("test_sync: STF detection, LTF timing, CFO estimation\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("  [ FAIL ] could not create FFT plan\n");
        return 1;
    }

    test_sync_clean_waveform(plan);
    test_sync_ltf_timing(plan);
    test_sync_cfo_estimation(plan);
    test_cfo_correction(plan);

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
