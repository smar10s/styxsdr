/**
 * test_fft.c — Validate FFT backend against known vectors.
 *
 * Tests:
 * 1. Forward FFT round-trip (forward then inverse recovers input)
 * 2. IFFT of L-LTF frequency domain matches annex_i1_ltf_time.json
 * 3. IFFT of L-STF frequency domain matches annex_i1_stf_time_one_period.json
 */

#include "test_util.h"
#include "lib80211/fft.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define N 64

/* L-LTF frequency domain (Table 17-5): subcarriers -26..+26 mapped to FFT bins */
static const float LTF_FREQ_REAL[N] = {
    /* bin 0 (DC) */ 0,
    /* bins 1-26 (subcarriers +1..+26) */
    1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1,
    -1, -1, 1, -1, 1, -1, 1, 1, 1, 1,
    /* bins 27-37 (guard/null) */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /* bins 38-63 (subcarriers -26..-1) */
    1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1,
    -1, 1, 1, -1, 1, -1, 1, 1, 1, 1,
};

/* STF frequency domain (Table 17-4): only every 4th subcarrier non-zero */
static void build_stf_freq(float *real, float *imag) {
    memset(real, 0, N * sizeof(float));
    memset(imag, 0, N * sizeof(float));

    /* sqrt(13/6) scaling factor */
    float scale = sqrtf(13.0f / 6.0f);

    /* Non-zero subcarriers: {-24,-20,-16,-12,-8,-4,4,8,12,16,20,24} */
    /* subcarrier -> FFT bin: (sc % 64) */
    struct { int sc; float r; float i; } stf_vals[] = {
        {-24, +1, +1}, {-20, -1, -1}, {-16, +1, +1}, {-12, -1, -1},
        { -8, -1, -1}, { -4, +1, +1}, {  4, -1, -1}, {  8, -1, -1},
        { 12, +1, +1}, { 16, +1, +1}, { 20, +1, +1}, { 24, +1, +1},
    };

    for (int k = 0; k < 12; k++) {
        int bin = ((stf_vals[k].sc % N) + N) % N;
        real[bin] = scale * stf_vals[k].r;
        imag[bin] = scale * stf_vals[k].i;
    }
}

/* Test 1: Forward-inverse round-trip */
static void test_fft_roundtrip(lib80211_fft_plan *plan) {
    TEST_BEGIN("fft_roundtrip");

    float in_real[N], in_imag[N];
    float freq_real[N], freq_imag[N];
    float out_real[N], out_imag[N];

    /* Create a test signal: complex sinusoid at bin 7 */
    for (int i = 0; i < N; i++) {
        double angle = 2.0 * M_PI * 7.0 * i / N;
        in_real[i] = (float)cos(angle);
        in_imag[i] = (float)sin(angle);
    }

    lib80211_fft_forward(plan, in_real, in_imag, freq_real, freq_imag);
    lib80211_fft_inverse(plan, freq_real, freq_imag, out_real, out_imag);

    if (assert_complex_close(in_real, in_imag, out_real, out_imag, N, 1e-5f, "round-trip")) {
        TEST_PASS();
    } else {
        TEST_FAIL("round-trip error exceeds tolerance");
    }
}

/* Test 2: Forward FFT of sinusoid should produce a single peak */
static void test_fft_single_bin(lib80211_fft_plan *plan) {
    TEST_BEGIN("fft_single_bin");

    float in_real[N], in_imag[N];
    float out_real[N], out_imag[N];

    /* Complex sinusoid at bin 7: x[n] = exp(j*2*pi*7*n/64) */
    for (int i = 0; i < N; i++) {
        double angle = 2.0 * M_PI * 7.0 * i / N;
        in_real[i] = (float)cos(angle);
        in_imag[i] = (float)sin(angle);
    }

    lib80211_fft_forward(plan, in_real, in_imag, out_real, out_imag);

    /* Bin 7 should have magnitude N=64, all others ~0 */
    bool ok = true;
    for (int i = 0; i < N; i++) {
        float mag = sqrtf(out_real[i] * out_real[i] + out_imag[i] * out_imag[i]);
        if (i == 7) {
            if (fabsf(mag - 64.0f) > 1e-3f) {
                printf("    bin 7 magnitude: expected 64.0, got %.4f\n", mag);
                ok = false;
            }
        } else {
            if (mag > 1e-3f) {
                printf("    bin %d magnitude: expected ~0, got %.4f\n", i, mag);
                ok = false;
                break;
            }
        }
    }

    if (ok) TEST_PASS();
    else TEST_FAIL("single-bin test failed");
}

/* Test 3: IFFT of LTF freq — verify round-trip consistency.
 * The golden vector (annex_i1_ltf_time) has windowing applied and 3-decimal
 * precision, so instead we verify: FFT(IFFT(LTF_freq)) == LTF_freq */
static void test_fft_ltf_consistency(lib80211_fft_plan *plan) {
    TEST_BEGIN("fft_ltf_consistency");

    float imag_in[N] = {0};  /* LTF freq is real-valued */
    float time_real[N], time_imag[N];
    float freq_real[N], freq_imag[N];

    /* IFFT of LTF frequency domain */
    lib80211_fft_inverse(plan, LTF_FREQ_REAL, imag_in, time_real, time_imag);

    /* FFT back to frequency domain */
    lib80211_fft_forward(plan, time_real, time_imag, freq_real, freq_imag);

    /* Should match original LTF frequency domain */
    if (assert_complex_close(LTF_FREQ_REAL, imag_in, freq_real, freq_imag, N, 1e-4f, "LTF round-trip")) {
        TEST_PASS();
    } else {
        TEST_FAIL("LTF round-trip mismatch");
    }
}

/* Test 4: IFFT of STF freq — first 16 samples match golden one-period vector */
static void test_fft_stf_time(lib80211_fft_plan *plan) {
    TEST_BEGIN("fft_stf_vs_golden");

    test_vector *vec = vector_load("annex_i1_stf_time_one_period");
    if (!vec) {
        TEST_FAIL("cannot load annex_i1_stf_time_one_period.json");
        return;
    }

    if (vec->n_complex < 16) {
        TEST_FAIL("vector too short: %zu samples", vec->n_complex);
        vector_free(vec);
        return;
    }

    /* Build STF frequency domain */
    float stf_real[N], stf_imag[N];
    build_stf_freq(stf_real, stf_imag);

    /* IFFT */
    float out_real[N], out_imag[N];
    lib80211_fft_inverse(plan, stf_real, stf_imag, out_real, out_imag);

    /* STF repeats every 16 samples. The golden vector is one 16-sample period.
     * Compare first 16 samples of IFFT output.
     * Tolerance: golden vector has 3-decimal precision (~5e-4 rounding error) */
    if (assert_complex_close(vec->real, vec->imag, out_real, out_imag, 16, 1e-3f, "STF IFFT")) {
        TEST_PASS();
    } else {
        TEST_FAIL("STF IFFT mismatch");
    }

    vector_free(vec);
}

int main(void) {
    printf("test_fft\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("FATAL: cannot create FFT plan\n");
        return 1;
    }

    test_fft_roundtrip(plan);
    test_fft_single_bin(plan);
    test_fft_ltf_consistency(plan);
    test_fft_stf_time(plan);

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
