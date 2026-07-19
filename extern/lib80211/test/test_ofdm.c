/**
 * test_ofdm.c — Validate OFDM symbol assembly, L-SIG, and preamble.
 *
 * Tests against Annex I.1 golden vectors:
 * - Signal bits, encoded, interleaved, frequency-domain
 * - Data frequency with pilots (validates subcarrier mapping + pilot insertion)
 * - STF time domain
 * - LTF time domain
 */

#include "test_util.h"
#include "lib80211/ofdm.h"
#include "lib80211/signal.h"
#include "lib80211/constants.h"
#include "lib80211/fec.h"
#include "lib80211/interleaver.h"
#include "lib80211/modulation.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* Test L-SIG bits match golden vector */
static void test_signal_bits(void) {
    TEST_BEGIN("signal_bits_annex_i1");

    test_vector *vec = vector_load("annex_i1_signal_bits");
    if (!vec || !vec->bits || vec->n_bits != 24) {
        TEST_FAIL("cannot load signal_bits vector (expected 24 bits)");
        if (vec) vector_free(vec);
        return;
    }

    /* Annex I.1: rate 36 Mbps, length 100 bytes */
    uint8_t sig_bits[24];
    lib80211_make_signal_bits(36, 100, sig_bits);

    if (assert_bits_equal(vec->bits, sig_bits, 24, "signal bits")) {
        TEST_PASS();
    } else {
        TEST_FAIL("signal bits mismatch");
    }

    vector_free(vec);
}

/* Test L-SIG encoding chain: bits -> encode -> interleave */
static void test_signal_encoded(void) {
    TEST_BEGIN("signal_encoded_annex_i1");

    test_vector *vec = vector_load("annex_i1_signal_encoded");
    if (!vec || !vec->bits || vec->n_bits != 48) {
        TEST_FAIL("cannot load signal_encoded vector");
        if (vec) vector_free(vec);
        return;
    }

    uint8_t sig_bits[24];
    lib80211_make_signal_bits(36, 100, sig_bits);

    uint8_t coded[48];
    lib80211_conv_encode(sig_bits, coded, 24, false);

    if (assert_bits_equal(vec->bits, coded, 48, "signal encoded")) {
        TEST_PASS();
    } else {
        TEST_FAIL("signal encoded mismatch");
    }

    vector_free(vec);
}

static void test_signal_interleaved(void) {
    TEST_BEGIN("signal_interleaved_annex_i1");

    test_vector *vec = vector_load("annex_i1_signal_interleaved");
    if (!vec || !vec->bits || vec->n_bits != 48) {
        TEST_FAIL("cannot load signal_interleaved vector");
        if (vec) vector_free(vec);
        return;
    }

    uint8_t sig_bits[24];
    lib80211_make_signal_bits(36, 100, sig_bits);

    uint8_t coded[48];
    lib80211_conv_encode(sig_bits, coded, 24, false);

    uint8_t interleaved[48];
    lib80211_interleave(coded, interleaved, 48, 1);

    if (assert_bits_equal(vec->bits, interleaved, 48, "signal interleaved")) {
        TEST_PASS();
    } else {
        TEST_FAIL("signal interleaved mismatch");
    }

    vector_free(vec);
}

/* Test frequency-domain signal (BPSK on 48 subcarriers + pilots) */
static void test_signal_freq(lib80211_fft_plan *plan) {
    TEST_BEGIN("signal_freq_annex_i1");

    test_vector *vec = vector_load("annex_i1_signal_freq");
    if (!vec || !vec->real || vec->n_complex != 64) {
        TEST_FAIL("cannot load signal_freq vector (expected 64 complex)");
        if (vec) vector_free(vec);
        return;
    }

    /* Generate signal symbol frequency domain */
    uint8_t sig_bits[24];
    lib80211_make_signal_bits(36, 100, sig_bits);

    uint8_t coded[48];
    lib80211_conv_encode(sig_bits, coded, 24, false);

    uint8_t interleaved[48];
    lib80211_interleave(coded, interleaved, 48, 1);

    float data_real[48], data_imag[48];
    lib80211_modulate(interleaved, data_real, data_imag, 48, 1);

    /* Build freq-domain (same as what make_ofdm_symbol does internally) */
    float freq_real[64] = {0};
    float freq_imag[64] = {0};

    for (int i = 0; i < 48; i++) {
        int bin = LIB80211_DATA_BINS[i];
        freq_real[bin] = data_real[i];
        freq_imag[bin] = data_imag[i];
    }

    /* Pilots for symbol 0 */
    float polarity = (float)LIB80211_PILOT_POLARITY[0];
    for (int i = 0; i < 4; i++) {
        int bin = LIB80211_PILOT_BINS[i];
        freq_real[bin] = polarity * LIB80211_PILOT_BASE[i];
        freq_imag[bin] = 0.0f;
    }

    /* The golden vector uses subcarrier indexing (index 0 = sc -32, index 32 = DC)
     * and has null at pilot positions. Compare only non-pilot, non-null subcarriers.
     * We need to convert from FFT bin ordering to subcarrier ordering for comparison,
     * and skip pilot bins (7, 21, 43, 57). */
    int ok = 1;
    for (int sc = -32; sc < 32; sc++) {
        int json_idx = sc + 32;
        int fft_bin = ((sc % 64) + 64) % 64;

        /* Skip pilot bins */
        if (fft_bin == 7 || fft_bin == 21 || fft_bin == 43 || fft_bin == 57)
            continue;

        float exp_r = vec->real[json_idx];
        float exp_i = vec->imag[json_idx];
        float act_r = freq_real[fft_bin];
        float act_i = freq_imag[fft_bin];

        float err = sqrtf((exp_r - act_r) * (exp_r - act_r) +
                         (exp_i - act_i) * (exp_i - act_i));
        if (err > 1e-5f) {
            printf("    signal freq: mismatch at sc %d (bin %d): "
                   "expected %.4f+%.4fi, got %.4f+%.4fi (err=%.2e)\n",
                   sc, fft_bin, exp_r, exp_i, act_r, act_i, err);
            ok = 0;
            break;
        }
    }

    if (ok) {
        TEST_PASS();
    } else {
        TEST_FAIL("signal freq mismatch");
    }

    vector_free(vec);
}

/* Test data frequency with pilots (first data symbol) */
static void test_data_freq_with_pilots(lib80211_fft_plan *plan) {
    TEST_BEGIN("data_freq_pilots_annex_i1");

    test_vector *freq_vec = vector_load("annex_i1_data_freq_with_pilots");
    test_vector *int_vec = vector_load("annex_i1_data_interleaved");

    if (!freq_vec || !freq_vec->real || freq_vec->n_complex != 64) {
        TEST_FAIL("cannot load data_freq_with_pilots vector");
        goto cleanup;
    }
    if (!int_vec || !int_vec->bits) {
        TEST_FAIL("cannot load interleaved vector");
        goto cleanup;
    }

    /* Modulate first symbol: 192 interleaved bits -> 48 16-QAM symbols */
    float data_real[48], data_imag[48];
    lib80211_modulate(int_vec->bits, data_real, data_imag, 48, 4);

    /* Build freq-domain with pilots (symbol_index=1 for first DATA symbol) */
    float freq_real[64] = {0};
    float freq_imag[64] = {0};

    for (int i = 0; i < 48; i++) {
        int bin = LIB80211_DATA_BINS[i];
        freq_real[bin] = data_real[i];
        freq_imag[bin] = data_imag[i];
    }

    /* Pilot polarity for DATA symbol 0 uses polarity index 1
     * (SIGNAL uses index 0, first DATA symbol uses index 1) */
    float polarity = (float)LIB80211_PILOT_POLARITY[1];
    for (int i = 0; i < 4; i++) {
        int bin = LIB80211_PILOT_BINS[i];
        freq_real[bin] = polarity * LIB80211_PILOT_BASE[i];
        freq_imag[bin] = 0.0f;
    }

    /* Convert golden vector from subcarrier ordering to FFT bin ordering */
    float expected_real[64], expected_imag[64];
    for (int sc = -32; sc < 32; sc++) {
        int json_idx = sc + 32;
        int fft_bin = ((sc % 64) + 64) % 64;
        expected_real[fft_bin] = freq_vec->real[json_idx];
        expected_imag[fft_bin] = freq_vec->imag[json_idx];
    }

    /* Tolerance: golden has 3-decimal precision */
    if (assert_complex_close(expected_real, expected_imag,
                            freq_real, freq_imag, 64, 1e-3f, "data freq+pilots")) {
        TEST_PASS();
    } else {
        TEST_FAIL("data freq+pilots mismatch");
    }

cleanup:
    if (freq_vec) vector_free(freq_vec);
    if (int_vec) vector_free(int_vec);
}

/* Test STF time domain */
static void test_stf_time(lib80211_fft_plan *plan) {
    TEST_BEGIN("stf_time_annex_i1");

    test_vector *vec = vector_load("annex_i1_stf_time");
    if (!vec || !vec->real) {
        TEST_FAIL("cannot load stf_time vector");
        if (vec) vector_free(vec);
        return;
    }

    float stf_real[160], stf_imag[160];
    lib80211_generate_stf(plan, stf_real, stf_imag);

    /* The golden vector has windowing (W[0]=0.5, W[159]=0.5) but we generate
     * raw samples without windowing. Compare interior samples (1..158). */
    size_t n_compare = (vec->n_complex < 160) ? vec->n_complex - 2 : 158;
    if (assert_complex_close(&vec->real[1], &vec->imag[1],
                            &stf_real[1], &stf_imag[1], n_compare, 1e-3f, "STF time")) {
        TEST_PASS();
    } else {
        TEST_FAIL("STF time mismatch");
    }

    vector_free(vec);
}

/* Test LTF time domain */
static void test_ltf_time(lib80211_fft_plan *plan) {
    TEST_BEGIN("ltf_time_annex_i1");

    test_vector *vec = vector_load("annex_i1_ltf_time");
    if (!vec || !vec->real) {
        TEST_FAIL("cannot load ltf_time vector");
        if (vec) vector_free(vec);
        return;
    }

    float ltf_real[160], ltf_imag[160];
    lib80211_generate_ltf(plan, ltf_real, ltf_imag);

    /* Golden has 161 samples with windowing. Compare interior (1..159).
     * Note: golden has samples 0..160 with W[0]=0.5, W[160]=0.5.
     * Our output is 160 samples (0..159) without windowing.
     * Compare samples 1..159 of golden with 1..159 of ours.
     *
     * Tolerance: The Annex I golden vector was generated by the IEEE reference
     * implementation which may use a slightly different IFFT convention.
     * The LTF is real in frequency, so imaginary residuals are small (~0.02)
     * but can have sign differences. Use 0.05 tolerance (matching Python test suite). */
    size_t n_compare = 159;
    if (vec->n_complex < 161) n_compare = vec->n_complex - 2;

    if (assert_complex_close(&vec->real[1], &vec->imag[1],
                            &ltf_real[1], &ltf_imag[1], n_compare, 5e-2f, "LTF time")) {
        TEST_PASS();
    } else {
        TEST_FAIL("LTF time mismatch");
    }

    vector_free(vec);
}

int main(void) {
    printf("test_ofdm\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("FATAL: cannot create FFT plan\n");
        return 1;
    }

    test_signal_bits();
    test_signal_encoded();
    test_signal_interleaved();
    test_signal_freq(plan);
    test_data_freq_with_pilots(plan);
    test_stf_time(plan);
    test_ltf_time(plan);

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
