/**
 * test_coverage.c -- Cover gaps identified in code review:
 *   - rate_lookup
 *   - sync_detect on non-frame inputs
 *   - channel estimation on clean LTF
 *   - equalization round-trip
 *   - LDPC encode/decode data
 *   - RX classify legacy frame
 *   - RNG normal distribution
 */

#include "test_util.h"
#include "lib80211/constants.h"
#include "lib80211/sync.h"
#include "lib80211/ofdm.h"
#include "lib80211/channel.h"
#include "lib80211/ldpc.h"
#include "lib80211/tx.h"
#include "lib80211/rx.h"
#include "lib80211/impairments.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ========================================================================
 * Test 1: rate_lookup
 * ======================================================================== */

static void test_rate_lookup(void) {
    TEST_BEGIN("rate_lookup");

    int valid_rates[] = {6, 9, 12, 18, 24, 36, 48, 54};
    int n_valid = (int)(sizeof(valid_rates) / sizeof(valid_rates[0]));

    for (int i = 0; i < n_valid; i++) {
        const lib80211_rate_info *info = lib80211_rate_lookup(valid_rates[i]);
        if (!info) {
            TEST_FAIL("rate %d returned NULL", valid_rates[i]);
            return;
        }
        if (info->rate_mbps != valid_rates[i]) {
            TEST_FAIL("rate %d: entry has rate_mbps=%d", valid_rates[i], info->rate_mbps);
            return;
        }
    }

    /* Invalid rates should return NULL */
    int invalid_rates[] = {0, 5, 7, 100};
    int n_invalid = (int)(sizeof(invalid_rates) / sizeof(invalid_rates[0]));

    for (int i = 0; i < n_invalid; i++) {
        const lib80211_rate_info *info = lib80211_rate_lookup(invalid_rates[i]);
        if (info != NULL) {
            TEST_FAIL("rate %d should return NULL, got rate_mbps=%d",
                      invalid_rates[i], info->rate_mbps);
            return;
        }
    }

    TEST_PASS();
}

/* ========================================================================
 * Test 2: sync_detect returns -1 for non-frame inputs
 * ======================================================================== */

static void test_sync_no_frame(lib80211_fft_plan *plan) {
    TEST_BEGIN("sync_no_frame");

    lib80211_sync_result result;

    /* All-zero buffer (1000 samples) */
    size_t n = 1000;
    float *re = (float *)calloc(n, sizeof(float));
    float *im = (float *)calloc(n, sizeof(float));
    if (!re || !im) {
        TEST_FAIL("malloc failed");
        free(re); free(im);
        return;
    }

    int rc = lib80211_sync_detect(plan, re, im, n, &result);
    if (rc != -1) {
        TEST_FAIL("all-zero buffer: expected -1, got %d", rc);
        free(re); free(im);
        return;
    }

    /* Random noise (small uncorrelated values via simple LCG) */
    uint32_t seed = 12345;
    for (size_t i = 0; i < n; i++) {
        seed = seed * 1103515245u + 12345u;
        re[i] = ((float)(seed >> 16) / 65536.0f - 0.5f) * 0.001f;
        seed = seed * 1103515245u + 12345u;
        im[i] = ((float)(seed >> 16) / 65536.0f - 0.5f) * 0.001f;
    }

    rc = lib80211_sync_detect(plan, re, im, n, &result);
    if (rc != -1) {
        TEST_FAIL("noise buffer: expected -1, got %d", rc);
        free(re); free(im);
        return;
    }

    free(re);
    free(im);

    /* Buffer too short (100 samples) */
    size_t short_n = 100;
    float *short_re = (float *)calloc(short_n, sizeof(float));
    float *short_im = (float *)calloc(short_n, sizeof(float));
    if (!short_re || !short_im) {
        TEST_FAIL("malloc failed");
        free(short_re); free(short_im);
        return;
    }

    rc = lib80211_sync_detect(plan, short_re, short_im, short_n, &result);
    if (rc != -1) {
        TEST_FAIL("short buffer (100): expected -1, got %d", rc);
        free(short_re); free(short_im);
        return;
    }

    free(short_re);
    free(short_im);

    TEST_PASS();
}

/* ========================================================================
 * Test 3: channel estimation on clean LTF
 * ======================================================================== */

static void test_channel_estimate(lib80211_fft_plan *plan) {
    TEST_BEGIN("channel_estimate");

    /* Generate LTF: 160 samples = GI2(32) + T1(64) + T2(64) */
    float ltf_re[160], ltf_im[160];
    lib80211_generate_ltf(plan, ltf_re, ltf_im);

    /* Pass the two 64-sample FFT symbols (samples 32..159) */
    float H_real[64], H_imag[64];
    float noise_var = 0.0f;

    lib80211_estimate_channel(plan,
                              ltf_re + 32, ltf_im + 32,
                              H_real, H_imag, &noise_var);

    /* On a perfect channel, H should be ~1+0j on active subcarriers.
     * Check data + pilot subcarriers. */
    int ok = 1;
    float max_err = 0.0f;

    for (int k = 0; k < 64; k++) {
        /* Skip DC and unused subcarriers — check only active ones.
         * Active: bins 1-6, 7-28, 38-42, 43-58 (subcarriers -26..-1, 1..26 + pilots)
         * Simpler: just check subcarriers that should be non-zero in LTF_FREQ_REAL. */
        if (LIB80211_LTF_FREQ_REAL[k] == 0.0f)
            continue;

        float err_r = fabsf(H_real[k] - 1.0f);
        float err_i = fabsf(H_imag[k]);
        float err = (err_r > err_i) ? err_r : err_i;
        if (err > max_err)
            max_err = err;
    }

    if (max_err > 0.01f) {
        TEST_FAIL("channel estimate error %.4f exceeds 0.01 on active subcarriers", max_err);
        ok = 0;
    }

    if (noise_var > 0.01f) {
        TEST_FAIL("noise_var=%.6f exceeds 0.01", noise_var);
        ok = 0;
    }

    if (ok) {
        printf("    max_H_error=%.2e, noise_var=%.2e\n", max_err, noise_var);
        TEST_PASS();
    }
}

/* ========================================================================
 * Test 4: equalization round-trip
 * ======================================================================== */

static void test_equalize(void) {
    TEST_BEGIN("equalize");

    /* Non-trivial channel: H[k] = 1.5 + 0.5j for all k */
    float H_real[64], H_imag[64];
    float Y_real[64], Y_imag[64];
    float known_real[64], known_imag[64];

    for (int k = 0; k < 64; k++) {
        H_real[k] = 1.5f;
        H_imag[k] = 0.5f;

        /* Known data: alternating pattern */
        known_real[k] = (k % 2 == 0) ? 1.0f : -1.0f;
        known_imag[k] = (k % 3 == 0) ? 0.5f : -0.5f;

        /* Y = H * known_data (complex multiply) */
        Y_real[k] = H_real[k] * known_real[k] - H_imag[k] * known_imag[k];
        Y_imag[k] = H_real[k] * known_imag[k] + H_imag[k] * known_real[k];
    }

    float out_real[64], out_imag[64];
    lib80211_equalize(Y_real, Y_imag, H_real, H_imag, 0.0f, out_real, out_imag);

    /* Verify equalized output matches known data */
    float max_err = 0.0f;
    for (int k = 0; k < 64; k++) {
        float err_r = fabsf(out_real[k] - known_real[k]);
        float err_i = fabsf(out_imag[k] - known_imag[k]);
        float err = (err_r > err_i) ? err_r : err_i;
        if (err > max_err)
            max_err = err;
    }

    if (max_err < 1e-5f) {
        printf("    max_error=%.2e (excellent)\n", max_err);
        TEST_PASS();
    } else {
        TEST_FAIL("max equalization error %.2e exceeds 1e-5", max_err);
    }
}

/* ========================================================================
 * Test 5: LDPC encode/decode data round-trip
 * ======================================================================== */

static void test_ldpc_encode_decode_data(void) {
    TEST_BEGIN("ldpc_encode_decode_data");

    /* MCS 0 params: BPSK, rate 1/2, n_dbps=26, n_cbps=52 */
    int n_dbps = 26;
    int n_cbps = 52;
    int cr_n = 1;
    int cr_d = 2;

    /* Create a known payload: 100 bytes = 800 bits, plus 16 SERVICE bits = 816 bits */
    int n_payload = 816;
    uint8_t *payload = (uint8_t *)calloc(n_payload, sizeof(uint8_t));
    if (!payload) {
        TEST_FAIL("malloc failed");
        return;
    }

    /* Fill with a pattern (simulating scrambled bits) */
    for (int i = 0; i < n_payload; i++) {
        payload[i] = (uint8_t)((i * 7 + 3) & 1);
    }

    /* Encode */
    int n_sym_out = 0;
    int ldpc_extra_out = 0;
    /* Max output: conservative upper bound */
    int max_syms = (n_payload + n_dbps - 1) / n_dbps + 2;
    uint8_t *coded = (uint8_t *)calloc((size_t)max_syms * (size_t)n_cbps, sizeof(uint8_t));
    if (!coded) {
        TEST_FAIL("malloc failed");
        free(payload);
        return;
    }

    int rc = lib80211_ldpc_encode_data(payload, n_payload,
                                       n_dbps, n_cbps, cr_n, cr_d,
                                       coded, &n_sym_out, &ldpc_extra_out);
    if (rc != 0) {
        TEST_FAIL("ldpc_encode_data returned %d", rc);
        free(payload); free(coded);
        return;
    }

    if (n_sym_out <= 0) {
        TEST_FAIL("n_sym_out=%d (expected > 0)", n_sym_out);
        free(payload); free(coded);
        return;
    }

    if (ldpc_extra_out != 0 && ldpc_extra_out != 1) {
        TEST_FAIL("ldpc_extra_out=%d (expected 0 or 1)", ldpc_extra_out);
        free(payload); free(coded);
        return;
    }

    printf("    n_sym_out=%d, ldpc_extra=%d\n", n_sym_out, ldpc_extra_out);

    /* Convert coded bits to LLRs: bit 0 -> +1.0, bit 1 -> -1.0
     * (positive LLR = bit 0 more likely, matches LDPC decode convention) */
    size_t total_soft = (size_t)n_sym_out * (size_t)n_cbps;
    float *llrs = (float *)malloc(total_soft * sizeof(float));
    if (!llrs) {
        TEST_FAIL("malloc failed");
        free(payload); free(coded);
        return;
    }

    for (size_t i = 0; i < total_soft; i++) {
        llrs[i] = (coded[i] == 0) ? 1.0f : -1.0f;
    }

    /* Decode */
    uint8_t *decoded = (uint8_t *)calloc((size_t)n_sym_out * (size_t)n_dbps, sizeof(uint8_t));
    if (!decoded) {
        TEST_FAIL("malloc failed");
        free(payload); free(coded); free(llrs);
        return;
    }

    rc = lib80211_ldpc_decode_data(llrs, total_soft,
                                   decoded, n_sym_out, n_dbps, n_cbps,
                                   cr_n, cr_d);
    if (rc != 0) {
        TEST_FAIL("ldpc_decode_data returned %d", rc);
        free(payload); free(coded); free(llrs); free(decoded);
        return;
    }

    /* Verify round-trip: first n_payload bits should match */
    int mismatches = 0;
    for (int i = 0; i < n_payload; i++) {
        if (decoded[i] != payload[i]) {
            if (mismatches == 0) {
                printf("    first mismatch at bit %d: expected %d, got %d\n",
                       i, payload[i], decoded[i]);
            }
            mismatches++;
        }
    }

    if (mismatches == 0) {
        TEST_PASS();
    } else {
        TEST_FAIL("round-trip: %d/%d bit mismatches", mismatches, n_payload);
    }

    free(payload);
    free(coded);
    free(llrs);
    free(decoded);
}

/* ========================================================================
 * Test 6: RX classify legacy frame (non-6Mbps)
 * ======================================================================== */

static void test_rx_classify_legacy(lib80211_fft_plan *plan) {
    TEST_BEGIN("rx_classify_legacy");

    /* Generate a legacy 24 Mbps frame */
    uint8_t psdu[50];
    memset(psdu, 0xAB, sizeof(psdu));

    lib80211_tx_legacy_params params = {
        .rate_mbps = 24,
        .psdu = psdu,
        .psdu_len = sizeof(psdu),
        .scrambler_seed = 0x5D,
    };

    size_t n_samp = lib80211_tx_legacy_samples(&params);
    if (n_samp == 0) {
        TEST_FAIL("tx_legacy_samples returned 0");
        return;
    }

    float *tx_re = (float *)malloc(n_samp * sizeof(float));
    float *tx_im = (float *)malloc(n_samp * sizeof(float));
    if (!tx_re || !tx_im) {
        TEST_FAIL("malloc failed");
        free(tx_re); free(tx_im);
        return;
    }

    size_t written = lib80211_tx_legacy(plan, &params, tx_re, tx_im);
    if (written == 0) {
        TEST_FAIL("tx_legacy returned 0");
        free(tx_re); free(tx_im);
        return;
    }

    /* Decode the frame */
    lib80211_rx_result result;
    memset(&result, 0, sizeof(result));

    int rc = lib80211_rx_decode(plan, tx_re, tx_im, written, &result);
    if (rc != 0) {
        TEST_FAIL("rx_decode returned %d (no frame detected)", rc);
        free(tx_re); free(tx_im);
        return;
    }

    int ok = 1;
    if (result.type != LIB80211_FRAME_LEGACY) {
        printf("    type=%d (expected %d=LEGACY)\n", result.type, LIB80211_FRAME_LEGACY);
        ok = 0;
    }
    if (result.rate_mbps != 24) {
        printf("    rate_mbps=%d (expected 24)\n", result.rate_mbps);
        ok = 0;
    }

    if (ok) {
        printf("    type=LEGACY, rate=24 Mbps, fcs_valid=%d\n", result.fcs_valid);
        TEST_PASS();
    } else {
        TEST_FAIL("classification mismatch");
    }

    free(tx_re);
    free(tx_im);
}

/* ========================================================================
 * Test 7: RNG normal distribution statistics
 * ======================================================================== */

static void test_rng_normal(void) {
    TEST_BEGIN("rng_normal");

    lib80211_rng rng;
    lib80211_rng_seed(&rng, 42);

    int n = 10000;
    double sum = 0.0;
    double sum_sq = 0.0;

    for (int i = 0; i < n; i++) {
        float x = lib80211_rng_normal(&rng);
        sum += (double)x;
        sum_sq += (double)x * (double)x;
    }

    double mean = sum / (double)n;
    double variance = sum_sq / (double)n - mean * mean;
    double stddev = sqrt(variance);

    printf("    mean=%.4f, stddev=%.4f (n=%d)\n", mean, stddev, n);

    int ok = 1;
    if (fabs(mean) > 0.05) {
        printf("    |mean|=%.4f exceeds 0.05\n", fabs(mean));
        ok = 0;
    }
    if (stddev < 0.9 || stddev > 1.1) {
        printf("    stddev=%.4f outside [0.9, 1.1]\n", stddev);
        ok = 0;
    }

    if (ok) {
        TEST_PASS();
    } else {
        TEST_FAIL("normal distribution statistics out of range");
    }
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(void) {
    printf("test_coverage: additional coverage tests\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("  [ FAIL ] could not create FFT plan\n");
        return 1;
    }

    test_rate_lookup();
    test_sync_no_frame(plan);
    test_channel_estimate(plan);
    test_equalize();
    test_ldpc_encode_decode_data();
    test_rx_classify_legacy(plan);
    test_rng_normal();

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
