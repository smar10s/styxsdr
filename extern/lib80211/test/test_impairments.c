/**
 * test_impairments.c -- Statistical robustness tests under channel impairments.
 *
 * Tests decode success rate under AWGN, CFO, multipath, and combined
 * impairments. Each test uses 20 trials with deterministic seeds and
 * requires >= 80% success rate (16/20) unless otherwise noted.
 *
 * These are NOT unit tests — they're statistical regression gates that
 * verify the decoder operates correctly under realistic conditions.
 */

#include "test_util.h"
#include "lib80211/tx.h"
#include "lib80211/rx.h"
#include "lib80211/fft.h"
#include "lib80211/impairments.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ========================================================================
 * Test infrastructure
 * ======================================================================== */

#define N_TRIALS     20
#define THRESHOLD_80 16   /* 80% of 20 */
#define THRESHOLD_70 14   /* 70% of 20 */
#define PAYLOAD_LEN  80   /* 80 bytes payload + 4 FCS = 84 bytes PSDU */
#define PSDU_LEN     84
#define PAD_SAMPLES  100

static uint32_t test_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static void make_payload(uint8_t *psdu, int seed)
{
    /* Deterministic payload matching Python: (i*7 + 13) % 256 XOR seed */
    for (int i = 0; i < PAYLOAD_LEN; i++)
        psdu[i] = (uint8_t)(((i * 7 + 13) ^ seed) & 0xFF);
    uint32_t fcs = test_crc32(psdu, PAYLOAD_LEN);
    psdu[PAYLOAD_LEN + 0] = (uint8_t)(fcs & 0xFF);
    psdu[PAYLOAD_LEN + 1] = (uint8_t)((fcs >> 8) & 0xFF);
    psdu[PAYLOAD_LEN + 2] = (uint8_t)((fcs >> 16) & 0xFF);
    psdu[PAYLOAD_LEN + 3] = (uint8_t)((fcs >> 24) & 0xFF);
}

/* Frame type for generating test frames */
typedef enum {
    FRAME_LEGACY,
    FRAME_HT,
    FRAME_VHT,
} frame_gen_type;

typedef struct {
    frame_gen_type type;
    int rate_or_mcs;
    bool ldpc;
    bool short_gi;
} frame_config;

/**
 * Generate a frame and return buffer with padding.
 * Caller must free re/im.
 */
static int gen_frame(lib80211_fft_plan *plan, const frame_config *cfg,
                     int seed, float **out_re, float **out_im, size_t *out_n)
{
    uint8_t psdu[PSDU_LEN];
    make_payload(psdu, seed);

    size_t max_samples;
    size_t n_tx;

    switch (cfg->type) {
    case FRAME_LEGACY: {
        lib80211_tx_legacy_params p = {
            .rate_mbps = cfg->rate_or_mcs,
            .psdu = psdu, .psdu_len = PSDU_LEN,
            .scrambler_seed = 0x5D,
        };
        max_samples = lib80211_tx_legacy_samples(&p) + 2 * PAD_SAMPLES;
        *out_re = (float *)calloc(max_samples, sizeof(float));
        *out_im = (float *)calloc(max_samples, sizeof(float));
        if (!*out_re || !*out_im) return -1;
        n_tx = lib80211_tx_legacy(plan, &p, *out_re + PAD_SAMPLES, *out_im + PAD_SAMPLES);
        break;
    }
    case FRAME_HT: {
        lib80211_tx_ht_params p = {
            .mcs = cfg->rate_or_mcs, .psdu = psdu, .psdu_len = PSDU_LEN,
            .scrambler_seed = 0x5D, .short_gi = cfg->short_gi, .ldpc = cfg->ldpc,
        };
        max_samples = lib80211_tx_ht_samples(&p) + 2 * PAD_SAMPLES;
        *out_re = (float *)calloc(max_samples, sizeof(float));
        *out_im = (float *)calloc(max_samples, sizeof(float));
        if (!*out_re || !*out_im) return -1;
        n_tx = lib80211_tx_ht(plan, &p, *out_re + PAD_SAMPLES, *out_im + PAD_SAMPLES);
        break;
    }
    case FRAME_VHT: {
        lib80211_tx_vht_params p = {
            .mcs = cfg->rate_or_mcs, .psdu = psdu, .psdu_len = PSDU_LEN,
            .scrambler_seed = 0x5D, .short_gi = cfg->short_gi, .ldpc = cfg->ldpc,
        };
        max_samples = lib80211_tx_vht_samples(&p) + 2 * PAD_SAMPLES;
        *out_re = (float *)calloc(max_samples, sizeof(float));
        *out_im = (float *)calloc(max_samples, sizeof(float));
        if (!*out_re || !*out_im) return -1;
        n_tx = lib80211_tx_vht(plan, &p, *out_re + PAD_SAMPLES, *out_im + PAD_SAMPLES);
        break;
    }
    default:
        return -1;
    }

    if (n_tx == 0) return -1;
    *out_n = PAD_SAMPLES + n_tx + PAD_SAMPLES;
    if (*out_n > max_samples) *out_n = max_samples;
    return 0;
}

/**
 * Try to decode and verify payload matches.
 */
static int try_decode(lib80211_fft_plan *plan, const frame_config *cfg,
                      int seed, float *re, float *im, size_t n)
{
    uint8_t expected_psdu[PSDU_LEN];
    make_payload(expected_psdu, seed);

    lib80211_rx_result result;
    int rc = lib80211_rx_decode(plan, re, im, n, &result);
    if (rc != 0) return 0;
    if (!result.fcs_valid) return 0;
    if (result.psdu_len != PSDU_LEN) return 0;

    /* Verify payload content */
    if (memcmp(result.psdu, expected_psdu, PAYLOAD_LEN) != 0)
        return 0;

    return 1;  /* success */
}

/* ========================================================================
 * AWGN tests
 * ======================================================================== */

static void test_awgn(lib80211_fft_plan *plan, const char *label,
                      const frame_config *cfg, float snr_db, int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "awgn_%s_%.0fdB", label, snr_db);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        lib80211_rng rng;
        lib80211_rng_seed(&rng, (uint64_t)t);
        lib80211_add_awgn(re, im, n, snr_db, &rng);

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * CFO tests
 * ======================================================================== */

static void test_cfo(lib80211_fft_plan *plan, const char *label,
                     const frame_config *cfg, float cfo_hz, float snr_db,
                     int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "cfo_%s_%.0fHz", label, cfo_hz);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        /* Apply CFO */
        lib80211_add_cfo(re, im, n, cfo_hz, 20e6f, 0.0f);

        /* Add noise backdrop */
        lib80211_rng rng;
        lib80211_rng_seed(&rng, (uint64_t)(t + 1000));
        lib80211_add_awgn(re, im, n, snr_db, &rng);

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * Multipath tests
 * ======================================================================== */

static void test_multipath(lib80211_fft_plan *plan, const char *label,
                           const frame_config *cfg,
                           const lib80211_multipath_tap *base_taps, int n_taps,
                           float snr_db, int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "multipath_%s", label);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        /* Per-trial tap variation: ±30% magnitude, ±45° phase */
        lib80211_rng rng;
        lib80211_rng_seed(&rng, (uint64_t)(t + 2000));

        lib80211_multipath_tap varied_taps[8];
        for (int k = 0; k < n_taps && k < 8; k++) {
            varied_taps[k].delay = base_taps[k].delay;
            if (k == 0) {
                /* Direct path: keep gain=1 */
                varied_taps[k].gain_re = base_taps[k].gain_re;
                varied_taps[k].gain_im = base_taps[k].gain_im;
            } else {
                /* Vary magnitude by ±30% and phase by ±45° */
                float mag_scale = 1.0f + 0.3f * (2.0f * lib80211_rng_float(&rng) - 1.0f);
                float phase_var = 0.785f * (2.0f * lib80211_rng_float(&rng) - 1.0f);  /* ±pi/4 */
                float orig_mag = sqrtf(base_taps[k].gain_re * base_taps[k].gain_re +
                                       base_taps[k].gain_im * base_taps[k].gain_im);
                float orig_phase = atan2f(base_taps[k].gain_im, base_taps[k].gain_re);
                float new_mag = orig_mag * mag_scale;
                float new_phase = orig_phase + phase_var;
                varied_taps[k].gain_re = new_mag * cosf(new_phase);
                varied_taps[k].gain_im = new_mag * sinf(new_phase);
            }
        }

        /* Allocate temp buffers for multipath */
        float *tmp_re = (float *)malloc(n * sizeof(float));
        float *tmp_im = (float *)malloc(n * sizeof(float));
        if (tmp_re && tmp_im) {
            lib80211_apply_multipath(re, im, n, varied_taps, n_taps, tmp_re, tmp_im);
        }
        free(tmp_re); free(tmp_im);

        /* Optional SNR backdrop */
        if (snr_db < 100.0f) {
            lib80211_rng rng2;
            lib80211_rng_seed(&rng2, (uint64_t)(t + 3000));
            lib80211_add_awgn(re, im, n, snr_db, &rng2);
        }

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * Combined impairment tests
 * ======================================================================== */

static void test_combined(lib80211_fft_plan *plan, const char *label,
                          const frame_config *cfg,
                          const lib80211_multipath_tap *taps, int n_taps,
                          float cfo_hz, float snr_db, int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "combined_%s", label);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        /* Multipath (no per-trial variation for combined tests) */
        float *tmp_re = (float *)malloc(n * sizeof(float));
        float *tmp_im = (float *)malloc(n * sizeof(float));
        if (tmp_re && tmp_im) {
            lib80211_apply_multipath(re, im, n, taps, n_taps, tmp_re, tmp_im);
        }
        free(tmp_re); free(tmp_im);

        /* CFO */
        lib80211_add_cfo(re, im, n, cfo_hz, 20e6f, 0.0f);

        /* AWGN */
        lib80211_rng rng;
        lib80211_rng_seed(&rng, (uint64_t)(t + 4000));
        lib80211_add_awgn(re, im, n, snr_db, &rng);

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * SFO tests
 * ======================================================================== */

static void test_sfo(lib80211_fft_plan *plan, const char *label,
                     const frame_config *cfg, float ppm, float snr_db,
                     int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "sfo_%s_%.0fppm", label, ppm);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        /* Resample */
        size_t n_out = lib80211_sfo_output_len(n, ppm);
        float *sfo_re = (float *)malloc(n_out * sizeof(float));
        float *sfo_im = (float *)malloc(n_out * sizeof(float));
        if (!sfo_re || !sfo_im) {
            free(re); free(im); free(sfo_re); free(sfo_im);
            continue;
        }
        n_out = lib80211_add_sfo(re, im, n, ppm, sfo_re, sfo_im);
        free(re); free(im);

        /* Optional noise */
        if (snr_db < 100.0f) {
            lib80211_rng rng;
            lib80211_rng_seed(&rng, (uint64_t)(t + 5000));
            lib80211_add_awgn(sfo_re, sfo_im, n_out, snr_db, &rng);
        }

        /* Decode */
        uint8_t expected_psdu[PSDU_LEN];
        make_payload(expected_psdu, t);
        lib80211_rx_result result;
        int rc = lib80211_rx_decode(plan, sfo_re, sfo_im, n_out, &result);
        if (rc == 0 && result.fcs_valid && result.psdu_len == PSDU_LEN &&
            memcmp(result.psdu, expected_psdu, PAYLOAD_LEN) == 0)
            successes++;

        free(sfo_re); free(sfo_im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * DC offset tests
 * ======================================================================== */

static void test_dc_offset(lib80211_fft_plan *plan, const char *label,
                           const frame_config *cfg, float dc_i, float dc_q,
                           float snr_db, int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "dc_%s", label);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        lib80211_add_dc_offset(re, im, n, dc_i, dc_q);

        if (snr_db < 100.0f) {
            lib80211_rng rng;
            lib80211_rng_seed(&rng, (uint64_t)(t + 6000));
            lib80211_add_awgn(re, im, n, snr_db, &rng);
        }

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * IQ imbalance tests
 * ======================================================================== */

static void test_iq_imbalance(lib80211_fft_plan *plan, const char *label,
                              const frame_config *cfg,
                              float gain_db, float phase_deg,
                              float snr_db, int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "iq_%s", label);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        lib80211_add_iq_imbalance(re, im, n, gain_db, phase_deg);

        if (snr_db < 100.0f) {
            lib80211_rng rng;
            lib80211_rng_seed(&rng, (uint64_t)(t + 7000));
            lib80211_add_awgn(re, im, n, snr_db, &rng);
        }

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * Phase noise tests
 * ======================================================================== */

static void test_phase_noise(lib80211_fft_plan *plan, const char *label,
                             const frame_config *cfg,
                             float strength, float corner_hz,
                             float snr_db, int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "pn_%s", label);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        lib80211_rng rng;
        lib80211_rng_seed(&rng, (uint64_t)(t + 8000));
        lib80211_add_phase_noise(re, im, n, strength, 20e6f, corner_hz, &rng);

        if (snr_db < 100.0f) {
            lib80211_rng rng2;
            lib80211_rng_seed(&rng2, (uint64_t)(t + 8500));
            lib80211_add_awgn(re, im, n, snr_db, &rng2);
        }

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * AGC ramp tests
 * ======================================================================== */

static void test_agc_ramp(lib80211_fft_plan *plan, const char *label,
                          const frame_config *cfg,
                          int settle_samples, float initial_gain_db,
                          float snr_db, int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "agc_%s", label);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        lib80211_add_agc_ramp(re, im, n, settle_samples, initial_gain_db);

        if (snr_db < 100.0f) {
            lib80211_rng rng;
            lib80211_rng_seed(&rng, (uint64_t)(t + 9000));
            lib80211_add_awgn(re, im, n, snr_db, &rng);
        }

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * Quantization tests
 * ======================================================================== */

static void test_quantization(lib80211_fft_plan *plan, const char *label,
                              const frame_config *cfg,
                              int bits, float snr_db, int threshold)
{
    char name[64];
    snprintf(name, sizeof(name), "quant_%s", label);
    TEST_BEGIN(name);

    int successes = 0;
    for (int t = 0; t < N_TRIALS; t++) {
        float *re, *im;
        size_t n;
        if (gen_frame(plan, cfg, t, &re, &im, &n) != 0) {
            free(re); free(im);
            continue;
        }

        lib80211_add_quantization(re, im, n, bits);

        if (snr_db < 100.0f) {
            lib80211_rng rng;
            lib80211_rng_seed(&rng, (uint64_t)(t + 9500));
            lib80211_add_awgn(re, im, n, snr_db, &rng);
        }

        successes += try_decode(plan, cfg, t, re, im, n);
        free(re); free(im);
    }

    if (successes >= threshold) {
        printf("    %d/%d passed (%.0f%%)\n", successes, N_TRIALS,
               100.0 * successes / N_TRIALS);
        TEST_PASS();
    } else {
        TEST_FAIL("%d/%d (need %d)", successes, N_TRIALS, threshold);
    }
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(void)
{
    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("  [ FAIL ] could not create FFT plan\n");
        return 1;
    }

    /* ==== AWGN tests ==== */
    printf("test_impairments: AWGN robustness\n");
    {
        /* Legacy rates */
        struct { int rate; float snr; } legacy_awgn[] = {
            {6, 10}, {9, 10}, {12, 12}, {18, 14},
            {24, 18}, {36, 20}, {48, 24}, {54, 26},
        };
        for (int i = 0; i < 8; i++) {
            char label[16];
            snprintf(label, sizeof(label), "leg%d", legacy_awgn[i].rate);
            frame_config cfg = { FRAME_LEGACY, legacy_awgn[i].rate, false };
            test_awgn(plan, label, &cfg, legacy_awgn[i].snr, THRESHOLD_80);
        }

        /* HT MCS */
        float ht_snr[] = {8, 10, 12, 14, 18, 22, 24, 26};
        for (int mcs = 0; mcs <= 7; mcs++) {
            char label[16];
            snprintf(label, sizeof(label), "ht%d", mcs);
            frame_config cfg = { FRAME_HT, mcs, false };
            test_awgn(plan, label, &cfg, ht_snr[mcs], THRESHOLD_80);
        }

        /* VHT MCS */
        float vht_snr[] = {10, 11, 13, 15, 18, 21, 23, 25, 28};
        for (int mcs = 0; mcs <= 8; mcs++) {
            char label[16];
            snprintf(label, sizeof(label), "vht%d", mcs);
            frame_config cfg = { FRAME_VHT, mcs, false };
            test_awgn(plan, label, &cfg, vht_snr[mcs], THRESHOLD_80);
        }
    }

    /* ==== CFO tests ==== */
    printf("\ntest_impairments: CFO robustness\n");
    {
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_cfo(plan, "leg6_+5k",  &leg6,  5000, 25, THRESHOLD_80);
        test_cfo(plan, "leg6_-5k",  &leg6, -5000, 25, THRESHOLD_80);
        test_cfo(plan, "leg6_+10k", &leg6, 10000, 25, THRESHOLD_80);
        test_cfo(plan, "leg6_-10k", &leg6,-10000, 25, THRESHOLD_80);
        test_cfo(plan, "leg6_+15k", &leg6, 15000, 25, THRESHOLD_80);
        test_cfo(plan, "leg6_-15k", &leg6,-15000, 25, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_cfo(plan, "ht0_+5k",  &ht0,  5000, 25, THRESHOLD_80);
        test_cfo(plan, "ht0_-5k",  &ht0, -5000, 25, THRESHOLD_80);
        test_cfo(plan, "ht0_+10k", &ht0, 10000, 25, THRESHOLD_80);
        test_cfo(plan, "ht0_-10k", &ht0,-10000, 25, THRESHOLD_80);

        frame_config vht0 = { FRAME_VHT, 0, false };
        test_cfo(plan, "vht0_+25k", &vht0, 25000, 15, THRESHOLD_80);
        test_cfo(plan, "vht0_-25k", &vht0,-25000, 15, THRESHOLD_80);
        test_cfo(plan, "vht0_+50k", &vht0, 50000, 15, THRESHOLD_80);
        test_cfo(plan, "vht0_-50k", &vht0,-50000, 15, THRESHOLD_80);

        frame_config vht5 = { FRAME_VHT, 5, false };
        test_cfo(plan, "vht5_+25k", &vht5, 25000, 28, THRESHOLD_80);
        test_cfo(plan, "vht5_-25k", &vht5,-25000, 28, THRESHOLD_80);
        test_cfo(plan, "vht5_+50k", &vht5, 50000, 28, THRESHOLD_80);
        test_cfo(plan, "vht5_-50k", &vht5,-50000, 28, THRESHOLD_80);

        frame_config vht8 = { FRAME_VHT, 8, false };
        test_cfo(plan, "vht8_+25k", &vht8, 25000, 35, THRESHOLD_80);
        test_cfo(plan, "vht8_-25k", &vht8,-25000, 35, THRESHOLD_80);
    }

    /* ==== Multipath tests ==== */
    printf("\ntest_impairments: Multipath robustness\n");
    {
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_multipath(plan, "leg6_mild", &leg6,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 20, THRESHOLD_80);
        test_multipath(plan, "leg6_moderate", &leg6,
                       LIB80211_MULTIPATH_MODERATE, LIB80211_MULTIPATH_MODERATE_N, 20, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_multipath(plan, "ht0_mild", &ht0,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 20, THRESHOLD_80);
        test_multipath(plan, "ht0_moderate", &ht0,
                       LIB80211_MULTIPATH_MODERATE, LIB80211_MULTIPATH_MODERATE_N, 20, THRESHOLD_80);

        /* VHT mild (no AWGN — 999 means effectively none) */
        int vht_mild_mcs[] = {0, 3, 5, 7, 8};
        for (int i = 0; i < 5; i++) {
            char label[32];
            snprintf(label, sizeof(label), "vht%d_mild", vht_mild_mcs[i]);
            frame_config cfg = { FRAME_VHT, vht_mild_mcs[i], false };
            test_multipath(plan, label, &cfg,
                           LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 999, THRESHOLD_80);
        }

        /* VHT moderate */
        int vht_mod_mcs[] = {0, 3, 5};
        for (int i = 0; i < 3; i++) {
            char label[32];
            snprintf(label, sizeof(label), "vht%d_moderate", vht_mod_mcs[i]);
            frame_config cfg = { FRAME_VHT, vht_mod_mcs[i], false };
            test_multipath(plan, label, &cfg,
                           LIB80211_MULTIPATH_MODERATE, LIB80211_MULTIPATH_MODERATE_N, 999, THRESHOLD_80);
        }
    }

    /* ==== Combined tests ==== */
    printf("\ntest_impairments: Combined impairments\n");
    {
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_combined(plan, "leg6_mild_3k_18dB", &leg6,
                      LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N,
                      3000, 18, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_combined(plan, "ht0_mild_3k_18dB", &ht0,
                      LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N,
                      3000, 18, THRESHOLD_80);

        frame_config vht5 = { FRAME_VHT, 5, false };
        test_combined(plan, "vht5_mild_3k_25dB", &vht5,
                      LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N,
                      3000, 25, THRESHOLD_70);

        frame_config vht8 = { FRAME_VHT, 8, false };
        test_combined(plan, "vht8_mild_35dB", &vht8,
                      LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N,
                      0, 35, THRESHOLD_70);
    }

    /* ==== SFO tests ==== */
    printf("\ntest_impairments: SFO robustness\n");
    {
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_sfo(plan, "leg6_+20", &leg6,  20.0f, 999, THRESHOLD_80);
        test_sfo(plan, "leg6_-20", &leg6, -20.0f, 999, THRESHOLD_80);
        test_sfo(plan, "leg6_+40", &leg6,  40.0f, 999, THRESHOLD_80);
        test_sfo(plan, "leg6_-40", &leg6, -40.0f, 999, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_sfo(plan, "ht0_+20", &ht0,  20.0f, 999, THRESHOLD_80);
        test_sfo(plan, "ht0_-20", &ht0, -20.0f, 999, THRESHOLD_80);
        test_sfo(plan, "ht0_+40", &ht0,  40.0f, 999, THRESHOLD_80);
        test_sfo(plan, "ht0_-40", &ht0, -40.0f, 999, THRESHOLD_80);

        frame_config vht0 = { FRAME_VHT, 0, false };
        test_sfo(plan, "vht0_+20", &vht0,  20.0f, 999, THRESHOLD_80);
        test_sfo(plan, "vht0_-20", &vht0, -20.0f, 999, THRESHOLD_80);
        test_sfo(plan, "vht0_+40", &vht0,  40.0f, 999, THRESHOLD_70);
        test_sfo(plan, "vht0_-40", &vht0, -40.0f, 999, THRESHOLD_70);
    }

    /* ==== DC offset tests ==== */
    printf("\ntest_impairments: DC offset robustness\n");
    {
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_dc_offset(plan, "leg6_5pct", &leg6, 0.05f, 0.03f, 999, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_dc_offset(plan, "ht0_8pct", &ht0, 0.08f, 0.05f, 999, THRESHOLD_80);

        frame_config vht0 = { FRAME_VHT, 0, false };
        test_dc_offset(plan, "vht0_5pct", &vht0, 0.05f, 0.05f, 999, THRESHOLD_80);

        frame_config vht5 = { FRAME_VHT, 5, false };
        test_dc_offset(plan, "vht5_10pct_20dB", &vht5, 0.10f, 0.05f, 20, THRESHOLD_70);
    }

    /* ==== IQ imbalance tests ==== */
    printf("\ntest_impairments: IQ imbalance robustness\n");
    {
        /* Moderate imbalance: 1 dB gain, 3° phase — should be tolerable */
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_iq_imbalance(plan, "leg6_1dB_3deg", &leg6, 1.0f, 3.0f, 999, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_iq_imbalance(plan, "ht0_1dB_3deg", &ht0, 1.0f, 3.0f, 999, THRESHOLD_80);

        frame_config vht0 = { FRAME_VHT, 0, false };
        test_iq_imbalance(plan, "vht0_1dB_3deg", &vht0, 1.0f, 3.0f, 999, THRESHOLD_80);

        /* Stronger: 2 dB gain, 5° phase — harder, especially for higher MCS */
        frame_config vht5 = { FRAME_VHT, 5, false };
        test_iq_imbalance(plan, "vht5_2dB_5deg_25dB", &vht5, 2.0f, 5.0f, 25, THRESHOLD_70);

        frame_config ht7 = { FRAME_HT, 7, false };
        test_iq_imbalance(plan, "ht7_1dB_3deg_28dB", &ht7, 1.0f, 3.0f, 28, THRESHOLD_80);
    }

    /* ==== Phase noise tests ==== */
    printf("\ntest_impairments: Phase noise robustness\n");
    {
        /* Mild phase noise: strength=0.005 (RMS~0.02 rad) — low MCS should cope */
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_phase_noise(plan, "leg6_mild", &leg6, 0.005f, 100e3f, 999, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_phase_noise(plan, "ht0_mild", &ht0, 0.005f, 100e3f, 999, THRESHOLD_80);

        frame_config vht0 = { FRAME_VHT, 0, false };
        test_phase_noise(plan, "vht0_mild", &vht0, 0.005f, 100e3f, 999, THRESHOLD_80);

        /* Moderate: strength=0.01 (RMS~0.04 rad) */
        frame_config ht3 = { FRAME_HT, 3, false };
        test_phase_noise(plan, "ht3_moderate", &ht3, 0.01f, 100e3f, 999, THRESHOLD_80);

        frame_config vht3 = { FRAME_VHT, 3, false };
        test_phase_noise(plan, "vht3_moderate", &vht3, 0.01f, 100e3f, 999, THRESHOLD_80);
    }

    /* ==== AGC ramp tests ==== */
    printf("\ntest_impairments: AGC ramp robustness\n");
    {
        /* Short settle (64 samples, -10 dB): well within STF (160 samples) */
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_agc_ramp(plan, "leg6_64samp_-10dB", &leg6, 64, -10.0f, 999, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_agc_ramp(plan, "ht0_64samp_-10dB", &ht0, 64, -10.0f, 999, THRESHOLD_80);

        frame_config vht0 = { FRAME_VHT, 0, false };
        test_agc_ramp(plan, "vht0_64samp_-10dB", &vht0, 64, -10.0f, 999, THRESHOLD_80);

        /* Longer settle (128 samples, -20 dB): still within STF+pad */
        test_agc_ramp(plan, "leg6_128samp_-20dB", &leg6, 128, -20.0f, 999, THRESHOLD_80);
        test_agc_ramp(plan, "ht0_128samp_-20dB", &ht0, 128, -20.0f, 999, THRESHOLD_80);
        test_agc_ramp(plan, "vht0_128samp_-20dB", &vht0, 128, -20.0f, 999, THRESHOLD_80);
    }

    /* ==== Quantization tests ==== */
    printf("\ntest_impairments: Quantization robustness\n");
    {
        /* 12-bit ADC (PlutoSDR) — should be transparent */
        frame_config leg6 = { FRAME_LEGACY, 6, false };
        test_quantization(plan, "leg6_12bit", &leg6, 12, 999, THRESHOLD_80);

        frame_config ht7 = { FRAME_HT, 7, false };
        test_quantization(plan, "ht7_12bit", &ht7, 12, 999, THRESHOLD_80);

        frame_config vht8 = { FRAME_VHT, 8, false };
        test_quantization(plan, "vht8_12bit", &vht8, 12, 999, THRESHOLD_80);

        /* 8-bit ADC — coarser, but should still work for lower MCS */
        test_quantization(plan, "leg6_8bit", &leg6, 8, 999, THRESHOLD_80);

        frame_config ht0 = { FRAME_HT, 0, false };
        test_quantization(plan, "ht0_8bit", &ht0, 8, 999, THRESHOLD_80);

        frame_config vht0 = { FRAME_VHT, 0, false, false };
        test_quantization(plan, "vht0_8bit", &vht0, 8, 999, THRESHOLD_80);
    }

    /* ==== Short GI under impairments ==== */
    printf("\ntest_impairments: Short GI robustness\n");
    {
        /* SGI AWGN — needs ~1-2 dB more SNR than long GI (shorter CP = less margin) */
        frame_config ht0_sgi = { FRAME_HT, 0, false, true };
        test_awgn(plan, "ht0_sgi", &ht0_sgi, 10, THRESHOLD_80);

        frame_config ht1_sgi = { FRAME_HT, 1, false, true };
        test_awgn(plan, "ht1_sgi", &ht1_sgi, 12, THRESHOLD_80);

        frame_config ht3_sgi = { FRAME_HT, 3, false, true };
        test_awgn(plan, "ht3_sgi", &ht3_sgi, 16, THRESHOLD_80);

        frame_config ht5_sgi = { FRAME_HT, 5, false, true };
        test_awgn(plan, "ht5_sgi", &ht5_sgi, 24, THRESHOLD_80);

        frame_config ht7_sgi = { FRAME_HT, 7, false, true };
        test_awgn(plan, "ht7_sgi", &ht7_sgi, 28, THRESHOLD_80);

        frame_config vht0_sgi = { FRAME_VHT, 0, false, true };
        test_awgn(plan, "vht0_sgi", &vht0_sgi, 12, THRESHOLD_80);

        frame_config vht3_sgi = { FRAME_VHT, 3, false, true };
        test_awgn(plan, "vht3_sgi", &vht3_sgi, 17, THRESHOLD_80);

        frame_config vht5_sgi = { FRAME_VHT, 5, false, true };
        test_awgn(plan, "vht5_sgi", &vht5_sgi, 23, THRESHOLD_80);

        frame_config vht7_sgi = { FRAME_VHT, 7, false, true };
        test_awgn(plan, "vht7_sgi", &vht7_sgi, 27, THRESHOLD_80);

        frame_config vht8_sgi = { FRAME_VHT, 8, false, true };
        test_awgn(plan, "vht8_sgi", &vht8_sgi, 30, THRESHOLD_80);

        /* SGI CFO */
        test_cfo(plan, "ht0_sgi_+5k",  &ht0_sgi,  5000, 25, THRESHOLD_80);
        test_cfo(plan, "ht0_sgi_+10k", &ht0_sgi, 10000, 25, THRESHOLD_80);
        test_cfo(plan, "ht0_sgi_-5k",  &ht0_sgi, -5000, 25, THRESHOLD_80);
        test_cfo(plan, "ht0_sgi_-10k", &ht0_sgi,-10000, 25, THRESHOLD_80);

        /* SGI multipath — only mild (max delay 3 < 8-sample short CP) */
        test_multipath(plan, "ht0_sgi_mild", &ht0_sgi,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 22, THRESHOLD_80);
        test_multipath(plan, "ht3_sgi_mild", &ht3_sgi,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 22, THRESHOLD_80);
        test_multipath(plan, "ht7_sgi_mild", &ht7_sgi,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 30, THRESHOLD_80);
        test_multipath(plan, "vht0_sgi_mild", &vht0_sgi,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 999, THRESHOLD_80);
        test_multipath(plan, "vht3_sgi_mild", &vht3_sgi,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 999, THRESHOLD_80);
        test_multipath(plan, "vht5_sgi_mild", &vht5_sgi,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 999, THRESHOLD_80);
        test_multipath(plan, "vht8_sgi_mild", &vht8_sgi,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 999, THRESHOLD_80);

        /* SGI combined: mild multipath + CFO + AWGN */
        test_combined(plan, "ht0_sgi_mild_3k_20dB", &ht0_sgi,
                      LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N,
                      3000, 20, THRESHOLD_80);
        test_combined(plan, "vht3_sgi_mild_3k_22dB", &vht3_sgi,
                      LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N,
                      3000, 22, THRESHOLD_70);
    }

    /* ==== LDPC under impairments ==== */
    printf("\ntest_impairments: LDPC robustness\n");
    {
        /* LDPC AWGN: HT MCS 0, 3, 7 and VHT MCS 0, 5, 8 */
        frame_config ht0_ldpc = { FRAME_HT, 0, true };
        test_awgn(plan, "ht0_ldpc", &ht0_ldpc, 8, THRESHOLD_80);

        frame_config ht3_ldpc = { FRAME_HT, 3, true };
        test_awgn(plan, "ht3_ldpc", &ht3_ldpc, 14, THRESHOLD_80);

        frame_config ht7_ldpc = { FRAME_HT, 7, true };
        test_awgn(plan, "ht7_ldpc", &ht7_ldpc, 26, THRESHOLD_80);

        frame_config vht0_ldpc = { FRAME_VHT, 0, true };
        test_awgn(plan, "vht0_ldpc", &vht0_ldpc, 10, THRESHOLD_80);

        frame_config vht5_ldpc = { FRAME_VHT, 5, true };
        test_awgn(plan, "vht5_ldpc", &vht5_ldpc, 21, THRESHOLD_80);

        frame_config vht8_ldpc = { FRAME_VHT, 8, true };
        test_awgn(plan, "vht8_ldpc", &vht8_ldpc, 28, THRESHOLD_80);

        /* LDPC CFO */
        test_cfo(plan, "ht0_ldpc_+5k", &ht0_ldpc, 5000, 25, THRESHOLD_80);
        test_cfo(plan, "vht0_ldpc_+25k", &vht0_ldpc, 25000, 15, THRESHOLD_80);
        test_cfo(plan, "vht5_ldpc_+25k", &vht5_ldpc, 25000, 28, THRESHOLD_80);

        /* LDPC multipath */
        test_multipath(plan, "ht0_ldpc_mild", &ht0_ldpc,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 20, THRESHOLD_80);
        test_multipath(plan, "vht0_ldpc_mild", &vht0_ldpc,
                       LIB80211_MULTIPATH_MILD, LIB80211_MULTIPATH_MILD_N, 999, THRESHOLD_80);
        test_multipath(plan, "vht5_ldpc_moderate", &vht5_ldpc,
                       LIB80211_MULTIPATH_MODERATE, LIB80211_MULTIPATH_MODERATE_N, 999, THRESHOLD_70);
    }

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
