/**
 * bench_decode.c -- Decode throughput benchmark for lib80211.
 *
 * Pre-generates frames via TX, then measures decode speed in a tight loop.
 * No I/O in the hot path. Reports frames/sec and us/frame.
 *
 * NOT a ctest — run manually for performance measurement.
 */

#include "lib80211/tx.h"
#include "lib80211/rx.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

/* ========================================================================
 * Timing
 * ======================================================================== */

static double get_time_sec(void)
{
#ifdef __APPLE__
    static mach_timebase_info_data_t tb;
    if (tb.denom == 0) mach_timebase_info(&tb);
    uint64_t t = mach_absolute_time();
    return (double)t * (double)tb.numer / (double)tb.denom / 1e9;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
#endif
}

/* ========================================================================
 * Test frame generation
 * ======================================================================== */

static const int PSDU_LEN = 100;

static uint32_t bench_crc32(const uint8_t *data, size_t len)
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

static void make_psdu(uint8_t *psdu, int seed)
{
    for (int i = 0; i < PSDU_LEN - 4; i++)
        psdu[i] = (uint8_t)((i * 7 + seed) & 0xFF);
    uint32_t fcs = bench_crc32(psdu, PSDU_LEN - 4);
    psdu[PSDU_LEN - 4] = (uint8_t)(fcs & 0xFF);
    psdu[PSDU_LEN - 3] = (uint8_t)((fcs >> 8) & 0xFF);
    psdu[PSDU_LEN - 2] = (uint8_t)((fcs >> 16) & 0xFF);
    psdu[PSDU_LEN - 1] = (uint8_t)((fcs >> 24) & 0xFF);
}

/* ========================================================================
 * Benchmark runner
 * ======================================================================== */

typedef struct {
    float *re;
    float *im;
    size_t n_samples;
} bench_frame;

static bench_frame generate_legacy(lib80211_fft_plan *plan, int rate_mbps)
{
    bench_frame f = {0};
    uint8_t psdu[100];
    make_psdu(psdu, rate_mbps);

    lib80211_tx_legacy_params p = {
        .rate_mbps = rate_mbps,
        .psdu = psdu,
        .psdu_len = PSDU_LEN,
        .scrambler_seed = 0x5D,
    };

    size_t max_samp = lib80211_tx_legacy_samples(&p) + 200;
    f.re = (float *)calloc(max_samp, sizeof(float));
    f.im = (float *)calloc(max_samp, sizeof(float));

    size_t n_tx = lib80211_tx_legacy(plan, &p, f.re + 100, f.im + 100);
    f.n_samples = 100 + n_tx + 100;
    if (f.n_samples > max_samp) f.n_samples = max_samp;
    return f;
}

static bench_frame generate_ht(lib80211_fft_plan *plan, int mcs, bool ldpc)
{
    bench_frame f = {0};
    uint8_t psdu[100];
    make_psdu(psdu, mcs + 100);

    lib80211_tx_ht_params p = {
        .mcs = mcs,
        .psdu = psdu,
        .psdu_len = PSDU_LEN,
        .scrambler_seed = 0x5D,
        .short_gi = false,
        .ldpc = ldpc,
    };

    size_t max_samp = lib80211_tx_ht_samples(&p) + 200;
    f.re = (float *)calloc(max_samp, sizeof(float));
    f.im = (float *)calloc(max_samp, sizeof(float));

    size_t n_tx = lib80211_tx_ht(plan, &p, f.re + 100, f.im + 100);
    f.n_samples = 100 + n_tx + 100;
    if (f.n_samples > max_samp) f.n_samples = max_samp;
    return f;
}

static bench_frame generate_vht(lib80211_fft_plan *plan, int mcs, bool ldpc)
{
    bench_frame f = {0};
    uint8_t psdu[100];
    make_psdu(psdu, mcs + 200);

    lib80211_tx_vht_params p = {
        .mcs = mcs,
        .psdu = psdu,
        .psdu_len = PSDU_LEN,
        .scrambler_seed = 0x5D,
        .short_gi = false,
        .ldpc = ldpc,
    };

    size_t max_samp = lib80211_tx_vht_samples(&p) + 200;
    f.re = (float *)calloc(max_samp, sizeof(float));
    f.im = (float *)calloc(max_samp, sizeof(float));

    size_t n_tx = lib80211_tx_vht(plan, &p, f.re + 100, f.im + 100);
    f.n_samples = 100 + n_tx + 100;
    if (f.n_samples > max_samp) f.n_samples = max_samp;
    return f;
}

static void free_frame(bench_frame *f)
{
    free(f->re);
    free(f->im);
    f->re = NULL;
    f->im = NULL;
}

/**
 * Run decode benchmark for a given pre-generated frame.
 * Adjusts iteration count to run for at least 1 second.
 */
static void bench_decode_one(lib80211_fft_plan *plan, const char *name, bench_frame *f)
{
    /* Warm up */
    lib80211_rx_result result;
    for (int i = 0; i < 5; i++) {
        lib80211_rx_decode(plan, f->re, f->im, f->n_samples, &result);
    }

    /* Calibrate: find N so total time >= 1 second */
    int n_iters = 100;
    double elapsed;
    while (1) {
        double t0 = get_time_sec();
        for (int i = 0; i < n_iters; i++) {
            lib80211_rx_decode(plan, f->re, f->im, f->n_samples, &result);
        }
        elapsed = get_time_sec() - t0;
        if (elapsed >= 1.0) break;
        n_iters *= 2;
        if (n_iters > 100000) break; /* safety */
    }

    double fps = (double)n_iters / elapsed;
    double us = elapsed * 1e6 / (double)n_iters;

    printf("  %-24s: %7.0f frames/sec  %7.1f us/frame  (%d iters)\n",
           name, fps, us, n_iters);
}

/* ========================================================================
 * Encode benchmark helpers
 * ======================================================================== */

typedef void (*encode_fn)(lib80211_fft_plan *plan, void *params,
                          float *out_re, float *out_im);

typedef struct {
    const char *name;
    void *params;
    size_t max_samples;
    encode_fn fn;
} bench_encode_config;

static void encode_legacy_wrapper(lib80211_fft_plan *plan, void *params,
                                  float *out_re, float *out_im)
{
    lib80211_tx_legacy(plan, (const lib80211_tx_legacy_params *)params,
                       out_re, out_im);
}

static void encode_ht_wrapper(lib80211_fft_plan *plan, void *params,
                              float *out_re, float *out_im)
{
    lib80211_tx_ht(plan, (const lib80211_tx_ht_params *)params,
                   out_re, out_im);
}

static void encode_vht_wrapper(lib80211_fft_plan *plan, void *params,
                               float *out_re, float *out_im)
{
    lib80211_tx_vht(plan, (const lib80211_tx_vht_params *)params,
                    out_re, out_im);
}

static void bench_encode_one(lib80211_fft_plan *plan, const char *name,
                             encode_fn fn, void *params, size_t max_samples)
{
    float *out_re = (float *)calloc(max_samples, sizeof(float));
    float *out_im = (float *)calloc(max_samples, sizeof(float));
    if (!out_re || !out_im) { free(out_re); free(out_im); return; }

    /* Warm up */
    for (int i = 0; i < 5; i++) {
        fn(plan, params, out_re, out_im);
    }

    /* Calibrate */
    int n_iters = 100;
    double elapsed;
    while (1) {
        double t0 = get_time_sec();
        for (int i = 0; i < n_iters; i++) {
            fn(plan, params, out_re, out_im);
        }
        elapsed = get_time_sec() - t0;
        if (elapsed >= 1.0) break;
        n_iters *= 2;
        if (n_iters > 100000) break;
    }

    double fps = (double)n_iters / elapsed;
    double us = elapsed * 1e6 / (double)n_iters;

    printf("  %-24s: %7.0f frames/sec  %7.1f us/frame  (%d iters)\n",
           name, fps, us, n_iters);

    free(out_re);
    free(out_im);
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(void)
{
    printf("bench: lib80211 encode/decode throughput (100-byte PSDU)\n");
    printf("============================================================\n\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "Failed to create FFT plan\n");
        return 1;
    }

    uint8_t psdu[100];
    make_psdu(psdu, 42);

    /* ================================================================
     * DECODE benchmarks
     * ================================================================ */
    printf("DECODE\n------\n");

    /* Legacy */
    printf("  Legacy:\n");
    {
        int rates[] = {6, 12, 24, 36, 48, 54};
        int n_rates = (int)(sizeof(rates) / sizeof(rates[0]));
        for (int i = 0; i < n_rates; i++) {
            char name[32];
            snprintf(name, sizeof(name), "legacy_%d_mbps", rates[i]);
            bench_frame f = generate_legacy(plan, rates[i]);
            bench_decode_one(plan, name, &f);
            free_frame(&f);
        }
    }

    /* HT BCC */
    printf("\n  HT (BCC):\n");
    {
        int mcs_list[] = {0, 2, 4, 7};
        int n_mcs = (int)(sizeof(mcs_list) / sizeof(mcs_list[0]));
        for (int i = 0; i < n_mcs; i++) {
            char name[32];
            snprintf(name, sizeof(name), "ht_mcs%d_bcc", mcs_list[i]);
            bench_frame f = generate_ht(plan, mcs_list[i], false);
            bench_decode_one(plan, name, &f);
            free_frame(&f);
        }
    }

    /* HT LDPC */
    printf("\n  HT (LDPC):\n");
    {
        int mcs_list[] = {0, 4, 7};
        int n_mcs = (int)(sizeof(mcs_list) / sizeof(mcs_list[0]));
        for (int i = 0; i < n_mcs; i++) {
            char name[32];
            snprintf(name, sizeof(name), "ht_mcs%d_ldpc", mcs_list[i]);
            bench_frame f = generate_ht(plan, mcs_list[i], true);
            bench_decode_one(plan, name, &f);
            free_frame(&f);
        }
    }

    /* VHT BCC */
    printf("\n  VHT (BCC):\n");
    {
        int mcs_list[] = {0, 2, 4, 6, 8};
        int n_mcs = (int)(sizeof(mcs_list) / sizeof(mcs_list[0]));
        for (int i = 0; i < n_mcs; i++) {
            char name[32];
            snprintf(name, sizeof(name), "vht_mcs%d_bcc", mcs_list[i]);
            bench_frame f = generate_vht(plan, mcs_list[i], false);
            bench_decode_one(plan, name, &f);
            free_frame(&f);
        }
    }

    /* VHT LDPC */
    printf("\n  VHT (LDPC):\n");
    {
        int mcs_list[] = {0, 4, 8};
        int n_mcs = (int)(sizeof(mcs_list) / sizeof(mcs_list[0]));
        for (int i = 0; i < n_mcs; i++) {
            char name[32];
            snprintf(name, sizeof(name), "vht_mcs%d_ldpc", mcs_list[i]);
            bench_frame f = generate_vht(plan, mcs_list[i], true);
            bench_decode_one(plan, name, &f);
            free_frame(&f);
        }
    }

    /* ================================================================
     * ENCODE benchmarks
     * ================================================================ */
    printf("\nENCODE\n------\n");

    /* Legacy encode */
    printf("  Legacy:\n");
    {
        int rates[] = {6, 12, 24, 36, 48, 54};
        int n_rates = (int)(sizeof(rates) / sizeof(rates[0]));
        for (int i = 0; i < n_rates; i++) {
            char name[32];
            snprintf(name, sizeof(name), "legacy_%d_mbps", rates[i]);
            lib80211_tx_legacy_params p = {
                .rate_mbps = rates[i], .psdu = psdu,
                .psdu_len = PSDU_LEN, .scrambler_seed = 0x5D,
            };
            size_t max_s = lib80211_tx_legacy_samples(&p) + 100;
            bench_encode_one(plan, name, encode_legacy_wrapper, &p, max_s);
        }
    }

    /* HT encode BCC */
    printf("\n  HT (BCC):\n");
    {
        int mcs_list[] = {0, 2, 4, 7};
        int n_mcs = (int)(sizeof(mcs_list) / sizeof(mcs_list[0]));
        for (int i = 0; i < n_mcs; i++) {
            char name[32];
            snprintf(name, sizeof(name), "ht_mcs%d_bcc", mcs_list[i]);
            lib80211_tx_ht_params p = {
                .mcs = mcs_list[i], .psdu = psdu,
                .psdu_len = PSDU_LEN, .scrambler_seed = 0x5D,
                .short_gi = false, .ldpc = false,
            };
            size_t max_s = lib80211_tx_ht_samples(&p) + 100;
            bench_encode_one(plan, name, encode_ht_wrapper, &p, max_s);
        }
    }

    /* HT encode LDPC */
    printf("\n  HT (LDPC):\n");
    {
        int mcs_list[] = {0, 4, 7};
        int n_mcs = (int)(sizeof(mcs_list) / sizeof(mcs_list[0]));
        for (int i = 0; i < n_mcs; i++) {
            char name[32];
            snprintf(name, sizeof(name), "ht_mcs%d_ldpc", mcs_list[i]);
            lib80211_tx_ht_params p = {
                .mcs = mcs_list[i], .psdu = psdu,
                .psdu_len = PSDU_LEN, .scrambler_seed = 0x5D,
                .short_gi = false, .ldpc = true,
            };
            size_t max_s = lib80211_tx_ht_samples(&p) + 100;
            bench_encode_one(plan, name, encode_ht_wrapper, &p, max_s);
        }
    }

    /* VHT encode BCC */
    printf("\n  VHT (BCC):\n");
    {
        int mcs_list[] = {0, 2, 4, 6, 8};
        int n_mcs = (int)(sizeof(mcs_list) / sizeof(mcs_list[0]));
        for (int i = 0; i < n_mcs; i++) {
            char name[32];
            snprintf(name, sizeof(name), "vht_mcs%d_bcc", mcs_list[i]);
            lib80211_tx_vht_params p = {
                .mcs = mcs_list[i], .psdu = psdu,
                .psdu_len = PSDU_LEN, .scrambler_seed = 0x5D,
                .short_gi = false, .ldpc = false,
            };
            size_t max_s = lib80211_tx_vht_samples(&p) + 100;
            bench_encode_one(plan, name, encode_vht_wrapper, &p, max_s);
        }
    }

    /* VHT encode LDPC */
    printf("\n  VHT (LDPC):\n");
    {
        int mcs_list[] = {0, 4, 8};
        int n_mcs = (int)(sizeof(mcs_list) / sizeof(mcs_list[0]));
        for (int i = 0; i < n_mcs; i++) {
            char name[32];
            snprintf(name, sizeof(name), "vht_mcs%d_ldpc", mcs_list[i]);
            lib80211_tx_vht_params p = {
                .mcs = mcs_list[i], .psdu = psdu,
                .psdu_len = PSDU_LEN, .scrambler_seed = 0x5D,
                .short_gi = false, .ldpc = true,
            };
            size_t max_s = lib80211_tx_vht_samples(&p) + 100;
            bench_encode_one(plan, name, encode_vht_wrapper, &p, max_s);
        }
    }

    printf("\n");
    lib80211_fft_plan_destroy(plan);
    return 0;
}
