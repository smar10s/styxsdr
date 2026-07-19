// SPDX-License-Identifier: MIT
/*
 * pluto_loopback — TX/RX round-trip test at all 8 legacy rates
 *
 * Transmits a known data frame, captures via RX DMA, and verifies
 * decode + payload match. Exercises the full DDR DMA + AD9361 path.
 *
 * Uses the same two-phase TX + direct DDR read pattern as pluto_sigladder:
 *   1. Start RX DMA (ring buffer fills continuously)
 *   2. dma_tx_load() — write waveform to DDR, configure registers (slow)
 *   3. Record t0 = dma_rx_wr_ptr() — our time reference
 *   4. dma_tx_trigger() — single register write (fast, deterministic)
 *   5. Wait for propagation
 *   6. Read from ring buffer at t0 — frame is guaranteed to be there
 *   7. Remove DC offset, then decode
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include "hal.h"
#include "dma_tx.h"
#include "dma_rx.h"
#include "convert.h"

#include <lib80211/fft.h>
#include <lib80211/sync.h>
#include <lib80211/rx.h>
#include <lib80211/tx.h>
#include <lib80211/mac.h>

/* -------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ   20000000ULL
#define BANDWIDTH_HZ     28000000ULL
#define SILENCE_PAD      2000   /* samples of silence before/after frame */
#define DEFAULT_PAYLOAD  80     /* bytes */
#define MAX_PAYLOAD      1400   /* bytes */

static const int RATES[] = {6, 9, 12, 18, 24, 36, 48, 54};
#define N_RATES 8

static void usage(const char *progname) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -f MHz           LO frequency in MHz (default: 3456)\n"
        "  -n trials        Trials per rate (default: 1, max: 100)\n"
        "  -p payload_bytes Payload size in bytes (default: 80, max: 1400)\n"
        "  -a tx_atten_dB   TX attenuation in dB (default: 3)\n"
        "  -g rx_gain_dB    RX gain in dB (default: 22)\n"
        "  -v               Verbose: print per-trial decode details\n"
        "  -h               Show this help\n",
        progname);
}

/* Build the test PSDU: data frame, broadcast, known payload */
static size_t build_test_psdu(uint8_t *buf, int payload_len) {
    size_t psdu_len = 24 + payload_len + 4;
    /* Frame Control: Data (type=2, subtype=0), To DS=0, From DS=0 → 0x0008 LE = 0x08, 0x00 */
    buf[0] = 0x08; buf[1] = 0x00;
    /* Duration */
    buf[2] = 0x00; buf[3] = 0x00;
    /* DA = broadcast */
    memset(&buf[4], 0xFF, 6);
    /* SA = 02:00:00:DE:AD:01 */
    buf[10] = 0x02; buf[11] = 0x00; buf[12] = 0x00;
    buf[13] = 0xDE; buf[14] = 0xAD; buf[15] = 0x01;
    /* BSSID = same as SA */
    buf[16] = 0x02; buf[17] = 0x00; buf[18] = 0x00;
    buf[19] = 0xDE; buf[20] = 0xAD; buf[21] = 0x01;
    /* Sequence Control = 0 */
    buf[22] = 0x00; buf[23] = 0x00;
    /* Payload: bytes 0x00 through 0xFF repeating */
    for (int i = 0; i < payload_len; i++)
        buf[24 + i] = (uint8_t)(i & 0xFF);
    /* Append FCS */
    lib80211_append_fcs(buf, 24 + payload_len);

    return psdu_len;
}

/* Check if decoded payload matches expected */
static bool payload_matches(const uint8_t *psdu, size_t psdu_len,
                            int payload_len) {
    size_t expected_len = (size_t)(24 + payload_len + 4);
    if (psdu_len < expected_len) return false;
    /* Compare payload bytes at offset 24 */
    for (int i = 0; i < payload_len; i++) {
        if (psdu[24 + i] != (uint8_t)(i & 0xFF))
            return false;
    }
    return true;
}

/*
 * Two-phase TX + direct DDR capture (same pattern as pluto_sigladder).
 *
 * Returns number of captured samples, or -1 on error.
 * Caller must free *out_real and *out_imag.
 */
static int tx_and_capture(const float *tx_real, const float *tx_imag,
                          size_t tx_samples, size_t capture_samples,
                          float **out_real, float **out_imag)
{
    *out_real = malloc(capture_samples * sizeof(float));
    *out_imag = malloc(capture_samples * sizeof(float));
    if (!*out_real || !*out_imag) {
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

    /* Start RX DMA — ring buffer filling continuously */
    if (dma_rx_start() != 0) {
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

    /* Phase 1: Load waveform into DDR and configure DMA (slow) */
    if (dma_tx_load(tx_real, tx_imag, tx_samples, false) != 0) {
        dma_rx_stop();
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

    /* Record t0 — our time reference.  The frame will appear a few
     * samples after t0 (DAC pipeline latency). */
    uint32_t t0 = dma_rx_wr_ptr();

    /* Phase 2: Trigger TX — single register write, deterministic */
    if (dma_tx_trigger() != 0) {
        dma_rx_stop();
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

    /* Wait for TX to complete + propagation margin */
    unsigned int tx_us = (unsigned int)(tx_samples / 20);  /* 20 samples/us */
    usleep(tx_us + 2000);

    /* Verify enough data has been captured since t0 */
    uint32_t cur = dma_rx_wr_ptr();
    uint32_t available = (cur >= t0) ? cur - t0
                       : (DMA_RX_BUF_SAMPLES - t0) + cur;
    if (available < (uint32_t)capture_samples) {
        usleep(5000);
        cur = dma_rx_wr_ptr();
        available = (cur >= t0) ? cur - t0
                  : (DMA_RX_BUF_SAMPLES - t0) + cur;
    }

    /* Stop both DMAs */
    dma_tx_stop();
    dma_rx_stop();

    if (available < (uint32_t)capture_samples) {
        fprintf(stderr, "tx_and_capture: insufficient data (%u/%zu)\n",
                available, capture_samples);
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

    /* Read directly from DDR ring buffer starting at t0 */
    volatile uint32_t *rx_buf = hal_ddr_rx_buf();
    uint32_t read_ptr = t0;
    if (read_ptr + (uint32_t)capture_samples <= DMA_RX_BUF_SAMPLES) {
        convert_rx_to_float(&rx_buf[read_ptr], capture_samples,
                            *out_real, *out_imag);
    } else {
        size_t first = DMA_RX_BUF_SAMPLES - read_ptr;
        size_t second = capture_samples - first;
        convert_rx_to_float(&rx_buf[read_ptr], first,
                            *out_real, *out_imag);
        convert_rx_to_float(&rx_buf[0], second,
                            &(*out_real)[first], &(*out_imag)[first]);
    }

    /* Remove residual DC offset (same as sigladder) */
    {
        double sum_re = 0.0, sum_im = 0.0;
        for (size_t i = 0; i < capture_samples; i++) {
            sum_re += (*out_real)[i];
            sum_im += (*out_imag)[i];
        }
        float dc_re = (float)(sum_re / (double)capture_samples);
        float dc_im = (float)(sum_im / (double)capture_samples);
        for (size_t i = 0; i < capture_samples; i++) {
            (*out_real)[i] -= dc_re;
            (*out_imag)[i] -= dc_im;
        }
    }

    return (int)capture_samples;
}

typedef struct {
    int rate;
    int trials;
    int pass_count;
    int fcs_fail_count;
    int no_sync_count;
    int skip_count;
} rate_result;

/*
 * Run one loopback trial for a given rate.
 * Returns: 1 = pass (FCS OK + payload match), 0 = FCS fail, -1 = no sync, -2 = skip
 */
static int run_one_trial(lib80211_fft_plan *plan,
                         const uint8_t *psdu, size_t psdu_len,
                         int payload_len, int rate,
                         bool verbose)
{
    lib80211_tx_legacy_params tx_params = {
        .rate_mbps = rate,
        .psdu = psdu,
        .psdu_len = psdu_len,
        .scrambler_seed = 0x5D,
    };

    size_t frame_samples = lib80211_tx_legacy_samples(&tx_params);
    size_t total_tx = SILENCE_PAD + frame_samples + SILENCE_PAD;
    if (total_tx > DMA_TX_MAX_SAMPLES)
        return -2;

    float *tx_real = calloc(total_tx, sizeof(float));
    float *tx_imag = calloc(total_tx, sizeof(float));
    if (!tx_real || !tx_imag) {
        free(tx_real); free(tx_imag);
        return -2;
    }

    size_t gen = lib80211_tx_legacy(plan, &tx_params,
                                    tx_real + SILENCE_PAD,
                                    tx_imag + SILENCE_PAD);
    if (gen == 0) {
        free(tx_real); free(tx_imag);
        return -2;
    }

    /* TX and capture */
    float *rx_real = NULL, *rx_imag = NULL;
    size_t capture_samples = total_tx + 4000;
    int captured = tx_and_capture(tx_real, tx_imag, total_tx,
                                  capture_samples, &rx_real, &rx_imag);
    free(tx_real); free(tx_imag);

    if (captured <= 0)
        return -2;

    /* Decode: find first frame at expected rate.
     *
     * Start searching at SILENCE_PAD - 200 to skip stale ring buffer
     * data from previous trials.  The real frame arrives at approximately
     * SILENCE_PAD + pipeline_latency (~2100 samples from t0).  Anything
     * before ~1800 is guaranteed to be old data from prior iterations
     * (the ring buffer is never cleared). */
    size_t cap = (size_t)captured;
    size_t skip = SILENCE_PAD;  /* frame cannot arrive before silence pad completes */
    size_t offset = skip;
    int trial_result = -1;  /* default: no sync */

    while (offset < cap) {
        size_t remaining = cap - offset;
        if (remaining < 320) break;

        lib80211_sync_result sync;
        int sync_rc = lib80211_sync_detect(plan,
                                           rx_real + offset,
                                           rx_imag + offset,
                                           remaining, &sync);
        if (sync_rc < 0) break;

        size_t frame_offset = offset + sync.frame_start;
        size_t frame_remaining = cap - frame_offset;
        if (frame_remaining < 320) break;

        lib80211_rx_result result;
        int dec_rc = lib80211_rx_decode(plan,
                                       rx_real + frame_offset,
                                       rx_imag + frame_offset,
                                       frame_remaining, &result);

        if (dec_rc < 0) {
            offset = frame_offset + 160;
            continue;
        }

        if (verbose) {
            fprintf(stderr, " [sync@%zu rate=%d fcs=%s]",
                    frame_offset, result.rate_mbps,
                    result.fcs_valid ? "OK" : "FAIL");
        }

        if (result.rate_mbps == rate) {
            if (result.fcs_valid && payload_matches(result.psdu, result.psdu_len,
                                                    payload_len)) {
                trial_result = 1;  /* pass */
            } else {
                trial_result = 0;  /* FCS or payload fail */
            }
            break;
        }

        /* Not our rate — advance past it */
        size_t frame_len = 320 + (size_t)result.n_symbols * 80;
        if (frame_len < 320) frame_len = 320;
        offset = frame_offset + frame_len;
    }

    free(rx_real); free(rx_imag);
    return trial_result;
}

int main(int argc, char *argv[]) {
    uint64_t freq = 3456000000ULL;
    double tx_atten = 3.0;
    double rx_gain = 22.0;
    int n_trials = 1;
    int payload_len = DEFAULT_PAYLOAD;
    bool verbose = false;
    int opt;

    while ((opt = getopt(argc, argv, "f:a:g:n:p:vh")) != -1) {
        switch (opt) {
        case 'f': freq = (uint64_t)(atof(optarg) * 1e6); break;
        case 'a': tx_atten = atof(optarg); break;
        case 'g': rx_gain = atof(optarg); break;
        case 'n': n_trials = atoi(optarg); break;
        case 'p': payload_len = atoi(optarg); break;
        case 'v': verbose = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (n_trials < 1) n_trials = 1;
    if (n_trials > 100) n_trials = 100;
    if (payload_len < 1) payload_len = 1;
    if (payload_len > MAX_PAYLOAD) payload_len = MAX_PAYLOAD;

    size_t psdu_len = 24 + payload_len + 4;

    /* Build test PSDU */
    uint8_t psdu[24 + MAX_PAYLOAD + 4];
    build_test_psdu(psdu, payload_len);

    /* Initialize HAL */
    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    /* Configure radio (both TX and RX) */
    hal_ad9361_set_sample_rate(SAMPLE_RATE_HZ);
    hal_ad9361_set_tx_lo(freq);
    hal_ad9361_set_rx_lo(freq);
    hal_ad9361_set_tx_bandwidth(BANDWIDTH_HZ);
    hal_ad9361_set_rx_bandwidth(BANDWIDTH_HZ);
    hal_ad9361_set_rx_gain_mode("manual");
    hal_ad9361_set_rx_gain(rx_gain);
    hal_ad9361_set_tx_attenuation(tx_atten);

    /* Allow PLL lock and ADC settling */
    usleep(100000);

    fprintf(stderr, "Loopback test: freq=%llu MHz, atten=%.1f dB, gain=%.1f dB, trials=%d, payload=%d bytes\n",
            (unsigned long long)(freq / 1000000ULL), tx_atten, rx_gain, n_trials, payload_len);

    /* Create FFT plan */
    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "ERROR: fft_plan_create failed\n");
        hal_cleanup();
        return 1;
    }

    /* Warm-up: TX a short silence burst to flush DAC pipeline and settle
     * the DC offset filter.  Without this, the first rate test sometimes
     * sees residual energy from radio initialization. */
    {
        float warmup[4096] = {0};
        float *discard_re = NULL, *discard_im = NULL;
        int wrc = tx_and_capture(warmup, warmup, 4096, 8000,
                                 &discard_re, &discard_im);
        (void)wrc;
        free(discard_re); free(discard_im);
    }

    rate_result results[N_RATES] = {0};
    int rates_passed = 0;

    for (int r = 0; r < N_RATES; r++) {
        int rate = RATES[r];
        results[r].rate = rate;
        results[r].trials = n_trials;

        fprintf(stderr, "  Rate %2d Mbps: ", rate);

        for (int t = 0; t < n_trials; t++) {
            int rc = run_one_trial(plan, psdu, psdu_len, payload_len, rate, verbose);
            switch (rc) {
            case 1:  results[r].pass_count++; break;
            case 0:  results[r].fcs_fail_count++; break;
            case -1: results[r].no_sync_count++; break;
            case -2: results[r].skip_count++; break;
            }
        }

        int pct = (results[r].pass_count * 100) / n_trials;
        bool rate_pass = (results[r].pass_count == n_trials);

        if (rate_pass) {
            fprintf(stderr, "%d/%d (100%%) PASS\n", results[r].pass_count, n_trials);
            rates_passed++;
        } else {
            fprintf(stderr, "%d/%d (%d%%)", results[r].pass_count, n_trials, pct);
            if (results[r].fcs_fail_count > 0)
                fprintf(stderr, " fcs_fail=%d", results[r].fcs_fail_count);
            if (results[r].no_sync_count > 0)
                fprintf(stderr, " no_sync=%d", results[r].no_sync_count);
            if (results[r].skip_count > 0)
                fprintf(stderr, " skip=%d", results[r].skip_count);
            fprintf(stderr, " FAIL\n");
        }
    }

    lib80211_fft_plan_destroy(plan);
    hal_cleanup();

    /* Output JSON */
    printf("{\"rates\":[");
    for (int i = 0; i < N_RATES; i++) {
        if (i > 0) printf(",");
        printf("{\"rate\":%d,\"trials\":%d,\"pass\":%d,\"fcs_fail\":%d,"
               "\"no_sync\":%d,\"skip\":%d,\"pct\":%d}",
               results[i].rate, results[i].trials, results[i].pass_count,
               results[i].fcs_fail_count, results[i].no_sync_count,
               results[i].skip_count,
               (results[i].pass_count * 100) / n_trials);
    }
    printf("],\"summary\":{\"rates_passed\":%d,\"rates_total\":%d,\"trials\":%d}}\n",
           rates_passed, N_RATES, n_trials);

    fprintf(stderr, "\nResult: %d/%d rates passed (%d trials each)\n",
            rates_passed, N_RATES, n_trials);

    return (rates_passed == N_RATES) ? 0 : 1;
}
