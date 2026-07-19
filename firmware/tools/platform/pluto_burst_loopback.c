// SPDX-License-Identifier: MIT
/*
 * pluto_burst_loopback — ARM-decode burst loopback
 *
 * Multi-frame burst waveform (mixed rates,
 * single DMA buffer), decoded by the ARM lib80211 RX instead of the
 * FPGA fabric. This isolates whether FCS failures are caused by:
 *   - The analog/RF path (both ARM and fabric would fail)
 *   - The fabric pipeline (ARM passes, fabric fails)
 *
 * Method:
 *   1. Generate burst (multi-frame manifest + build_frame)
 *   2. TX via DMA, simultaneously capture RX IQ from DDR
 *   3. Decode captured IQ using lib80211_sync_detect + lib80211_rx_decode
 *   4. Match decoded frames against manifest
 *   5. Report pass/fail and compare with fabric results
 *
 * Usage:
 *   pluto_burst_loopback -n 10 -r 6 -p 80 -g 20000 -c 149
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <getopt.h>
#include <time.h>

#include "hal.h"
#include "dma_tx.h"
#include "dma_rx.h"
#include "convert.h"

#include <lib80211/fft.h>
#include <lib80211/tx.h>
#include <lib80211/rx.h>
#include <lib80211/sync.h>
#include <lib80211/mac.h>

/* -------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ       20000000ULL
#define BANDWIDTH_HZ         28000000ULL
#define DEFAULT_FREQ_HZ      3456000000ULL  /* 3456 MHz — clear of WiFi/BT */
#define DEFAULT_GAP_SAMPLES  20000   /* 1 ms */
#define SILENCE_PAD_PRE      2000
#define SILENCE_PAD_POST     2000
#define MAX_BURST_FRAMES     200
#define MAX_PSDU_LEN         1500

static const int SUPPORTED_RATES[] = {6, 12, 24};
#define N_SUPPORTED_RATES 3

/* --------------------------------------------------------------------------
 * Frame builders
 * -------------------------------------------------------------------------- */

static size_t build_frame(uint8_t *buf, int payload_bytes,
                          uint8_t seq_num, bool eapol)
{
    int offset = 0;

    /* MAC header (24 bytes) */
    buf[offset++] = 0x08;  /* FC: Data */
    buf[offset++] = 0x02;  /* FC: From DS */
    buf[offset++] = 0x00;  /* Duration */
    buf[offset++] = 0x00;
    memset(&buf[offset], 0xFF, 6); offset += 6;  /* DA = broadcast */
    buf[offset++] = 0x02; buf[offset++] = 0x00; buf[offset++] = 0x00;
    buf[offset++] = 0xDE; buf[offset++] = 0xAD; buf[offset++] = 0x01;  /* BSSID */
    buf[offset++] = 0x02; buf[offset++] = 0x00; buf[offset++] = 0x00;
    buf[offset++] = 0xDE; buf[offset++] = 0xAD; buf[offset++] = 0x02;  /* SA */
    buf[offset++] = (uint8_t)(seq_num & 0xFF);
    buf[offset++] = 0x00;  /* SeqCtrl */

    /* LLC/SNAP header (8 bytes) */
    buf[offset++] = 0xAA;
    buf[offset++] = 0xAA;
    buf[offset++] = 0x03;
    buf[offset++] = 0x00;
    buf[offset++] = 0x00;
    buf[offset++] = 0x00;
    if (eapol) {
        buf[offset++] = 0x88;
        buf[offset++] = 0x8E;
    } else {
        buf[offset++] = 0x08;
        buf[offset++] = 0x00;
    }

    /* Payload: magic + seq_num + pattern */
    buf[offset++] = 0xDE;
    buf[offset++] = 0x10;
    buf[offset++] = 0x05;
    buf[offset++] = seq_num;
    for (int i = 4; i < payload_bytes; i++)
        buf[offset++] = (uint8_t)((seq_num + i) & 0xFF);

    lib80211_append_fcs(buf, offset);
    return offset + 4;
}

typedef struct {
    int rate_mbps;
    size_t psdu_len;
    uint8_t psdu[MAX_PSDU_LEN + 4];
    bool is_eapol;
    uint8_t seq_num;
} frame_manifest_t;

typedef struct {
    int n_frames;
    frame_manifest_t frames[MAX_BURST_FRAMES];
} burst_manifest_t;

static void build_manifest(burst_manifest_t *m, int n_frames,
                           int fixed_rate, int payload_size,
                           bool inject_eapol, unsigned int seed)
{
    srand(seed);
    m->n_frames = n_frames;

    int min_payload = (payload_size > 0) ? payload_size : 40;
    int max_payload = (payload_size > 0) ? payload_size : 400;
    int eapol_interval = inject_eapol ? 5 : 0;

    for (int i = 0; i < n_frames; i++) {
        frame_manifest_t *f = &m->frames[i];
        f->seq_num = (uint8_t)(i & 0xFF);

        if (fixed_rate > 0)
            f->rate_mbps = fixed_rate;
        else
            f->rate_mbps = SUPPORTED_RATES[rand() % N_SUPPORTED_RATES];

        f->is_eapol = (eapol_interval > 0 && ((i % eapol_interval) == 2));

        int plen;
        if (f->is_eapol)
            plen = 95;
        else if (payload_size > 0)
            plen = payload_size;
        else
            plen = min_payload + (rand() % (max_payload - min_payload + 1));

        f->psdu_len = build_frame(f->psdu, plen, f->seq_num, f->is_eapol);
    }
}

/* --------------------------------------------------------------------------
 * TX + RX capture (from pluto_loopback pattern)
 * -------------------------------------------------------------------------- */

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

    if (dma_rx_start() != 0) {
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

    if (dma_tx_load(tx_real, tx_imag, tx_samples, false) != 0) {
        dma_rx_stop();
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

    uint32_t t0 = dma_rx_wr_ptr();

    if (dma_tx_trigger() != 0) {
        dma_rx_stop();
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

    unsigned int tx_us = (unsigned int)(tx_samples / 20);
    usleep(tx_us + 5000);  /* TX duration + 5ms margin for longer bursts */

    uint32_t cur = dma_rx_wr_ptr();
    uint32_t available = (cur >= t0) ? cur - t0
                       : (DMA_RX_BUF_SAMPLES - t0) + cur;
    if (available < (uint32_t)capture_samples) {
        usleep(10000);
        cur = dma_rx_wr_ptr();
        available = (cur >= t0) ? cur - t0
                   : (DMA_RX_BUF_SAMPLES - t0) + cur;
    }

    dma_tx_stop();
    dma_rx_stop();

    if (available < (uint32_t)capture_samples) {
        fprintf(stderr, "tx_and_capture: insufficient data (%u/%zu)\n",
                available, capture_samples);
        free(*out_real); free(*out_imag);
        *out_real = NULL; *out_imag = NULL;
        return -1;
    }

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

    /* DC offset removal */
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

    return (int)capture_samples;
}

/* --------------------------------------------------------------------------
 * ARM decode: find all frames in capture buffer
 * -------------------------------------------------------------------------- */

typedef struct {
    int rate_mbps;
    size_t psdu_len;
    bool fcs_valid;
    uint8_t seq_num;   /* extracted from psdu[35] if available */
    size_t sample_offset;  /* where in capture buffer */
} arm_decode_result_t;

static int decode_all_frames(lib80211_fft_plan *plan,
                             const float *rx_real, const float *rx_imag,
                             size_t n_samples,
                             arm_decode_result_t *results, int max_results)
{
    int count = 0;
    size_t offset = SILENCE_PAD_PRE / 2;  /* start searching slightly before expected */

    while (offset < n_samples && count < max_results) {
        size_t remaining = n_samples - offset;
        if (remaining < 320) break;

        lib80211_sync_result sync;
        int sync_rc = lib80211_sync_detect(plan,
                                           rx_real + offset,
                                           rx_imag + offset,
                                           remaining, &sync);
        if (sync_rc < 0) break;

        size_t frame_offset = offset + sync.frame_start;
        size_t frame_remaining = n_samples - frame_offset;
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

        arm_decode_result_t *r = &results[count];
        r->rate_mbps = result.rate_mbps;
        r->psdu_len = result.psdu_len;
        r->fcs_valid = result.fcs_valid;
        r->sample_offset = frame_offset;
        r->seq_num = 0xFF;

        /* Extract seq_num from payload if we have enough bytes */
        if (result.psdu_len >= 36)
            r->seq_num = result.psdu[35];

        count++;

        /* Advance past this frame */
        size_t frame_len = 320 + (size_t)result.n_symbols * 80;
        if (frame_len < 320) frame_len = 320;
        offset = frame_offset + frame_len;
    }

    return count;
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n\n"
        "  Multi-frame burst loopback.\n"
        "  Decoded by lib80211 on ARM.\n\n"
        "Options:\n"
        "  -n count       Number of frames (default: 10, max: %d)\n"
        "  -r rate        Fixed rate (6/12/24); default: mixed\n"
        "  -p payload     Fixed payload size (default: 80)\n"
        "  -g gap         Inter-frame gap in samples (default: %d)\n"
        "  -f MHz         LO frequency in MHz (default: 3456)\n"
        "  -s seed        Random seed (default: from time)\n"
        "  -v             Verbose\n"
        "  -h             Help\n",
        prog, MAX_BURST_FRAMES, DEFAULT_GAP_SAMPLES);
}

int main(int argc, char *argv[])
{
    int n_frames = 10;
    int fixed_rate = 6;
    int payload_size = 80;
    int gap_samples = DEFAULT_GAP_SAMPLES;
    uint64_t freq = DEFAULT_FREQ_HZ;
    unsigned int seed = 0;
    bool seed_set = false;
    bool verbose = false;
    int opt;

    while ((opt = getopt(argc, argv, "n:r:p:g:f:s:vh")) != -1) {
        switch (opt) {
        case 'n': n_frames = atoi(optarg); break;
        case 'r': fixed_rate = atoi(optarg); break;
        case 'p': payload_size = atoi(optarg); break;
        case 'g': gap_samples = atoi(optarg); break;
        case 'f': freq = (uint64_t)(atof(optarg) * 1e6); break;
        case 's': seed = (unsigned int)atoi(optarg); seed_set = true; break;
        case 'v': verbose = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (!seed_set) seed = (unsigned int)time(NULL);
    if (n_frames < 1 || n_frames > MAX_BURST_FRAMES) {
        fprintf(stderr, "ERROR: frame count 1-%d\n", MAX_BURST_FRAMES);
        return 1;
    }

    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    uint64_t freq_local = freq;
    hal_ad9361_set_sample_rate(SAMPLE_RATE_HZ);
    hal_ad9361_set_rx_lo(freq_local);
    hal_ad9361_set_tx_lo(freq_local);
    hal_ad9361_set_rx_bandwidth(BANDWIDTH_HZ);
    hal_ad9361_set_tx_bandwidth(BANDWIDTH_HZ);
    hal_ad9361_set_rx_gain_mode("manual");
    hal_ad9361_set_rx_gain(22.0);
    hal_ad9361_set_tx_attenuation(3000);  /* -3 dB (match pluto_loopback) */
    usleep(100000);

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "ERROR: FFT plan failed\n");
        hal_cleanup();
        return 1;
    }

    /* Build manifest */
    burst_manifest_t manifest;
    build_manifest(&manifest, n_frames, fixed_rate, payload_size, false, seed);

    fprintf(stderr, "pluto_burst_loopback (ARM decode): %d frames, freq=%llu MHz, seed=%u\n",
            n_frames, (unsigned long long)(freq / 1000000ULL), seed);
    fprintf(stderr, "  rate=%d, payload=%d, gap=%d samples (%.0f us)\n",
            fixed_rate, payload_size, gap_samples,
            gap_samples * 1e6 / (double)SAMPLE_RATE_HZ);

    /* Generate burst IQ */
    size_t total_samples = SILENCE_PAD_PRE;
    for (int i = 0; i < n_frames; i++) {
        const frame_manifest_t *f = &manifest.frames[i];
        lib80211_tx_legacy_params params = {
            .rate_mbps = f->rate_mbps,
            .psdu = f->psdu,
            .psdu_len = f->psdu_len,
            .scrambler_seed = (uint8_t)(0x5D + i),
        };
        total_samples += lib80211_tx_legacy_samples(&params);
        if (i < n_frames - 1)
            total_samples += gap_samples;
    }
    total_samples += SILENCE_PAD_POST;

    if (total_samples > DMA_TX_MAX_SAMPLES) {
        fprintf(stderr, "ERROR: burst too large (%zu > %d)\n",
                total_samples, DMA_TX_MAX_SAMPLES);
        lib80211_fft_plan_destroy(plan);
        hal_cleanup();
        return 1;
    }

    float *tx_real = calloc(total_samples, sizeof(float));
    float *tx_imag = calloc(total_samples, sizeof(float));
    if (!tx_real || !tx_imag) {
        fprintf(stderr, "ERROR: malloc\n");
        lib80211_fft_plan_destroy(plan);
        hal_cleanup();
        return 1;
    }

    size_t tx_offset = SILENCE_PAD_PRE;
    for (int i = 0; i < n_frames; i++) {
        const frame_manifest_t *f = &manifest.frames[i];
        lib80211_tx_legacy_params params = {
            .rate_mbps = f->rate_mbps,
            .psdu = f->psdu,
            .psdu_len = f->psdu_len,
            .scrambler_seed = (uint8_t)(0x5D + i),
        };
        size_t gen = lib80211_tx_legacy(plan, &params,
                                        tx_real + tx_offset, tx_imag + tx_offset);
        if (gen == 0) {
            fprintf(stderr, "ERROR: tx_legacy frame %d\n", i);
            free(tx_real); free(tx_imag);
            lib80211_fft_plan_destroy(plan);
            hal_cleanup();
            return 1;
        }
        tx_offset += gen;
        if (i < n_frames - 1)
            tx_offset += gap_samples;
    }

    fprintf(stderr, "  TX buffer: %zu samples (%.1f ms)\n",
            total_samples, total_samples * 1000.0 / SAMPLE_RATE_HZ);

    /* TX and capture */
    float *rx_real = NULL, *rx_imag = NULL;
    size_t capture_samples = total_samples + 10000;  /* extra margin */
    int captured = tx_and_capture(tx_real, tx_imag, total_samples,
                                  capture_samples, &rx_real, &rx_imag);
    free(tx_real); free(tx_imag);

    if (captured <= 0) {
        fprintf(stderr, "ERROR: tx_and_capture failed\n");
        lib80211_fft_plan_destroy(plan);
        hal_cleanup();
        return 1;
    }

    fprintf(stderr, "  Captured %d samples\n", captured);

    /* Decode all frames from capture buffer */
    arm_decode_result_t results[MAX_BURST_FRAMES * 2];
    int n_decoded = decode_all_frames(plan, rx_real, rx_imag,
                                      (size_t)captured,
                                      results, MAX_BURST_FRAMES * 2);
    free(rx_real); free(rx_imag);

    /* Match and report */
    int fcs_pass = 0, fcs_fail = 0, content_match = 0;

    fprintf(stderr, "\n");
    fprintf(stderr, "============================================================\n");
    fprintf(stderr, "ARM BURST LOOPBACK RESULTS\n");
    fprintf(stderr, "============================================================\n");

    for (int i = 0; i < n_decoded; i++) {
        arm_decode_result_t *r = &results[i];
        if (r->fcs_valid) fcs_pass++;
        else fcs_fail++;

        /* Match by seq_num */
        int matched = -1;
        if (r->fcs_valid && r->seq_num != 0xFF) {
            for (int j = 0; j < n_frames; j++) {
                if (manifest.frames[j].seq_num == r->seq_num &&
                    manifest.frames[j].rate_mbps == r->rate_mbps) {
                    matched = j;
                    content_match++;
                    break;
                }
            }
        }

        if (verbose) {
            fprintf(stderr, "  [%2d] @%6zu rate=%2d fcs=%s len=%zu seq=%02x match=%d\n",
                    i, r->sample_offset, r->rate_mbps,
                    r->fcs_valid ? "OK" : "!!", r->psdu_len,
                    r->seq_num, matched);
        }
    }

    fprintf(stderr, "  TX frames:       %d\n", n_frames);
    fprintf(stderr, "  Decoded (ARM):   %d\n", n_decoded);
    fprintf(stderr, "  FCS pass:        %d\n", fcs_pass);
    fprintf(stderr, "  FCS fail:        %d\n", fcs_fail);
    fprintf(stderr, "  Content match:   %d / %d\n", content_match, fcs_pass);
    fprintf(stderr, "  RECEIVE RATE:    %d%% (%d/%d)\n",
            n_frames > 0 ? (content_match * 100 / n_frames) : 0,
            content_match, n_frames);
    fprintf(stderr, "============================================================\n");

    /* JSON to stdout */
    printf("{\"decoder\":\"arm\",\"tx_frames\":%d,\"decoded\":%d,"
           "\"fcs_pass\":%d,\"fcs_fail\":%d,\"content_match\":%d,"
           "\"gap_samples\":%d,\"seed\":%u}\n",
           n_frames, n_decoded, fcs_pass, fcs_fail, content_match,
           gap_samples, seed);

    lib80211_fft_plan_destroy(plan);
    hal_cleanup();
    return (content_match == n_frames) ? 0 : 1;
}
