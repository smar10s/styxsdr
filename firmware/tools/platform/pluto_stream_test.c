// SPDX-License-Identifier: MIT
/*
 * pluto_stream_test — Continuous TX streaming validation
 *
 * Generates a continuous waveform on the ARM and feeds it into the
 * streaming TX DMA.  Simultaneously captures the loopback on RX and
 * verifies:
 *   1. No DAC underruns (RX power envelope has no dropouts)
 *   2. Throughput: feed keeps up with drain
 *   3. Buffer pressure: available space remains above threshold
 *
 * Links against styx_hal only — no lib80211 or FFTW dependency.
 *
 * Usage:
 *   pluto_stream_test [-d seconds] [-f MHz] [-v]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "hal.h"
#include "dma_tx.h"
#include "dma_rx.h"
#include "convert.h"

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ    1000000ULL   /* 1 MSPS — matches LoRa vector rate */
#define CHIRP_F0_HZ        100000ULL   /* 100 kHz  */
#define CHIRP_F1_HZ        400000ULL   /* 400 kHz → stays within 500 kHz BW */
#define CHIRP_PERIOD      (SAMPLE_RATE_HZ / 10)  /* 100 ms per sweep */
#define DEFAULT_DURATION  5            /* seconds */
#define SILENCE_PAD       2000
#define CAPTURE_EXTRA     10000        /* extra RX capture margin */

/* --------------------------------------------------------------------------
 * Chirp generator
 * -------------------------------------------------------------------------- */

/*
 * Generate a linear chirp from f0 to f1 spanning chirp_samples.
 * Phase-continuous across calls: caller must preserve t_next between calls.
 */
static void gen_chirp_chunk(float *re, float *im, size_t n,
                            double f0, double f1, size_t chirp_len,
                            size_t *t_offset)
{
    double k = (f1 - f0) / ((double)chirp_len / (double)SAMPLE_RATE_HZ);
    for (size_t i = 0; i < n; i++) {
        size_t t = *t_offset + i;
        double tau = (double)t / (double)SAMPLE_RATE_HZ;
        /* Wrap phase to keep chirp repeating */
        double t_wrapped = tau;
        while (t_wrapped >= (double)chirp_len / (double)SAMPLE_RATE_HZ)
            t_wrapped -= (double)chirp_len / (double)SAMPLE_RATE_HZ;

        double phase = 2.0 * M_PI * (f0 * t_wrapped + 0.5 * k * t_wrapped * t_wrapped);
        re[i] = (float)cos(phase) * 0.5f;  /* half-scale for safe headroom */
        im[i] = (float)sin(phase) * 0.5f;
    }
    *t_offset += n;
}

/* --------------------------------------------------------------------------
 * Power envelope analysis
 * -------------------------------------------------------------------------- */

typedef struct {
    double mean;
    double variance;
    double min_val;
    double max_val;
    int    n_samples;
} envelope_stats_t;

static void compute_envelope_stats(const float *re, const float *im,
                                   size_t n, size_t block_size,
                                   envelope_stats_t *stats)
{
    size_t n_blocks = n / block_size;
    if (n_blocks < 2) return;

    double sum = 0, sum_sq = 0;
    double min_e = 1e30, max_e = -1e30;

    for (size_t b = 0; b < n_blocks; b++) {
        double energy = 0;
        for (size_t i = 0; i < block_size; i++) {
            size_t idx = b * block_size + i;
            float val = re[idx] * re[idx] + im[idx] * im[idx];
            energy += (double)val;
        }
        double block_power = energy / (double)block_size;

        sum += block_power;
        sum_sq += block_power * block_power;
        if (block_power < min_e) min_e = block_power;
        if (block_power > max_e) max_e = block_power;
    }

    stats->mean = sum / (double)n_blocks;
    stats->variance = (sum_sq / (double)n_blocks) - (stats->mean * stats->mean);
    stats->min_val = min_e;
    stats->max_val = max_e;
    stats->n_samples = (int)n;
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n\n"
        "  Generates a repeating chirp and streams it through the TX DMA.\n"
        "  Captures RX loopback and validates there are no DAC underruns.\n\n"
        "Options:\n"
        "  -d seconds   Test duration (default: %d)\n"
        "  -f MHz       LO frequency in MHz (default: 915)\n"
        "  -a dB        TX attenuation in dB (default: 30)\n"
        "  -v           Verbose\n"
        "  -h           Help\n",
        prog, DEFAULT_DURATION);
}

int main(int argc, char *argv[])
{
    int duration_sec = DEFAULT_DURATION;
    uint64_t freq_mhz = 915;
    double tx_atten = 30.0;
    bool verbose = false;
    int opt;

    while ((opt = getopt(argc, argv, "d:f:a:vh")) != -1) {
        switch (opt) {
        case 'd': duration_sec = atoi(optarg); break;
        case 'f': freq_mhz = (uint64_t)atoll(optarg); break;
        case 'a': tx_atten = atof(optarg); break;
        case 'v': verbose = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (duration_sec < 1) duration_sec = 1;
    if (duration_sec > 60) duration_sec = 60;

    uint64_t freq_hz = freq_mhz * 1000000ULL;

    fprintf(stderr, "pluto_stream_test: %d seconds, %llu MHz, %.1f dB atten\n",
            duration_sec, (unsigned long long)freq_mhz, tx_atten);

    /* Initialize HAL */
    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    /* Configure AD9361 */
    hal_ad9361_set_sample_rate(SAMPLE_RATE_HZ);
    hal_ad9361_set_tx_lo(freq_hz);
    hal_ad9361_set_rx_lo(freq_hz);
    hal_ad9361_set_tx_bandwidth(5000000ULL);   /* 5 MHz — room for chirp */
    hal_ad9361_set_rx_bandwidth(5000000ULL);
    hal_ad9361_set_rx_gain_mode("manual");
    hal_ad9361_set_rx_gain(22.0);
    hal_ad9361_set_tx_attenuation(tx_atten);

    usleep(100000);

    /* Start RX DMA */
    if (dma_rx_start() != 0) {
        fprintf(stderr, "ERROR: dma_rx_start failed\n");
        hal_cleanup();
        return 1;
    }

    /* Record RX start position */
    uint32_t rx_t0 = dma_rx_wr_ptr();

    /* Start streaming TX */
    if (dma_tx_stream_start() != 0) {
        fprintf(stderr, "ERROR: dma_tx_stream_start failed\n");
        dma_rx_stop();
        hal_cleanup();
        return 1;
    }

    /* Feed continuous chirp for the test duration */
    size_t total_fed = 0;
    size_t t_offset = 0;
    int min_available = DMA_TX_MAX_SAMPLES;
    int max_available = 0;
    int feed_cycles = 0;
    int stall_cycles = 0;
    uint64_t start_ms = 0;

    /* Heap-allocated chunk buffers (stack would overflow on Pluto's ARM) */
    float *chunk_re = malloc(DMA_TX_STREAM_CHUNK * sizeof(float));
    float *chunk_im = malloc(DMA_TX_STREAM_CHUNK * sizeof(float));
    if (!chunk_re || !chunk_im) {
        fprintf(stderr, "ERROR: failed to alloc chunk buffers\n");
        free(chunk_re); free(chunk_im);
        dma_tx_stream_stop();
        dma_rx_stop();
        hal_cleanup();
        return 1;
    }

    fprintf(stderr, "Streaming for %d seconds...\n", duration_sec);

    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        start_ms = (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
    }

    while (1) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_ms = (uint64_t)ts.tv_sec * 1000
                        + (uint64_t)ts.tv_nsec / 1000000;

        if (now_ms - start_ms >= (uint64_t)duration_sec * 1000ULL)
            break;

        int avail = dma_tx_stream_available();
        if (avail < min_available) min_available = avail;
        if (avail > max_available) max_available = avail;

        if (avail < DMA_TX_STREAM_CHUNK) {
            stall_cycles++;
            usleep(500);
            continue;
        }

        feed_cycles++;
        gen_chirp_chunk(chunk_re, chunk_im, DMA_TX_STREAM_CHUNK,
                        CHIRP_F0_HZ, CHIRP_F1_HZ, CHIRP_PERIOD, &t_offset);

        if (dma_tx_stream_feed(chunk_re, chunk_im,
                               DMA_TX_STREAM_CHUNK) != 0) {
            fprintf(stderr, "ERROR: stream feed failed at %zu samples\n",
                    total_fed);
            break;
        }
        total_fed += DMA_TX_STREAM_CHUNK;
    }

    fprintf(stderr, "  Fed %zu samples in %d cycles (%.1f MSPS effective)\n",
            total_fed, feed_cycles,
            (double)total_fed / (double)duration_sec / 1e6);
    fprintf(stderr, "  Buffer pressure: min_avail=%d, max_avail=%d, stalls=%d\n",
            min_available, max_available, stall_cycles);

    /* Let the last chunk play out before stopping */
    unsigned int drain_ms = (unsigned int)(DMA_TX_MAX_SAMPLES * 1000ULL
                            / SAMPLE_RATE_HZ) + 500;
    usleep(drain_ms * 1000);

    dma_tx_stream_stop();

    free(chunk_re); free(chunk_im);

    /* Capture RX data */
    uint32_t rx_cur = dma_rx_wr_ptr();
    dma_rx_stop();

    uint32_t rx_available = (rx_cur >= rx_t0) ? rx_cur - rx_t0
                          : (DMA_RX_BUF_SAMPLES - rx_t0) + rx_cur;

    if (rx_available < 1000) {
        fprintf(stderr, "WARNING: RX capture too small (%u samples), "
                "skipping validation\n", rx_available);
        hal_cleanup();
        return 1;
    }

    /* Read RX capture */
    size_t cap_samples = rx_available;
    float *rx_re = calloc(cap_samples, sizeof(float));
    float *rx_im = calloc(cap_samples, sizeof(float));
    if (!rx_re || !rx_im) {
        fprintf(stderr, "ERROR: alloc for %zu samples failed\n", cap_samples);
        free(rx_re); free(rx_im);
        hal_cleanup();
        return 1;
    }

    volatile uint32_t *rx_buf = hal_ddr_rx_buf();
    if (rx_t0 + (uint32_t)cap_samples <= DMA_RX_BUF_SAMPLES) {
        convert_rx_to_float(&rx_buf[rx_t0], cap_samples, rx_re, rx_im);
    } else {
        size_t first = DMA_RX_BUF_SAMPLES - rx_t0;
        size_t second = cap_samples - first;
        convert_rx_to_float(&rx_buf[rx_t0], first, rx_re, rx_im);
        convert_rx_to_float(&rx_buf[0], second,
                            &rx_re[first], &rx_im[first]);
    }

    /* Power envelope analysis: check for dropouts */
    size_t block = 1024;  /* ~1 ms blocks at 1 MSPS */
    envelope_stats_t env;
    compute_envelope_stats(rx_re, rx_im, cap_samples, block, &env);

    /* A dropout is when block power drops below 10% of the mean.
     * Normal operation shows small variance (the chirp sweeps through
     * frequency, but amplitude is constant).  A DAC underrun causes a
     * sharp drop to noise floor. */
    double dropout_threshold = env.mean * 0.1;
    int dropouts = 0;
    size_t n_blocks = cap_samples / block;
    for (size_t b = 0; b < n_blocks; b++) {
        double energy = 0;
        for (size_t i = 0; i < block; i++) {
            size_t idx = b * block + i;
            energy += (double)(rx_re[idx] * rx_re[idx]
                             + rx_im[idx] * rx_im[idx]);
        }
        if (energy / (double)block < dropout_threshold) {
            dropouts++;
            if (verbose)
                fprintf(stderr, "  dropout at block %zu (ms %.0f)\n",
                        b, (double)b * block * 1000.0 / SAMPLE_RATE_HZ);
        }
    }

    free(rx_re); free(rx_im);

    /* Report */
    fprintf(stderr, "\n=== STREAM TEST RESULTS ===\n");
    fprintf(stderr, "  Duration:         %d s\n", duration_sec);
    fprintf(stderr, "  Samples fed:      %zu\n", total_fed);
    fprintf(stderr, "  Feed cycles:      %d\n", feed_cycles);
    fprintf(stderr, "  Stall cycles:     %d\n", stall_cycles);
    fprintf(stderr, "  Min buffer avail: %d samples (%.1f ms)\n",
            min_available,
            (double)min_available * 1000.0 / SAMPLE_RATE_HZ);
    fprintf(stderr, "  RX captured:      %zu samples\n", cap_samples);
    fprintf(stderr, "  Dropouts:         %d / %zu blocks (%.1f%%)\n",
            dropouts, n_blocks,
            n_blocks > 0 ? 100.0 * dropouts / n_blocks : 0.0);
    fprintf(stderr, "  Envelope mean:    %.6f\n", env.mean);
    fprintf(stderr, "  Envelope var:     %.6f\n", env.variance);

    bool passed = (dropouts == 0) && (total_fed > 0)
               && (min_available > DMA_TX_STREAM_CHUNK * 2);

    fprintf(stderr, "\n  RESULT: %s\n", passed ? "PASS" : "FAIL");
    printf("{\"passed\":%s,\"duration_sec\":%d,\"fed\":%zu,"
           "\"cycles\":%d,\"stalls\":%d,\"min_avail\":%d,"
           "\"dropouts\":%d,\"n_blocks\":%zu}\n",
           passed ? "true" : "false", duration_sec, total_fed,
           feed_cycles, stall_cycles, min_available,
           dropouts, n_blocks);

    hal_cleanup();
    return passed ? 0 : 1;
}
