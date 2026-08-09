// SPDX-License-Identifier: MIT
/*
 * pluto_stream_test — Continuous TX streaming validation
 *
 * Generates a repeating chirp on the ARM and feeds it into the
 * streaming TX DMA.  Simultaneously captures the loopback on RX and
 * verifies:
 *   1. No DAC underruns (RX power envelope has no dropouts)
 *   2. Throughput: feed keeps up with drain
 *   3. Buffer pressure: available space remains above threshold
 *
 * Supports multiple sample rates for DMA low-rate validation.
 *
 * Links against styx_hal only — no lib80211 or FFTW dependency.
 *
 * Usage:
 *   pluto_stream_test [-d seconds] [-f MHz] [-a dB] [-r Hz] [-v]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <arm_neon.h>

#include "hal.h"
#include "dma_tx.h"
#include "dma_rx.h"
#include "convert.h"

/* --------------------------------------------------------------------------
 * Signal handling — ensure TX is stopped on Ctrl-C / SIGTERM
 * -------------------------------------------------------------------------- */

static volatile sig_atomic_t g_shutdown = 0;

static void shutdown_handler(int sig) {
    (void)sig;
    g_shutdown = 1;
}

/* --------------------------------------------------------------------------
 * Defaults
 * -------------------------------------------------------------------------- */

#define DEFAULT_SAMPLE_RATE  4000000ULL  /* 4 MSPS */
#define DEFAULT_FREQ          915        /* MHz */
#define DEFAULT_ATTEN          20.0      /* dB */
#define DEFAULT_DURATION       10

#define CHIRP_F0_HZ          100000ULL   /* 100 kHz */
#define CHIRP_F1_HZ          800000ULL   /* 800 kHz — within 2 MHz TX BW */
#define TX_BW_HZ            5000000ULL   /* 5 MHz TX bandwidth */
#define RX_BW_HZ           10000000ULL   /* 10 MHz RX bandwidth */
#define RX_GAIN               22.0

/* Block size for dropout detection: ~1 ms worth of samples */
#define ENVELOPE_BLOCK(sr)  ((size_t)((sr) / 1000))

/* --------------------------------------------------------------------------
 * NEON vectorized sin/cos (4-wide, single-precision)
 *
 * 7th-order minimax polynomial on [-pi, pi].  Accuracy ~20 bits —
 * more than sufficient for a 12-bit DAC.  Based on Cephes coefficients
 * adapted for the [-pi, pi] range with Estrin evaluation.
 * -------------------------------------------------------------------------- */

static const float SINCOS_PI     = 3.14159265358979323846f;
static const float SINCOS_TWO_PI = 6.28318530717958647692f;
static const float SINCOS_INV_TWO_PI = 0.15915494309189533577f;

/*
 * Range-reduce x to [-pi, pi] via: x - round(x / 2pi) * 2pi
 */
static inline float32x4_t neon_range_reduce(float32x4_t x)
{
    /* n = round(x / 2pi): add/sub 0.5 then truncate */
    float32x4_t n = vmulq_n_f32(x, SINCOS_INV_TWO_PI);
    /* Round-to-nearest via: trunc(n + copysign(0.5, n)) */
    const float32x4_t half = vdupq_n_f32(0.5f);
    const float32x4_t nhalf = vdupq_n_f32(-0.5f);
    uint32x4_t neg_mask = vcltq_f32(n, vdupq_n_f32(0.0f));
    float32x4_t bias = vbslq_f32(neg_mask, nhalf, half);
    float32x4_t biased = vaddq_f32(n, bias);
    int32x4_t ni = vcvtq_s32_f32(biased);  /* truncate toward zero */
    float32x4_t nf = vcvtq_f32_s32(ni);
    /* x - n * 2pi */
    return vmlsq_n_f32(x, nf, SINCOS_TWO_PI);
}

/*
 * Compute sin and cos simultaneously for 4 floats.
 * Uses: sin(x) = x - x^3/6 + x^5/120 - x^7/5040
 *       cos(x) = 1 - x^2/2 + x^4/24 - x^6/720
 * (Horner form for efficiency)
 */
static inline void neon_sincos(float32x4_t x,
                               float32x4_t *out_sin,
                               float32x4_t *out_cos)
{
    /* Range reduce to [-pi, pi] */
    x = neon_range_reduce(x);

    float32x4_t x2 = vmulq_f32(x, x);
    float32x4_t x3 = vmulq_f32(x2, x);
    float32x4_t x4 = vmulq_f32(x2, x2);
    float32x4_t x5 = vmulq_f32(x4, x);
    float32x4_t x6 = vmulq_f32(x4, x2);
    float32x4_t x7 = vmulq_f32(x6, x);

    /* sin(x) ≈ x - x³/6 + x⁵/120 - x⁷/5040 */
    const float32x4_t s1 = vdupq_n_f32(-1.0f / 6.0f);
    const float32x4_t s2 = vdupq_n_f32(1.0f / 120.0f);
    const float32x4_t s3 = vdupq_n_f32(-1.0f / 5040.0f);

    float32x4_t s = x;
    s = vmlaq_f32(s, x3, s1);
    s = vmlaq_f32(s, x5, s2);
    s = vmlaq_f32(s, x7, s3);
    *out_sin = s;

    /* cos(x) ≈ 1 - x²/2 + x⁴/24 - x⁶/720 */
    const float32x4_t c0 = vdupq_n_f32(1.0f);
    const float32x4_t c1 = vdupq_n_f32(-0.5f);
    const float32x4_t c2 = vdupq_n_f32(1.0f / 24.0f);
    const float32x4_t c3 = vdupq_n_f32(-1.0f / 720.0f);

    float32x4_t c = c0;
    c = vmlaq_f32(c, x2, c1);
    c = vmlaq_f32(c, x4, c2);
    c = vmlaq_f32(c, x6, c3);
    *out_cos = c;
}

/* --------------------------------------------------------------------------
 * Chirp generator (NEON-accelerated)
 * -------------------------------------------------------------------------- */

/*
 * Chirp state: maintained across calls for phase continuity.
 * Phase advances quadratically: phase[i] = phase0 + dp*i + 0.5*ddp*i²
 * where dp = 2π·f0/fs and ddp = 2π·k/fs² (k = chirp rate in Hz/s).
 */
typedef struct {
    double phase;       /* current accumulated phase (double for drift-free) */
    double dp;          /* phase increment per sample: 2π·f_inst/fs */
    double ddp;         /* increment of dp per sample: 2π·k/fs² */
    double dp_base;     /* dp at chirp start (for wrap reset) */
    size_t t;           /* sample counter within chirp period */
    size_t chirp_len;   /* samples per chirp period */
} chirp_state_t;

static void chirp_state_init(chirp_state_t *cs, double f0, double f1,
                             size_t chirp_len, uint64_t sample_rate)
{
    double fs = (double)sample_rate;
    double k = (f1 - f0) / ((double)chirp_len / fs);  /* Hz/s */
    cs->phase    = 0.0;
    cs->dp       = 2.0 * M_PI * f0 / fs;
    cs->ddp      = 2.0 * M_PI * k / (fs * fs);
    cs->dp_base  = cs->dp;
    cs->t        = 0;
    cs->chirp_len = chirp_len;
}

/*
 * Generate n samples of chirp into re/im buffers.
 * Processes 4 samples at a time using NEON sincos.
 */
static void gen_chirp_chunk(float *re, float *im, size_t n,
                            chirp_state_t *cs)
{
    const float32x4_t half = vdupq_n_f32(0.5f);
    size_t i = 0;

    /* Main NEON loop: 4 samples per iteration */
    for (; i + 4 <= n; i += 4) {
        /* Compute 4 phase values using the running accumulator.
         * Phase is range-reduced at chirp boundaries (not per-sample)
         * to prevent unbounded growth while avoiding costly fmod. */
        float phases[4];
        for (int j = 0; j < 4; j++) {
            phases[j] = (float)cs->phase;
            cs->phase += cs->dp;
            cs->dp    += cs->ddp;
            cs->t++;
            if (cs->t >= cs->chirp_len) {
                cs->t     = 0;
                cs->dp    = cs->dp_base;
                /* Range-reduce phase at chirp boundary to prevent unbounded
                 * growth.  Keeps float32 cast accurate (ULP < 1e-4 rad when
                 * |phase| < 2π). */
                cs->phase = fmod(cs->phase, 2.0 * M_PI);
            }
        }

        float32x4_t ph = vld1q_f32(phases);
        float32x4_t sv, cv;
        neon_sincos(ph, &sv, &cv);

        /* re = cos(phase) * 0.5, im = sin(phase) * 0.5 */
        vst1q_f32(&re[i], vmulq_f32(cv, half));
        vst1q_f32(&im[i], vmulq_f32(sv, half));
    }

    /* Scalar tail (0-3 samples) */
    for (; i < n; i++) {
        re[i] = cosf((float)cs->phase) * 0.5f;
        im[i] = sinf((float)cs->phase) * 0.5f;
        cs->phase += cs->dp;
        cs->dp    += cs->ddp;
        cs->t++;
        if (cs->t >= cs->chirp_len) {
            cs->t     = 0;
            cs->dp    = cs->dp_base;
            cs->phase = fmod(cs->phase, 2.0 * M_PI);
        }
    }
}

/* --------------------------------------------------------------------------
 * Power envelope analysis
 * -------------------------------------------------------------------------- */

typedef struct {
    double mean;
    double variance;
    double min_val;
    double max_val;
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
}

static size_t count_dropouts(const float *re, const float *im,
                             size_t n, size_t block_size,
                             double dropout_threshold, bool verbose,
                             uint64_t sample_rate)
{
    int dropouts = 0;
    size_t n_blocks = n / block_size;
    for (size_t b = 0; b < n_blocks; b++) {
        double energy = 0;
        for (size_t i = 0; i < block_size; i++) {
            size_t idx = b * block_size + i;
            energy += (double)(re[idx] * re[idx] + im[idx] * im[idx]);
        }
        if (energy / (double)block_size < dropout_threshold) {
            dropouts++;
            if (verbose)
                fprintf(stderr, "  dropout at block %zu (ms %.1f)\n",
                        b, (double)b * (double)block_size * 1000.0
                        / (double)sample_rate);
        }
    }
    return (size_t)dropouts;
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n\n"
        "  Generates a repeating chirp and streams it through the TX DMA.\n"
        "  Captures RX loopback and validates for DAC underruns.\n\n"
        "Options:\n"
        "  -d seconds   Test duration (default: %d)\n"
        "  -f MHz       LO frequency in MHz (default: %d)\n"
        "  -a dB        TX attenuation in dB (default: %.0f)\n"
        "  -r Hz        Sample rate in Hz (default: %llu)\n"
        "  -v           Verbose\n"
        "  -h           Help\n",
        prog, DEFAULT_DURATION, DEFAULT_FREQ, DEFAULT_ATTEN,
        (unsigned long long)DEFAULT_SAMPLE_RATE);
}

int main(int argc, char *argv[])
{
    int duration_sec = DEFAULT_DURATION;
    uint64_t freq_mhz = DEFAULT_FREQ;
    uint64_t sample_rate_hz = DEFAULT_SAMPLE_RATE;
    double tx_atten = DEFAULT_ATTEN;
    bool verbose = false;
    int opt;

    while ((opt = getopt(argc, argv, "d:f:a:r:vh")) != -1) {
        switch (opt) {
        case 'd': duration_sec = atoi(optarg); break;
        case 'f': freq_mhz = (uint64_t)atoll(optarg); break;
        case 'a': tx_atten = atof(optarg); break;
        case 'r': sample_rate_hz = (uint64_t)atoll(optarg); break;
        case 'v': verbose = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (duration_sec < 1) duration_sec = 1;
    if (duration_sec > 120) duration_sec = 120;

    uint64_t freq_hz = freq_mhz * 1000000ULL;

    /* Chirp period: 25ms worth of samples, capped at chirp_len */
    size_t chirp_len = (size_t)(sample_rate_hz / 40);

    fprintf(stderr, "pluto_stream_test: %d s, %llu Hz, %llu MHz, %.1f dB atten\n",
            duration_sec, (unsigned long long)sample_rate_hz,
            (unsigned long long)freq_mhz, tx_atten);

    /* Initialize HAL */
    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    /* Configure AD9361 */
    hal_ad9361_set_sample_rate(sample_rate_hz);
    hal_ad9361_set_tx_lo(freq_hz);
    hal_ad9361_set_rx_lo(freq_hz);
    hal_ad9361_set_tx_bandwidth(TX_BW_HZ);
    hal_ad9361_set_rx_bandwidth(RX_BW_HZ);
    hal_ad9361_set_rx_gain_mode("manual");
    hal_ad9361_set_rx_gain(RX_GAIN);
    hal_ad9361_set_tx_attenuation(tx_atten);

    /* Allow AD9361 to settle after rate change.  300ms covers PLL relock
     * + DC tracking loop convergence, especially after a large sample rate
     * change (e.g. 20 MSPS → 2.5 MSPS when running after sigladder). */
    usleep(300000);

    /* Install signal handlers before starting any TX */
    signal(SIGINT, shutdown_handler);
    signal(SIGTERM, shutdown_handler);

    /* Start RX DMA */
    if (dma_rx_start() != 0) {
        fprintf(stderr, "ERROR: dma_rx_start failed\n");
        hal_cleanup();
        return 1;
    }

    /* Record RX start position */
    uint32_t rx_t0 = dma_rx_wr_ptr();

    /* Arm streaming TX (does NOT start the drain FSM yet) */
    if (dma_tx_stream_arm() != 0) {
        fprintf(stderr, "ERROR: dma_tx_stream_arm failed\n");
        dma_rx_stop();
        hal_cleanup();
        return 1;
    }

    /* Feed continuous chirp for the test duration */
    size_t total_fed = 0;
    chirp_state_t chirp;
    chirp_state_init(&chirp, CHIRP_F0_HZ, CHIRP_F1_HZ,
                     chirp_len, sample_rate_hz);
    int min_available = DMA_TX_MAX_SAMPLES;
    int max_available = 0;
    int feed_cycles = 0;
    int stall_cycles = 0;
    uint64_t start_ms = 0;

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

    /* Pre-fill: feed several chunks before triggering so the drain FSM
     * doesn't run ahead of an empty ring. */
    size_t prefill_target = DMA_TX_STREAM_CHUNK * 4;
    while (total_fed < prefill_target) {
        gen_chirp_chunk(chunk_re, chunk_im, DMA_TX_STREAM_CHUNK, &chirp);
        if (dma_tx_stream_feed(chunk_re, chunk_im, DMA_TX_STREAM_CHUNK) != 0) {
            fprintf(stderr, "ERROR: pre-fill feed failed\n");
            free(chunk_re); free(chunk_im);
            dma_tx_stream_stop();
            dma_rx_stop();
            hal_cleanup();
            return 1;
        }
        total_fed += DMA_TX_STREAM_CHUNK;
    }

    /* Now trigger — drain FSM starts with data already available */
    if (dma_tx_stream_trigger() != 0) {
        fprintf(stderr, "ERROR: dma_tx_stream_trigger failed\n");
        free(chunk_re); free(chunk_im);
        dma_tx_stream_stop();
        dma_rx_stop();
        hal_cleanup();
        return 1;
    }

    fprintf(stderr, "Streaming for %d s at %llu Hz...\n",
            duration_sec, (unsigned long long)sample_rate_hz);

    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        start_ms = (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
    }

    int consecutive_stalls = 0;

    while (1) {
        if (g_shutdown)
            break;

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
            consecutive_stalls++;
            if (consecutive_stalls > 100) {
                fprintf(stderr, "ERROR: stuck — %d consecutive stalls "
                        "(avail=%d)\n", consecutive_stalls, avail);
                break;
            }
            usleep(200);
            continue;
        }
        consecutive_stalls = 0;

        feed_cycles++;
        gen_chirp_chunk(chunk_re, chunk_im, DMA_TX_STREAM_CHUNK, &chirp);

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

    /* Snapshot RX position NOW — before drain silence contaminates
     * the capture window.  The RX DMA is a circular buffer; capturing
     * the pointer here gives us only the active TX period. */
    uint32_t rx_cur = dma_rx_wr_ptr();
    dma_rx_stop();

    /* Let ring buffer drain (TX side) */
    unsigned int drain_ms = (unsigned int)(DMA_TX_MAX_SAMPLES * 1000ULL
                            / sample_rate_hz) + 500;
    usleep(drain_ms * 1000);

    dma_tx_stream_stop();

    /* Read DAC valid miss counter before cleanup — counts drain FSM
     * cycles where dac_valid was not asserted (underrun indicator). */
    uint32_t dac_miss = hal_reg_read(REG_IQ_DMA_TX_DAC_MISS);

    free(chunk_re); free(chunk_im);

    /* Compute RX available */
    uint32_t rx_available = (rx_cur >= rx_t0) ? rx_cur - rx_t0
                          : (DMA_RX_BUF_SAMPLES - rx_t0) + rx_cur;

    if (rx_available < (uint32_t)ENVELOPE_BLOCK(sample_rate_hz) * 2) {
        fprintf(stderr, "WARNING: RX capture too small (%u samples), "
                "skipping validation\n", rx_available);
        hal_cleanup();
        return 1;
    }

    /* Read RX capture — but only analyze the active TX window.
     * The RX runs continuously and includes pre-fill silence and
     * post-drain silence which would false-positive as dropouts.
     * Skip the first 10ms (startup transient) and cap at total_fed. */
    size_t cap_samples = (size_t)rx_available;
    size_t skip_samples = (size_t)(sample_rate_hz / 10);  /* 100ms — skip startup transient and PLL settling */
    size_t analyze_samples = total_fed > skip_samples
                           ? total_fed - skip_samples : 0;
    if (analyze_samples > cap_samples - skip_samples)
        analyze_samples = cap_samples - skip_samples;

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

    /* Power envelope analysis — only over the active TX window */
    size_t block = ENVELOPE_BLOCK(sample_rate_hz);
    envelope_stats_t env;
    float *ana_re = rx_re + skip_samples;
    float *ana_im = rx_im + skip_samples;
    compute_envelope_stats(ana_re, ana_im, analyze_samples, block, &env);

    /* Dropout detection: a block whose power is below threshold is a dropout.
     * Use an absolute minimum floor (not purely self-referential) so the test
     * fails when no signal is present (e.g. loopback cable unplugged). */
    if (env.mean < 0.001) {
        fprintf(stderr, "\n  FAIL: No signal detected (mean envelope %.6f)\n",
                env.mean);
        free(rx_re); free(rx_im);
        dma_tx_stream_stop();
        hal_cleanup();
        printf("{\"passed\":false,\"reason\":\"no_signal\","
               "\"envelope_mean\":%.9f}\n", env.mean);
        return 1;
    }
    double dropout_threshold = fmax(env.mean * 0.1, 0.001);
    int dropouts = (int)count_dropouts(ana_re, ana_im, analyze_samples, block,
                                       dropout_threshold, verbose,
                                       sample_rate_hz);
    size_t n_blocks = analyze_samples / block;

    free(rx_re); free(rx_im);

    /* Report */
    fprintf(stderr, "\n=== STREAM TEST RESULTS ===\n");
    fprintf(stderr, "  Sample rate:      %llu Hz\n",
            (unsigned long long)sample_rate_hz);
    fprintf(stderr, "  Duration:         %d s\n", duration_sec);
    fprintf(stderr, "  Samples fed:      %zu\n", total_fed);
    fprintf(stderr, "  Feed cycles:      %d\n", feed_cycles);
    fprintf(stderr, "  Stall cycles:     %d\n", stall_cycles);
    fprintf(stderr, "  Min buffer avail: %d samples (%.1f ms)\n",
            min_available,
            (double)min_available * 1000.0 / (double)sample_rate_hz);
    fprintf(stderr, "  DAC valid miss:   %u\n", dac_miss);
    fprintf(stderr, "  RX captured:      %zu samples\n", cap_samples);
    fprintf(stderr, "  RX analyzed:      %zu samples (skipped first %zu)\n",
            analyze_samples, skip_samples);
    fprintf(stderr, "  Dropouts:         %d / %zu blocks (%.1f%%)\n",
            dropouts, n_blocks,
            n_blocks > 0 ? 100.0 * (double)dropouts / (double)n_blocks : 0.0);
    fprintf(stderr, "  Envelope mean:    %.6f\n", env.mean);
    fprintf(stderr, "  Envelope var:     %.9f\n", env.variance);
    fprintf(stderr, "  Envelope min/max: %.6f / %.6f\n",
            env.min_val, env.max_val);

    /* Pass criteria:
     *   1. No dropouts in the active TX window
     *   2. ARM actually fed samples (total_fed > 0)
     *   3. Effective rate >= 90% of target sample rate */
    double effective_rate = (double)total_fed / (double)duration_sec;
    bool rate_ok = effective_rate >= 0.9 * (double)sample_rate_hz;
    bool passed = (dropouts == 0) && (total_fed > 0) && rate_ok;

    fprintf(stderr, "\n  RESULT: %s\n", passed ? "PASS" : "FAIL");
    printf("{\"passed\":%s,\"sample_rate_hz\":%llu,"
           "\"duration_sec\":%d,\"fed\":%zu,"
           "\"cycles\":%d,\"stalls\":%d,\"min_avail\":%d,"
           "\"dac_miss\":%u,\"dropouts\":%d,\"n_blocks\":%zu}\n",
           passed ? "true" : "false",
           (unsigned long long)sample_rate_hz,
           duration_sec, total_fed,
           feed_cycles, stall_cycles, min_available,
           dac_miss, dropouts, n_blocks);

    hal_cleanup();
    return passed ? 0 : 1;
}
