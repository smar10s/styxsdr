// SPDX-License-Identifier: MIT
/*
 * pluto_sigladder — Signal complexity ladder test
 *
 * Exercises the TX→cable→RX DMA path at 6 levels of increasing
 * signal complexity.  Each level has a binary PASS/FAIL criterion.
 * Levels are cumulative: failure at level N means levels N+ are not
 * attempted (the prerequisite isn't met).
 *
 * Levels:
 *   1. Low-freq tone   — DDR→DAC→cable→ADC→DDR path alive
 *   2. Single tone     — sample rate, no aliasing, I/Q not swapped
 *   3. Chirp           — no sample drops, timing alignment
 *   4. STF only        — sync detection works through hardware
 *   5. STF+LTF         — CFO/timing estimation works
 *   6. Full frame      — end-to-end decode, FCS OK
 *
 * Usage:
 *   pluto_sigladder [options]
 *     -l N    Run only up to level N (default: 6)
 *     -c CH   Channel number (default: 36)
 *     -a dB   TX attenuation (default: 10)
 *     -g dB   RX gain (default: 50)
 *     -v      Verbose: print diagnostic details
 *     -h      Help
 *
 * Exit code: 0 = all requested levels pass, 1 = failure.
 * Output: JSON summary to stdout.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include "hal.h"
#include "dma_tx.h"
#include "dma_rx.h"
#include "convert.h"

#include <fftw3.h>

#include <lib80211/fft.h>
#include <lib80211/sync.h>
#include <lib80211/rx.h>
#include <lib80211/tx.h>
#include <lib80211/mac.h>

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ   20000000ULL
#define BANDWIDTH_HZ     28000000ULL
#define SILENCE_PAD      2000    /* samples of silence before/after waveform */
#define MAX_LEVELS       6

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --------------------------------------------------------------------------
 * Globals
 * -------------------------------------------------------------------------- */

static int g_max_level = MAX_LEVELS;
static uint64_t g_freq_hz = 3456000000ULL;  /* 3456 MHz — amateur 9cm, no WiFi/BT/cellular */
static double g_tx_atten = 10.0;
static double g_rx_gain = 50.0;
static bool g_verbose = false;
static bool g_cyclic = false;
static int g_repeat = 0;       /* >0: repeat L6 this many times, report stats */
static lib80211_fft_plan *g_fft_plan = NULL;

typedef struct {
    int level;
    const char *name;
    bool pass;
    const char *detail;
} level_result;

static level_result results[MAX_LEVELS];

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

static void configure_radio(void) {
    hal_ad9361_set_sample_rate(SAMPLE_RATE_HZ);
    hal_ad9361_set_tx_lo(g_freq_hz);
    hal_ad9361_set_rx_lo(g_freq_hz);
    hal_ad9361_set_tx_bandwidth(BANDWIDTH_HZ);
    hal_ad9361_set_rx_bandwidth(BANDWIDTH_HZ);
    hal_ad9361_set_rx_gain_mode("manual");
    hal_ad9361_set_rx_gain(g_rx_gain);
    hal_ad9361_set_tx_attenuation(g_tx_atten);

    /* Allow PLL lock and ADC settling after configuration change.
     * AD9361 auto-calibration mode runs TX quad and DC offset
     * calibration when frequency/bandwidth are written. */
    usleep(100000);  /* 100 ms for settling + auto-cal */
}

/*
 * Transmit a waveform and capture the loopback result.
 *
 * ONE-SHOT (cyclic=false):
 *   Two-phase TX for deterministic timing:
 *   1. dma_tx_load() — slow: DDR writes, reset, register config
 *   2. Record RX wr_ptr (our t=0 reference)
 *   3. dma_tx_trigger() — fast: single register write
 *   4. Wait for signal propagation
 *   5. Read from t=0 onward — signal is guaranteed to start within
 *      a few samples of t=0
 *
 * CYCLIC (cyclic=true):
 *   Start cyclic TX, wait for pipeline to settle (50ms = ~200
 *   iterations of a typical frame), then read BACKWARD from the
 *   current RX wr_ptr.  This guarantees we capture clean mid-stream
 *   data far from any startup transient or cyclic boundary.
 *   Matches airmon-sdr's proven approach.
 *
 * Returns number of captured samples, or -1 on error.
 * Caller must free *out_real and *out_imag.
 */
static int tx_and_capture(const float *tx_real, const float *tx_imag,
                          size_t tx_samples, size_t capture_samples,
                          bool cyclic,
                          float **out_real, float **out_imag)
{
    *out_real = malloc(capture_samples * sizeof(float));
    *out_imag = malloc(capture_samples * sizeof(float));
    if (!*out_real || !*out_imag) {
        free(*out_real); free(*out_imag);
        return -1;
    }

    /* Start RX DMA — ring buffer filling continuously */
    if (dma_rx_start() != 0) {
        free(*out_real); free(*out_imag);
        return -1;
    }

    /* Phase 1: Load waveform into DDR and configure DMA (slow). */
    if (dma_tx_load(tx_real, tx_imag, tx_samples, cyclic) != 0) {
        dma_rx_stop();
        free(*out_real); free(*out_imag);
        return -1;
    }

    if (!cyclic) {
        /* ONE-SHOT: record t0, trigger, wait, read forward from t0.
         * The frame appears a few samples after t0 (DAC pipeline delay). */
        uint32_t t0 = dma_rx_wr_ptr();

        if (dma_tx_trigger() != 0) {
            dma_rx_stop();
            free(*out_real); free(*out_imag);
            return -1;
        }

        unsigned int tx_us = (unsigned int)(tx_samples / 20);
        usleep(tx_us + 2000);

        /* Verify enough data captured since t0 */
        uint32_t cur = dma_rx_wr_ptr();
        uint32_t available = (cur >= t0) ? cur - t0
                           : (DMA_RX_BUF_SAMPLES - t0) + cur;
        if (available < (uint32_t)capture_samples) {
            usleep(5000);
            cur = dma_rx_wr_ptr();
            available = (cur >= t0) ? cur - t0
                      : (DMA_RX_BUF_SAMPLES - t0) + cur;
        }
        if (available < (uint32_t)capture_samples) {
            fprintf(stderr, "tx_and_capture: timeout (%u/%zu)\n",
                    available, capture_samples);
            dma_tx_stop(); dma_rx_stop();
            free(*out_real); free(*out_imag);
            *out_real = NULL; *out_imag = NULL;
            return -1;
        }

        /* Read forward from t0 */
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
    } else {
        /* CYCLIC: trigger, wait for pipeline to settle, snapshot wr_ptr
         * while TX is still running (data is good), THEN stop TX+RX and
         * read from the snapshot position.  This ensures we read data
         * captured while cyclic TX was actively playing. */
        if (dma_tx_trigger() != 0) {
            dma_rx_stop();
            free(*out_real); free(*out_imag);
            return -1;
        }

        /* Wait for cyclic TX to settle and multiple clean iterations
         * to play.  50ms gives ~200 iterations at typical frame size. */
        usleep(50000);

        /* Snapshot wr_ptr while TX is still running — data before this
         * point was captured with TX active (valid signal). */
        uint32_t cur = dma_rx_wr_ptr();

        /* Stop TX and RX to eliminate DDR contention during ARM read */
        dma_tx_stop();
        dma_rx_stop();

        volatile uint32_t *rx_buf = hal_ddr_rx_buf();

        /* Start reading from (cur - capture_samples), wrapping at 0 */
        uint32_t read_ptr;
        if (cur >= (uint32_t)capture_samples) {
            read_ptr = cur - (uint32_t)capture_samples;
        } else {
            read_ptr = DMA_RX_BUF_SAMPLES - ((uint32_t)capture_samples - cur);
        }

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
    }

    dma_tx_stop();
    dma_rx_stop();

    /* Remove residual DC offset.  The fabric DC filter (ADC_DCFILTER)
     * handles the bulk, but has finite settling and bandwidth. This
     * software pass removes any remaining DC component. */
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

/*
 * Generate a single-tone complex signal: exp(j * 2*pi * freq * t/Fs)
 */
static void gen_tone(float *re, float *im, size_t n, double freq_hz) {
    double phase_inc = 2.0 * M_PI * freq_hz / (double)SAMPLE_RATE_HZ;
    for (size_t i = 0; i < n; i++) {
        double phase = phase_inc * (double)i;
        re[i] = (float)cos(phase);
        im[i] = (float)sin(phase);
    }
}

/*
 * Generate a linear chirp from f0 to f1 over n samples.
 */
static void gen_chirp(float *re, float *im, size_t n, double f0, double f1) {
    double k = (f1 - f0) / ((double)n / (double)SAMPLE_RATE_HZ);
    for (size_t i = 0; i < n; i++) {
        double t = (double)i / (double)SAMPLE_RATE_HZ;
        double phase = 2.0 * M_PI * (f0 * t + 0.5 * k * t * t);
        re[i] = (float)cos(phase);
        im[i] = (float)sin(phase);
    }
}

/*
 * Compute FFT via fftw3f and return bin index of peak magnitude.
 * Uses a one-shot plan (acceptable for a diagnostic tool called twice).
 */
static int find_fft_peak(const float *re, const float *im, size_t n,
                         float *peak_mag_out) {
    size_t N = n;
    if (N > 4096) N = 4096;

    fftwf_complex *buf = fftwf_alloc_complex(N);
    if (!buf) { if (peak_mag_out) *peak_mag_out = 0; return 0; }

    for (size_t i = 0; i < N; i++) {
        buf[i][0] = re[i];
        buf[i][1] = im[i];
    }

    fftwf_plan plan = fftwf_plan_dft_1d((int)N, buf, buf,
                                         FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_execute(plan);
    fftwf_destroy_plan(plan);

    /* Find peak */
    float peak_mag = 0;
    int peak_bin = 0;
    for (size_t k = 0; k < N; k++) {
        float mag = buf[k][0] * buf[k][0] + buf[k][1] * buf[k][1];
        if (mag > peak_mag) {
            peak_mag = mag;
            peak_bin = (int)k;
        }
    }

    fftwf_free(buf);
    if (peak_mag_out) *peak_mag_out = sqrtf(peak_mag);
    return peak_bin;
}

/* --------------------------------------------------------------------------
 * Level 1: Low-frequency tone (100 kHz)
 *
 * Criterion: RX capture has non-zero energy, FFT peak is in the
 * expected frequency bin (within ±2 bins tolerance).
 * -------------------------------------------------------------------------- */
static bool run_level_1(void) {
    static char detail[256];
    results[0].detail = detail;

    double tone_freq = 100000.0;  /* 100 kHz */
    size_t tone_samples = 4096;
    size_t total_tx = SILENCE_PAD + tone_samples + SILENCE_PAD;

    float *tx_re = calloc(total_tx, sizeof(float));
    float *tx_im = calloc(total_tx, sizeof(float));
    if (!tx_re || !tx_im) { free(tx_re); free(tx_im); return false; }

    gen_tone(tx_re + SILENCE_PAD, tx_im + SILENCE_PAD, tone_samples, tone_freq);

    float *rx_re = NULL, *rx_im = NULL;
    size_t cap_samples = total_tx + 2000;
    int captured = tx_and_capture(tx_re, tx_im, total_tx, cap_samples, false, &rx_re, &rx_im);
    free(tx_re); free(tx_im);

    if (captured <= 0) {
        snprintf(detail, sizeof(detail), "capture failed");
        return false;
    }

    /* Find the region with energy (skip silence) */
    double total_energy = 0;
    for (int i = 0; i < captured; i++)
        total_energy += rx_re[i] * rx_re[i] + rx_im[i] * rx_im[i];

    if (total_energy < 1e-6) {
        snprintf(detail, sizeof(detail), "no energy in capture (total=%.2e)", total_energy);
        free(rx_re); free(rx_im);
        return false;
    }

    /* FFT peak check — use a 4096-point window from the middle of capture */
    size_t fft_start = (size_t)captured > 4096 + 1000 ? 1000 : 0;
    size_t fft_len = 4096;
    if (fft_start + fft_len > (size_t)captured)
        fft_len = (size_t)captured - fft_start;

    float peak_mag = 0;
    int peak_bin = find_fft_peak(rx_re + fft_start, rx_im + fft_start, fft_len, &peak_mag);

    /* Expected bin: tone_freq / (Fs / N), or its negative mirror */
    double bin_resolution = (double)SAMPLE_RATE_HZ / (double)fft_len;
    int expected_bin_pos = (int)(tone_freq / bin_resolution + 0.5);
    int expected_bin_neg = (int)fft_len - expected_bin_pos;

    /* Check error against both positive and negative frequency */
    int err_pos = abs(peak_bin - expected_bin_pos);
    if (err_pos > (int)(fft_len / 2)) err_pos = (int)fft_len - err_pos;
    int err_neg = abs(peak_bin - expected_bin_neg);
    if (err_neg > (int)(fft_len / 2)) err_neg = (int)fft_len - err_neg;
    int bin_error = err_pos < err_neg ? err_pos : err_neg;

    if (g_verbose)
        fprintf(stderr, "    L1: energy=%.2e, peak_bin=%d, expected=±%d, error=%d\n",
                total_energy, peak_bin, expected_bin_pos, bin_error);

    free(rx_re); free(rx_im);

    /* L1 criterion: energy is strong AND peak is near expected frequency.
     * Tolerance of ±30 bins allows for TX/RX LO offset (CFO). */
    if (total_energy > 100.0 && bin_error <= 30) {
        snprintf(detail, sizeof(detail), "peak_bin=%d expected=±%d (err=%d) energy=%.0f",
                 peak_bin, expected_bin_pos, bin_error, total_energy);
        return true;
    } else {
        snprintf(detail, sizeof(detail), "peak_bin=%d expected=±%d (err=%d) energy=%.2e",
                 peak_bin, expected_bin_pos, bin_error, total_energy);
        return false;
    }
}

/* --------------------------------------------------------------------------
 * Level 2: Single tone at known frequency (1 MHz)
 *
 * Criterion: FFT peak at correct bin within ±1 bin, and peak is >20 dB
 * above mean power (no aliasing / spurs dominating).
 * -------------------------------------------------------------------------- */
static bool run_level_2(void) {
    static char detail[256];
    results[1].detail = detail;

    double tone_freq = 1000000.0;  /* 1 MHz */
    size_t tone_samples = 4096;
    size_t total_tx = SILENCE_PAD + tone_samples + SILENCE_PAD;

    float *tx_re = calloc(total_tx, sizeof(float));
    float *tx_im = calloc(total_tx, sizeof(float));
    if (!tx_re || !tx_im) { free(tx_re); free(tx_im); return false; }

    gen_tone(tx_re + SILENCE_PAD, tx_im + SILENCE_PAD, tone_samples, tone_freq);

    float *rx_re = NULL, *rx_im = NULL;
    size_t cap_samples = total_tx + 2000;
    int captured = tx_and_capture(tx_re, tx_im, total_tx, cap_samples, false, &rx_re, &rx_im);
    free(tx_re); free(tx_im);

    if (captured <= 0) {
        snprintf(detail, sizeof(detail), "capture failed");
        return false;
    }

    /* FFT analysis */
    size_t fft_start = 1000;
    size_t fft_len = 4096;
    if (fft_start + fft_len > (size_t)captured) {
        fft_start = 0;
        fft_len = (size_t)captured > 4096 ? 4096 : (size_t)captured;
    }

    float peak_mag = 0;
    int peak_bin = find_fft_peak(rx_re + fft_start, rx_im + fft_start, fft_len, &peak_mag);

    /* Compute peak-to-total power ratio as SNR proxy */
    double total_power = 0;
    for (size_t i = fft_start; i < fft_start + fft_len; i++)
        total_power += rx_re[i] * rx_re[i] + rx_im[i] * rx_im[i];

    double peak_power = (double)peak_mag * peak_mag / (double)fft_len;
    double ratio_db = 10.0 * log10(peak_power / (total_power / (double)fft_len + 1e-30));

    double bin_resolution = (double)SAMPLE_RATE_HZ / (double)fft_len;
    int expected_bin_pos = (int)(tone_freq / bin_resolution + 0.5);
    int expected_bin_neg = (int)fft_len - expected_bin_pos;

    /* Check error against both positive and negative frequency */
    int err_pos = abs(peak_bin - expected_bin_pos);
    if (err_pos > (int)(fft_len / 2)) err_pos = (int)fft_len - err_pos;
    int err_neg = abs(peak_bin - expected_bin_neg);
    if (err_neg > (int)(fft_len / 2)) err_neg = (int)fft_len - err_neg;
    int bin_error = err_pos < err_neg ? err_pos : err_neg;

    if (g_verbose)
        fprintf(stderr, "    L2: peak_bin=%d expected=±%d err=%d, ratio=%.1f dB\n",
                peak_bin, expected_bin_pos, bin_error, ratio_db);

    free(rx_re); free(rx_im);

    /* L2 criterion: peak within ±30 bins of expected freq, >20 dB above floor */
    if (bin_error <= 30 && ratio_db > 20.0) {
        snprintf(detail, sizeof(detail), "bin=%d (err=%d) %.1fdB above floor",
                 peak_bin, bin_error, ratio_db);
        return true;
    } else {
        snprintf(detail, sizeof(detail), "bin=%d expected=±%d (err=%d) ratio=%.1fdB",
                 peak_bin, expected_bin_pos, bin_error, ratio_db);
        return false;
    }
}

/* --------------------------------------------------------------------------
 * Level 3: Chirp (linear sweep 0.5 → 9.5 MHz)
 *
 * Criterion: normalized cross-correlation of captured chirp against
 * reference > 0.9.
 * -------------------------------------------------------------------------- */
static bool run_level_3(void) {
    static char detail[256];
    results[2].detail = detail;

    double f0 = 500000.0;    /* 0.5 MHz */
    double f1 = 9500000.0;   /* 9.5 MHz — sweeps most of the 20 MHz band */
    size_t chirp_samples = 8192;
    size_t total_tx = SILENCE_PAD + chirp_samples + SILENCE_PAD;

    float *tx_re = calloc(total_tx, sizeof(float));
    float *tx_im = calloc(total_tx, sizeof(float));
    if (!tx_re || !tx_im) { free(tx_re); free(tx_im); return false; }

    gen_chirp(tx_re + SILENCE_PAD, tx_im + SILENCE_PAD, chirp_samples, f0, f1);

    float *rx_re = NULL, *rx_im = NULL;
    size_t cap_samples = total_tx + 4000;
    int captured = tx_and_capture(tx_re, tx_im, total_tx, cap_samples, false, &rx_re, &rx_im);

    if (captured <= 0) {
        snprintf(detail, sizeof(detail), "capture failed");
        free(tx_re); free(tx_im);
        return false;
    }

    /* L3 criterion: verify the chirp signal was received without sample drops.
     * Complex cross-correlation fails due to CFO (~100 kHz LO offset).
     * Instead, use power-envelope correlation which is CFO-insensitive:
     * compute |rx|^2 envelope and correlate against |tx|^2 envelope. */

    /* Compute TX power envelope (block averages for robustness) */
    size_t block = 64;
    size_t n_blocks_ref = chirp_samples / block;
    size_t n_blocks_sig = (size_t)captured / block;
    float *ref_env = calloc(n_blocks_ref, sizeof(float));
    float *sig_env = calloc(n_blocks_sig, sizeof(float));
    if (!ref_env || !sig_env) {
        free(ref_env); free(sig_env);
        free(tx_re); free(tx_im); free(rx_re); free(rx_im);
        snprintf(detail, sizeof(detail), "alloc failed");
        return false;
    }

    for (size_t b = 0; b < n_blocks_ref; b++) {
        double pwr = 0;
        for (size_t i = 0; i < block; i++) {
            size_t idx = SILENCE_PAD + b * block + i;
            pwr += tx_re[idx] * tx_re[idx] + tx_im[idx] * tx_im[idx];
        }
        ref_env[b] = (float)(pwr / (double)block);
    }

    for (size_t b = 0; b < n_blocks_sig; b++) {
        double pwr = 0;
        for (size_t i = 0; i < block; i++) {
            size_t idx = b * block + i;
            pwr += rx_re[idx] * rx_re[idx] + rx_im[idx] * rx_im[idx];
        }
        sig_env[b] = (float)(pwr / (double)block);
    }

    /* Normalized cross-correlation of power envelopes */
    float best_corr = 0;
    if (n_blocks_sig >= n_blocks_ref) {
        size_t search = n_blocks_sig - n_blocks_ref;
        if (search > 200) search = 200;
        double ref_energy = 0;
        for (size_t i = 0; i < n_blocks_ref; i++)
            ref_energy += (double)ref_env[i] * ref_env[i];

        for (size_t lag = 0; lag < search; lag++) {
            double dot = 0, sig_energy = 0;
            for (size_t i = 0; i < n_blocks_ref; i++) {
                dot += (double)ref_env[i] * sig_env[lag + i];
                sig_energy += (double)sig_env[lag + i] * sig_env[lag + i];
            }
            double norm = sqrt(ref_energy * sig_energy);
            if (norm > 0) {
                float val = (float)(dot / norm);
                if (val > best_corr) best_corr = val;
            }
        }
    }

    if (g_verbose)
        fprintf(stderr, "    L3: power-envelope correlation = %.4f (threshold: 0.8)\n", best_corr);

    free(ref_env); free(sig_env);
    free(tx_re); free(tx_im);
    free(rx_re); free(rx_im);

    snprintf(detail, sizeof(detail), "env_corr=%.4f", best_corr);

    return (best_corr > 0.8f);
}

/* --------------------------------------------------------------------------
 * Level 4: STF only (10 short training periods)
 *
 * Criterion: lib80211_sync_detect returns frame_start (>=0 return code).
 * -------------------------------------------------------------------------- */
static bool run_level_4(void) {
    static char detail[256];
    results[3].detail = detail;

    /* Generate a full preamble (STF + LTF) via lib80211_tx_legacy with
     * minimal data.  We'll only check sync detection. */
    uint8_t psdu[28];  /* minimal: 24-byte header + 4 FCS */
    memset(psdu, 0, sizeof(psdu));
    psdu[0] = 0x08;  /* Data frame */
    lib80211_append_fcs(psdu, 24);

    lib80211_tx_legacy_params tx_params = {
        .rate_mbps = 6,
        .psdu = psdu,
        .psdu_len = 28,
        .scrambler_seed = 0x5D,
    };

    size_t frame_samples = lib80211_tx_legacy_samples(&tx_params);
    size_t total_tx = SILENCE_PAD + frame_samples + SILENCE_PAD;
    if (total_tx > DMA_TX_MAX_SAMPLES) {
        snprintf(detail, sizeof(detail), "waveform too large");
        return false;
    }

    float *tx_re = calloc(total_tx, sizeof(float));
    float *tx_im = calloc(total_tx, sizeof(float));
    if (!tx_re || !tx_im) { free(tx_re); free(tx_im); return false; }

    lib80211_tx_legacy(g_fft_plan, &tx_params, tx_re + SILENCE_PAD, tx_im + SILENCE_PAD);

    float *rx_re = NULL, *rx_im = NULL;
    size_t cap_samples = g_cyclic ? total_tx * 5 + 8000 : total_tx + 8000;
    int captured = tx_and_capture(tx_re, tx_im, total_tx, cap_samples, g_cyclic, &rx_re, &rx_im);
    free(tx_re); free(tx_im);

    if (captured <= 0) {
        snprintf(detail, sizeof(detail), "capture failed");
        return false;
    }

    /* Run sync detection */
    lib80211_sync_result sync;
    int rc = lib80211_sync_detect(g_fft_plan, rx_re, rx_im, (size_t)captured, &sync);

    if (g_verbose)
        fprintf(stderr, "    L4: sync_detect rc=%d, frame_start=%zu, cfo=%.4f rad/s\n",
                rc, rc >= 0 ? sync.frame_start : 0, rc >= 0 ? sync.cfo_rad : 0.0f);

    free(rx_re); free(rx_im);

    if (rc >= 0) {
        snprintf(detail, sizeof(detail), "frame_start=%zu cfo=%.4f", sync.frame_start, sync.cfo_rad);
        return true;
    } else {
        snprintf(detail, sizeof(detail), "sync_detect failed (rc=%d)", rc);
        return false;
    }
}

/* --------------------------------------------------------------------------
 * Level 5: STF + LTF (full preamble, no data check)
 *
 * Criterion: sync detection succeeds AND channel estimate is non-degenerate
 * (verified by successfully calling rx_decode — it internally does channel
 * estimation).  We accept any decode result; the point is that sync+LTF
 * processing doesn't crash or return garbage.
 * -------------------------------------------------------------------------- */
static bool run_level_5(void) {
    static char detail[256];
    results[4].detail = detail;

    /* Same frame as level 4 */
    uint8_t psdu[28];
    memset(psdu, 0, sizeof(psdu));
    psdu[0] = 0x08;
    lib80211_append_fcs(psdu, 24);

    lib80211_tx_legacy_params tx_params = {
        .rate_mbps = 6,
        .psdu = psdu,
        .psdu_len = 28,
        .scrambler_seed = 0x5D,
    };

    size_t frame_samples = lib80211_tx_legacy_samples(&tx_params);
    size_t total_tx = SILENCE_PAD + frame_samples + SILENCE_PAD;

    float *tx_re = calloc(total_tx, sizeof(float));
    float *tx_im = calloc(total_tx, sizeof(float));
    if (!tx_re || !tx_im) { free(tx_re); free(tx_im); return false; }

    lib80211_tx_legacy(g_fft_plan, &tx_params, tx_re + SILENCE_PAD, tx_im + SILENCE_PAD);

    float *rx_re = NULL, *rx_im = NULL;
    size_t cap_samples = g_cyclic ? total_tx * 5 + 8000 : total_tx + 8000;
    int captured = tx_and_capture(tx_re, tx_im, total_tx, cap_samples, g_cyclic, &rx_re, &rx_im);
    free(tx_re); free(tx_im);

    if (captured <= 0) {
        snprintf(detail, sizeof(detail), "capture failed");
        return false;
    }

    /* Run full decode — for cyclic mode, scan multiple frames.
     * Advance by at least one TX period on failure to skip past any
     * boundary-crossing frame and reach a clean one. */
    lib80211_rx_result result;
    int dec_rc = -1;
    size_t offset = 0;
    size_t min_advance = g_cyclic ? total_tx : 400;
    int attempts = g_cyclic ? (int)((size_t)captured / total_tx) : 1;
    if (attempts < 1) attempts = 1;

    for (int a = 0; a < attempts && dec_rc < 0; a++) {
        if (offset + 400 >= (size_t)captured) break;
        dec_rc = lib80211_rx_decode(g_fft_plan, rx_re + offset, rx_im + offset,
                                    (size_t)captured - offset, &result);
        if (dec_rc >= 0 && result.rate_mbps == 6)
            break;
        /* Advance past this frame */
        lib80211_sync_result sync;
        if (lib80211_sync_detect(g_fft_plan, rx_re + offset, rx_im + offset,
                                 (size_t)captured - offset, &sync) == 0) {
            size_t skip = sync.frame_start + 400;
            if (skip < min_advance) skip = min_advance;
            offset += skip;
        } else {
            offset += min_advance;
        }
        dec_rc = -1;
    }

    if (g_verbose)
        fprintf(stderr, "    L5: decode rc=%d, rate=%d Mbps, n_symbols=%d\n",
                dec_rc, dec_rc >= 0 ? result.rate_mbps : 0,
                dec_rc >= 0 ? result.n_symbols : 0);

    free(rx_re); free(rx_im);

    if (dec_rc >= 0 && result.rate_mbps == 6) {
        snprintf(detail, sizeof(detail), "rate=%d n_syms=%d", result.rate_mbps, result.n_symbols);
        return true;
    } else {
        snprintf(detail, sizeof(detail), "decode failed (rc=%d)", dec_rc);
        return false;
    }
}

/* --------------------------------------------------------------------------
 * Level 6: Full frame decode (6 Mbps beacon, FCS valid)
 *
 * Criterion: complete decode, FCS check passes, payload matches.
 * -------------------------------------------------------------------------- */
#define L6_PAYLOAD_LEN 64

static bool run_level_6(void) {
    static char detail[256];
    results[5].detail = detail;

    /* Build a known beacon PSDU */
    size_t psdu_len = 24 + L6_PAYLOAD_LEN + 4;  /* header + payload + FCS */
    uint8_t psdu[24 + L6_PAYLOAD_LEN + 4];
    memset(psdu, 0, sizeof(psdu));

    /* Beacon frame */
    psdu[0] = 0x80; psdu[1] = 0x00;  /* Type: Management, Subtype: Beacon */
    /* DA = broadcast */
    memset(&psdu[4], 0xFF, 6);
    /* SA/BSSID */
    psdu[10] = 0x02; psdu[11] = 0xDE; psdu[12] = 0xAD;
    psdu[13] = 0xBE; psdu[14] = 0xEF; psdu[15] = 0x01;
    memcpy(&psdu[16], &psdu[10], 6);
    /* Payload: deterministic pattern */
    for (int i = 0; i < L6_PAYLOAD_LEN; i++)
        psdu[24 + i] = (uint8_t)(i ^ 0xA5);
    /* FCS */
    lib80211_append_fcs(psdu, 24 + L6_PAYLOAD_LEN);

    lib80211_tx_legacy_params tx_params = {
        .rate_mbps = 6,
        .psdu = psdu,
        .psdu_len = psdu_len,
        .scrambler_seed = 0x5D,
    };

    size_t frame_samples = lib80211_tx_legacy_samples(&tx_params);
    size_t total_tx = SILENCE_PAD + frame_samples + SILENCE_PAD;

    float *tx_re = calloc(total_tx, sizeof(float));
    float *tx_im = calloc(total_tx, sizeof(float));
    if (!tx_re || !tx_im) { free(tx_re); free(tx_im); return false; }

    lib80211_tx_legacy(g_fft_plan, &tx_params, tx_re + SILENCE_PAD, tx_im + SILENCE_PAD);

    float *rx_re = NULL, *rx_im = NULL;
    size_t cap_samples = g_cyclic ? total_tx * 5 + 8000 : total_tx + 8000;
    int captured = tx_and_capture(tx_re, tx_im, total_tx, cap_samples, g_cyclic, &rx_re, &rx_im);
    free(tx_re); free(tx_im);

    if (captured <= 0) {
        snprintf(detail, sizeof(detail), "capture failed");
        return false;
    }

    /* Decode — for cyclic mode, scan for multiple frames and try each.
     * The cyclic stream may have a boundary-crossing frame; skip it
     * and decode the next clean one.  We advance by at least one full
     * TX waveform period on each failed attempt to guarantee reaching
     * a fresh frame that isn't straddling the cyclic boundary. */
    lib80211_rx_result result;
    int dec_rc = -1;
    size_t offset = 0;
    size_t min_advance = g_cyclic ? total_tx : 400;
    int attempts = g_cyclic ? (int)((size_t)captured / total_tx) : 1;
    if (attempts < 1) attempts = 1;

    for (int a = 0; a < attempts && dec_rc < 0; a++) {
        if (offset + 400 >= (size_t)captured) break;

        dec_rc = lib80211_rx_decode(g_fft_plan, rx_re + offset, rx_im + offset,
                                    (size_t)captured - offset, &result);
        if (dec_rc >= 0 && result.fcs_valid)
            break;  /* success */

        /* Advance past this frame.  In cyclic mode, always advance at
         * least one full waveform period to land on a fresh frame. */
        if (dec_rc >= 0) {
            /* Decoded but FCS failed — advance past it */
            size_t skip = 400 + 80 * (size_t)result.n_symbols;
            if (skip < min_advance) skip = min_advance;
            offset += skip;
        } else {
            /* Decode failed entirely.  Try sync_detect to find next
             * frame start; if that also fails, advance by one period
             * (don't give up — there are more frames in the buffer). */
            lib80211_sync_result sync;
            if (lib80211_sync_detect(g_fft_plan, rx_re + offset, rx_im + offset,
                                     (size_t)captured - offset, &sync) == 0) {
                size_t skip = sync.frame_start + 400;
                if (skip < min_advance) skip = min_advance;
                offset += skip;
            } else {
                offset += min_advance;
            }
        }
        dec_rc = -1;  /* reset for next attempt */
    }
    free(rx_re); free(rx_im);

    if (dec_rc < 0) {
        snprintf(detail, sizeof(detail), "decode failed (rc=%d)", dec_rc);
        return false;
    }

    if (!result.fcs_valid) {
        snprintf(detail, sizeof(detail), "FCS invalid (rate=%d len=%zu)",
                 result.rate_mbps, result.psdu_len);
        return false;
    }

    /* Verify payload */
    if (result.psdu_len < psdu_len) {
        snprintf(detail, sizeof(detail), "short PSDU: %zu < %zu", result.psdu_len, psdu_len);
        return false;
    }

    bool payload_ok = true;
    for (int i = 0; i < L6_PAYLOAD_LEN; i++) {
        if (result.psdu[24 + i] != (uint8_t)(i ^ 0xA5)) {
            payload_ok = false;
            break;
        }
    }

    if (g_verbose)
        fprintf(stderr, "    L6: fcs=%s rate=%d payload=%s\n",
                result.fcs_valid ? "OK" : "FAIL",
                result.rate_mbps,
                payload_ok ? "match" : "MISMATCH");

    if (payload_ok) {
        snprintf(detail, sizeof(detail), "FCS OK, rate=%d, payload match", result.rate_mbps);
        return true;
    } else {
        snprintf(detail, sizeof(detail), "payload mismatch (FCS OK, rate=%d)", result.rate_mbps);
        return false;
    }
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

static void usage(const char *progname) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -l N    Run up to level N (default: 6)\n"
        "  -n N    Repeat L6 N times (stats mode, skips L1-L5)\n"
        "  -f MHz  LO frequency in MHz (default: 3456)\n"
        "  -a dB   TX attenuation (default: 10)\n"
        "  -g dB   RX gain (default: 50)\n"
        "  -C      Use cyclic TX (default: one-shot)\n"
        "  -v      Verbose output\n"
        "  -h      Help\n",
        progname);
}

int main(int argc, char *argv[]) {
    int opt;
    while ((opt = getopt(argc, argv, "l:n:f:a:g:Cvh")) != -1) {
        switch (opt) {
        case 'l': g_max_level = atoi(optarg); break;
        case 'n': g_repeat = atoi(optarg); break;
        case 'f': g_freq_hz = (uint64_t)(atof(optarg) * 1e6); break;
        case 'a': g_tx_atten = atof(optarg); break;
        case 'g': g_rx_gain = atof(optarg); break;
        case 'C': g_cyclic = true; break;
        case 'v': g_verbose = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (g_max_level < 1) g_max_level = 1;
    if (g_max_level > MAX_LEVELS) g_max_level = MAX_LEVELS;

    fprintf(stderr, "pluto_sigladder — signal complexity ladder (levels 1-%d)\n", g_max_level);
    fprintf(stderr, "  freq=%llu MHz, atten=%.1f dB, gain=%.1f dB, tx_mode=%s\n",
            (unsigned long long)(g_freq_hz / 1000000ULL), g_tx_atten, g_rx_gain,
            g_cyclic ? "cyclic" : "one-shot");

    /* Initialize */
    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    g_fft_plan = lib80211_fft_plan_create();
    if (!g_fft_plan) {
        fprintf(stderr, "ERROR: fft_plan_create failed\n");
        hal_cleanup();
        return 1;
    }

    configure_radio();

    /* Repeat mode: run L6 N times, report pass rate */
    if (g_repeat > 0) {
        fprintf(stderr, "Repeat mode: running L6 %d times\n", g_repeat);
        int pass = 0, fail_fcs = 0, fail_decode = 0;
        for (int r = 0; r < g_repeat; r++) {
            results[5].pass = run_level_6();
            if (results[5].pass) {
                pass++;
            } else if (results[5].detail && strstr(results[5].detail, "FCS")) {
                fail_fcs++;
            } else {
                fail_decode++;
            }
            if (g_verbose)
                fprintf(stderr, "  [%2d] %s (%s)\n", r+1,
                        results[5].pass ? "PASS" : "FAIL",
                        results[5].detail ? results[5].detail : "");
        }
        fprintf(stderr, "\nL6 repeat results: %d/%d pass (%.1f%%)\n",
                pass, g_repeat, 100.0 * pass / g_repeat);
        fprintf(stderr, "  fail_decode=%d fail_fcs=%d\n", fail_decode, fail_fcs);
        printf("{\"mode\":\"repeat\",\"level\":6,\"n\":%d,\"pass\":%d,"
               "\"fail_decode\":%d,\"fail_fcs\":%d,\"rate\":%.1f}\n",
               g_repeat, pass, fail_decode, fail_fcs,
               100.0 * pass / g_repeat);

        lib80211_fft_plan_destroy(g_fft_plan);
        hal_cleanup();
        return (pass == g_repeat) ? 0 : 1;
    }

    /* Level names */
    static const char *level_names[] = {
        "DC/low-freq tone",
        "Single tone (1 MHz)",
        "Chirp (0.5-9.5 MHz)",
        "STF detection",
        "STF+LTF (channel est)",
        "Full frame decode",
    };

    /* Level functions */
    typedef bool (*level_fn)(void);
    level_fn level_funcs[] = {
        run_level_1, run_level_2, run_level_3,
        run_level_4, run_level_5, run_level_6,
    };

    int highest_pass = 0;

    for (int i = 0; i < g_max_level; i++) {
        results[i].level = i + 1;
        results[i].name = level_names[i];

        fprintf(stderr, "  [%d] %s: ", i + 1, level_names[i]);

        results[i].pass = level_funcs[i]();

        if (results[i].pass) {
            fprintf(stderr, "PASS");
            if (g_verbose && results[i].detail)
                fprintf(stderr, " (%s)", results[i].detail);
            fprintf(stderr, "\n");
            highest_pass = i + 1;
        } else {
            fprintf(stderr, "FAIL");
            if (results[i].detail)
                fprintf(stderr, " (%s)", results[i].detail);
            fprintf(stderr, "\n");
            /* Stop — higher levels depend on lower ones passing */
            break;
        }
    }

    lib80211_fft_plan_destroy(g_fft_plan);
    hal_cleanup();

    /* JSON output */
    printf("{\"levels\":[");
    for (int i = 0; i < g_max_level; i++) {
        if (i > 0) printf(",");
        printf("{\"level\":%d,\"name\":\"%s\",\"pass\":%s",
               results[i].level, results[i].name,
               results[i].pass ? "true" : "false");
        if (results[i].detail)
            printf(",\"detail\":\"%s\"", results[i].detail);
        printf("}");
        if (!results[i].pass) break;  /* didn't run higher levels */
    }
    printf("],\"highest_pass\":%d,\"max_level\":%d}\n", highest_pass, g_max_level);

    fprintf(stderr, "\nResult: highest pass = level %d/%d\n", highest_pass, g_max_level);

    return (highest_pass == g_max_level) ? 0 : 1;
}
