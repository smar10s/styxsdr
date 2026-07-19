/**
 * ota_tx_beacon.c — On-device beacon TX via libiio cyclic DMA.
 *
 * Generates beacon frame IQ using lib80211, scales to int16, and pushes
 * to the AD9361 TX via libiio cyclic buffer. The hardware repeats the
 * buffer continuously until stopped (Ctrl+C).
 *
 * Designed for ADALM-Pluto (Cortex-A9). Minimal memory usage.
 *
 * Usage: ota_tx_beacon [options]
 *   -s <ssid>       SSID to broadcast (default: "lib80211-test")
 *   -c <channel>    Channel number (default: 36)
 *   -i <interval>   Beacon interval in TU (default: 100)
 *   -a <atten_db>   TX attenuation in dB (default: 10)
 *   -h              Help
 */

#include "lib80211/lib80211.h"
#include "lib80211/mac.h"
#include "lib80211/tx.h"
#include "lib80211/fft.h"

#include <iio.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>

/* ========================================================================
 * Defaults
 * ======================================================================== */

#define SAMPLE_RATE       20000000LL
#define SAMPLES_PER_TU    20480        /* 1 TU = 1024 us @ 20 MSPS */

/* Channel → frequency map (5 GHz only for now) */
static long long channel_to_freq(int ch)
{
    if (ch >= 36 && ch <= 64)
        return 5000000000LL + (long long)ch * 5000000LL;
    if (ch >= 100 && ch <= 144)
        return 5000000000LL + (long long)ch * 5000000LL;
    if (ch >= 149 && ch <= 165)
        return 5000000000LL + (long long)ch * 5000000LL;
    return -1;
}

/* ========================================================================
 * Signal handling
 * ======================================================================== */

static volatile sig_atomic_t g_stop = 0;

static void sig_handler(int sig)
{
    (void)sig;
    g_stop = 1;
}

/* ========================================================================
 * Usage
 * ======================================================================== */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -s <ssid>       SSID (default: \"lib80211-test\")\n"
        "  -c <channel>    Channel number (default: 36)\n"
        "  -i <interval>   Beacon interval in TU (default: 100)\n"
        "  -a <atten_db>   TX attenuation in dB (default: 10)\n"
        "  -h              Help\n",
        prog);
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(int argc, char *argv[])
{
    const char *ssid = "lib80211-test";
    int channel = 36;
    int interval_tu = 100;
    double attenuation = 10.0;

    /* Parse options */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            ssid = argv[++i];
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            channel = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            interval_tu = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
            attenuation = atof(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    long long freq_hz = channel_to_freq(channel);
    if (freq_hz < 0) {
        fprintf(stderr, "Error: unsupported channel %d\n", channel);
        return 1;
    }
    if (interval_tu < 1 || interval_tu > 10000) {
        fprintf(stderr, "Error: invalid interval %d TU\n", interval_tu);
        return 1;
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* ---- Generate beacon IQ ---- */
    fprintf(stderr, "ota_tx_beacon: generating beacon...\n");

    uint8_t psdu[512];
    uint8_t bssid[6] = {0x02, 0x00, 0x00, 0xDE, 0xAD, 0x01};

    size_t psdu_len = lib80211_build_beacon(psdu, sizeof(psdu) - 4,
                                            ssid, bssid, (uint8_t)channel,
                                            (uint16_t)interval_tu, 0);
    if (psdu_len == 0) {
        fprintf(stderr, "Error: lib80211_build_beacon failed\n");
        return 1;
    }

    lib80211_append_fcs(psdu, psdu_len);
    psdu_len += 4;

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "Error: cannot create FFT plan\n");
        return 1;
    }

    lib80211_tx_legacy_params params = {
        .rate_mbps = 6,
        .psdu = psdu,
        .psdu_len = psdu_len,
        .scrambler_seed = 0x5D,
    };

    size_t beacon_samples = lib80211_tx_legacy_samples(&params);
    float *tx_real = (float *)calloc(beacon_samples, sizeof(float));
    float *tx_imag = (float *)calloc(beacon_samples, sizeof(float));
    if (!tx_real || !tx_imag) {
        fprintf(stderr, "Error: cannot allocate TX buffers\n");
        lib80211_fft_plan_destroy(plan);
        return 1;
    }

    size_t actual_samples = lib80211_tx_legacy(plan, &params, tx_real, tx_imag);
    lib80211_fft_plan_destroy(plan);

    if (actual_samples == 0) {
        fprintf(stderr, "Error: lib80211_tx_legacy failed\n");
        free(tx_real);
        free(tx_imag);
        return 1;
    }

    /* Build one full period: beacon IQ + silence gap */
    size_t period_samples = (size_t)interval_tu * SAMPLES_PER_TU;
    if (actual_samples > period_samples) {
        fprintf(stderr, "Warning: beacon (%zu samples) exceeds interval, no gap\n",
                actual_samples);
        period_samples = actual_samples;
    }

    /* Convert to int16 (IIO native format) with scaling.
     * AD9361 TX DAC is 12-bit; use 11-bit range for headroom. */
    int16_t *tx_buf = (int16_t *)calloc(period_samples * 2, sizeof(int16_t));
    if (!tx_buf) {
        fprintf(stderr, "Error: cannot allocate TX period buffer\n");
        free(tx_real);
        free(tx_imag);
        return 1;
    }

    /* Find peak amplitude for normalization */
    float peak = 0;
    for (size_t s = 0; s < actual_samples; s++) {
        float mag = fabsf(tx_real[s]);
        if (mag > peak) peak = mag;
        mag = fabsf(tx_imag[s]);
        if (mag > peak) peak = mag;
    }
    if (peak == 0) peak = 1.0f;

    /* Scale to 2^14 with one bit headroom below int16 max.
     * This matches pyadi-iio's scaling for maximum TX power. */
    float scale = 16384.0f / peak;
    for (size_t s = 0; s < actual_samples; s++) {
        tx_buf[2 * s]     = (int16_t)(tx_real[s] * scale);
        tx_buf[2 * s + 1] = (int16_t)(tx_imag[s] * scale);
    }
    /* Silence gap is already zero from calloc */

    free(tx_real);
    free(tx_imag);

    fprintf(stderr, "ota_tx_beacon: beacon=%zu samples, period=%zu samples (%.1f ms)\n",
            actual_samples, period_samples,
            (double)period_samples / SAMPLE_RATE * 1000.0);

    /* ---- IIO setup ---- */
    fprintf(stderr, "ota_tx_beacon: connecting to IIO...\n");

    struct iio_context *ctx = iio_create_local_context();
    if (!ctx) {
        fprintf(stderr, "Error: cannot create local IIO context\n");
        free(tx_buf);
        return 1;
    }

    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    if (!phy) {
        fprintf(stderr, "Error: ad9361-phy not found\n");
        free(tx_buf);
        iio_context_destroy(ctx);
        return 1;
    }

    /* Configure TX */
    struct iio_channel *tx_lo = iio_device_find_channel(phy, "altvoltage1", true);
    struct iio_channel *tx_bb = iio_device_find_channel(phy, "voltage0", true);
    if (!tx_lo || !tx_bb) {
        fprintf(stderr, "Error: TX LO or baseband channel not found\n");
        free(tx_buf);
        iio_context_destroy(ctx);
        return 1;
    }

    iio_channel_attr_write_longlong(tx_lo, "frequency", freq_hz);
    iio_channel_attr_write_longlong(tx_bb, "sampling_frequency", SAMPLE_RATE);
    iio_channel_attr_write_longlong(tx_bb, "rf_bandwidth", SAMPLE_RATE);
    iio_channel_attr_write_double(tx_bb, "hardwaregain", -attenuation);

    /* Open TX streaming device */
    struct iio_device *txdev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (!txdev) {
        fprintf(stderr, "Error: cf-ad9361-dds-core-lpc (TX DMA) not found\n");
        free(tx_buf);
        iio_context_destroy(ctx);
        return 1;
    }

    struct iio_channel *tx_i = iio_device_find_channel(txdev, "voltage0", true);
    struct iio_channel *tx_q = iio_device_find_channel(txdev, "voltage1", true);
    if (!tx_i || !tx_q) {
        fprintf(stderr, "Error: TX I/Q channels not found\n");
        free(tx_buf);
        iio_context_destroy(ctx);
        return 1;
    }
    iio_channel_enable(tx_i);
    iio_channel_enable(tx_q);

    /* Create cyclic TX buffer */
    struct iio_buffer *iio_txbuf = iio_device_create_buffer(txdev, period_samples, true);
    if (!iio_txbuf) {
        fprintf(stderr, "Error: cannot create TX cyclic buffer (%zu samples)\n",
                period_samples);
        free(tx_buf);
        iio_context_destroy(ctx);
        return 1;
    }

    /* Copy samples into IIO buffer */
    void *buf_start = iio_buffer_start(iio_txbuf);
    memcpy(buf_start, tx_buf, period_samples * 4);  /* 2 * int16 per sample */
    free(tx_buf);

    /* Push — starts continuous cyclic TX */
    ssize_t nbytes = iio_buffer_push(iio_txbuf);
    if (nbytes < 0) {
        fprintf(stderr, "Error: iio_buffer_push failed: %zd\n", nbytes);
        iio_buffer_destroy(iio_txbuf);
        iio_context_destroy(ctx);
        return 1;
    }

    fprintf(stderr, "\nota_tx_beacon: TX active\n");
    fprintf(stderr, "  SSID:      \"%s\"\n", ssid);
    fprintf(stderr, "  Channel:   %d (%.0f MHz)\n", channel, freq_hz / 1e6);
    fprintf(stderr, "  Interval:  %d TU\n", interval_tu);
    fprintf(stderr, "  Atten:     %.1f dB\n", attenuation);
    fprintf(stderr, "  Buffer:    %zu samples (%zd bytes pushed)\n",
            period_samples, nbytes);
    fprintf(stderr, "\nPress Ctrl+C to stop.\n");
    fprintf(stderr, "Verify: macOS WiFi scanner or `iw dev wlan0 scan`\n\n");

    /* Wait for signal */
    while (!g_stop) {
        usleep(100000);  /* 100 ms */
    }

    fprintf(stderr, "\nota_tx_beacon: stopping TX...\n");
    iio_buffer_destroy(iio_txbuf);
    iio_context_destroy(ctx);

    fprintf(stderr, "ota_tx_beacon: done.\n");
    return 0;
}
