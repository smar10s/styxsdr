// SPDX-License-Identifier: MIT
/*
 * pluto_tx_beacon — Cyclic beacon transmitter
 *
 * Generates an 802.11 beacon frame at 6 Mbps and plays it in a loop
 * via DDR DMA with configurable silence between beacons.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include "hal.h"
#include "dma_tx.h"

#include <lib80211/fft.h>
#include <lib80211/mac.h>
#include <lib80211/tx.h>

/* -------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ   20000000ULL
#define BANDWIDTH_HZ     28000000ULL
#define SAMPLES_PER_TU   20  /* 1 TU = 1024 us ≈ 20 samples/us * 1024, but at 20 MSPS: 20 samples/us */

static volatile sig_atomic_t g_running = 1;

static void sigint_handler(int sig) {
    (void)sig;
    g_running = 0;
}

/* -------------------------------------------------------------------------- */

static void usage(const char *progname) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -s SSID         SSID (default: Styx)\n"
        "  -f MHz          LO frequency in MHz (default: 3456)\n"
        "  -a atten_dB     TX attenuation in dB (default: 10)\n"
        "  -i interval_TU  Beacon interval in TU (default: 100)\n"
        "  -h              Show this help\n",
        progname);
}

int main(int argc, char *argv[]) {
    const char *ssid = "Styx";
    uint64_t freq = 3456000000ULL;
    double tx_atten = 10.0;
    int interval_tu = 100;
    int opt;

    while ((opt = getopt(argc, argv, "s:f:a:i:h")) != -1) {
        switch (opt) {
        case 's': ssid = optarg; break;
        case 'f': freq = (uint64_t)(atof(optarg) * 1e6); break;
        case 'a': tx_atten = atof(optarg); break;
        case 'i': interval_tu = atoi(optarg); break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    /* Build beacon PSDU */
    uint8_t psdu[512];
    const uint8_t bssid[6] = {0x02, 0x00, 0x00, 0xDE, 0x1A, 0x05};
    /* Derive 802.11 channel number for beacon IE (0 if non-standard freq) */
    uint8_t chan_num = 0;
    uint64_t freq_mhz = freq / 1000000ULL;
    if (freq_mhz >= 2412 && freq_mhz <= 2472)
        chan_num = (uint8_t)((freq_mhz - 2407) / 5);
    else if (freq_mhz == 2484)
        chan_num = 14;
    else if (freq_mhz >= 5180 && freq_mhz <= 5825)
        chan_num = (uint8_t)((freq_mhz - 5000) / 5);
    size_t psdu_len = lib80211_build_beacon(psdu, sizeof(psdu) - 4,
                                            ssid, bssid, chan_num,
                                            (uint16_t)interval_tu, 0);
    if (psdu_len == 0) {
        fprintf(stderr, "ERROR: lib80211_build_beacon failed\n");
        return 1;
    }
    lib80211_append_fcs(psdu, psdu_len);
    psdu_len += 4;

    fprintf(stderr, "Beacon PSDU: %zu bytes, SSID=\"%s\", freq=%llu MHz\n",
            psdu_len, ssid, (unsigned long long)freq_mhz);

    /* Compute TX waveform length */
    lib80211_tx_legacy_params tx_params = {
        .rate_mbps = 6,
        .psdu = psdu,
        .psdu_len = psdu_len,
        .scrambler_seed = 0x5D,
    };

    size_t beacon_samples = lib80211_tx_legacy_samples(&tx_params);
    if (beacon_samples == 0) {
        fprintf(stderr, "ERROR: tx_legacy_samples returned 0\n");
        return 1;
    }

    /* Full period: beacon + silence to fill interval */
    /* interval_tu TU, 1 TU = 1024 us, at 20 MSPS = 20 samples/us */
    size_t period_samples = (size_t)interval_tu * 1024 * SAMPLES_PER_TU;
    size_t silence_samples;

    if (period_samples > DMA_TX_MAX_SAMPLES) {
        fprintf(stderr, "WARNING: period (%zu) exceeds DMA_TX_MAX_SAMPLES (%d), "
                "reducing silence to fit\n", period_samples, DMA_TX_MAX_SAMPLES);
        period_samples = DMA_TX_MAX_SAMPLES;
    }

    if (beacon_samples >= period_samples) {
        /* No room for silence — just use the beacon */
        silence_samples = 0;
        period_samples = beacon_samples;
        fprintf(stderr, "WARNING: beacon longer than interval, no silence padding\n");
    } else {
        silence_samples = period_samples - beacon_samples;
    }

    fprintf(stderr, "Waveform: %zu beacon + %zu silence = %zu total samples\n",
            beacon_samples, silence_samples, period_samples);

    /* Generate IQ */
    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "ERROR: fft_plan_create failed\n");
        return 1;
    }

    float *tx_real = calloc(period_samples, sizeof(float));
    float *tx_imag = calloc(period_samples, sizeof(float));
    if (!tx_real || !tx_imag) {
        fprintf(stderr, "ERROR: alloc failed for %zu samples\n", period_samples);
        lib80211_fft_plan_destroy(plan);
        return 1;
    }

    size_t gen = lib80211_tx_legacy(plan, &tx_params, tx_real, tx_imag);
    if (gen == 0) {
        fprintf(stderr, "ERROR: lib80211_tx_legacy failed\n");
        free(tx_real); free(tx_imag);
        lib80211_fft_plan_destroy(plan);
        return 1;
    }
    /* Silence is already zero from calloc */

    lib80211_fft_plan_destroy(plan);

    /* Initialize HAL and configure AD9361 */
    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        free(tx_real); free(tx_imag);
        return 1;
    }

    hal_ad9361_set_sample_rate(SAMPLE_RATE_HZ);
    hal_ad9361_set_tx_lo(freq);
    hal_ad9361_set_tx_bandwidth(BANDWIDTH_HZ);
    hal_ad9361_set_tx_attenuation(tx_atten);

    fprintf(stderr, "TX: freq=%llu MHz, atten=%.1f dB, rate=20 MSPS\n",
            (unsigned long long)(freq / 1000000ULL), tx_atten);

    /* Start cyclic TX DMA */
    if (dma_tx_start(tx_real, tx_imag, period_samples, true) != 0) {
        fprintf(stderr, "ERROR: dma_tx_start failed\n");
        free(tx_real); free(tx_imag);
        hal_cleanup();
        return 1;
    }

    fprintf(stderr, "Transmitting beacon (SIGINT to stop)...\n");

    /* Wait for SIGINT */
    signal(SIGINT, sigint_handler);
    while (g_running) {
        usleep(100000);  /* 100 ms */
    }

    fprintf(stderr, "\nStopping TX...\n");
    dma_tx_stop();
    free(tx_real);
    free(tx_imag);
    hal_cleanup();

    fprintf(stderr, "Done.\n");
    return 0;
}
