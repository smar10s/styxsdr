/**
 * tx_beacon.c — Generate beacon frame IQ and write as cf32 file.
 *
 * Builds beacon PSDUs, appends FCS, generates IQ via lib80211_tx_legacy
 * at 6 Mbps, and writes repeated beacons with inter-beacon silence
 * to a cf32 (interleaved float32 I/Q) file.
 *
 * Usage: tx_beacon [options] -o <output.cf32>
 */

#include "lib80211/lib80211.h"
#include "lib80211/mac.h"
#include "lib80211/tx.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define SAMPLE_RATE     20000000   /* 20 MSPS */
#define SAMPLES_PER_TU  20480      /* 1 TU = 1024 us @ 20 MSPS */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options] -o <output.cf32>\n"
        "  -s <ssid>       SSID (default: \"lib80211-test\")\n"
        "  -c <channel>    Channel number for DS Parameter IE (default: 36)\n"
        "  -i <interval>   Beacon interval in TU (default: 20)\n"
        "  -n <count>      Number of beacons in output (default: 50)\n"
        "  -o <file>       Output cf32 file (required)\n"
        "  -h              Help\n",
        prog);
}

int main(int argc, char *argv[])
{
    const char *ssid = "lib80211-test";
    int channel = 36;
    int interval_tu = 20;
    int count = 50;
    const char *output_path = NULL;

    int opt;
    while ((opt = getopt(argc, argv, "s:c:i:n:o:h")) != -1) {
        switch (opt) {
        case 's': ssid = optarg; break;
        case 'c': channel = atoi(optarg); break;
        case 'i': interval_tu = atoi(optarg); break;
        case 'n': count = atoi(optarg); break;
        case 'o': output_path = optarg; break;
        case 'h':
        default:
            usage(argv[0]);
            return (opt == 'h') ? 0 : 1;
        }
    }

    if (!output_path) {
        fprintf(stderr, "Error: -o <output.cf32> is required\n");
        usage(argv[0]);
        return 1;
    }

    if (channel < 1 || channel > 200) {
        fprintf(stderr, "Error: invalid channel %d\n", channel);
        return 1;
    }
    if (interval_tu < 1) {
        fprintf(stderr, "Error: invalid interval %d TU\n", interval_tu);
        return 1;
    }
    if (count < 1) {
        fprintf(stderr, "Error: invalid count %d\n", count);
        return 1;
    }

    /* Build beacon PSDU */
    uint8_t psdu[512];
    uint8_t bssid[6] = {0x02, 0x00, 0x00, 0xDE, 0xAD, 0x01};

    size_t psdu_len = lib80211_build_beacon(psdu, sizeof(psdu) - 4,
                                            ssid, bssid, (uint8_t)channel,
                                            (uint16_t)interval_tu, 0);
    if (psdu_len == 0) {
        fprintf(stderr, "Error: lib80211_build_beacon failed\n");
        return 1;
    }

    /* Append FCS */
    lib80211_append_fcs(psdu, psdu_len);
    psdu_len += 4;

    /* Create FFT plan */
    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "Error: cannot create FFT plan\n");
        return 1;
    }

    /* Generate IQ at 6 Mbps (mandatory beacon rate) */
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
    if (actual_samples == 0) {
        fprintf(stderr, "Error: lib80211_tx_legacy failed\n");
        lib80211_fft_plan_destroy(plan);
        free(tx_real);
        free(tx_imag);
        return 1;
    }

    /* Compute period size */
    size_t period_samples = (size_t)interval_tu * SAMPLES_PER_TU;
    if (actual_samples > period_samples) {
        fprintf(stderr, "Warning: beacon (%zu samples) exceeds interval (%zu samples), no silence gap\n",
                actual_samples, period_samples);
        period_samples = actual_samples;
    }
    size_t silence_samples = period_samples - actual_samples;

    /* Open output file */
    FILE *fout = fopen(output_path, "wb");
    if (!fout) {
        fprintf(stderr, "Error: cannot open '%s' for writing\n", output_path);
        lib80211_fft_plan_destroy(plan);
        free(tx_real);
        free(tx_imag);
        return 1;
    }

    /* Write beacons: [IQ][silence] repeated count times */
    /* Interleave beacon IQ into a buffer */
    float *beacon_cf32 = (float *)malloc(actual_samples * 2 * sizeof(float));
    if (!beacon_cf32) {
        fprintf(stderr, "Error: cannot allocate interleave buffer\n");
        fclose(fout);
        lib80211_fft_plan_destroy(plan);
        free(tx_real);
        free(tx_imag);
        return 1;
    }
    for (size_t s = 0; s < actual_samples; s++) {
        beacon_cf32[2 * s]     = tx_real[s];
        beacon_cf32[2 * s + 1] = tx_imag[s];
    }

    /* Silence buffer (zeros) */
    float *silence_cf32 = (float *)calloc(silence_samples * 2, sizeof(float));
    if (!silence_cf32 && silence_samples > 0) {
        fprintf(stderr, "Error: cannot allocate silence buffer\n");
        fclose(fout);
        free(beacon_cf32);
        lib80211_fft_plan_destroy(plan);
        free(tx_real);
        free(tx_imag);
        return 1;
    }

    size_t total_written = 0;
    for (int i = 0; i < count; i++) {
        size_t w = fwrite(beacon_cf32, sizeof(float), actual_samples * 2, fout);
        if (w != actual_samples * 2) {
            fprintf(stderr, "Error: short write at beacon %d\n", i);
            break;
        }
        total_written += actual_samples;

        if (silence_samples > 0) {
            w = fwrite(silence_cf32, sizeof(float), silence_samples * 2, fout);
            if (w != silence_samples * 2) {
                fprintf(stderr, "Error: short write at silence %d\n", i);
                break;
            }
            total_written += silence_samples;
        }
    }
    fclose(fout);

    size_t total_samples = (size_t)count * period_samples;
    fprintf(stderr, "tx_beacon: wrote %d beacons to %s\n", count, output_path);
    fprintf(stderr, "  SSID: \"%s\", channel: %d, interval: %d TU\n",
            ssid, channel, interval_tu);
    fprintf(stderr, "  Beacon: %zu samples, period: %zu samples, silence: %zu samples\n",
            actual_samples, period_samples, silence_samples);
    fprintf(stderr, "  Total: %zu samples (%zu bytes, %.3f sec @ %d MSPS)\n",
            total_samples, total_samples * 8,
            (double)total_samples / SAMPLE_RATE, SAMPLE_RATE / 1000000);

    free(beacon_cf32);
    free(silence_cf32);
    lib80211_fft_plan_destroy(plan);
    free(tx_real);
    free(tx_imag);

    return 0;
}
