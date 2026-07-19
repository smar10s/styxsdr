/**
 * gen_test_cf32.c — Generate a test cf32 file containing a beacon frame.
 *
 * Builds a beacon PSDU, appends FCS, generates IQ via lib80211_tx_legacy
 * at 6 Mbps, interleaves split complex → cf32 format, writes to file.
 *
 * Usage: gen_test_cf32 <output.cf32>
 */

#include "lib80211/lib80211.h"
#include "lib80211/mac.h"
#include "lib80211/tx.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <output.cf32>\n", argv[0]);
        return 1;
    }
    const char *output_path = argv[1];

    /* Build beacon PSDU */
    uint8_t psdu[256];
    uint8_t bssid[6] = {0x02, 0x00, 0x00, 0xDE, 0xAD, 0x01};
    const char *ssid = "TestNet80211";

    size_t psdu_len = lib80211_build_beacon(psdu, sizeof(psdu) - 4,
                                            ssid, bssid, 36, 100, 0);
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

    /* Generate IQ at 6 Mbps */
    lib80211_tx_legacy_params params = {
        .rate_mbps = 6,
        .psdu = psdu,
        .psdu_len = psdu_len,
        .scrambler_seed = 0x5D,
    };

    size_t n_samples = lib80211_tx_legacy_samples(&params);
    float *tx_real = (float *)calloc(n_samples + 200, sizeof(float));
    float *tx_imag = (float *)calloc(n_samples + 200, sizeof(float));
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

    /* Add some leading/trailing silence for realism */
    size_t lead_silence = 200;
    size_t trail_silence = 200;
    size_t total_samples = lead_silence + actual_samples + trail_silence;

    /* Interleave to cf32 (I0, Q0, I1, Q1, ...) */
    float *cf32 = (float *)calloc(total_samples * 2, sizeof(float));
    if (!cf32) {
        fprintf(stderr, "Error: cannot allocate cf32 buffer\n");
        lib80211_fft_plan_destroy(plan);
        free(tx_real);
        free(tx_imag);
        return 1;
    }

    for (size_t s = 0; s < actual_samples; s++) {
        cf32[2 * (lead_silence + s)]     = tx_real[s];
        cf32[2 * (lead_silence + s) + 1] = tx_imag[s];
    }

    /* Write to file */
    FILE *fout = fopen(output_path, "wb");
    if (!fout) {
        fprintf(stderr, "Error: cannot open '%s' for writing\n", output_path);
        lib80211_fft_plan_destroy(plan);
        free(tx_real);
        free(tx_imag);
        free(cf32);
        return 1;
    }

    size_t written = fwrite(cf32, sizeof(float), total_samples * 2, fout);
    fclose(fout);

    if (written != total_samples * 2) {
        fprintf(stderr, "Error: short write\n");
        lib80211_fft_plan_destroy(plan);
        free(tx_real);
        free(tx_imag);
        free(cf32);
        return 1;
    }

    fprintf(stderr, "gen_test_cf32: wrote %zu samples (%zu bytes) to %s\n",
            total_samples, total_samples * 8, output_path);
    fprintf(stderr, "  PSDU: %zu bytes, SSID: \"%s\", BSSID: %02x:%02x:%02x:%02x:%02x:%02x\n",
            psdu_len, ssid,
            bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);

    lib80211_fft_plan_destroy(plan);
    free(tx_real);
    free(tx_imag);
    free(cf32);

    return 0;
}
