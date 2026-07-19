/**
 * fuzz_rx.c -- libFuzzer target for lib80211_rx_decode.
 *
 * Exercises the RX path with arbitrary IQ samples.
 * Build: cmake -B build-fuzz -DLIB80211_BUILD_FUZZ=ON
 * Run:   ./build-fuzz/test/fuzz_rx -max_len=65536
 */

#include "lib80211/rx.h"
#include "lib80211/fft.h"
#include <stdint.h>
#include <stddef.h>

static lib80211_fft_plan *g_plan = NULL;

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    if (!g_plan) {
        g_plan = lib80211_fft_plan_create();
        if (!g_plan) return 0;
    }

    /* Need enough bytes for at least STF + LTF worth of IQ samples */
    size_t n_floats = size / sizeof(float);
    if (n_floats < 640) return 0;  /* 320 real + 320 imag minimum */

    size_t n_samples = n_floats / 2;
    const float *re = (const float *)data;
    const float *im = re + n_samples;

    lib80211_rx_result result;
    lib80211_rx_decode(g_plan, re, im, n_samples, &result);

    return 0;
}
