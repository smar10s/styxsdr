/**
 * FFT backend: Apple vDSP (Accelerate.framework)
 *
 * Uses vDSP_fft_zop for out-of-place complex FFT on split-complex data.
 * This is the native format for vDSP — no packing/unpacking needed.
 */

#include "lib80211/fft.h"

#include <Accelerate/Accelerate.h>
#include <stdlib.h>
#include <math.h>

struct lib80211_fft_plan {
    size_t n;
    int log2_n;
    FFTSetup setup;
};

lib80211_fft_plan *lib80211_fft_plan_create_n(size_t n) {
    lib80211_fft_plan *plan = malloc(sizeof(lib80211_fft_plan));
    if (!plan) return NULL;

    plan->n = n;
    plan->log2_n = (int)log2f((float)n);
    plan->setup = vDSP_create_fftsetup(plan->log2_n, kFFTRadix2);
    if (!plan->setup) {
        free(plan);
        return NULL;
    }
    return plan;
}

lib80211_fft_plan *lib80211_fft_plan_create(void) {
    return lib80211_fft_plan_create_n(64);
}

void lib80211_fft_plan_destroy(lib80211_fft_plan *plan) {
    if (plan) {
        vDSP_destroy_fftsetup(plan->setup);
        free(plan);
    }
}

void lib80211_fft_forward(lib80211_fft_plan *plan,
                          const float *in_real, const float *in_imag,
                          float *out_real, float *out_imag) {
    DSPSplitComplex input  = { .realp = (float *)in_real,  .imagp = (float *)in_imag };
    DSPSplitComplex output = { .realp = out_real, .imagp = out_imag };

    vDSP_fft_zop(plan->setup, &input, 1, &output, 1,
                 plan->log2_n, kFFTDirection_Forward);
}

void lib80211_fft_inverse(lib80211_fft_plan *plan,
                          const float *in_real, const float *in_imag,
                          float *out_real, float *out_imag) {
    DSPSplitComplex input  = { .realp = (float *)in_real,  .imagp = (float *)in_imag };
    DSPSplitComplex output = { .realp = out_real, .imagp = out_imag };

    vDSP_fft_zop(plan->setup, &input, 1, &output, 1,
                 plan->log2_n, kFFTDirection_Inverse);

    float scale = 1.0f / (float)plan->n;
    vDSP_vsmul(out_real, 1, &scale, out_real, 1, plan->n);
    vDSP_vsmul(out_imag, 1, &scale, out_imag, 1, plan->n);
}
