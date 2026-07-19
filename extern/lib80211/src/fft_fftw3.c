/**
 * FFT backend: FFTW3 single-precision (libfftw3f)
 *
 * Uses fftwf_plan_dft_1d for 64-point complex FFT.
 * Input/output is split complex (separate real/imag arrays) —
 * we pack into FFTW's interleaved format and unpack after.
 *
 * FFTW3 provides optimized codelet paths for power-of-2 sizes
 * and uses SIMD (NEON on ARM, SSE/AVX on x86) automatically.
 */

#include "lib80211/fft.h"

#include <fftw3.h>
#include <stdlib.h>

struct lib80211_fft_plan {
    size_t n;
    fftwf_plan forward;
    fftwf_plan inverse;
    fftwf_complex *in;
    fftwf_complex *out;
};

lib80211_fft_plan *lib80211_fft_plan_create_n(size_t n)
{
    lib80211_fft_plan *plan = malloc(sizeof(lib80211_fft_plan));
    if (!plan) return NULL;

    plan->n = n;
    plan->in = fftwf_alloc_complex(n);
    plan->out = fftwf_alloc_complex(n);
    if (!plan->in || !plan->out) {
        fftwf_free(plan->in);
        fftwf_free(plan->out);
        free(plan);
        return NULL;
    }

    plan->forward = fftwf_plan_dft_1d(n, plan->in, plan->out,
                                       FFTW_FORWARD, FFTW_ESTIMATE);
    plan->inverse = fftwf_plan_dft_1d(n, plan->in, plan->out,
                                       FFTW_BACKWARD, FFTW_ESTIMATE);

    if (!plan->forward || !plan->inverse) {
        fftwf_destroy_plan(plan->forward);
        fftwf_destroy_plan(plan->inverse);
        fftwf_free(plan->in);
        fftwf_free(plan->out);
        free(plan);
        return NULL;
    }

    return plan;
}

lib80211_fft_plan *lib80211_fft_plan_create(void)
{
    return lib80211_fft_plan_create_n(64);
}

void lib80211_fft_plan_destroy(lib80211_fft_plan *plan)
{
    if (plan) {
        fftwf_destroy_plan(plan->forward);
        fftwf_destroy_plan(plan->inverse);
        fftwf_free(plan->in);
        fftwf_free(plan->out);
        free(plan);
    }
}

void lib80211_fft_forward(lib80211_fft_plan *plan,
                          const float *in_real, const float *in_imag,
                          float *out_real, float *out_imag)
{
    for (size_t i = 0; i < plan->n; i++) {
        plan->in[i][0] = in_real[i];
        plan->in[i][1] = in_imag[i];
    }

    fftwf_execute(plan->forward);

    for (size_t i = 0; i < plan->n; i++) {
        out_real[i] = plan->out[i][0];
        out_imag[i] = plan->out[i][1];
    }
}

void lib80211_fft_inverse(lib80211_fft_plan *plan,
                          const float *in_real, const float *in_imag,
                          float *out_real, float *out_imag)
{
    for (size_t i = 0; i < plan->n; i++) {
        plan->in[i][0] = in_real[i];
        plan->in[i][1] = in_imag[i];
    }

    fftwf_execute(plan->inverse);

    const float scale = 1.0f / (float)plan->n;
    for (size_t i = 0; i < plan->n; i++) {
        out_real[i] = plan->out[i][0] * scale;
        out_imag[i] = plan->out[i][1] * scale;
    }
}
