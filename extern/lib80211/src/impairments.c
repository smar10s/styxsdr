/**
 * impairments.c -- Channel impairment models for testing.
 *
 * Provides AWGN, CFO, and multipath simulation with deterministic PRNG.
 * All functions operate in-place for zero-allocation usage.
 */

#include "lib80211/impairments.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 * xoshiro256** PRNG
 * ======================================================================== */

static uint64_t splitmix64(uint64_t *state)
{
    uint64_t z = (*state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

void lib80211_rng_seed(lib80211_rng *rng, uint64_t seed)
{
    uint64_t sm = seed;
    rng->s[0] = splitmix64(&sm);
    rng->s[1] = splitmix64(&sm);
    rng->s[2] = splitmix64(&sm);
    rng->s[3] = splitmix64(&sm);
}

static inline uint64_t rotl(uint64_t x, int k)
{
    return (x << k) | (x >> (64 - k));
}

static uint64_t xoshiro256ss_next(lib80211_rng *rng)
{
    uint64_t *s = rng->s;
    uint64_t result = rotl(s[1] * 5, 7) * 9;
    uint64_t t = s[1] << 17;

    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];
    s[2] ^= t;
    s[3] = rotl(s[3], 45);

    return result;
}

float lib80211_rng_float(lib80211_rng *rng)
{
    /* Use upper 24 bits for float mantissa precision */
    uint64_t x = xoshiro256ss_next(rng);
    return (float)(x >> 40) * (1.0f / 16777216.0f);  /* / 2^24 */
}

float lib80211_rng_normal(lib80211_rng *rng)
{
    /* Box-Muller transform */
    float u1, u2;
    do {
        u1 = lib80211_rng_float(rng);
    } while (u1 < 1e-30f);  /* avoid log(0) */
    u2 = lib80211_rng_float(rng);

    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

/* ========================================================================
 * AWGN
 * ======================================================================== */

void lib80211_add_awgn(float *re, float *im, size_t n,
                       float snr_db, lib80211_rng *rng)
{
    if (n == 0) return;

    /* Compute signal power */
    double sig_pwr = 0.0;
    for (size_t i = 0; i < n; i++) {
        sig_pwr += (double)re[i] * re[i] + (double)im[i] * im[i];
    }
    sig_pwr /= (double)n;

    if (sig_pwr < 1e-20) return;  /* silent signal, nothing to add */

    /* Noise power per component */
    double noise_pwr = sig_pwr / pow(10.0, (double)snr_db / 10.0);
    float sigma = sqrtf((float)(noise_pwr / 2.0));

    /* Add Gaussian noise */
    for (size_t i = 0; i < n; i++) {
        re[i] += sigma * lib80211_rng_normal(rng);
        im[i] += sigma * lib80211_rng_normal(rng);
    }
}

/* ========================================================================
 * CFO
 * ======================================================================== */

void lib80211_add_cfo(float *re, float *im, size_t n,
                      float cfo_hz, float sample_rate, float phi0)
{
    if (n == 0) return;

    float phase_inc = 2.0f * (float)M_PI * cfo_hz / sample_rate;

    /* Incremental rotation to avoid per-sample trig */
    float cos_inc = cosf(phase_inc);
    float sin_inc = sinf(phase_inc);

    float cos_phase = cosf(phi0);
    float sin_phase = sinf(phi0);

    for (size_t i = 0; i < n; i++) {
        float r = re[i];
        float q = im[i];

        /* (r + jq) * (cos_phase + j*sin_phase) */
        re[i] = r * cos_phase - q * sin_phase;
        im[i] = r * sin_phase + q * cos_phase;

        /* Advance phase: (cos,sin) *= (cos_inc, sin_inc) */
        float new_cos = cos_phase * cos_inc - sin_phase * sin_inc;
        float new_sin = sin_phase * cos_inc + cos_phase * sin_inc;
        cos_phase = new_cos;
        sin_phase = new_sin;

        /* Periodic renormalization to prevent drift (every 1024 samples) */
        if ((i & 1023) == 1023) {
            float norm = 1.0f / sqrtf(cos_phase * cos_phase + sin_phase * sin_phase);
            cos_phase *= norm;
            sin_phase *= norm;
        }
    }
}

/* ========================================================================
 * Multipath
 * ======================================================================== */

/* Preset channel models */
const lib80211_multipath_tap LIB80211_MULTIPATH_MILD[2] = {
    { .delay = 0,  .gain_re = 1.0f,  .gain_im = 0.0f },
    { .delay = 3,  .gain_re = -0.3f, .gain_im = 0.1f },
};

const lib80211_multipath_tap LIB80211_MULTIPATH_MODERATE[3] = {
    { .delay = 0,  .gain_re = 1.0f,  .gain_im = 0.0f },
    { .delay = 5,  .gain_re = -0.4f, .gain_im = 0.3f },
    { .delay = 12, .gain_re = 0.2f,  .gain_im = -0.1f },
};

const lib80211_multipath_tap LIB80211_MULTIPATH_SEVERE[3] = {
    { .delay = 0,  .gain_re = 1.0f,  .gain_im = 0.0f },
    { .delay = 8,  .gain_re = 0.8f,  .gain_im = 0.0f },
    { .delay = 15, .gain_re = -0.5f, .gain_im = 0.0f },
};

void lib80211_apply_multipath(float *re, float *im, size_t n,
                               const lib80211_multipath_tap *taps, int n_taps,
                               float *tmp_re, float *tmp_im)
{
    if (n == 0 || n_taps == 0) return;

    /* Save original into temp buffers */
    memcpy(tmp_re, re, n * sizeof(float));
    memcpy(tmp_im, im, n * sizeof(float));

    /* Zero output */
    memset(re, 0, n * sizeof(float));
    memset(im, 0, n * sizeof(float));

    /* Accumulate each tap */
    for (int t = 0; t < n_taps; t++) {
        int d = taps[t].delay;
        float gr = taps[t].gain_re;
        float gi = taps[t].gain_im;

        if (d < 0 || (size_t)d >= n) continue;

        /* out[i] += (tmp[i-d] * gain) for i >= d */
        for (size_t i = (size_t)d; i < n; i++) {
            float sr = tmp_re[i - d];
            float si = tmp_im[i - d];
            /* complex multiply: (sr + j*si) * (gr + j*gi) */
            re[i] += sr * gr - si * gi;
            im[i] += sr * gi + si * gr;
        }
    }
}

/* ========================================================================
 * SFO (Sampling Frequency Offset)
 * ======================================================================== */

size_t lib80211_sfo_output_len(size_t n_in, float ppm)
{
    double ratio = 1.0 + (double)ppm * 1e-6;
    return (size_t)((double)n_in * ratio);
}

size_t lib80211_add_sfo(const float *re_in, const float *im_in, size_t n_in,
                        float ppm, float *re_out, float *im_out)
{
    if (n_in == 0) return 0;

    double ratio = 1.0 + (double)ppm * 1e-6;
    size_t n_out = (size_t)((double)n_in * ratio);

    for (size_t i = 0; i < n_out; i++) {
        double in_idx = (double)i / ratio;
        size_t lo = (size_t)in_idx;
        size_t hi = lo + 1;
        if (lo >= n_in) lo = n_in - 1;
        if (hi >= n_in) hi = n_in - 1;
        float frac = (float)(in_idx - (double)lo);

        re_out[i] = re_in[lo] * (1.0f - frac) + re_in[hi] * frac;
        im_out[i] = im_in[lo] * (1.0f - frac) + im_in[hi] * frac;
    }

    return n_out;
}

/* ========================================================================
 * DC offset
 * ======================================================================== */

void lib80211_add_dc_offset(float *re, float *im, size_t n,
                            float dc_i, float dc_q)
{
    if (n == 0) return;

    /* Compute signal RMS */
    double pwr = 0.0;
    for (size_t i = 0; i < n; i++) {
        pwr += (double)re[i] * re[i] + (double)im[i] * im[i];
    }
    pwr /= (double)n;
    if (pwr < 1e-20) return;

    float rms = sqrtf((float)pwr);
    float offset_re = rms * dc_i;
    float offset_im = rms * dc_q;

    for (size_t i = 0; i < n; i++) {
        re[i] += offset_re;
        im[i] += offset_im;
    }
}

/* ========================================================================
 * IQ imbalance
 * ======================================================================== */

void lib80211_add_iq_imbalance(float *re, float *im, size_t n,
                               float gain_db, float phase_deg)
{
    if (n == 0) return;

    float gain_lin = powf(10.0f, gain_db / 20.0f);
    float g_i = 1.0f + (gain_lin - 1.0f) / 2.0f;   /* excess I gain */
    float g_q = 1.0f - (gain_lin - 1.0f) / 2.0f;   /* reduced Q gain */
    float sin_phi = sinf(phase_deg * (float)M_PI / 180.0f);

    for (size_t i = 0; i < n; i++) {
        float r = re[i];
        re[i] = r * g_i;
        im[i] = im[i] * g_q + r * sin_phi;
    }
}

/* ========================================================================
 * Phase noise
 * ======================================================================== */

void lib80211_add_phase_noise(float *re, float *im, size_t n,
                              float strength, float sample_rate,
                              float corner_hz, lib80211_rng *rng)
{
    if (n == 0) return;

    float alpha = expf(-2.0f * (float)M_PI * corner_hz / sample_rate);
    float phi = 0.0f;

    for (size_t i = 0; i < n; i++) {
        phi = alpha * phi + strength * lib80211_rng_normal(rng);
        float cos_phi = cosf(phi);
        float sin_phi = sinf(phi);
        float r = re[i];
        float q = im[i];
        re[i] = r * cos_phi - q * sin_phi;
        im[i] = r * sin_phi + q * cos_phi;
    }
}

/* ========================================================================
 * AGC settling ramp
 * ======================================================================== */

void lib80211_add_agc_ramp(float *re, float *im, size_t n,
                           int settle_samples, float initial_gain_db)
{
    if (n == 0 || settle_samples <= 0) return;

    float initial_lin = powf(10.0f, initial_gain_db / 20.0f);
    float deficit = 1.0f - initial_lin;
    float tau = (float)settle_samples / 3.0f;  /* 3 time constants ≈ 95% */
    float inv_tau = 1.0f / tau;

    size_t ramp_end = (size_t)settle_samples;
    if (ramp_end > n) ramp_end = n;

    for (size_t i = 0; i < ramp_end; i++) {
        float gain = 1.0f - deficit * expf(-(float)i * inv_tau);
        re[i] *= gain;
        im[i] *= gain;
    }
}

/* ========================================================================
 * ADC quantization
 * ======================================================================== */

void lib80211_add_quantization(float *re, float *im, size_t n, int bits)
{
    if (n == 0 || bits <= 0) return;

    /* Find peak across I and Q */
    float peak = 0.0f;
    for (size_t i = 0; i < n; i++) {
        float ar = fabsf(re[i]);
        float ai = fabsf(im[i]);
        if (ar > peak) peak = ar;
        if (ai > peak) peak = ai;
    }
    if (peak == 0.0f) return;

    float levels = (float)(1 << (bits - 1));
    float inv_peak = 1.0f / peak;

    for (size_t i = 0; i < n; i++) {
        re[i] = roundf(re[i] * inv_peak * levels) / levels * peak;
        im[i] = roundf(im[i] * inv_peak * levels) / levels * peak;
    }
}
