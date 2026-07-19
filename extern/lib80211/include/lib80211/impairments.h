#ifndef LIB80211_IMPAIRMENTS_H
#define LIB80211_IMPAIRMENTS_H

#include <stddef.h>
#include <stdint.h>

/* ========================================================================
 * PRNG (xoshiro256** — deterministic, platform-independent)
 * ======================================================================== */

typedef struct {
    uint64_t s[4];
} lib80211_rng;

/**
 * Seed the PRNG. Expands a single 64-bit seed into full state via splitmix64.
 */
void lib80211_rng_seed(lib80211_rng *rng, uint64_t seed);

/**
 * Generate a uniform float in [0, 1).
 */
float lib80211_rng_float(lib80211_rng *rng);

/**
 * Generate a standard normal (Gaussian) sample via Box-Muller.
 */
float lib80211_rng_normal(lib80211_rng *rng);

/* ========================================================================
 * AWGN
 * ======================================================================== */

/**
 * Add AWGN to IQ signal in-place.
 *
 * @param re      I samples (modified in-place)
 * @param im      Q samples (modified in-place)
 * @param n       Number of samples
 * @param snr_db  Target SNR in dB (signal power / noise power)
 * @param rng     PRNG state (caller-owned, for deterministic tests)
 */
void lib80211_add_awgn(float *re, float *im, size_t n,
                       float snr_db, lib80211_rng *rng);

/* ========================================================================
 * CFO (Carrier Frequency Offset)
 * ======================================================================== */

/**
 * Apply carrier frequency offset in-place.
 *
 * @param re          I samples (modified in-place)
 * @param im          Q samples (modified in-place)
 * @param n           Number of samples
 * @param cfo_hz      Frequency offset in Hz
 * @param sample_rate Sample rate in Hz (typically 20e6)
 * @param phi0        Initial phase in radians
 */
void lib80211_add_cfo(float *re, float *im, size_t n,
                      float cfo_hz, float sample_rate, float phi0);

/* ========================================================================
 * Multipath
 * ======================================================================== */

typedef struct {
    int delay;       /* delay in samples */
    float gain_re;   /* complex tap gain, real part */
    float gain_im;   /* complex tap gain, imag part */
} lib80211_multipath_tap;

/* Preset channel models (matching Python py80211 definitions) */
#define LIB80211_MULTIPATH_MILD_N 2
extern const lib80211_multipath_tap LIB80211_MULTIPATH_MILD[];

#define LIB80211_MULTIPATH_MODERATE_N 3
extern const lib80211_multipath_tap LIB80211_MULTIPATH_MODERATE[];

#define LIB80211_MULTIPATH_SEVERE_N 3
extern const lib80211_multipath_tap LIB80211_MULTIPATH_SEVERE[];

/**
 * Apply multipath channel. Requires a temporary buffer (caller provides).
 *
 * Output[n] = sum_k( tap[k].gain * input[n - tap[k].delay] )
 *
 * @param re      I samples (modified in-place)
 * @param im      Q samples (modified in-place)
 * @param n       Number of samples
 * @param taps    Array of multipath taps
 * @param n_taps  Number of taps
 * @param tmp_re  Temporary buffer (n floats, caller-owned)
 * @param tmp_im  Temporary buffer (n floats, caller-owned)
 */
void lib80211_apply_multipath(float *re, float *im, size_t n,
                              const lib80211_multipath_tap *taps, int n_taps,
                              float *tmp_re, float *tmp_im);

/* ========================================================================
 * SFO (Sampling Frequency Offset)
 * ======================================================================== */

/**
 * Apply sampling frequency offset via linear interpolation resampling.
 *
 * Models a receiver sampling at (1 + ppm*1e-6) times the TX rate.
 * Positive ppm means receiver clock is faster → more output samples.
 *
 * The output buffer must be pre-allocated by the caller. Use
 * lib80211_sfo_output_len() to compute the required size.
 *
 * @param re_in     Input I samples
 * @param im_in     Input Q samples
 * @param n_in      Number of input samples
 * @param ppm       SFO in parts-per-million (e.g., +20 or -40)
 * @param re_out    Output I samples (caller-allocated)
 * @param im_out    Output Q samples (caller-allocated)
 * @return Number of output samples written
 */
size_t lib80211_add_sfo(const float *re_in, const float *im_in, size_t n_in,
                        float ppm, float *re_out, float *im_out);

/**
 * Compute output sample count for SFO resampling.
 */
size_t lib80211_sfo_output_len(size_t n_in, float ppm);

/* ========================================================================
 * DC offset (direct-conversion receiver LO leakage)
 * ======================================================================== */

/**
 * Add fixed DC offset relative to signal RMS.
 *
 * Models LO leakage in direct-conversion receivers (PlutoSDR AD9361).
 * The offset appears at subcarrier 0 (which 802.11 nulls).
 *
 * @param re       I samples (modified in-place)
 * @param im       Q samples (modified in-place)
 * @param n        Number of samples
 * @param dc_i     DC offset on I channel (fraction of signal RMS, e.g. 0.05 = 5%)
 * @param dc_q     DC offset on Q channel (fraction of signal RMS)
 */
void lib80211_add_dc_offset(float *re, float *im, size_t n,
                            float dc_i, float dc_q);

/* ========================================================================
 * IQ imbalance (gain and phase mismatch)
 * ======================================================================== */

/**
 * Apply I/Q gain and phase imbalance in-place.
 *
 * Models analog front-end mismatch in direct-conversion receivers.
 * I' = I * (1 + g/2),  Q' = Q * (1 - g/2) + I * sin(phi)
 * where g = 10^(gain_db/20) - 1, phi = phase_deg * pi/180.
 *
 * @param re          I samples (modified in-place)
 * @param im          Q samples (modified in-place)
 * @param n           Number of samples
 * @param gain_db     I/Q gain imbalance in dB (positive = I channel excess)
 * @param phase_deg   Phase imbalance in degrees
 */
void lib80211_add_iq_imbalance(float *re, float *im, size_t n,
                               float gain_db, float phase_deg);

/* ========================================================================
 * Phase noise (AR(1) filtered random walk)
 * ======================================================================== */

/**
 * Add phase noise via AR(1) filtered random walk.
 *
 * Models oscillator phase noise. The AR(1) process:
 *   phi[n] = alpha * phi[n-1] + strength * w[n]
 * where alpha = exp(-2*pi*corner_hz/sample_rate), w ~ N(0,1).
 * Steady-state RMS ~ strength / sqrt(1 - alpha^2).
 *
 * @param re          I samples (modified in-place)
 * @param im          Q samples (modified in-place)
 * @param n           Number of samples
 * @param strength    Per-sample innovation std-dev in radians
 * @param sample_rate Sample rate in Hz (typically 20e6)
 * @param corner_hz   AR(1) corner frequency in Hz (controls correlation time)
 * @param rng         PRNG state (caller-owned)
 */
void lib80211_add_phase_noise(float *re, float *im, size_t n,
                              float strength, float sample_rate,
                              float corner_hz, lib80211_rng *rng);

/* ========================================================================
 * AGC settling ramp
 * ======================================================================== */

/**
 * Simulate AGC settling by ramping gain during the first N samples.
 *
 * Models the exponential convergence of automatic gain control.
 * gain(n) = 1 - (1 - initial_lin) * exp(-n / tau)
 * where tau = settle_samples / 3 (3 time constants ≈ 95% settled).
 *
 * @param re              I samples (modified in-place)
 * @param im              Q samples (modified in-place)
 * @param n               Number of samples
 * @param settle_samples  Number of samples for AGC to settle (e.g., 128-300)
 * @param initial_gain_db Initial gain deficit in dB (negative = attenuated start)
 */
void lib80211_add_agc_ramp(float *re, float *im, size_t n,
                           int settle_samples, float initial_gain_db);

/* ========================================================================
 * ADC quantization
 * ======================================================================== */

/**
 * Simulate ADC quantization with AGC full-scale normalization.
 *
 * Peak-normalizes, quantizes to `bits` resolution, then rescales.
 * Models a typical SDR ADC (e.g., 12-bit PlutoSDR).
 *
 * @param re       I samples (modified in-place)
 * @param im       Q samples (modified in-place)
 * @param n        Number of samples
 * @param bits     ADC resolution in bits (e.g., 12 for PlutoSDR)
 */
void lib80211_add_quantization(float *re, float *im, size_t n, int bits);

#endif /* LIB80211_IMPAIRMENTS_H */
