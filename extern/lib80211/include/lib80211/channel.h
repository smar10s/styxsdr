#ifndef LIB80211_CHANNEL_H
#define LIB80211_CHANNEL_H

#include "lib80211/fft.h"
#include <stddef.h>

/**
 * Pilot phase tracking state (EWMA across symbols).
 * Initialize slope=0, alpha=0.3, initialized=0 before first call.
 */
typedef struct {
    float slope;        /* EWMA-smoothed phase slope (radians/subcarrier) */
    float alpha;        /* EWMA smoothing factor (0.3 typical) */
    int initialized;    /* 0 = first symbol, slope not yet initialized */
    int update_h;       /* If non-zero, apply conjugate correction to H each symbol */
} lib80211_pilot_state;

/**
 * Estimate channel from L-LTF (two 64-sample symbols).
 *
 * H[k] = avg(FFT(sym1), FFT(sym2)) * conj(LTF_ref[k]) / |LTF_ref[k]|^2
 * on active subcarriers. Non-active subcarriers get H=1+0j.
 *
 * @param plan       FFT plan
 * @param ltf_real   128 samples: first LTF sym (64) + second LTF sym (64)
 *                   NOTE: caller should pass ltf_start pointing to first FFT sym
 *                   (after GI2), so this is 128 samples total
 * @param ltf_imag   Corresponding Q samples (128 samples)
 * @param H_real     Output: 64 channel estimates (real part)
 * @param H_imag     Output: 64 channel estimates (imag part)
 * @param noise_var  Output: estimated noise variance (nullable)
 */
void lib80211_estimate_channel(lib80211_fft_plan *plan,
                               const float *ltf_real, const float *ltf_imag,
                               float *H_real, float *H_imag,
                               float *noise_var);

/**
 * MMSE equalization: eq[k] = Y[k] * conj(H[k]) / (|H[k]|^2 + noise_var)
 *
 * @param Y_real      Received frequency-domain symbol (real), 64 values
 * @param Y_imag      Received frequency-domain symbol (imag), 64 values
 * @param H_real      Channel estimate real, 64 values
 * @param H_imag      Channel estimate imag, 64 values
 * @param noise_var   Noise variance for regularization
 * @param out_real    Equalized output (real), 64 values
 * @param out_imag    Equalized output (imag), 64 values
 */
void lib80211_equalize(const float *Y_real, const float *Y_imag,
                       const float *H_real, const float *H_imag,
                       float noise_var,
                       float *out_real, float *out_imag);

/**
 * Extract one legacy OFDM DATA symbol: FFT, equalize, pilot track, extract data.
 *
 * Performs the full pipeline for one symbol:
 *   1. Skip CP (16 samples), FFT the 64-sample body
 *   2. MMSE equalization using H
 *   3. Pilot phase tracking (CPE + slope) with EWMA
 *   4. If pilot_state->update_h is set, apply conjugate rotation to H
 *      (progressive tracking for long frames)
 *   5. Extract 48 data subcarrier values
 *
 * @param plan          FFT plan
 * @param sym_real      Input: 80 time-domain samples (CP+64)
 * @param sym_imag      Input: 80 time-domain samples (CP+64)
 * @param H_real        Channel estimate (64 values, may be updated if update_h)
 * @param H_imag        Channel estimate (64 values, may be updated if update_h)
 * @param noise_var     Noise variance for MMSE
 * @param symbol_idx    Symbol index (for pilot polarity, 0=first DATA symbol)
 * @param pilot_state   Pilot EWMA state (modified in-place), can be NULL
 * @param out_real      Output: 48 equalized data subcarrier values (I)
 * @param out_imag      Output: 48 equalized data subcarrier values (Q)
 */
void lib80211_extract_legacy_symbol(lib80211_fft_plan *plan,
                                    const float *sym_real, const float *sym_imag,
                                    float *H_real, float *H_imag,
                                    float noise_var, int symbol_idx,
                                    lib80211_pilot_state *pilot_state,
                                    float *out_real, float *out_imag);

#endif /* LIB80211_CHANNEL_H */
