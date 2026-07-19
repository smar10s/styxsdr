#ifndef LIB80211_RX_INTERNAL_H
#define LIB80211_RX_INTERNAL_H

/**
 * rx_internal.h — Internal RX pipeline functions shared between
 * rx_common.c, rx_legacy.c, rx_ht.c, and rx_vht.c.
 *
 * These are NOT part of the public API and may change without notice.
 */

#include "lib80211/rx.h"
#include "lib80211/scratch.h"
#include "lib80211/channel.h"
#include "lib80211/constants.h"

#include <math.h>

/**
 * Detect scrambler seed from first 7 decoded bits (SERVICE field).
 * Returns seed (1-127), or 0x5D as fallback.
 */
uint8_t lib80211_detect_scrambler_seed(const uint8_t *decoded_bits);

/**
 * Convert bit array (one bit per byte, LSB first) to packed bytes.
 */
void lib80211_bits_to_bytes_lsb(const uint8_t *bits, uint8_t *bytes, size_t n_bytes);

/**
 * L-SIG rate code to rate_info lookup. Returns NULL if not found.
 */
const lib80211_rate_info *lib80211_rate_lookup_by_code(uint8_t rate_code);

/**
 * Perform initial RX stages: sync, CFO correct, channel estimate, L-SIG decode.
 * Populates rx_state for subsequent classification and decode.
 * Scratch-aware version: allocates work buffers from scratch.
 *
 * @param plan       FFT plan
 * @param iq_real    Input I samples (raw)
 * @param iq_imag    Input Q samples (raw)
 * @param n_samples  Number of samples
 * @param scratch    Scratch allocator (must have space for 2*n_samples floats + overhead)
 * @param state      Output: populated RX state
 * @return 0 on success, -1 on failure
 */
int lib80211_rx_front_end_s(lib80211_fft_plan *plan,
                            const float *iq_real, const float *iq_imag,
                            size_t n_samples, lib80211_scratch *scratch,
                            lib80211_rx_state *state);

/**
 * Legacy front-end (allocates internally). Convenience wrapper.
 */
int lib80211_rx_front_end(lib80211_fft_plan *plan,
                          const float *iq_real, const float *iq_imag,
                          size_t n_samples, lib80211_rx_state *state);

/**
 * Classify frame type based on post-L-SIG symbol modulation.
 *
 * Examines the first symbol after L-SIG to detect HT (Q-BPSK) or VHT (BPSK).
 * If L-SIG rate != 6 Mbps, returns LEGACY immediately.
 *
 * @param state      RX state from lib80211_rx_front_end
 * @return Detected frame type
 */
lib80211_frame_type lib80211_rx_classify(const lib80211_rx_state *state);

/**
 * Decode legacy DATA field given front-end state.
 */
int lib80211_rx_decode_legacy(const lib80211_rx_state *state,
                              lib80211_scratch *scratch,
                              lib80211_rx_result *result);

/**
 * Decode HT frame given front-end state.
 */
int lib80211_rx_decode_ht(const lib80211_rx_state *state,
                          lib80211_scratch *scratch,
                          lib80211_rx_result *result);

/**
 * Decode VHT frame given front-end state.
 */
int lib80211_rx_decode_vht(const lib80211_rx_state *state,
                           lib80211_scratch *scratch,
                           lib80211_rx_result *result);

/* ========================================================================
 * Shared HT/VHT helpers
 * ======================================================================== */

/**
 * HT/VHT-LTF channel estimation (56 active subcarriers).
 *
 * Both HT-LTF and VHT-LTF use the same frequency-domain sequence
 * (LIB80211_HT_LTF_FREQ_REAL), so this function handles both.
 *
 * @param plan       FFT plan
 * @param ltf_real   Time-domain LTF symbol (80 samples, CP16 + 64 FFT)
 * @param ltf_imag   Time-domain LTF symbol Q
 * @param H_real     Output: 64 channel estimates (real)
 * @param H_imag     Output: 64 channel estimates (imag)
 * @param noise_var  Output: estimated noise variance (nullable)
 */
static inline void lib80211_ht_vht_channel_estimate(
    lib80211_fft_plan *plan,
    const float *ltf_real, const float *ltf_imag,
    float *H_real, float *H_imag, float *noise_var)
{
    float Y_real[64], Y_imag[64];
    lib80211_fft_forward(plan, ltf_real + LIB80211_NCP, ltf_imag + LIB80211_NCP,
                         Y_real, Y_imag);

    for (int k = 0; k < 64; k++) {
        float ref = LIB80211_HT_LTF_FREQ_REAL[k];

        if (ref > 0.5f || ref < -0.5f) {
            H_real[k] = Y_real[k] * ref;
            H_imag[k] = Y_imag[k] * ref;
        } else {
            H_real[k] = 1.0f;
            H_imag[k] = 0.0f;
        }
    }

    if (noise_var) {
        float diff_acc = 0.0f;
        int n_pairs = 0;
        for (int k = 1; k < 64; k++) {
            if ((H_real[k] == 1.0f && H_imag[k] == 0.0f) ||
                (H_real[k-1] == 1.0f && H_imag[k-1] == 0.0f))
                continue;
            float dr = H_real[k] - H_real[k-1];
            float di = H_imag[k] - H_imag[k-1];
            diff_acc += dr * dr + di * di;
            n_pairs++;
        }
        *noise_var = (n_pairs > 0) ? diff_acc / (2.0f * n_pairs) : 1e-4f;
    }
}

/**
 * Extract one HT/VHT OFDM DATA symbol: FFT, equalize, pilot track, extract 52 data sc.
 *
 * @param plan          FFT plan
 * @param sym_real      Time-domain symbol (ncp + 64 samples)
 * @param sym_imag      Time-domain symbol Q
 * @param H_real        Channel estimate (64 bins, may be updated if update_h)
 * @param H_imag        Channel estimate (64 bins, may be updated if update_h)
 * @param noise_var     Noise variance for MMSE
 * @param symbol_idx    DATA symbol index (0-based)
 * @param ncp           Cyclic prefix length (16 or 8)
 * @param z_start       Pilot polarity z_start (3 for HT, 4 for VHT)
 * @param pilot_state   Pilot EWMA state (modified in-place), can be NULL
 * @param out_real      Output: 52 equalized data subcarrier values (I)
 * @param out_imag      Output: 52 equalized data subcarrier values (Q)
 */
static inline void lib80211_extract_ht_vht_symbol(
    lib80211_fft_plan *plan,
    const float *sym_real, const float *sym_imag,
    float *H_real, float *H_imag,
    float noise_var, int symbol_idx, int ncp, int z_start,
    lib80211_pilot_state *pilot_state,
    float *out_real, float *out_imag)
{
    /* FFT the symbol body (skip CP) */
    float freq_real[64], freq_imag[64];
    lib80211_fft_forward(plan, sym_real + ncp, sym_imag + ncp,
                         freq_real, freq_imag);

    /* MMSE equalization */
    float eq_real[64], eq_imag[64];
    lib80211_equalize(freq_real, freq_imag, H_real, H_imag, noise_var,
                      eq_real, eq_imag);

    /* Pilot phase tracking */
    if (pilot_state) {
        int pol_idx = (z_start + symbol_idx) % 127;
        int polarity = LIB80211_PILOT_POLARITY[pol_idx];

        static const int pilot_sc[4] = { -21, -7, 7, 21 };
        float phase_err[4];

        for (int k = 0; k < 4; k++) {
            int bin = LIB80211_HT_PILOT_BINS[k];
            float expected = polarity * LIB80211_HT_PILOT_PATTERN[(symbol_idx + k) % 4];
            float r = eq_real[bin] * expected;
            float im = eq_imag[bin] * expected;
            phase_err[k] = atan2f(im, r);
        }

        /* CPE: mean */
        float a = (phase_err[0] + phase_err[1] + phase_err[2] + phase_err[3]) * 0.25f;

        /* Slope: b = sum(sc * phase) / sum(sc^2) = ... / 980 */
        float num = 0.0f;
        for (int k = 0; k < 4; k++)
            num += pilot_sc[k] * phase_err[k];
        float b = num / 980.0f;

        /* EWMA */
        if (!pilot_state->initialized) {
            pilot_state->slope = b;
            pilot_state->initialized = 1;
        } else {
            pilot_state->slope = pilot_state->alpha * b +
                                 (1.0f - pilot_state->alpha) * pilot_state->slope;
        }

        /* Apply correction using incremental rotation (6 trig calls vs 128) */
        float smoothed_slope = pilot_state->slope;
        float base_angle = -a;
        float delta = -smoothed_slope;

        float delta_r = cosf(delta);
        float delta_i = sinf(delta);

        /* Pass 1: bins 0..31 (sc = 0..31) */
        float rot_r = cosf(base_angle);
        float rot_i = sinf(base_angle);
        for (int k = 0; k < 32; k++) {
            float r = eq_real[k], im = eq_imag[k];
            eq_real[k] = r * rot_r - im * rot_i;
            eq_imag[k] = r * rot_i + im * rot_r;
            float nr = rot_r * delta_r - rot_i * delta_i;
            rot_i     = rot_r * delta_i + rot_i * delta_r;
            rot_r     = nr;
        }

        /* Pass 2: bins 32..63 (sc = -32..-1) */
        float start2 = -(a - 32.0f * smoothed_slope);
        rot_r = cosf(start2);
        rot_i = sinf(start2);
        for (int k = 32; k < 64; k++) {
            float r = eq_real[k], im = eq_imag[k];
            eq_real[k] = r * rot_r - im * rot_i;
            eq_imag[k] = r * rot_i + im * rot_r;
            float nr = rot_r * delta_r - rot_i * delta_i;
            rot_i     = rot_r * delta_i + rot_i * delta_r;
            rot_r     = nr;
        }

        /* Progressive H update: apply conjugate rotation to H for next symbol */
        if (pilot_state->update_h) {
            float conj_delta_r = cosf(smoothed_slope);
            float conj_delta_i = sinf(smoothed_slope);

            float hr = cosf(a);
            float hi = sinf(a);
            for (int k = 0; k < 32; k++) {
                float r2 = H_real[k], im2 = H_imag[k];
                H_real[k] = r2 * hr - im2 * hi;
                H_imag[k] = r2 * hi + im2 * hr;
                float nr2 = hr * conj_delta_r - hi * conj_delta_i;
                hi        = hr * conj_delta_i + hi * conj_delta_r;
                hr        = nr2;
            }

            float s2h = a - 32.0f * smoothed_slope;
            hr = cosf(s2h);
            hi = sinf(s2h);
            for (int k = 32; k < 64; k++) {
                float r2 = H_real[k], im2 = H_imag[k];
                H_real[k] = r2 * hr - im2 * hi;
                H_imag[k] = r2 * hi + im2 * hr;
                float nr2 = hr * conj_delta_r - hi * conj_delta_i;
                hi        = hr * conj_delta_i + hi * conj_delta_r;
                hr        = nr2;
            }
        }
    }

    /* Extract 52 HT/VHT data subcarriers */
    for (int i = 0; i < LIB80211_N_HT_DATA_SC; i++) {
        int bin = LIB80211_HT_DATA_BINS[i];
        out_real[i] = eq_real[bin];
        out_imag[i] = eq_imag[bin];
    }
}

/**
 * BCC decode pipeline: depuncture -> Viterbi -> descramble -> extract PSDU -> FCS.
 *
 * Shared by rx_legacy.c, rx_ht.c, and rx_vht.c (BCC paths).
 *
 * @param all_soft       Deinterleaved soft bits (total_soft floats, scratch-allocated)
 * @param total_soft     Length of all_soft
 * @param n_symbols      Number of OFDM data symbols
 * @param n_dbps         Data bits per symbol
 * @param cr_n           Code rate numerator
 * @param cr_d           Code rate denominator
 * @param psdu_length    Expected PSDU length in bytes
 * @param scramble_len   Number of bits to descramble (n_data_bits for legacy/HT,
 *                       16+8*psdu_length for VHT)
 * @param scratch        Scratch allocator for working buffers
 * @param result         Output: psdu, psdu_len, fcs_valid populated
 * @return 0 on success, -1 on failure
 */
int lib80211_rx_bcc_decode(float *all_soft, size_t total_soft,
                           int n_symbols, int n_dbps,
                           int cr_n, int cr_d,
                           uint16_t psdu_length, size_t scramble_len,
                           lib80211_scratch *scratch,
                           lib80211_rx_result *result);

#endif /* LIB80211_RX_INTERNAL_H */
