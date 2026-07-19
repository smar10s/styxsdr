#include "lib80211/channel.h"
#include "lib80211/constants.h"
#include "lib80211/fft.h"
#include <math.h>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

void lib80211_estimate_channel(lib80211_fft_plan *plan,
                               const float *ltf_real, const float *ltf_imag,
                               float *H_real, float *H_imag,
                               float *noise_var) {
    /* FFT each of the two LTF symbols */
    float S1_real[64], S1_imag[64];
    float S2_real[64], S2_imag[64];

    lib80211_fft_forward(plan, ltf_real, ltf_imag, S1_real, S1_imag);
    lib80211_fft_forward(plan, ltf_real + 64, ltf_imag + 64, S2_real, S2_imag);

    /* Average the two symbols and compute LS channel estimate */
    float noise_acc = 0.0f;
    int n_active = 0;

    for (int k = 0; k < 64; k++) {
        float ref = LIB80211_LTF_FREQ_REAL[k];

        if (ref > 0.5f || ref < -0.5f) {
            /* Active subcarrier: LTF_ref is ±1, so |ref|^2 = 1
             * H[k] = avg[k] * ref (since ref is real and ±1) */
            float avg_r = (S1_real[k] + S2_real[k]) * 0.5f;
            float avg_i = (S1_imag[k] + S2_imag[k]) * 0.5f;
            H_real[k] = avg_r * ref;
            H_imag[k] = avg_i * ref;

            /* Noise estimate from difference of the two symbols */
            float diff_r = S1_real[k] - S2_real[k];
            float diff_i = S1_imag[k] - S2_imag[k];
            noise_acc += diff_r * diff_r + diff_i * diff_i;
            n_active++;
        } else {
            /* Null subcarrier (DC, guard bands): set H = 1+0j */
            H_real[k] = 1.0f;
            H_imag[k] = 0.0f;
        }
    }

    if (noise_var) {
        /* noise_var = sum(|diff|^2) / (2 * n_active) */
        *noise_var = (n_active > 0) ? noise_acc / (2.0f * n_active) : 0.0f;
    }
}

#ifdef __ARM_NEON
void lib80211_equalize(const float *Y_real, const float *Y_imag,
                       const float *H_real, const float *H_imag,
                       float noise_var,
                       float *out_real, float *out_imag) {
    float32x4_t nv = vdupq_n_f32(noise_var);
    float32x4_t eps = vdupq_n_f32(1e-10f);

    for (int k = 0; k < 64; k += 4) {
        float32x4_t hr = vld1q_f32(H_real + k);
        float32x4_t hi = vld1q_f32(H_imag + k);
        float32x4_t yr = vld1q_f32(Y_real + k);
        float32x4_t yi = vld1q_f32(Y_imag + k);

        /* |H|^2 = hr*hr + hi*hi */
        float32x4_t h2 = vmlaq_f32(vmulq_f32(hr, hr), hi, hi);

        /* denom = max(|H|^2 + noise_var, eps) */
        float32x4_t denom = vmaxq_f32(vaddq_f32(h2, nv), eps);

        /* Y * conj(H) = (yr*hr + yi*hi) + j*(yi*hr - yr*hi) */
        float32x4_t num_r = vmlaq_f32(vmulq_f32(yr, hr), yi, hi);
        float32x4_t num_i = vmlsq_f32(vmulq_f32(yi, hr), yr, hi);

        /* Divide by denom */
#ifdef __aarch64__
        /* AArch64: hardware float divide */
        vst1q_f32(out_real + k, vdivq_f32(num_r, denom));
        vst1q_f32(out_imag + k, vdivq_f32(num_i, denom));
#else
        /* ARMv7: reciprocal estimate + 2 Newton steps (~24-bit precision) */
        float32x4_t inv_d = vrecpeq_f32(denom);
        inv_d = vmulq_f32(inv_d, vrecpsq_f32(denom, inv_d));
        inv_d = vmulq_f32(inv_d, vrecpsq_f32(denom, inv_d));
        vst1q_f32(out_real + k, vmulq_f32(num_r, inv_d));
        vst1q_f32(out_imag + k, vmulq_f32(num_i, inv_d));
#endif
    }
}
#else
void lib80211_equalize(const float *Y_real, const float *Y_imag,
                       const float *H_real, const float *H_imag,
                       float noise_var,
                       float *out_real, float *out_imag) {
    for (int k = 0; k < 64; k++) {
        float hr = H_real[k];
        float hi = H_imag[k];
        float yr = Y_real[k];
        float yi = Y_imag[k];

        /* |H|^2 + noise_var */
        float h_abs2 = hr * hr + hi * hi;
        float denom = h_abs2 + noise_var;
        if (denom < 1e-10f) {
            denom = 1e-10f;
        }

        /* Y * conj(H) / denom */
        out_real[k] = (yr * hr + yi * hi) / denom;
        out_imag[k] = (yi * hr - yr * hi) / denom;
    }
}
#endif

void lib80211_extract_legacy_symbol(lib80211_fft_plan *plan,
                                    const float *sym_real, const float *sym_imag,
                                    float *H_real, float *H_imag,
                                    float noise_var, int symbol_idx,
                                    lib80211_pilot_state *pilot_state,
                                    float *out_real, float *out_imag) {
    /* 1. Skip CP (16 samples), FFT the 64-sample body */
    float freq_real[64], freq_imag[64];
    lib80211_fft_forward(plan, sym_real + LIB80211_NCP, sym_imag + LIB80211_NCP,
                         freq_real, freq_imag);

    /* 2. MMSE equalization */
    float eq_real[64], eq_imag[64];
    lib80211_equalize(freq_real, freq_imag, H_real, H_imag, noise_var,
                      eq_real, eq_imag);

    /* 3. Pilot phase tracking (CPE + SFO slope) */
    if (pilot_state) {
        int polarity = LIB80211_PILOT_POLARITY[symbol_idx % 127];

        /* Pilot subcarrier indices (centered at DC, NOT FFT bins).
         * Used for slope regression: sum(pilot_sc[i] * phase_err[i]).
         * FFT-bin positions come from LIB80211_PILOT_BINS[] above. */
        static const int pilot_sc[4] = { 7, 21, -21, -7 };

        /* Compute phase error at each pilot */
        float phase_err[4];
        for (int i = 0; i < 4; i++) {
            int bin = LIB80211_PILOT_BINS[i];
            /* Expected pilot = PILOT_BASE[i] * polarity (real-valued ±1) */
            float expected = LIB80211_PILOT_BASE[i] * polarity;
            /* Multiply equalized by conj(expected): since expected is real ±1,
             * this just multiplies by expected */
            float r = eq_real[bin] * expected;
            float im = eq_imag[bin] * expected;
            phase_err[i] = atan2f(im, r);
        }

        /* CPE: mean of phase errors */
        float a = (phase_err[0] + phase_err[1] + phase_err[2] + phase_err[3]) * 0.25f;

        /* Slope: b = sum(pilot_sc * phase_err) / sum(pilot_sc^2)
         * sum(pilot_sc^2) = 49 + 441 + 441 + 49 = 980 */
        float num = 0.0f;
        for (int i = 0; i < 4; i++) {
            num += pilot_sc[i] * phase_err[i];
        }
        float b = num / 980.0f;

        /* EWMA on slope */
        if (!pilot_state->initialized) {
            pilot_state->slope = b;
            pilot_state->initialized = 1;
        } else {
            pilot_state->slope = pilot_state->alpha * b +
                                 (1.0f - pilot_state->alpha) * pilot_state->slope;
        }

        /* Apply correction to all 64 subcarriers using incremental rotation.
         * Phase at bin k: -(a + smoothed_slope * sc) where sc = k (k<32) or k-64.
         * Bins 0..31 (sc=0..31): start at -a, advance by -slope each bin.
         * Bins 32..63 (sc=-32..-1): start at -(a - 32*slope), advance by -slope.
         */
        float smoothed_slope = pilot_state->slope;
        float base_angle = -a;
        float delta = -smoothed_slope;

        /* Delta rotation per bin step (shared by both passes) */
        float delta_r = cosf(delta);
        float delta_i = sinf(delta);

        /* Pass 1: bins 0..31 (sc = 0, 1, ..., 31) */
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

        /* Pass 2: bins 32..63 (sc = -32, -31, ..., -1) */
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

        /* Progressive H update: apply conjugate rotation (+phase) to H.
         * This carries the correction forward so the next symbol's equalization
         * starts from a better channel estimate. Only active when update_h is set
         * (long frames where cumulative drift matters). */
        if (pilot_state->update_h) {
            float conj_delta_r = cosf(smoothed_slope);  /* +slope (conjugate) */
            float conj_delta_i = sinf(smoothed_slope);

            /* Pass 1: bins 0..31 (sc = 0..31) */
            float hr = cosf(a);   /* +a (conjugate of -a) */
            float hi = sinf(a);
            for (int k = 0; k < 32; k++) {
                float r = H_real[k], im = H_imag[k];
                H_real[k] = r * hr - im * hi;
                H_imag[k] = r * hi + im * hr;
                float nr2 = hr * conj_delta_r - hi * conj_delta_i;
                hi        = hr * conj_delta_i + hi * conj_delta_r;
                hr        = nr2;
            }

            /* Pass 2: bins 32..63 (sc = -32..-1) */
            float s2 = a - 32.0f * smoothed_slope;  /* conjugate of start2 */
            hr = cosf(s2);
            hi = sinf(s2);
            for (int k = 32; k < 64; k++) {
                float r = H_real[k], im = H_imag[k];
                H_real[k] = r * hr - im * hi;
                H_imag[k] = r * hi + im * hr;
                float nr2 = hr * conj_delta_r - hi * conj_delta_i;
                hi        = hr * conj_delta_i + hi * conj_delta_r;
                hr        = nr2;
            }
        }
    }

    /* 4. Extract 48 data subcarriers */
    for (int i = 0; i < 48; i++) {
        int bin = LIB80211_DATA_BINS[i];
        out_real[i] = eq_real[bin];
        out_imag[i] = eq_imag[bin];
    }
}
