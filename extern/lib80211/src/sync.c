/**
 * sync.c — STF detection, LTF timing, and CFO estimation.
 *
 * Algorithm:
 * 1. Sliding lag-16 normalized autocorrelation to detect STF
 * 2. Coarse CFO from STF autocorrelation angle
 * 3. LTF timing via cross-correlation with known LTF waveform
 * 4. Fine CFO from lag-64 autocorrelation of two LTF symbols
 */

#include "lib80211/sync.h"
#include "lib80211/fft.h"
#include "lib80211/constants.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* STF autocorrelation parameters */
#define STF_LAG         16
#define STF_WINDOW      64
#define STF_THRESHOLD   0.5f
#define STF_MIN_PERIODS 4
#define STF_DUR_THRESH  0.42f

/* ========================================================================
 * Internal: STF detection
 * ======================================================================== */

/**
 * Compute normalized autocorrelation at given lag/window for one position.
 * Returns |P| / sqrt(E1*E2) where P = sum(x[n]*conj(x[n+lag])).
 */
static float autocorr_metric(const float *re, const float *im,
                             size_t pos, int lag, int window)
{
    float pr = 0.0f, pi = 0.0f;
    float e1 = 0.0f, e2 = 0.0f;

    for (int k = 0; k < window; k++) {
        float r1 = re[pos + k];
        float i1 = im[pos + k];
        float r2 = re[pos + k + lag];
        float i2 = im[pos + k + lag];

        /* x1 * conj(x2) = (r1*r2 + i1*i2) + j*(i1*r2 - r1*i2) */
        pr += r1 * r2 + i1 * i2;
        pi += i1 * r2 - r1 * i2;

        e1 += r1 * r1 + i1 * i1;
        e2 += r2 * r2 + i2 * i2;
    }

    float denom = sqrtf(e1 * e2);
    if (denom < 1e-12f) return 0.0f;

    float p_mag = sqrtf(pr * pr + pi * pi);
    return p_mag / denom;
}

/**
 * Compute complex autocorrelation at given lag/window (returns real/imag parts).
 */
static void autocorr_complex(const float *re, const float *im,
                             size_t pos, int lag, int window,
                             float *out_re, float *out_im)
{
    float pr = 0.0f, pi = 0.0f;

    for (int k = 0; k < window; k++) {
        float r1 = re[pos + k];
        float i1 = im[pos + k];
        float r2 = re[pos + k + lag];
        float i2 = im[pos + k + lag];

        pr += r1 * r2 + i1 * i2;
        pi += i1 * r2 - r1 * i2;
    }

    *out_re = pr;
    *out_im = pi;
}

/**
 * DC rejection: verify that the candidate signal is AC-dominated (not just a
 * DC offset sitting in noise). Computes ratio of AC power to total power over
 * the initial-detection window. Real STF is zero-mean by design (no DC subcarrier),
 * so AC/total ≈ 1.0. A DC-offset region has AC/total ≈ noise/(DC²+noise) which
 * is typically < 0.5.
 */
static int dc_reject_check(const float *re, const float *im,
                           size_t pos, size_t n_samples)
{
    int len = STF_WINDOW;
    if (pos + (size_t)len > n_samples) return 0;

    /* Compute mean (DC component) */
    float sum_r = 0.0f, sum_i = 0.0f;
    for (int k = 0; k < len; k++) {
        sum_r += re[pos + k];
        sum_i += im[pos + k];
    }
    float mean_r = sum_r / (float)len;
    float mean_i = sum_i / (float)len;

    /* Compute total power and AC power (= total - DC) */
    float total_power = 0.0f;
    float ac_power = 0.0f;
    for (int k = 0; k < len; k++) {
        float r = re[pos + k];
        float i = im[pos + k];
        total_power += r * r + i * i;
        float ar = r - mean_r;
        float ai = i - mean_i;
        ac_power += ar * ar + ai * ai;
    }

    if (total_power < 1e-20f) return 0;

    /* Real STF: AC/total ≈ 0.98-1.0.  DC+noise: ≈ 0.3-0.5.
     * Threshold at 0.7 gives robust separation. */
    return ac_power >= 0.7f * total_power;
}

/**
 * Duration check: verify consecutive high-autocorrelation periods starting at pos.
 * Returns 1 if >= STF_MIN_PERIODS consecutive periods pass, 0 otherwise.
 * Also rejects DC-only signals via AC power ratio check.
 */
static int stf_duration_check(const float *re, const float *im,
                              size_t n_samples, size_t pos)
{
    int consec = 0;
    for (size_t n = pos; n + STF_LAG + STF_LAG <= n_samples; n += STF_LAG) {
        float m = autocorr_metric(re, im, n, STF_LAG, STF_LAG);
        if (m >= STF_DUR_THRESH) {
            consec++;
            if (consec >= STF_MIN_PERIODS) {
                /* Verify this isn't just DC — check modulation content */
                if (dc_reject_check(re, im, pos, n_samples))
                    return 1;
                /* DC-like signal: keep scanning */
                return 0;
            }
        } else {
            break;
        }
    }
    return 0;
}

/**
 * Detect STF start using sliding normalized autocorrelation.
 * Uses cumulative-sum sliding window and squared threshold comparison
 * to avoid sqrt/division at every sample. Only computes full metric
 * at candidate positions.
 * Returns index of first sample of detected STF, or -1.
 */
static int detect_stf(const float *re, const float *im, size_t n_samples)
{
    if (n_samples < (size_t)(STF_WINDOW + STF_LAG))
        return -1;

    size_t max_pos = n_samples - STF_WINDOW - STF_LAG;

    /* Squared threshold for cheap comparison: |P|^2 >= thresh^2 * E1 * E2 */
    const float thresh_sq = STF_THRESHOLD * STF_THRESHOLD;

    /* Initialize sliding accumulators at position 0 */
    float pr = 0.0f, pi_acc = 0.0f;
    float e1 = 0.0f, e2 = 0.0f;

    for (int k = 0; k < STF_WINDOW; k++) {
        float r1 = re[k], i1 = im[k];
        float r2 = re[k + STF_LAG], i2 = im[k + STF_LAG];
        pr += r1 * r2 + i1 * i2;
        pi_acc += i1 * r2 - r1 * i2;
        e1 += r1 * r1 + i1 * i1;
        e2 += r2 * r2 + i2 * i2;
    }

    for (size_t n = 0; n <= max_pos; n++) {
        /* Cheap squared comparison — no sqrt or division */
        float p_sq = pr * pr + pi_acc * pi_acc;
        float e_prod = e1 * e2;

        if (p_sq >= thresh_sq * e_prod && e_prod > 1e-20f) {
            /* Candidate: do duration check */
            if (stf_duration_check(re, im, n_samples, n))
                return (int)n;
        }

        /* Slide window: remove sample at n, add sample at n+STF_WINDOW */
        if (n < max_pos) {
            float r1_old = re[n], i1_old = im[n];
            float r2_old = re[n + STF_LAG], i2_old = im[n + STF_LAG];
            pr -= r1_old * r2_old + i1_old * i2_old;
            pi_acc -= i1_old * r2_old - r1_old * i2_old;
            e1 -= r1_old * r1_old + i1_old * i1_old;
            e2 -= r2_old * r2_old + i2_old * i2_old;

            size_t new_idx = n + STF_WINDOW;
            float r1_new = re[new_idx], i1_new = im[new_idx];
            float r2_new = re[new_idx + STF_LAG], i2_new = im[new_idx + STF_LAG];
            pr += r1_new * r2_new + i1_new * i2_new;
            pi_acc += i1_new * r2_new - r1_new * i2_new;
            e1 += r1_new * r1_new + i1_new * i1_new;
            e2 += r2_new * r2_new + i2_new * i2_new;
        }
    }

    return -1;
}

/* ========================================================================
 * Internal: Coarse CFO from STF
 * ======================================================================== */

static float estimate_coarse_cfo(const float *re, const float *im, size_t stf_start)
{
    /* Skip first 3 periods (48 samples) for ramp-up, use 96 samples (6 periods) */
    size_t offset = stf_start + 48;
    float cr, ci;
    autocorr_complex(re, im, offset, STF_LAG, 96, &cr, &ci);
    /* autocorr angle = -cfo * lag, so negate to get cfo */
    return -atan2f(ci, cr) / (float)STF_LAG;
}

/* ========================================================================
 * Internal: LTF timing via cross-correlation
 * ======================================================================== */

/**
 * Generate LTF time-domain reference (64 samples) via IFFT of LTF frequency sequence.
 */
static void generate_ltf_reference(lib80211_fft_plan *plan,
                                   float *ref_re, float *ref_im)
{
    float zeros[64] = {0};
    lib80211_fft_inverse(plan, LIB80211_LTF_FREQ_REAL, zeros, ref_re, ref_im);
}

/**
 * Cross-correlation magnitude at a given offset.
 * Computes |sum(x[pos+k] * conj(ref[k]))| for k=0..63.
 */
static float xcorr_mag(const float *re, const float *im,
                       const float *ref_re, const float *ref_im,
                       size_t pos)
{
    float pr = 0.0f, pi = 0.0f;
    for (int k = 0; k < 64; k++) {
        float xr = re[pos + k];
        float xi = im[pos + k];
        float rr = ref_re[k];
        float ri = ref_im[k];

        pr += xr * rr + xi * ri;
        pi += xi * rr - xr * ri;
    }
    return sqrtf(pr * pr + pi * pi);
}

/**
 * Find LTF start (first FFT symbol, after GI2) using cross-correlation.
 * Searches around expected position: stf_start + 160 (STF) + 32 (GI2).
 */
static size_t find_ltf_start(lib80211_fft_plan *plan,
                             const float *re, const float *im,
                             size_t n_samples, size_t stf_start)
{
    float ref_re[64], ref_im[64];
    generate_ltf_reference(plan, ref_re, ref_im);

    /* Expected position of first LTF FFT symbol */
    int expected = (int)stf_start + LIB80211_STF_SAMPLES + 32;

    /* Search window: ±32 samples */
    int search_lo = expected - 32;
    int search_hi = expected + 32;
    if (search_lo < 0) search_lo = 0;
    if ((size_t)(search_hi + 64) > n_samples)
        search_hi = (int)(n_samples - 64);

    float best_val = 0.0f;
    int best_pos = expected;

    for (int pos = search_lo; pos <= search_hi; pos++) {
        float val = xcorr_mag(re, im, ref_re, ref_im, (size_t)pos);
        if (val > best_val) {
            best_val = val;
            best_pos = pos;
        }
    }

    return (size_t)best_pos;
}

/* ========================================================================
 * Internal: Fine CFO from LTF
 * ======================================================================== */

static float estimate_fine_cfo(const float *re, const float *im, size_t ltf_start)
{
    float cr, ci;
    autocorr_complex(re, im, ltf_start, 64, 64, &cr, &ci);
    /* autocorr angle = -cfo * lag, so negate to get cfo */
    return -atan2f(ci, cr) / 64.0f;
}

/* ========================================================================
 * Public API
 * ======================================================================== */

/**
 * Scratch-aware sync: writes coarse-CFO-corrected IQ into work_re/work_im.
 * Caller can reuse these buffers for subsequent processing (saves a copy).
 */
int lib80211_sync_detect_with_work(lib80211_fft_plan *plan,
                                   const float *iq_real, const float *iq_imag,
                                   size_t n_samples,
                                   float *work_re, float *work_im,
                                   lib80211_sync_result *result)
{
    if (!plan || !iq_real || !iq_imag || !result || !work_re || !work_im || n_samples < 320)
        return -1;

    /* Step 1: Detect STF */
    int stf_pos = detect_stf(iq_real, iq_imag, n_samples);
    if (stf_pos < 0)
        return -1;

    result->frame_start = (size_t)stf_pos;

    /* Step 2: Coarse CFO from STF */
    float coarse_cfo = 0.0f;
    if ((size_t)(stf_pos + 48 + 96 + STF_LAG) <= n_samples) {
        coarse_cfo = estimate_coarse_cfo(iq_real, iq_imag, (size_t)stf_pos);
    }

    /* Step 3: Apply coarse CFO correction to the provided working buffers */
    memcpy(work_re, iq_real, n_samples * sizeof(float));
    memcpy(work_im, iq_imag, n_samples * sizeof(float));

    if (fabsf(coarse_cfo) > 1e-8f) {
        lib80211_cfo_correct(work_re, work_im, n_samples, coarse_cfo);
    }

    /* Step 4: Find LTF timing on corrected signal */
    size_t ltf_pos = find_ltf_start(plan, work_re, work_im, n_samples, (size_t)stf_pos);
    result->ltf_start = ltf_pos;

    /* Step 5: Fine CFO from corrected LTF */
    float fine_cfo = 0.0f;
    if (ltf_pos + 128 <= n_samples) {
        fine_cfo = estimate_fine_cfo(work_re, work_im, ltf_pos);
    }

    /* Step 6: Combined CFO */
    result->cfo_rad = coarse_cfo + fine_cfo;
    /* Store coarse CFO so caller can apply only the fine delta */
    result->frame_start = (size_t)stf_pos;  /* already set above, kept for clarity */

    return 0;
}

int lib80211_sync_detect(lib80211_fft_plan *plan,
                         const float *iq_real, const float *iq_imag,
                         size_t n_samples, lib80211_sync_result *result)
{
    if (!plan || !iq_real || !iq_imag || !result || n_samples < 320)
        return -1;

    /* Allocate temporary work buffers */
    float *work_re = (float *)malloc(n_samples * sizeof(float));
    float *work_im = (float *)malloc(n_samples * sizeof(float));
    if (!work_re || !work_im) {
        free(work_re);
        free(work_im);
        return -1;
    }

    int rc = lib80211_sync_detect_with_work(plan, iq_real, iq_imag, n_samples,
                                            work_re, work_im, result);

    free(work_re);
    free(work_im);
    return rc;
}

void lib80211_cfo_correct(float *iq_real, float *iq_imag,
                          size_t n_samples, float cfo_rad)
{
    /* Incremental rotation: avoids precision loss from large phase values.
     * Re-normalize every 256 samples to prevent magnitude drift. */
    float c = cosf(-cfo_rad);
    float s = sinf(-cfo_rad);
    float rot_r = 1.0f, rot_i = 0.0f;

    for (size_t n = 0; n < n_samples; n++) {
        float r = iq_real[n];
        float i = iq_imag[n];
        iq_real[n] = r * rot_r - i * rot_i;
        iq_imag[n] = r * rot_i + i * rot_r;

        /* Rotate accumulator: (rot_r + j*rot_i) *= (c + j*s) */
        float new_r = rot_r * c - rot_i * s;
        float new_i = rot_r * s + rot_i * c;
        rot_r = new_r;
        rot_i = new_i;

        /* Re-normalize every 256 samples to prevent drift */
        if ((n & 0xFF) == 0xFF) {
            float inv = 1.0f / sqrtf(rot_r * rot_r + rot_i * rot_i);
            rot_r *= inv;
            rot_i *= inv;
        }
    }
}
