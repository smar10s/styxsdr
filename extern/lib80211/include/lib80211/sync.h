#ifndef LIB80211_SYNC_H
#define LIB80211_SYNC_H

#include "lib80211/fft.h"
#include <stddef.h>

typedef struct {
    size_t frame_start;     /* sample index of L-STF start */
    float cfo_rad;          /* combined CFO in radians/sample */
    size_t ltf_start;       /* sample index of first LTF FFT symbol (after GI2) */
} lib80211_sync_result;

/**
 * Detect 802.11 frame and estimate timing/CFO.
 *
 * Uses STF lag-16 autocorrelation for detection and coarse CFO,
 * LTF cross-correlation for timing, and lag-64 autocorrelation
 * for fine CFO.
 *
 * @param plan       FFT plan (used to generate LTF reference)
 * @param iq_real    Input I samples
 * @param iq_imag    Input Q samples
 * @param n_samples  Number of input samples
 * @param result     Output: sync results
 * @return 0 on success, -1 if no frame detected
 */
int lib80211_sync_detect(lib80211_fft_plan *plan,
                         const float *iq_real, const float *iq_imag,
                         size_t n_samples, lib80211_sync_result *result);

/**
 * Apply CFO correction to IQ samples in-place.
 *
 * Multiplies each sample by exp(-j * cfo_rad * n) to remove frequency offset.
 *
 * @param iq_real    I samples (modified in-place)
 * @param iq_imag    Q samples (modified in-place)
 * @param n_samples  Number of samples
 * @param cfo_rad    CFO in radians/sample (as returned by sync_detect)
 */
void lib80211_cfo_correct(float *iq_real, float *iq_imag,
                          size_t n_samples, float cfo_rad);

#endif /* LIB80211_SYNC_H */
