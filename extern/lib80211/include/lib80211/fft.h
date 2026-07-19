#ifndef LIB80211_FFT_H
#define LIB80211_FFT_H

#include <stddef.h>

/**
 * Opaque FFT plan. Holds backend-specific state (vDSP setup, twiddle
 * factors, etc.) for a complex FFT of the size specified at creation.
 */
typedef struct lib80211_fft_plan lib80211_fft_plan;

/**
 * Create a 64-point FFT plan. Convenience wrapper for
 * lib80211_fft_plan_create_n(64).
 * The plan must be destroyed with lib80211_fft_plan_destroy()
 * when no longer needed.
 *
 * This function allocates memory for the plan object (one-time setup).
 * All data-path functions (RX decode, TX encode) use caller-provided
 * scratch buffers — see lib80211/scratch.h.
 */
lib80211_fft_plan *lib80211_fft_plan_create(void);

/**
 * Create an N-point FFT plan. The wavelet daemon uses this
 * for variable-size transforms (e.g. 4096-point for waterfall).
 */
lib80211_fft_plan *lib80211_fft_plan_create_n(size_t n);

/**
 * Destroy a previously created FFT plan.
 */
void lib80211_fft_plan_destroy(lib80211_fft_plan *plan);

/**
 * Forward FFT: time-domain -> frequency-domain (plan->n-point).
 *
 * Split complex format: separate real and imaginary arrays of length plan->n.
 * No scaling applied (DFT convention: X[k] = sum x[n] * exp(-j2pi*k*n/N)).
 *
 * NOTE: The plan object is mutated (internal workspace buffers).
 * Do not call concurrently from multiple threads with the same plan.
 */
void lib80211_fft_forward(lib80211_fft_plan *plan,
                          const float *in_real, const float *in_imag,
                          float *out_real, float *out_imag);

/**
 * Inverse FFT: frequency-domain -> time-domain (plan->n-point).
 *
 * Split complex format: separate real and imaginary arrays of length plan->n.
 * Includes 1/N scaling (IDFT convention: x[n] = (1/N) sum X[k] * exp(j2pi*k*n/N)).
 *
 * NOTE: The plan object is mutated (internal workspace buffers).
 * Do not call concurrently from multiple threads with the same plan.
 */
void lib80211_fft_inverse(lib80211_fft_plan *plan,
                          const float *in_real, const float *in_imag,
                          float *out_real, float *out_imag);

#endif /* LIB80211_FFT_H */
