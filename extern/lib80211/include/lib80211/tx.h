#ifndef LIB80211_TX_H
#define LIB80211_TX_H

#include "lib80211/fft.h"
#include "lib80211/scratch.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * Parameters for legacy 802.11a frame generation.
 */
typedef struct {
    int rate_mbps;              /* 6, 9, 12, 18, 24, 36, 48, or 54 */
    const uint8_t *psdu;       /* PSDU bytes (MAC frame including FCS) */
    size_t psdu_len;           /* PSDU length in bytes */
    uint8_t scrambler_seed;    /* 7-bit scrambler seed */
} lib80211_tx_legacy_params;

/**
 * Compute the output sample count for a legacy frame.
 * Useful for pre-allocating the output buffer.
 */
size_t lib80211_tx_legacy_samples(const lib80211_tx_legacy_params *params);

/**
 * Generate a complete legacy 802.11a frame (baseband IQ).
 *
 * Output structure: STF(160) + LTF(160) + SIGNAL(80) + DATA(n_sym * 80)
 *
 * @param plan       FFT plan
 * @param params     Frame parameters
 * @param out_real   Output I samples (must hold tx_legacy_samples() floats)
 * @param out_imag   Output Q samples
 * @return Number of output samples, or 0 on error
 */
size_t lib80211_tx_legacy(lib80211_fft_plan *plan,
                          const lib80211_tx_legacy_params *params,
                          float *out_real, float *out_imag);

/**
 * Scratch-based variant of lib80211_tx_legacy.
 * Caller provides a pre-initialized scratch allocator instead of heap.
 */
size_t lib80211_tx_legacy_s(lib80211_fft_plan *plan,
                            const lib80211_tx_legacy_params *params,
                            lib80211_scratch *scratch,
                            float *out_real, float *out_imag);

/* ========================================================================
 * HT TX (802.11n, MCS 0-7)
 * ======================================================================== */

/**
 * Parameters for HT-mixed frame generation.
 */
typedef struct {
    int mcs;                    /* HT MCS index (0-7) */
    const uint8_t *psdu;       /* PSDU bytes (MAC frame including FCS) */
    size_t psdu_len;           /* PSDU length in bytes */
    uint8_t scrambler_seed;    /* 7-bit scrambler seed */
    bool short_gi;             /* Use short guard interval (400 ns) */
    bool ldpc;                 /* Use LDPC coding (false = BCC) */
} lib80211_tx_ht_params;

/**
 * Compute the output sample count for an HT frame.
 *
 * Structure: L-STF(160) + L-LTF(160) + L-SIG(80) + HT-SIG(160)
 *          + HT-STF(80) + HT-LTF(80) + HT-DATA(n_sym * symbol_len)
 */
size_t lib80211_tx_ht_samples(const lib80211_tx_ht_params *params);

/**
 * Generate a complete HT-mixed frame (baseband IQ).
 *
 * @param plan       FFT plan
 * @param params     Frame parameters
 * @param out_real   Output I samples (must hold tx_ht_samples() floats)
 * @param out_imag   Output Q samples
 * @return Number of output samples, or 0 on error
 */
size_t lib80211_tx_ht(lib80211_fft_plan *plan,
                      const lib80211_tx_ht_params *params,
                      float *out_real, float *out_imag);

/**
 * Scratch-based variant of lib80211_tx_ht.
 * Caller provides a pre-initialized scratch allocator instead of heap.
 */
size_t lib80211_tx_ht_s(lib80211_fft_plan *plan,
                        const lib80211_tx_ht_params *params,
                        lib80211_scratch *scratch,
                        float *out_real, float *out_imag);

/* ========================================================================
 * VHT TX (802.11ac, MCS 0-8)
 * ======================================================================== */

/**
 * Parameters for VHT frame generation.
 */
typedef struct {
    int mcs;                    /* VHT MCS index (0-8) */
    const uint8_t *psdu;       /* PSDU bytes (MAC frame including FCS) */
    size_t psdu_len;           /* PSDU length in bytes */
    uint8_t scrambler_seed;    /* 7-bit scrambler seed */
    bool short_gi;             /* Use short guard interval (400 ns) */
    bool ldpc;                 /* Use LDPC coding (false = BCC) */
} lib80211_tx_vht_params;

/**
 * Compute the output sample count for a VHT frame.
 *
 * Structure: L-STF(160) + L-LTF(160) + L-SIG(80) + VHT-SIG-A(160)
 *          + VHT-STF(80) + VHT-LTF(80) + VHT-SIG-B(80) + VHT-DATA(...)
 */
size_t lib80211_tx_vht_samples(const lib80211_tx_vht_params *params);

/**
 * Generate a complete VHT frame (baseband IQ).
 *
 * @param plan       FFT plan
 * @param params     Frame parameters
 * @param out_real   Output I samples (must hold tx_vht_samples() floats)
 * @param out_imag   Output Q samples
 * @return Number of output samples, or 0 on error
 */
size_t lib80211_tx_vht(lib80211_fft_plan *plan,
                       const lib80211_tx_vht_params *params,
                       float *out_real, float *out_imag);

/**
 * Scratch-based variant of lib80211_tx_vht.
 * Caller provides a pre-initialized scratch allocator instead of heap.
 */
size_t lib80211_tx_vht_s(lib80211_fft_plan *plan,
                         const lib80211_tx_vht_params *params,
                         lib80211_scratch *scratch,
                         float *out_real, float *out_imag);

#endif /* LIB80211_TX_H */
