#ifndef LIB80211_RX_H
#define LIB80211_RX_H

#include "lib80211/fft.h"
#include "lib80211/scratch.h"
#include "lib80211/channel.h"
#include "lib80211/constants.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ========================================================================
 * Frame types
 * ======================================================================== */

typedef enum {
    LIB80211_FRAME_LEGACY = 0,
    LIB80211_FRAME_HT     = 1,
    LIB80211_FRAME_VHT    = 2,
} lib80211_frame_type;

/* ========================================================================
 * Decode result
 * ======================================================================== */

typedef struct {
    uint8_t psdu[4095];         /* Decoded PSDU bytes */
    size_t psdu_len;            /* PSDU length in bytes */
    lib80211_frame_type type;   /* Detected frame type */
    int rate_mbps;              /* Legacy rate or 0 for HT/VHT */
    int mcs;                    /* HT/VHT MCS index (-1 for legacy) */
    bool fcs_valid;             /* FCS check passed */
    bool short_gi;              /* Short GI used */
    bool ldpc;                  /* LDPC coding used */
    int n_symbols;              /* Number of OFDM data symbols decoded */
} lib80211_rx_result;

/* ========================================================================
 * Shared RX state (passed between sync/classify/decode stages)
 * ======================================================================== */

typedef struct {
    /* FFT plan (non-owning) */
    lib80211_fft_plan *plan;

    /* Synchronization results */
    size_t ltf_start;       /* sample index of first LTF FFT symbol */
    float cfo_rad;          /* CFO in rad/sample */

    /* CFO-corrected working buffer (caller-owned via rx_decode) */
    float *work_re;
    float *work_im;
    size_t n_samples;

    /* L-LTF channel estimate (52 active subcarriers, indexed by FFT bin) */
    float H_real[64];
    float H_imag[64];
    float noise_var;

    /* L-SIG decoded values */
    size_t sig_offset;      /* sample offset of SIGNAL symbol */
    int lsig_rate_mbps;     /* L-SIG rate field */
    uint16_t lsig_length;   /* L-SIG length field */

    /* Refined channel estimate (L-LTF blended with L-SIG decision-directed).
     * Used for HT-SIG / VHT-SIG-A decode — temporally closer to those symbols. */
    float H_refined_real[64];
    float H_refined_imag[64];
} lib80211_rx_state;

/* ========================================================================
 * Public API
 * ======================================================================== */

/**
 * CRC-32 FCS check. Returns true if frame has valid FCS.
 */
bool lib80211_fcs_check(const uint8_t *frame, size_t len);

/**
 * Decode an 802.11 frame from baseband IQ (scratch-based, zero allocation).
 *
 * Primary API for allocation-free operation.  The scratch buffer must be
 * initialized and sized for at least LIB80211_SCRATCH_RX_SIZE(n_samples).
 * The scratch is reset internally at the start of each call.
 *
 * @param plan       FFT plan
 * @param iq_real    Input I samples
 * @param iq_imag    Input Q samples
 * @param n_samples  Number of input samples
 * @param scratch    Caller-owned scratch buffer
 * @param result     Output: decode results
 * @return 0 on success (frame decoded, check result->fcs_valid),
 *         -1 on failure (no frame found or critical decode error)
 */
int lib80211_rx_decode_s(lib80211_fft_plan *plan,
                         const float *iq_real, const float *iq_imag,
                         size_t n_samples, lib80211_scratch *scratch,
                         lib80211_rx_result *result);

/**
 * Decode an 802.11 frame from baseband IQ.
 *
 * Convenience wrapper that allocates scratch internally.
 * For allocation-free operation, use lib80211_rx_decode_s() instead.
 *
 * @param plan       FFT plan
 * @param iq_real    Input I samples
 * @param iq_imag    Input Q samples
 * @param n_samples  Number of input samples
 * @param result     Output: decode results
 * @return 0 on success (frame decoded, check result->fcs_valid),
 *         -1 on failure (no frame found or critical decode error)
 */
int lib80211_rx_decode(lib80211_fft_plan *plan,
                       const float *iq_real, const float *iq_imag,
                       size_t n_samples, lib80211_rx_result *result);

#endif /* LIB80211_RX_H */
