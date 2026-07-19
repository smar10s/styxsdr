/**
 * rx_legacy.c -- Legacy 802.11a DATA field decoder.
 *
 * Given front-end state (sync, channel est, L-SIG decoded), decodes the
 * DATA field: extract symbols -> demap -> deinterleave -> BCC decode.
 */

#include "rx_internal.h"
#include "lib80211/constants.h"
#include "lib80211/modulation.h"
#include "lib80211/interleaver.h"
#include "lib80211/fec.h"
#include "lib80211/scrambler.h"

#include <stdlib.h>
#include <string.h>

int lib80211_rx_decode_legacy(const lib80211_rx_state *state,
                              lib80211_scratch *scratch,
                              lib80211_rx_result *result)
{
    result->type = LIB80211_FRAME_LEGACY;
    result->mcs = -1;
    result->short_gi = false;
    result->ldpc = false;

    /* Look up rate info */
    const lib80211_rate_info *rate_info = lib80211_rate_lookup(state->lsig_rate_mbps);
    if (!rate_info) return -1;

    result->rate_mbps = rate_info->rate_mbps;

    int n_dbps = rate_info->n_dbps;
    int n_cbps = rate_info->n_cbps;
    int n_bpsc = rate_info->n_bpsc;
    uint16_t length = state->lsig_length;
    int n_symbols = ((16 + 8 * (int)length + 6) + n_dbps - 1) / n_dbps;
    result->n_symbols = n_symbols;

    /* DATA symbols start after SIGNAL */
    size_t data_start = state->sig_offset + LIB80211_SYMBOL_LEN;

    if (data_start + (size_t)n_symbols * LIB80211_SYMBOL_LEN > state->n_samples)
        return -1;

    /* For long frames (>3 symbols), use progressive H tracking */
    int use_h_tracking = (n_symbols > 3);
    float H_track_real[64], H_track_imag[64];
    const float *h_re = state->H_real;
    const float *h_im = state->H_imag;

    if (use_h_tracking) {
        memcpy(H_track_real, state->H_real, 64 * sizeof(float));
        memcpy(H_track_imag, state->H_imag, 64 * sizeof(float));
        h_re = H_track_real;
        h_im = H_track_imag;
    }

    /* Allocate soft bit buffer from scratch */
    size_t total_soft = (size_t)n_symbols * (size_t)n_cbps;
    float *all_soft = (float *)lib80211_scratch_alloc(scratch, total_soft * sizeof(float), 0);
    if (!all_soft) return -1;

    /* Per-symbol working buffers (stack — max 288 floats = 1152 bytes each) */
    float sym_data_re[48], sym_data_im[48];
    float sym_soft[288];
    float sym_deint[288];

    lib80211_pilot_state data_pilot = {
        .slope = 0.0f, .alpha = 0.3f, .initialized = 0,
        .update_h = use_h_tracking
    };

    for (int i = 0; i < n_symbols; i++) {
        size_t offset = data_start + (size_t)i * LIB80211_SYMBOL_LEN;

        lib80211_extract_legacy_symbol(state->plan,
                                       state->work_re + offset,
                                       state->work_im + offset,
                                       (float *)h_re, (float *)h_im,
                                       state->noise_var,
                                       i + 1, &data_pilot,
                                       sym_data_re, sym_data_im);

        lib80211_soft_demap(sym_data_re, sym_data_im, sym_soft, 48, n_bpsc);
        lib80211_deinterleave(sym_soft, sym_deint, n_cbps, n_bpsc);
        memcpy(all_soft + (size_t)i * (size_t)n_cbps, sym_deint,
               (size_t)n_cbps * sizeof(float));
    }

    /* Shared BCC decode: depuncture -> Viterbi -> descramble -> PSDU -> FCS */
    size_t n_data_bits = (size_t)n_symbols * (size_t)n_dbps;
    return lib80211_rx_bcc_decode(all_soft, total_soft, n_symbols, n_dbps,
                                  rate_info->cr_n, rate_info->cr_d,
                                  length, n_data_bits, scratch, result);
}
