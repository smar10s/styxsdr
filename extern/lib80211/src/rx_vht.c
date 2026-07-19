/**
 * rx_vht.c -- VHT (802.11ac) frame decoder.
 *
 * Pipeline: VHT-SIG-A decode -> VHT-LTF channel est -> VHT-SIG-B decode
 *           -> D9 channel refinement -> VHT-DATA decode
 */

#include "rx_internal.h"
#include "lib80211/constants.h"
#include "lib80211/signal.h"
#include "lib80211/modulation.h"
#include "lib80211/interleaver.h"
#include "lib80211/fec.h"
#include "lib80211/scrambler.h"
#include "lib80211/channel.h"
#include "lib80211/ldpc.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ========================================================================
 * VHT-SIG-A decode
 * ======================================================================== */

static int decode_vhtsiga(const lib80211_rx_state *state,
                          int *mcs, bool *short_gi, bool *ldpc, int *ldpc_extra)
{
    size_t siga_offset = state->sig_offset + LIB80211_SYMBOL_LEN;

    if (siga_offset + 2 * LIB80211_SYMBOL_LEN > state->n_samples)
        return -1;

    float all_soft[96];

    for (int sym = 0; sym < 2; sym++) {
        size_t offset = siga_offset + (size_t)sym * LIB80211_SYMBOL_LEN;

        float data_re[48], data_im[48];
        lib80211_pilot_state ps = { .slope = 0.0f, .alpha = 0.3f, .initialized = 0 };
        lib80211_extract_legacy_symbol(state->plan,
                                       state->work_re + offset,
                                       state->work_im + offset,
                                       (float *)state->H_refined_real,
                                       (float *)state->H_refined_imag,
                                       state->noise_var,
                                       sym + 1, &ps,
                                       data_re, data_im);

        float sym_soft[48];

        if (sym == 0) {
            lib80211_soft_demap(data_re, data_im, sym_soft, 48, 1);
        } else {
            float derot_re[48], derot_im[48];
            for (int i = 0; i < 48; i++) {
                derot_re[i] = data_im[i];
                derot_im[i] = -data_re[i];
            }
            lib80211_soft_demap(derot_re, derot_im, sym_soft, 48, 1);
        }

        float sym_deint[48];
        lib80211_deinterleave(sym_soft, sym_deint, 48, 1);

        memcpy(all_soft + sym * 48, sym_deint, 48 * sizeof(float));
    }

    uint8_t decoded[42];
    lib80211_viterbi_decode(all_soft, decoded, 96, 42);

    *short_gi = (decoded[24] & 1) != 0;
    *ldpc = (decoded[26] & 1) != 0;
    *ldpc_extra = (decoded[27] & 1);

    *mcs = 0;
    for (int i = 0; i < 4; i++)
        *mcs |= (decoded[28 + i] & 1) << i;

    uint8_t computed_crc = lib80211_htsig_crc8(decoded, 34);

    uint8_t received_crc = 0;
    for (int i = 0; i < 8; i++)
        received_crc |= (decoded[34 + i] & 1) << (7 - i);

    uint8_t expected_crc = (uint8_t)(~computed_crc);
    if (received_crc != expected_crc)
        return -1;

    int group_id = 0;
    for (int i = 0; i < 6; i++)
        group_id |= (decoded[4 + i] & 1) << i;
    if (group_id != 0) return -1;

    if (*mcs < 0 || *mcs > 8) return -1;

    return 0;
}

/* ========================================================================
 * VHT-SIG-B decode
 * ======================================================================== */

static int decode_vhtsigb(lib80211_fft_plan *plan,
                          const float *sigb_re, const float *sigb_im,
                          const float *H_real, const float *H_imag,
                          float noise_var,
                          uint16_t *psdu_length, uint8_t *sigb_bits)
{
    float freq_real[64], freq_imag[64];
    lib80211_fft_forward(plan, sigb_re + LIB80211_NCP, sigb_im + LIB80211_NCP,
                         freq_real, freq_imag);

    float eq_real[64], eq_imag[64];
    lib80211_equalize(freq_real, freq_imag, H_real, H_imag, noise_var,
                      eq_real, eq_imag);

    float data_re[52], data_im[52];
    for (int i = 0; i < 52; i++) {
        int bin = LIB80211_HT_DATA_BINS[i];
        data_re[i] = eq_real[bin];
        data_im[i] = eq_imag[bin];
    }

    float soft[52];
    lib80211_soft_demap(data_re, data_im, soft, 52, 1);

    float deint[52];
    lib80211_deinterleave_ht(soft, deint, 52, 1);

    uint8_t decoded[20];
    lib80211_viterbi_decode(deint, decoded, 52, 20);

    int length_field = 0;
    for (int i = 0; i < 17; i++)
        length_field |= (decoded[i] & 1) << i;

    *psdu_length = (uint16_t)(length_field * 4);

    memcpy(sigb_bits, decoded, 20);
    memset(sigb_bits + 20, 0, 6);

    return 0;
}

/* ========================================================================
 * D9: VHT-SIG-B data-aided channel refinement
 * ======================================================================== */

static void refine_channel_from_sigb(lib80211_fft_plan *plan,
                                     const float *sigb_re, const float *sigb_im,
                                     float *H_real, float *H_imag,
                                     const uint8_t *sigb_bits)
{
    uint8_t full_bits[26];
    memcpy(full_bits, sigb_bits, 20);
    memset(full_bits + 20, 0, 6);

    uint8_t coded[52];
    lib80211_conv_encode(full_bits, coded, 26, false);

    uint8_t interleaved[52];
    lib80211_interleave_ht(coded, interleaved, 52, 1);

    float exp_re[52], exp_im[52];
    lib80211_modulate(interleaved, exp_re, exp_im, 52, 1);

    float Y_real[64], Y_imag[64];
    lib80211_fft_forward(plan, sigb_re + LIB80211_NCP, sigb_im + LIB80211_NCP,
                         Y_real, Y_imag);

    for (int i = 0; i < 52; i++) {
        int bin = LIB80211_HT_DATA_BINS[i];
        float h_sigb_re = Y_real[bin] * exp_re[i];
        float h_sigb_im = Y_imag[bin] * exp_re[i];

        H_real[bin] = 0.5f * H_real[bin] + 0.5f * h_sigb_re;
        H_imag[bin] = 0.5f * H_imag[bin] + 0.5f * h_sigb_im;
    }
}

/* ========================================================================
 * VHT frame decoder
 * ======================================================================== */

int lib80211_rx_decode_vht(const lib80211_rx_state *state,
                           lib80211_scratch *scratch,
                           lib80211_rx_result *result)
{
    /* Step 1: Decode VHT-SIG-A */
    int mcs;
    bool short_gi, ldpc;
    int ldpc_extra;

    if (decode_vhtsiga(state, &mcs, &short_gi, &ldpc, &ldpc_extra) != 0)
        return -1;

    const lib80211_ht_mcs_info *mcs_info = &LIB80211_VHT_MCS_TABLE[mcs];

    result->type = LIB80211_FRAME_VHT;
    result->mcs = mcs;
    result->rate_mbps = 0;
    result->short_gi = short_gi;
    result->ldpc = ldpc;

    int n_dbps = mcs_info->n_dbps;
    int n_cbps = mcs_info->n_cbps;
    int n_bpsc = mcs_info->n_bpsc;

    /* Preamble offsets */
    size_t siga_offset = state->sig_offset + LIB80211_SYMBOL_LEN;
    size_t vht_stf_offset = siga_offset + 2 * LIB80211_SYMBOL_LEN;
    size_t vht_ltf_offset = vht_stf_offset + LIB80211_SYMBOL_LEN;
    size_t vht_sigb_offset = vht_ltf_offset + LIB80211_SYMBOL_LEN;
    size_t vht_data_start = vht_sigb_offset + LIB80211_SYMBOL_LEN;

    /* Step 2: VHT-LTF channel estimation */
    float H_vht_real[64], H_vht_imag[64];
    float vht_noise_var;

    if (vht_ltf_offset + LIB80211_SYMBOL_LEN > state->n_samples)
        return -1;

    lib80211_ht_vht_channel_estimate(state->plan,
                         state->work_re + vht_ltf_offset,
                         state->work_im + vht_ltf_offset,
                         H_vht_real, H_vht_imag, &vht_noise_var);

    if (state->noise_var > vht_noise_var)
        vht_noise_var = state->noise_var;

    /* Step 3: Decode VHT-SIG-B */
    if (vht_sigb_offset + LIB80211_SYMBOL_LEN > state->n_samples)
        return -1;

    uint16_t sigb_psdu_length;
    uint8_t sigb_bits[26];

    if (decode_vhtsigb(state->plan,
                       state->work_re + vht_sigb_offset,
                       state->work_im + vht_sigb_offset,
                       H_vht_real, H_vht_imag, vht_noise_var,
                       &sigb_psdu_length, sigb_bits) != 0)
        return -1;

    /* Step 4: D9 channel refinement from VHT-SIG-B */
    refine_channel_from_sigb(state->plan,
                             state->work_re + vht_sigb_offset,
                             state->work_im + vht_sigb_offset,
                             H_vht_real, H_vht_imag, sigb_bits);

    /* Compute number of DATA symbols */
    int n_symbols;
    uint16_t vht_length = sigb_psdu_length;

    if (ldpc) {
        n_symbols = ((16 + 8 * (int)vht_length) + n_dbps - 1) / n_dbps;

        int n_pld_test = n_symbols * n_dbps;
        int l_cw_test = 1944, n_cw_test = 1;
        int k_per_cw_test = l_cw_test * mcs_info->cr_n / mcs_info->cr_d;

        if (n_pld_test <= 648 * mcs_info->cr_n / mcs_info->cr_d) {
            l_cw_test = 648;
        } else if (n_pld_test <= 1296 * mcs_info->cr_n / mcs_info->cr_d) {
            l_cw_test = 1296;
        } else if (n_pld_test <= 1944 * mcs_info->cr_n / mcs_info->cr_d) {
            l_cw_test = 1944;
        } else {
            l_cw_test = 1944;
            k_per_cw_test = l_cw_test * mcs_info->cr_n / mcs_info->cr_d;
            n_cw_test = (n_pld_test + k_per_cw_test - 1) / k_per_cw_test;
        }
        k_per_cw_test = l_cw_test * mcs_info->cr_n / mcs_info->cr_d;
        int n_avail_test = n_symbols * n_cbps;
        int n_shrt_test = n_cw_test * k_per_cw_test - n_pld_test;
        if (n_shrt_test < 0) n_shrt_test = 0;
        int n_punc_test = n_cw_test * l_cw_test - n_avail_test - n_shrt_test;
        if (n_punc_test < 0) n_punc_test = 0;
        int n_parity_total = n_cw_test * (l_cw_test - k_per_cw_test);

        if (n_parity_total > 0 && n_punc_test * 10 > n_parity_total) {
            n_symbols += 1;
        }

        if (ldpc_extra && n_symbols == ((16 + 8 * (int)vht_length) + n_dbps - 1) / n_dbps) {
            n_symbols += 1;
        }
    } else {
        n_symbols = ((16 + 8 * (int)vht_length + 6) + n_dbps - 1) / n_dbps;
    }
    result->n_symbols = n_symbols;

    int ncp = short_gi ? LIB80211_NCP_SHORT : LIB80211_NCP;
    int sym_len = LIB80211_NFFT + ncp;

    if (vht_data_start + (size_t)n_symbols * (size_t)sym_len > state->n_samples)
        return -1;

    /* Step 5: Extract and decode DATA symbols */
    if (ldpc) {
        /* LDPC path */
        size_t total_soft = (size_t)n_symbols * (size_t)n_cbps;
        float *all_soft = (float *)lib80211_scratch_alloc(scratch, total_soft * sizeof(float), 0);
        if (!all_soft) return -1;

        lib80211_pilot_state data_pilot = {
            .slope = 0.0f, .alpha = 0.3f, .initialized = 0,
            .update_h = (n_symbols > 3)
        };
        float sym_data_re[52], sym_data_im[52];
        float sym_soft[416];

        for (int i = 0; i < n_symbols; i++) {
            size_t offset = vht_data_start + (size_t)i * (size_t)sym_len;

            lib80211_extract_ht_vht_symbol(state->plan,
                               state->work_re + offset,
                               state->work_im + offset,
                               H_vht_real, H_vht_imag,
                               vht_noise_var, i, ncp, LIB80211_VHT_PILOT_Z_START,
                               &data_pilot,
                               sym_data_re, sym_data_im);

            lib80211_soft_demap(sym_data_re, sym_data_im, sym_soft,
                                LIB80211_N_HT_DATA_SC, n_bpsc);

            memcpy(all_soft + (size_t)i * (size_t)n_cbps, sym_soft,
                   (size_t)n_cbps * sizeof(float));
        }

        /* Negate LLRs */
        for (size_t i = 0; i < total_soft; i++)
            all_soft[i] = -all_soft[i];

        int n_pld = n_symbols * n_dbps;
        uint8_t *decoded_bits = (uint8_t *)lib80211_scratch_alloc(scratch, (size_t)n_pld, 0);
        if (!decoded_bits) return -1;

        int rc = lib80211_ldpc_decode_data(all_soft, total_soft, decoded_bits,
                                           n_symbols, n_dbps, n_cbps,
                                           mcs_info->cr_n, mcs_info->cr_d);
        if (rc < 0) return -1;

        /* VHT descramble: SERVICE + PSDU only (no tail) */
        int n_payload = 16 + 8 * (int)vht_length;
        if (n_payload > n_pld) n_payload = n_pld;

        uint8_t seed = lib80211_detect_scrambler_seed(decoded_bits);
        lib80211_scramble(decoded_bits, decoded_bits, (size_t)n_payload, seed);

        if (vht_length > 4095 || 16 + 8 * (int)vht_length > n_pld)
            return -1;

        result->psdu_len = vht_length;
        lib80211_bits_to_bytes_lsb(decoded_bits + 16, result->psdu, vht_length);
        result->fcs_valid = lib80211_fcs_check(result->psdu, vht_length);

        return 0;
    }

    /* BCC path */
    {
        size_t total_soft = (size_t)n_symbols * (size_t)n_cbps;
        float *all_soft = (float *)lib80211_scratch_alloc(scratch, total_soft * sizeof(float), 0);
        if (!all_soft) return -1;

        /* Per-symbol buffers on stack (max 416 floats = 1664 bytes each) */
        float sym_data_re[52], sym_data_im[52];
        float sym_soft[416];
        float sym_deint[416];

        lib80211_pilot_state data_pilot = {
            .slope = 0.0f, .alpha = 0.3f, .initialized = 0,
            .update_h = (n_symbols > 3)
        };

        for (int i = 0; i < n_symbols; i++) {
            size_t offset = vht_data_start + (size_t)i * (size_t)sym_len;

            lib80211_extract_ht_vht_symbol(state->plan,
                               state->work_re + offset,
                               state->work_im + offset,
                               H_vht_real, H_vht_imag,
                               vht_noise_var, i, ncp, LIB80211_VHT_PILOT_Z_START,
                               &data_pilot,
                               sym_data_re, sym_data_im);

            lib80211_soft_demap(sym_data_re, sym_data_im, sym_soft,
                                LIB80211_N_HT_DATA_SC, n_bpsc);

            lib80211_deinterleave_ht(sym_soft, sym_deint, n_cbps, n_bpsc);

            memcpy(all_soft + (size_t)i * (size_t)n_cbps, sym_deint,
                   (size_t)n_cbps * sizeof(float));
        }

        /* VHT BCC: scrambler applied to SERVICE+PSDU only */
        size_t scramble_len = (size_t)(16 + 8 * (int)vht_length);
        size_t n_data_bits = (size_t)n_symbols * (size_t)n_dbps;
        if (scramble_len > n_data_bits) scramble_len = n_data_bits;
        return lib80211_rx_bcc_decode(all_soft, total_soft, n_symbols, n_dbps,
                                      mcs_info->cr_n, mcs_info->cr_d,
                                      vht_length, scramble_len, scratch, result);
    }
}
