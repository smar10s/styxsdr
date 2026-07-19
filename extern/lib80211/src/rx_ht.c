/**
 * rx_ht.c -- HT-mixed (802.11n) frame decoder.
 *
 * Pipeline: HT-SIG decode -> HT-LTF channel est -> HT-DATA decode
 *   (extract symbols, demap, deinterleave, BCC/LDPC decode)
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
 * HT-SIG decode
 * ======================================================================== */

static int decode_htsig(const lib80211_rx_state *state,
                        int *mcs, uint16_t *ht_length,
                        bool *short_gi, bool *ldpc)
{
    size_t htsig_offset = state->sig_offset + LIB80211_SYMBOL_LEN;

    if (htsig_offset + 2 * LIB80211_SYMBOL_LEN > state->n_samples)
        return -1;

    float all_soft[96];

    for (int sym = 0; sym < 2; sym++) {
        size_t offset = htsig_offset + (size_t)sym * LIB80211_SYMBOL_LEN;

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

        float derot_re[48], derot_im[48];
        for (int i = 0; i < 48; i++) {
            derot_re[i] = data_im[i];
            derot_im[i] = -data_re[i];
        }

        float sym_soft[48];
        lib80211_soft_demap(derot_re, derot_im, sym_soft, 48, 1);

        float sym_deint[48];
        lib80211_deinterleave(sym_soft, sym_deint, 48, 1);

        memcpy(all_soft + sym * 48, sym_deint, 48 * sizeof(float));
    }

    uint8_t decoded[42];
    lib80211_viterbi_decode(all_soft, decoded, 96, 42);

    *mcs = 0;
    for (int i = 0; i < 7; i++)
        *mcs |= (decoded[i] & 1) << i;

    *ht_length = 0;
    for (int i = 8; i <= 23; i++)
        *ht_length |= (uint16_t)(decoded[i] & 1) << (i - 8);

    *ldpc = (decoded[30] & 1) != 0;
    *short_gi = (decoded[31] & 1) != 0;

    uint8_t computed_crc = lib80211_htsig_crc8(decoded, 34);

    uint8_t received_crc = 0;
    for (int i = 0; i < 8; i++)
        received_crc |= (decoded[34 + i] & 1) << (7 - i);

    uint8_t expected_crc = (uint8_t)(~computed_crc);
    if (received_crc != expected_crc)
        return -1;

    if (*mcs < 0 || *mcs > 7) return -1;

    return 0;
}

/* ========================================================================
 * HT frame decoder (BCC + LDPC paths)
 * ======================================================================== */

int lib80211_rx_decode_ht(const lib80211_rx_state *state,
                          lib80211_scratch *scratch,
                          lib80211_rx_result *result)
{
    /* Step 1: Decode HT-SIG */
    int mcs;
    uint16_t ht_length;
    bool short_gi, ldpc;

    if (decode_htsig(state, &mcs, &ht_length, &short_gi, &ldpc) != 0)
        return -1;

    const lib80211_ht_mcs_info *mcs_info = &LIB80211_HT_MCS_TABLE[mcs];

    result->type = LIB80211_FRAME_HT;
    result->mcs = mcs;
    result->rate_mbps = 0;
    result->short_gi = short_gi;
    result->ldpc = ldpc;

    int n_dbps = mcs_info->n_dbps;
    int n_cbps = mcs_info->n_cbps;
    int n_bpsc = mcs_info->n_bpsc;

    /* Number of DATA symbols */
    int n_symbols;
    if (ldpc) {
        n_symbols = ((16 + 8 * (int)ht_length) + n_dbps - 1) / n_dbps;

        /* Check for LDPC extra symbol (§19.3.11.7.5) */
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
    } else {
        n_symbols = ((16 + 8 * (int)ht_length + 6) + n_dbps - 1) / n_dbps;
    }
    result->n_symbols = n_symbols;

    /* Preamble offsets */
    size_t htsig_offset = state->sig_offset + LIB80211_SYMBOL_LEN;
    size_t ht_stf_offset = htsig_offset + 2 * LIB80211_SYMBOL_LEN;
    size_t ht_ltf_offset = ht_stf_offset + LIB80211_SYMBOL_LEN;
    size_t ht_data_start = ht_ltf_offset + LIB80211_SYMBOL_LEN;

    int ncp = short_gi ? LIB80211_NCP_SHORT : LIB80211_NCP;
    int sym_len = LIB80211_NFFT + ncp;

    if (ht_data_start + (size_t)n_symbols * (size_t)sym_len > state->n_samples)
        return -1;

    /* Step 2: HT-LTF channel estimation */
    float H_ht_real[64], H_ht_imag[64];
    float ht_noise_var;

    if (ht_ltf_offset + LIB80211_SYMBOL_LEN > state->n_samples)
        return -1;

    lib80211_ht_vht_channel_estimate(state->plan,
                        state->work_re + ht_ltf_offset,
                        state->work_im + ht_ltf_offset,
                        H_ht_real, H_ht_imag, &ht_noise_var);

    if (state->noise_var > ht_noise_var)
        ht_noise_var = state->noise_var;

    /* Step 3: Extract and decode DATA symbols */
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
            size_t offset = ht_data_start + (size_t)i * (size_t)sym_len;

            lib80211_extract_ht_vht_symbol(state->plan,
                              state->work_re + offset,
                              state->work_im + offset,
                              H_ht_real, H_ht_imag,
                              ht_noise_var, i, ncp, LIB80211_HT_PILOT_Z_START,
                              &data_pilot,
                              sym_data_re, sym_data_im);

            lib80211_soft_demap(sym_data_re, sym_data_im, sym_soft,
                                LIB80211_N_HT_DATA_SC, n_bpsc);

            memcpy(all_soft + (size_t)i * (size_t)n_cbps, sym_soft,
                   (size_t)n_cbps * sizeof(float));
        }

        /* Negate LLRs: soft_demap produces positive=bit1, LDPC wants positive=bit0 */
        for (size_t i = 0; i < total_soft; i++)
            all_soft[i] = -all_soft[i];

        int n_pld = n_symbols * n_dbps;
        uint8_t *decoded_bits = (uint8_t *)lib80211_scratch_alloc(scratch, (size_t)n_pld, 0);
        if (!decoded_bits) return -1;

        int rc = lib80211_ldpc_decode_data(all_soft, total_soft, decoded_bits,
                                           n_symbols, n_dbps, n_cbps,
                                           mcs_info->cr_n, mcs_info->cr_d);
        if (rc < 0) return -1;

        /* Descramble */
        uint8_t seed = lib80211_detect_scrambler_seed(decoded_bits);
        lib80211_scramble(decoded_bits, decoded_bits, (size_t)n_pld, seed);

        /* Extract PSDU */
        if (ht_length > 4095 || 16 + 8 * (int)ht_length > n_pld)
            return -1;

        result->psdu_len = ht_length;
        lib80211_bits_to_bytes_lsb(decoded_bits + 16, result->psdu, ht_length);
        result->fcs_valid = lib80211_fcs_check(result->psdu, ht_length);

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
            size_t offset = ht_data_start + (size_t)i * (size_t)sym_len;

            lib80211_extract_ht_vht_symbol(state->plan,
                              state->work_re + offset,
                              state->work_im + offset,
                              H_ht_real, H_ht_imag,
                              ht_noise_var, i, ncp, LIB80211_HT_PILOT_Z_START,
                              &data_pilot,
                              sym_data_re, sym_data_im);

            lib80211_soft_demap(sym_data_re, sym_data_im, sym_soft,
                                LIB80211_N_HT_DATA_SC, n_bpsc);

            lib80211_deinterleave_ht(sym_soft, sym_deint, n_cbps, n_bpsc);

            memcpy(all_soft + (size_t)i * (size_t)n_cbps, sym_deint,
                   (size_t)n_cbps * sizeof(float));
        }

        /* Shared BCC decode */
        size_t n_data_bits = (size_t)n_symbols * (size_t)n_dbps;
        return lib80211_rx_bcc_decode(all_soft, total_soft, n_symbols, n_dbps,
                                      mcs_info->cr_n, mcs_info->cr_d,
                                      ht_length, n_data_bits, scratch, result);
    }
}
