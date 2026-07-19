/**
 * rx_common.c -- Shared RX utilities and front-end pipeline.
 *
 * Implements: sync -> CFO correct -> channel estimate -> L-SIG decode.
 * These stages are common to legacy, HT, and VHT frame reception.
 */

#include "rx_internal.h"
#include "crc32_internal.h"
#include "lib80211/sync.h"
#include "lib80211/scratch.h"
#include "lib80211/channel.h"
#include "lib80211/constants.h"
#include "lib80211/modulation.h"
#include "lib80211/interleaver.h"
#include "lib80211/fec.h"
#include "lib80211/scrambler.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ========================================================================
 * CRC-32 FCS check
 * ======================================================================== */

bool lib80211_fcs_check(const uint8_t *frame, size_t len)
{
    if (len < 4) return false;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc = lib80211_crc32_byte(crc, frame[i]);
    }
    return (crc == 0xDEBB20E3u);
}

/* ========================================================================
 * Scrambler seed detection
 * ======================================================================== */

uint8_t lib80211_detect_scrambler_seed(const uint8_t *decoded_bits)
{
    for (uint8_t seed = 1; seed <= 127; seed++) {
        uint8_t state = seed;
        int match = 1;
        for (int i = 0; i < 7; i++) {
            uint8_t feedback = ((state >> 6) ^ (state >> 3)) & 1;
            if (feedback != decoded_bits[i]) {
                match = 0;
                break;
            }
            state = ((state << 1) | feedback) & 0x7F;
        }
        if (match) return seed;
    }
    return 0x5D;  /* fallback */
}

/* ========================================================================
 * Bits <-> bytes
 * ======================================================================== */

void lib80211_bits_to_bytes_lsb(const uint8_t *bits, uint8_t *bytes, size_t n_bytes)
{
    for (size_t i = 0; i < n_bytes; i++) {
        uint8_t byte = 0;
        for (int b = 0; b < 8; b++) {
            byte |= (bits[i * 8 + b] & 1) << b;
        }
        bytes[i] = byte;
    }
}

/* ========================================================================
 * Rate lookup by rate_bits code
 * ======================================================================== */

const lib80211_rate_info *lib80211_rate_lookup_by_code(uint8_t rate_code)
{
    for (int i = 0; i < 8; i++) {
        if (LIB80211_RATE_TABLE[i].rate_bits == rate_code)
            return &LIB80211_RATE_TABLE[i];
    }
    return NULL;
}

/* ========================================================================
 * Front-end: sync -> CFO -> channel estimate -> L-SIG decode
 * ======================================================================== */

/* Internal: declared in sync.c, used here to avoid redundant copy */
int lib80211_sync_detect_with_work(lib80211_fft_plan *plan,
                                   const float *iq_real, const float *iq_imag,
                                   size_t n_samples,
                                   float *work_re, float *work_im,
                                   lib80211_sync_result *result);

int lib80211_rx_front_end_s(lib80211_fft_plan *plan,
                            const float *iq_real, const float *iq_imag,
                            size_t n_samples, lib80211_scratch *scratch,
                            lib80211_rx_state *state)
{
    if (!plan || !iq_real || !iq_imag || !state || !scratch || n_samples < 400)
        return -1;

    memset(state, 0, sizeof(*state));
    state->plan = plan;
    state->n_samples = n_samples;

    /* Allocate work buffers from scratch */
    state->work_re = (float *)lib80211_scratch_alloc(scratch, n_samples * sizeof(float), 0);
    state->work_im = (float *)lib80211_scratch_alloc(scratch, n_samples * sizeof(float), 0);
    if (!state->work_re || !state->work_im)
        return -1;

    /* Step 1: Synchronization — writes coarse-CFO-corrected IQ into work buffers.
     * We use sync_detect_with_work which fills work_re/work_im with the
     * coarse-corrected signal. Then we re-apply the full combined correction
     * for best accuracy (single rotation pass avoids cascaded phase errors). */
    lib80211_sync_result sync;
    if (lib80211_sync_detect_with_work(plan, iq_real, iq_imag, n_samples,
                                       state->work_re, state->work_im, &sync) != 0)
        return -1;

    state->ltf_start = sync.ltf_start;
    state->cfo_rad = sync.cfo_rad;

    /* Step 2: Apply full combined CFO correction to raw input.
     * sync_detect_with_work applied only coarse CFO to find LTF timing.
     * For decode accuracy, reapply full correction from raw. */
    memcpy(state->work_re, iq_real, n_samples * sizeof(float));
    memcpy(state->work_im, iq_imag, n_samples * sizeof(float));

    if (fabsf(sync.cfo_rad) > 1e-8f) {
        lib80211_cfo_correct(state->work_re, state->work_im, n_samples, sync.cfo_rad);
    }

    /* Step 2b: IQ imbalance correction (estimated from L-LTF) */
    if (sync.ltf_start + 128 <= n_samples) {
        const float *ltf_re = state->work_re + sync.ltf_start;
        const float *ltf_im = state->work_im + sync.ltf_start;

        double ei2 = 0.0, eq2 = 0.0, eiq = 0.0;
        for (size_t i = 0; i < 128; i++) {
            ei2 += (double)ltf_re[i] * ltf_re[i];
            eq2 += (double)ltf_im[i] * ltf_im[i];
            eiq += (double)ltf_re[i] * ltf_im[i];
        }
        ei2 /= 128.0;
        eq2 /= 128.0;
        eiq /= 128.0;

        if (ei2 > 1e-20 && eq2 > 1e-20) {
            float g_ratio = sqrtf((float)(ei2 / eq2));
            float phi_est = (float)(eiq / ei2);

            if (fabsf(g_ratio - 1.0f) >= 0.01f || fabsf(phi_est) >= 0.01f) {
                for (size_t i = 0; i < n_samples; i++) {
                    state->work_im[i] = (state->work_im[i] - phi_est * state->work_re[i]) * g_ratio;
                }
            }
        }
    }

    /* Step 3: Channel estimation from L-LTF */
    if (sync.ltf_start + 128 > n_samples)
        return -1;

    lib80211_estimate_channel(plan,
                              state->work_re + sync.ltf_start,
                              state->work_im + sync.ltf_start,
                              state->H_real, state->H_imag, &state->noise_var);

    if (state->noise_var < 1e-6f) state->noise_var = 1e-6f;

    /* Step 4: L-SIG decode */
    state->sig_offset = sync.ltf_start + 128;

    if (state->sig_offset + 80 > n_samples)
        return -1;

    float sig_data_re[48], sig_data_im[48];
    lib80211_pilot_state sig_pilot = { .slope = 0.0f, .alpha = 0.3f, .initialized = 0 };
    lib80211_extract_legacy_symbol(plan,
                                   state->work_re + state->sig_offset,
                                   state->work_im + state->sig_offset,
                                   state->H_real, state->H_imag, state->noise_var,
                                   0, &sig_pilot,
                                   sig_data_re, sig_data_im);

    float sig_soft[48];
    lib80211_soft_demap(sig_data_re, sig_data_im, sig_soft, 48, 1);

    float sig_deint[48];
    lib80211_deinterleave(sig_soft, sig_deint, 48, 1);

    uint8_t sig_decoded[18];
    if (lib80211_viterbi_decode(sig_deint, sig_decoded, 48, 18) != 0)
        return -1;

    /* Parse L-SIG fields */
    uint8_t rate_code = 0;
    for (int i = 0; i < 4; i++)
        rate_code |= (sig_decoded[i] & 1) << i;

    uint16_t length = 0;
    for (int i = 5; i <= 16; i++)
        length |= (uint16_t)(sig_decoded[i] & 1) << (i - 5);

    uint8_t parity_bit = sig_decoded[17] & 1;
    uint8_t parity_sum = 0;
    for (int i = 0; i < 17; i++)
        parity_sum ^= (sig_decoded[i] & 1);

    if (parity_sum != parity_bit)
        return -1;

    const lib80211_rate_info *rate_info = lib80211_rate_lookup_by_code(rate_code);
    if (!rate_info)
        return -1;

    state->lsig_rate_mbps = rate_info->rate_mbps;
    state->lsig_length = length;

    /* Step 5: Decision-directed channel refinement from L-SIG */
    {
        uint8_t lsig_bits[24];
        memset(lsig_bits, 0, sizeof(lsig_bits));

        for (int i = 0; i < 4; i++)
            lsig_bits[i] = (rate_code >> i) & 1;
        for (int i = 5; i <= 16; i++)
            lsig_bits[i] = (length >> (i - 5)) & 1;
        lsig_bits[17] = parity_bit;

        uint8_t coded[48];
        lib80211_conv_encode(lsig_bits, coded, 24, false);

        uint8_t interleaved[48];
        lib80211_interleave(coded, interleaved, 48, 1);

        float exp_data_re[48], exp_data_im[48];
        lib80211_modulate(interleaved, exp_data_re, exp_data_im, 48, 1);

        int sig_polarity = LIB80211_PILOT_POLARITY[0];

        float Y_sig_real[64], Y_sig_imag[64];
        lib80211_fft_forward(plan,
                             state->work_re + state->sig_offset + LIB80211_NCP,
                             state->work_im + state->sig_offset + LIB80211_NCP,
                             Y_sig_real, Y_sig_imag);

        memcpy(state->H_refined_real, state->H_real, 64 * sizeof(float));
        memcpy(state->H_refined_imag, state->H_imag, 64 * sizeof(float));

        for (int i = 0; i < 48; i++) {
            int bin = LIB80211_DATA_BINS[i];
            float x = exp_data_re[i];
            float h_new_re = Y_sig_real[bin] * x;
            float h_new_im = Y_sig_imag[bin] * x;

            state->H_refined_real[bin] = 0.5f * state->H_real[bin] + 0.5f * h_new_re;
            state->H_refined_imag[bin] = 0.5f * state->H_imag[bin] + 0.5f * h_new_im;
        }

        for (int i = 0; i < 4; i++) {
            int bin = LIB80211_PILOT_BINS[i];
            float expected_pilot = LIB80211_PILOT_BASE[i] * sig_polarity;
            float h_new_re = Y_sig_real[bin] * expected_pilot;
            float h_new_im = Y_sig_imag[bin] * expected_pilot;

            state->H_refined_real[bin] = 0.5f * state->H_real[bin] + 0.5f * h_new_re;
            state->H_refined_imag[bin] = 0.5f * state->H_imag[bin] + 0.5f * h_new_im;
        }
    }

    return 0;
}

int lib80211_rx_front_end(lib80211_fft_plan *plan,
                          const float *iq_real, const float *iq_imag,
                          size_t n_samples, lib80211_rx_state *state)
{
    if (!plan || !iq_real || !iq_imag || !state || n_samples < 400)
        return -1;

    /* Convenience path: malloc a scratch, run _s, copy work buffers out to heap */
    size_t scratch_size = LIB80211_SCRATCH_RX_SIZE(n_samples);
    uint8_t *mem = (uint8_t *)malloc(scratch_size);
    if (!mem) return -1;

    lib80211_scratch scratch;
    lib80211_scratch_init(&scratch, mem, scratch_size);

    int rc = lib80211_rx_front_end_s(plan, iq_real, iq_imag, n_samples, &scratch, state);
    if (rc != 0) {
        free(mem);
        state->work_re = NULL;
        state->work_im = NULL;
        return -1;
    }

    /* Copy work buffers from scratch onto heap (old API: caller frees them) */
    float *heap_re = (float *)malloc(n_samples * sizeof(float));
    float *heap_im = (float *)malloc(n_samples * sizeof(float));
    if (!heap_re || !heap_im) {
        free(heap_re);
        free(heap_im);
        free(mem);
        state->work_re = NULL;
        state->work_im = NULL;
        return -1;
    }
    memcpy(heap_re, state->work_re, n_samples * sizeof(float));
    memcpy(heap_im, state->work_im, n_samples * sizeof(float));
    state->work_re = heap_re;
    state->work_im = heap_im;

    free(mem);
    return 0;
}

/* ========================================================================
 * Classification
 * ======================================================================== */

lib80211_frame_type lib80211_rx_classify(const lib80211_rx_state *state)
{
    /* HT/VHT require L-SIG rate = 6 Mbps (IEEE 802.11-2020 S19.3.9.3.5) */
    if (state->lsig_rate_mbps != 6)
        return LIB80211_FRAME_LEGACY;

    /* Examine the first symbol after L-SIG for modulation type */
    size_t first_sym_offset = state->sig_offset + LIB80211_SYMBOL_LEN;

    if (first_sym_offset + LIB80211_SYMBOL_LEN > state->n_samples)
        return LIB80211_FRAME_LEGACY;

    /* Extract symbol using refined channel estimate (closer to HT-SIG in time) */
    float data_re[48], data_im[48];
    lib80211_pilot_state ps = { .slope = 0.0f, .alpha = 0.3f, .initialized = 0 };
    lib80211_extract_legacy_symbol(state->plan,
                                   state->work_re + first_sym_offset,
                                   state->work_im + first_sym_offset,
                                   (float *)state->H_refined_real,
                                   (float *)state->H_refined_imag,
                                   state->noise_var,
                                   1, &ps,
                                   data_re, data_im);

    float i_energy = 0.0f, q_energy = 0.0f;
    for (int i = 0; i < 48; i++) {
        i_energy += data_re[i] * data_re[i];
        q_energy += data_im[i] * data_im[i];
    }

    if (q_energy > 1.5f * i_energy)
        return LIB80211_FRAME_HT;
    if (i_energy > 1.5f * q_energy)
        return LIB80211_FRAME_VHT;

    return LIB80211_FRAME_LEGACY;
}

/* ========================================================================
 * Shared BCC decode pipeline (scratch-based)
 * ======================================================================== */

int lib80211_rx_bcc_decode(float *all_soft, size_t total_soft,
                           int n_symbols, int n_dbps,
                           int cr_n, int cr_d,
                           uint16_t psdu_length, size_t scramble_len,
                           lib80211_scratch *scratch,
                           lib80211_rx_result *result)
{
    /* Step 1: Depuncture */
    size_t depunct_len;
    float *depunctured = NULL;

    if (cr_n == 1 && cr_d == 2) {
        depunct_len = total_soft;
        depunctured = all_soft;
    } else {
        size_t max_depunct = total_soft * 3;
        depunctured = (float *)lib80211_scratch_alloc(scratch, max_depunct * sizeof(float), 0);
        if (!depunctured) return -1;
        depunct_len = lib80211_depuncture(all_soft, depunctured, total_soft, cr_n, cr_d);
        if (depunct_len == 0)
            return -1;
    }

    /* Step 2: Viterbi decode */
    size_t n_data_bits = (size_t)n_symbols * (size_t)n_dbps;
    size_t n_coded_needed = 2 * (n_data_bits + 6);

    if (depunct_len < n_coded_needed) {
        /* Need zero-padded buffer */
        float *viterbi_input = (float *)lib80211_scratch_alloc(scratch, n_coded_needed * sizeof(float), 0);
        if (!viterbi_input) return -1;
        memcpy(viterbi_input, depunctured, depunct_len * sizeof(float));
        memset(viterbi_input + depunct_len, 0,
               (n_coded_needed - depunct_len) * sizeof(float));
        depunctured = viterbi_input;
        depunct_len = n_coded_needed;
    }

    uint8_t *decoded_bits = (uint8_t *)lib80211_scratch_alloc(scratch, n_data_bits, 0);
    if (!decoded_bits) return -1;

    if (lib80211_viterbi_decode(depunctured, decoded_bits, depunct_len, n_data_bits) != 0)
        return -1;

    /* Step 3: Descramble */
    uint8_t seed = lib80211_detect_scrambler_seed(decoded_bits);
    lib80211_scramble(decoded_bits, decoded_bits, scramble_len, seed);

    /* Step 4: Extract PSDU */
    if (psdu_length > 4095 || (size_t)(16 + 8 * psdu_length) > n_data_bits)
        return -1;

    result->psdu_len = psdu_length;
    lib80211_bits_to_bytes_lsb(decoded_bits + 16, result->psdu, psdu_length);

    /* Step 5: FCS check */
    result->fcs_valid = lib80211_fcs_check(result->psdu, psdu_length);

    return 0;
}

/* ========================================================================
 * Unified decoder entry points
 * ======================================================================== */

int lib80211_rx_decode_s(lib80211_fft_plan *plan,
                         const float *iq_real, const float *iq_imag,
                         size_t n_samples, lib80211_scratch *scratch,
                         lib80211_rx_result *result)
{
    if (!result || !scratch) return -1;
    memset(result, 0, sizeof(*result));
    result->mcs = -1;

    lib80211_scratch_reset(scratch);

    lib80211_rx_state state;
    int rc = lib80211_rx_front_end_s(plan, iq_real, iq_imag, n_samples, scratch, &state);
    if (rc != 0) return -1;

    lib80211_frame_type ftype = lib80211_rx_classify(&state);

    switch (ftype) {
    case LIB80211_FRAME_HT:
        rc = lib80211_rx_decode_ht(&state, scratch, result);
        if (rc != 0) {
            memset(result, 0, sizeof(*result));
            result->mcs = -1;
            rc = lib80211_rx_decode_vht(&state, scratch, result);
            if (rc != 0) {
                memset(result, 0, sizeof(*result));
                result->mcs = -1;
                rc = lib80211_rx_decode_legacy(&state, scratch, result);
            }
        }
        break;
    case LIB80211_FRAME_VHT:
        rc = lib80211_rx_decode_vht(&state, scratch, result);
        if (rc != 0) {
            memset(result, 0, sizeof(*result));
            result->mcs = -1;
            rc = lib80211_rx_decode_ht(&state, scratch, result);
            if (rc != 0) {
                memset(result, 0, sizeof(*result));
                result->mcs = -1;
                rc = lib80211_rx_decode_legacy(&state, scratch, result);
            }
        }
        break;
    case LIB80211_FRAME_LEGACY:
    default:
        if (state.lsig_rate_mbps == 6) {
            rc = lib80211_rx_decode_ht(&state, scratch, result);
            if (rc == 0)
                break;
            memset(result, 0, sizeof(*result));
            result->mcs = -1;
            rc = lib80211_rx_decode_vht(&state, scratch, result);
            if (rc == 0)
                break;
            memset(result, 0, sizeof(*result));
            result->mcs = -1;
        }
        rc = lib80211_rx_decode_legacy(&state, scratch, result);
        break;
    }

    return rc;
}

int lib80211_rx_decode(lib80211_fft_plan *plan,
                       const float *iq_real, const float *iq_imag,
                       size_t n_samples, lib80211_rx_result *result)
{
    if (!result) return -1;

    size_t scratch_size = LIB80211_SCRATCH_RX_SIZE(n_samples);
    uint8_t *mem = (uint8_t *)malloc(scratch_size);
    if (!mem) return -1;

    lib80211_scratch scratch;
    lib80211_scratch_init(&scratch, mem, scratch_size);

    int rc = lib80211_rx_decode_s(plan, iq_real, iq_imag, n_samples, &scratch, result);

    free(mem);
    return rc;
}
