/**
 * tx_vht.c — VHT (802.11ac) frame generation, MCS 0-8, BCC.
 *
 * Frame structure:
 *   L-STF(160) | L-LTF(160) | L-SIG(80) | VHT-SIG-A(160)
 *   | VHT-STF(80) | VHT-LTF(80) | VHT-SIG-B(80) | VHT-DATA(n_sym * symbol_len)
 *
 * DATA path: bytes->bits -> scramble -> conv_encode -> puncture ->
 *            HT-interleave -> modulate -> VHT-OFDM symbols (52 subcarriers)
 *
 * Key VHT differences from HT:
 *   - Scrambler input does NOT include tail; tail is appended after scrambling
 *   - SERVICE field bits 8-15 carry CRC-8 of VHT-SIG-B first 20 bits
 *   - Pilot z_start = 4 (vs HT z_start = 3)
 *   - SGI last symbol has 1 extra sample (disambiguity)
 *
 * TX normalization (IEEE TGac reference):
 *   Legacy fields (L-STF through VHT-STF, samples 0-639): NFFT/sqrt(52)
 *   VHT fields (VHT-LTF onward, from sample 640): NFFT/sqrt(56)
 *
 * Memory: All intermediate buffers are scratch-allocated.
 * Maximum PSDU length is LIB80211_VHT_MAX_PSDU (4095 bytes).
 */

#include "lib80211/tx.h"
#include "lib80211/scratch.h"
#include "lib80211/constants.h"
#include "lib80211/scrambler.h"
#include "lib80211/fec.h"
#include "lib80211/interleaver.h"
#include "lib80211/modulation.h"
#include "lib80211/ofdm.h"
#include "lib80211/signal.h"
#include "lib80211/ldpc.h"
#include "tx_internal.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

/* TX normalization factors */
#define VHT_NORM_LEGACY  (64.0f / sqrtf(52.0f))   /* ~8.875 */
#define VHT_NORM_VHT     (64.0f / sqrtf(56.0f))   /* ~8.552 */

/*
 * Maximum PSDU size for buffer allocation.
 * This matches the legacy maximum (L-SIG LENGTH field is 12 bits = 4095).
 * Worst case at MCS 0 (n_dbps=26): ceil((16+32760+6)/26)=1261 symbols.
 */
#define VHT_MAX_PSDU      4095
#define VHT_MAX_DATA_BITS (16 + VHT_MAX_PSDU * 8 + 6 + 416)  /* SERVICE+PSDU+TAIL+maxPAD */
#define VHT_MAX_CODED     (VHT_MAX_DATA_BITS * 2)
/* Max windowing boundaries: 8 preamble + ceil((16+32760+6)/26) DATA = 1269 */
#define VHT_MAX_BOUNDS    1280

/**
 * Compute VHT n_sym: ceil((16 + 8*psdu_len + 6) / n_dbps)
 * Note: n_data_bits = 16 + 8*psdu_len (SERVICE + PSDU, no tail in data),
 * but +6 accounts for tail bits in the ceiling.
 */
static int vht_n_sym(int psdu_len, int n_dbps) {
    int n_bits = 16 + 8 * psdu_len + 6;
    return (n_bits + n_dbps - 1) / n_dbps;
}

size_t lib80211_tx_vht_samples(const lib80211_tx_vht_params *params) {
    if (params->mcs < 0 || params->mcs > 8) return 0;
    if (params->psdu_len > VHT_MAX_PSDU) return 0;

    const lib80211_ht_mcs_info *mcs = &LIB80211_VHT_MCS_TABLE[params->mcs];
    int n_sym = vht_n_sym((int)params->psdu_len, mcs->n_dbps);

    /* Preamble: L-STF(160) + L-LTF(160) + L-SIG(80) + VHT-SIG-A(160)
     *         + VHT-STF(80) + VHT-LTF(80) + VHT-SIG-B(80) = 800 */
    size_t preamble = 800;

    if (params->short_gi) {
        /* SGI: n_sym * 72 + 1 (last symbol has extra sample for disambiguity) */
        return preamble + (size_t)n_sym * LIB80211_SYMBOL_LEN_SGI + 1;
    } else {
        return preamble + (size_t)n_sym * LIB80211_SYMBOL_LEN;
    }
}

size_t lib80211_tx_vht_s(lib80211_fft_plan *plan,
                         const lib80211_tx_vht_params *params,
                         lib80211_scratch *scratch,
                         float *out_real, float *out_imag) {
    if (params->mcs < 0 || params->mcs > 8) return 0;
    if (params->psdu_len > VHT_MAX_PSDU) return 0;

    lib80211_scratch_reset(scratch);

    const lib80211_ht_mcs_info *mcs = &LIB80211_VHT_MCS_TABLE[params->mcs];
    int n_bpsc = mcs->n_bpsc;
    int n_cbps = mcs->n_cbps;
    int n_dbps = mcs->n_dbps;
    int cr_n = mcs->cr_n;
    int cr_d = mcs->cr_d;

    /* Compute frame dimensions */
    int n_sym = vht_n_sym((int)params->psdu_len, n_dbps);
    int n_data_bits_total = n_sym * n_dbps;  /* total data bits across all symbols */

    /* Pre-compute LDPC extra symbol flag (needed for VHT-SIG-A) */
    int ldpc_extra = 0;
    if (params->ldpc) {
        /* Dry-run: compute ldpc_extra without actually encoding */
        int n_payload = 16 + 8 * (int)params->psdu_len;
        int n_sym_init = (n_payload + n_dbps - 1) / n_dbps;
        int n_pld = n_sym_init * n_dbps;

        /* Select codeword params */
        int l_cw = 1944, n_cw = 1;
        int k_cap;
        if (n_pld <= 648 * cr_n / cr_d) { l_cw = 648; }
        else if (n_pld <= 1296 * cr_n / cr_d) { l_cw = 1296; }
        else if (n_pld <= 1944 * cr_n / cr_d) { l_cw = 1944; }
        else { l_cw = 1944; n_cw = (n_pld * cr_d + 1944 * cr_n - 1) / (1944 * cr_n); }
        k_cap = l_cw * cr_n / cr_d;

        int n_avail_check = n_sym_init * n_cbps;
        int n_shrt_check = n_cw * k_cap - n_pld;
        if (n_shrt_check < 0) n_shrt_check = 0;
        int n_punc_check = n_cw * l_cw - n_avail_check - n_shrt_check;
        if (n_punc_check < 0) n_punc_check = 0;
        int n_parity_check = n_cw * (l_cw - k_cap);
        if (n_parity_check > 0 && n_punc_check * 10 > n_parity_check) {
            ldpc_extra = 1;
        }
    }

    int cp_len = params->short_gi ? LIB80211_NCP_SHORT : LIB80211_NCP;
    int sym_len = cp_len + LIB80211_NFFT;

    size_t out_idx = 0;

    /* === L-STF (160 samples) === */
    lib80211_generate_stf(plan, &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], LIB80211_STF_SAMPLES, VHT_NORM_LEGACY);
    out_idx += LIB80211_STF_SAMPLES;

    /* === L-LTF (160 samples) === */
    lib80211_generate_ltf(plan, &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], LIB80211_LTF_SAMPLES, VHT_NORM_LEGACY);
    out_idx += LIB80211_LTF_SAMPLES;

    /* === L-SIG (80 samples) === */
    int lsig_length;
    if (params->short_gi) {
        /* SGI: 3*(ceil(n_sym * 0.9) + 5) - 3 */
        lsig_length = 3 * ((int)ceil(n_sym * 0.9) + 5) - 3;
    } else {
        /* Normal GI: (5 + n_sym) * 3 - 3 */
        lsig_length = (5 + n_sym) * 3 - 3;
    }
    lib80211_make_lsig_symbol(plan, 6, lsig_length,
                              &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], LIB80211_SYMBOL_LEN, VHT_NORM_LEGACY);
    out_idx += LIB80211_SYMBOL_LEN;

    /* === VHT-SIG-A (160 samples, 2 OFDM symbols) === */
    lib80211_make_vhtsiga_symbols(plan, params->mcs, (int)params->psdu_len,
                                  params->short_gi, params->ldpc, ldpc_extra,
                                  &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], 160, VHT_NORM_LEGACY);
    out_idx += 160;

    /* === VHT-STF (80 samples) — identical to HT-STF === */
    lib80211_generate_ht_stf(plan, &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], 80, VHT_NORM_LEGACY);
    out_idx += 80;

    /* === VHT-LTF (80 samples) — identical to HT-LTF === */
    lib80211_generate_ht_ltf(plan, &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], 80, VHT_NORM_VHT);
    out_idx += 80;

    /* === VHT-SIG-B (80 samples) === */
    lib80211_make_vhtsigb_symbol(plan, (int)params->psdu_len,
                                 &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], 80, VHT_NORM_VHT);
    out_idx += 80;

    /* === VHT-DATA === */

    /* Step 1: Build SERVICE field with VHT-SIG-B CRC.
     * VHT SERVICE bits 0-7: scrambler init (set to 0, filled by scrambler).
     * VHT SERVICE bits 8-15: CRC-8 of VHT-SIG-B first 20 bits. */
    uint8_t sigb_bits[26];
    lib80211_make_vhtsigb_bits((int)params->psdu_len, sigb_bits);
    uint8_t sigb_crc = lib80211_htsig_crc8(sigb_bits, 20);
    uint8_t sigb_crc_inv = (uint8_t)(~sigb_crc & 0xFF);

    /* Allocate working buffers from scratch. */
    size_t alloc_data = (size_t)VHT_MAX_DATA_BITS;
    size_t alloc_coded = (size_t)VHT_MAX_CODED;
    uint8_t *data_bits = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_data, 0);
    uint8_t *coded     = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_coded, 0);
    if (!data_bits || !coded) return 0;

    if (params->ldpc) {
        /* --- LDPC path: no tail bits, no interleaving --- */

        /* Build payload: SERVICE(16) + PSDU bits */
        int n_payload = 16 + 8 * (int)params->psdu_len;
        memset(data_bits, 0, (size_t)n_payload);

        /* SERVICE field: bits 8-15 = SIG-B CRC (ones-complement, MSB-first) */
        for (int i = 0; i < 8; i++)
            data_bits[8 + i] = (uint8_t)((sigb_crc_inv >> (7 - i)) & 1);

        /* PSDU: LSB-first per byte */
        for (size_t byte_idx = 0; byte_idx < params->psdu_len; byte_idx++) {
            uint8_t val = params->psdu[byte_idx];
            for (int bit = 0; bit < 8; bit++) {
                data_bits[16 + byte_idx * 8 + bit] = (val >> bit) & 1;
            }
        }

        /* Pad to initial n_sym * n_dbps for scrambling */
        int n_sym_init = (n_payload + n_dbps - 1) / n_dbps;
        int scramble_len = n_sym_init * n_dbps;
        memset(&data_bits[n_payload], 0, (size_t)(scramble_len - n_payload));

        /* Scramble (use a separate buffer from scratch) */
        uint8_t *scrambled_buf = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_data, 0);
        if (!scrambled_buf) return 0;
        lib80211_scramble(data_bits, scrambled_buf, (size_t)scramble_len,
                          params->scrambler_seed);

        /* LDPC encode */
        int n_sym_ldpc = 0, ldpc_extra_actual = 0;
        lib80211_ldpc_encode_data(scrambled_buf, n_payload,
                                  n_dbps, n_cbps, cr_n, cr_d,
                                  coded, &n_sym_ldpc, &ldpc_extra_actual);

        /* Update n_sym for windowing */
        n_sym = n_sym_ldpc;

        /* Build OFDM symbols — NO interleaving for LDPC */
        for (int s = 0; s < n_sym; s++) {
            uint8_t *sym_bits = &coded[s * n_cbps];

            /* No interleaving — map directly to constellation */
            float mod_real[52], mod_imag[52];
            lib80211_modulate(sym_bits, mod_real, mod_imag, 52, n_bpsc);

            /* Build frequency-domain symbol with HT subcarrier mapping */
            float freq_real[64] = {0};
            float freq_imag[64] = {0};

            for (int i = 0; i < 52; i++) {
                int bin = LIB80211_HT_DATA_BINS[i];
                freq_real[bin] = mod_real[i];
                freq_imag[bin] = mod_imag[i];
            }

            /* VHT pilots: z_start=4 */
            float polarity = (float)LIB80211_PILOT_POLARITY[
                (LIB80211_VHT_PILOT_Z_START + s) % 127];
            for (int k = 0; k < 4; k++) {
                int bin = LIB80211_HT_PILOT_BINS[k];
                float pattern = LIB80211_HT_PILOT_PATTERN[(s + k) % 4];
                freq_real[bin] = polarity * pattern;
                freq_imag[bin] = 0.0f;
            }

            /* 64-point IFFT -> time domain */
            float time_real[64], time_imag[64];
            lib80211_fft_inverse(plan, freq_real, freq_imag, time_real, time_imag);

            /* Prepend cyclic prefix */
            int this_cp_len = cp_len;
            int this_sym_len = sym_len;

            if (params->short_gi && s == n_sym - 1) {
                this_sym_len = sym_len + 1;
            }

            memcpy(&out_real[out_idx], &time_real[64 - this_cp_len],
                   (size_t)this_cp_len * sizeof(float));
            memcpy(&out_imag[out_idx], &time_imag[64 - this_cp_len],
                   (size_t)this_cp_len * sizeof(float));
            memcpy(&out_real[out_idx + this_cp_len], time_real, 64 * sizeof(float));
            memcpy(&out_imag[out_idx + this_cp_len], time_imag, 64 * sizeof(float));

            if (params->short_gi && s == n_sym - 1) {
                out_real[out_idx + this_cp_len + 64] = time_real[0];
                out_imag[out_idx + this_cp_len + 64] = time_imag[0];
            }

            lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx],
                     (size_t)this_sym_len, VHT_NORM_VHT);
            out_idx += (size_t)this_sym_len;
        }
    } else {
        /* --- BCC path (original behavior) --- */
        uint8_t *punctured = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_coded, 0);
        if (!punctured) return 0;

        /* Scrambler input: SERVICE(16) + PSDU(8*len) + PAD(n_pad) */
        int n_data_bits = 16 + 8 * (int)params->psdu_len;
        int n_pad = n_data_bits_total - n_data_bits - 6;
        int scrambler_input_len = n_data_bits + n_pad;

        /* Build scrambler input in data_bits[] */
        memset(data_bits, 0, (size_t)scrambler_input_len);

        /* SERVICE field: bits 0-7 = 0, bits 8-15 = CRC MSB-first */
        for (int i = 0; i < 8; i++)
            data_bits[8 + i] = (uint8_t)((sigb_crc_inv >> (7 - i)) & 1);

        /* PSDU: LSB-first per byte */
        for (size_t byte_idx = 0; byte_idx < params->psdu_len; byte_idx++) {
            uint8_t val = params->psdu[byte_idx];
            for (int bit = 0; bit < 8; bit++) {
                data_bits[16 + byte_idx * 8 + bit] = (val >> bit) & 1;
            }
        }

        /* Scramble (use coded[] as temp) */
        lib80211_scramble(data_bits, coded, (size_t)scrambler_input_len,
                          params->scrambler_seed);

        /* Build data_with_tail: scrambled + 6 zero tail bits */
        memcpy(data_bits, coded, (size_t)scrambler_input_len);
        memset(&data_bits[scrambler_input_len], 0, 6);

        /* Convolutional encode (rate-1/2) */
        lib80211_conv_encode(data_bits, coded, (size_t)n_data_bits_total, false);

        /* Puncture to target rate */
        int total_coded_needed = n_sym * n_cbps;

        if (cr_n == 1 && cr_d == 2) {
            memcpy(punctured, coded, (size_t)total_coded_needed);
        } else {
            int pre_punct;
            if (cr_n == 2 && cr_d == 3)
                pre_punct = (total_coded_needed * 4) / 3;
            else if (cr_n == 3 && cr_d == 4)
                pre_punct = (total_coded_needed * 6) / 4;
            else
                pre_punct = (total_coded_needed * 10) / 6;

            lib80211_puncture(coded, punctured, (size_t)pre_punct, cr_n, cr_d);
        }

        /* Per-symbol: HT-interleave -> modulate -> VHT OFDM symbol */
        for (int s = 0; s < n_sym; s++) {
            uint8_t *sym_bits = &punctured[s * n_cbps];

            /* HT Interleave (N_col=13) */
            uint8_t interleaved[416];
            lib80211_interleave_ht(sym_bits, interleaved, n_cbps, n_bpsc);

            /* Modulate (52 subcarriers) */
            float mod_real[52], mod_imag[52];
            lib80211_modulate(interleaved, mod_real, mod_imag, 52, n_bpsc);

            /* Build frequency-domain symbol with HT subcarrier mapping */
            float freq_real[64] = {0};
            float freq_imag[64] = {0};

            for (int i = 0; i < 52; i++) {
                int bin = LIB80211_HT_DATA_BINS[i];
                freq_real[bin] = mod_real[i];
                freq_imag[bin] = mod_imag[i];
            }

            /* VHT pilots: z_start=4 */
            float polarity = (float)LIB80211_PILOT_POLARITY[
                (LIB80211_VHT_PILOT_Z_START + s) % 127];
            for (int k = 0; k < 4; k++) {
                int bin = LIB80211_HT_PILOT_BINS[k];
                float pattern = LIB80211_HT_PILOT_PATTERN[(s + k) % 4];
                freq_real[bin] = polarity * pattern;
                freq_imag[bin] = 0.0f;
            }

            /* 64-point IFFT -> time domain */
            float time_real[64], time_imag[64];
            lib80211_fft_inverse(plan, freq_real, freq_imag, time_real, time_imag);

            /* Prepend cyclic prefix */
            int this_cp_len = cp_len;
            int this_sym_len = sym_len;

            if (params->short_gi && s == n_sym - 1) {
                this_sym_len = sym_len + 1;
            }

            memcpy(&out_real[out_idx], &time_real[64 - this_cp_len],
                   (size_t)this_cp_len * sizeof(float));
            memcpy(&out_imag[out_idx], &time_imag[64 - this_cp_len],
                   (size_t)this_cp_len * sizeof(float));
            memcpy(&out_real[out_idx + this_cp_len], time_real, 64 * sizeof(float));
            memcpy(&out_imag[out_idx + this_cp_len], time_imag, 64 * sizeof(float));

            if (params->short_gi && s == n_sym - 1) {
                out_real[out_idx + this_cp_len + 64] = time_real[0];
                out_imag[out_idx + this_cp_len + 64] = time_imag[0];
            }

            lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx],
                     (size_t)this_sym_len, VHT_NORM_VHT);
            out_idx += (size_t)this_sym_len;
        }
    }

    /* === T_TR = 1 sample windowing with optional circular shift ===
     *
     * The VHT TGac reference waveform (11-14/0571r10) applies transmit windowing.
     *
     * Normal GI: circular shift of -1 sample + 1-sample overlap-add windowing
     * at all OFDM symbol boundaries.
     *
     * Short GI: half-amplitude at first and last samples only (no shift,
     * no boundary windowing).
     */

    if (!params->short_gi) {
        /* Normal GI: compute boundary values, circshift, then place boundaries */

        static const int preamble_boundary_pos[] = {159, 319, 399, 479, 559, 639, 719, 799};
        static const int preamble_body_first[] = {0, 192, 336, 416, 496, 576, 656, 736};
        const int n_preamble_bounds = 8;

        int n_data_bounds = n_sym;
        int n_total_bounds = n_preamble_bounds + n_data_bounds;

        float bound_real[VHT_MAX_BOUNDS];
        float bound_imag[VHT_MAX_BOUNDS];
        int bound_pos[VHT_MAX_BOUNDS];

        /* Preamble boundaries */
        for (int i = 0; i < n_preamble_bounds; i++) {
            int bpos = preamble_boundary_pos[i];
            int bf = preamble_body_first[i];
            bound_real[i] = 0.5f * (out_real[bf] + out_real[bpos + 1]);
            bound_imag[i] = 0.5f * (out_imag[bf] + out_imag[bpos + 1]);
            bound_pos[i] = bpos;
        }

        /* DATA symbol boundaries */
        {
            size_t pos = 800;
            for (int s = 0; s < n_sym; s++) {
                size_t bpos = pos + (size_t)sym_len - 1;
                size_t bf = pos + (size_t)cp_len;
                int bi = n_preamble_bounds + s;
                bound_pos[bi] = (int)bpos;

                if (s < n_sym - 1) {
                    bound_real[bi] = 0.5f * (out_real[bf] + out_real[bpos + 1]);
                    bound_imag[bi] = 0.5f * (out_imag[bf] + out_imag[bpos + 1]);
                } else {
                    /* Last symbol: fade to zero */
                    bound_real[bi] = 0.5f * out_real[bf];
                    bound_imag[bi] = 0.5f * out_imag[bf];
                }
                pos += (size_t)sym_len;
            }
        }

        /* Apply global circular shift of -1 */
        float first_real = out_real[0];
        float first_imag = out_imag[0];
        memmove(out_real, &out_real[1], (out_idx - 1) * sizeof(float));
        memmove(out_imag, &out_imag[1], (out_idx - 1) * sizeof(float));
        out_real[out_idx - 1] = first_real;
        out_imag[out_idx - 1] = first_imag;

        /* Place windowed boundary values */
        for (int i = 0; i < n_total_bounds; i++) {
            out_real[bound_pos[i]] = bound_real[i];
            out_imag[bound_pos[i]] = bound_imag[i];
        }
    } else {
        /* Short GI: windowing at first sample of each field (position boundary+1)
         * and half-amplitude at first and last waveform samples.
         * No circular shift applied. */

        /* Half-amplitude start */
        out_real[0] *= 0.5f;
        out_imag[0] *= 0.5f;

        /* Windowing at field boundaries: replace first sample of next field */
        static const int sgi_boundary_pos[] = {160, 320, 400, 480, 560, 640, 720, 800};
        static const int sgi_body_first[] = {0, 192, 336, 416, 496, 576, 656, 736};

        for (int i = 0; i < 8; i++) {
            int bpos = sgi_boundary_pos[i];
            int bf = sgi_body_first[i];
            /* Use the unhalved body first (bf=0 is already halved, need original) */
            float bf_real = out_real[bf];
            float bf_imag = out_imag[bf];
            if (bf == 0) {
                /* Position 0 was halved; recover original for windowing formula */
                bf_real *= 2.0f;
                bf_imag *= 2.0f;
            }
            out_real[bpos] = 0.5f * (bf_real + out_real[bpos]);
            out_imag[bpos] = 0.5f * (bf_imag + out_imag[bpos]);
        }

        /* DATA symbol boundaries: first sample of each DATA symbol (except first) */
        {
            size_t pos = 800;
            for (int s = 0; s < n_sym - 1; s++) {
                size_t bf = pos + (size_t)cp_len;
                size_t next_pos = pos + (size_t)sym_len;
                out_real[next_pos] = 0.5f * (out_real[bf] + out_real[next_pos]);
                out_imag[next_pos] = 0.5f * (out_imag[bf] + out_imag[next_pos]);
                pos += (size_t)sym_len;
            }
        }

        /* Last sample = 0.5 * body_first of last DATA symbol */
        {
            size_t last_sym_start = 800 + (size_t)(n_sym - 1) * (size_t)sym_len;
            size_t last_bf = last_sym_start + (size_t)cp_len;
            out_real[out_idx - 1] = 0.5f * out_real[last_bf];
            out_imag[out_idx - 1] = 0.5f * out_imag[last_bf];
        }
    }

    return out_idx;
}

size_t lib80211_tx_vht(lib80211_fft_plan *plan,
                       const lib80211_tx_vht_params *params,
                       float *out_real, float *out_imag)
{
    size_t scratch_size = LIB80211_SCRATCH_TX_SIZE;
    uint8_t *mem = (uint8_t *)malloc(scratch_size);
    if (!mem) return 0;

    lib80211_scratch scratch;
    lib80211_scratch_init(&scratch, mem, scratch_size);

    size_t result = lib80211_tx_vht_s(plan, params, &scratch, out_real, out_imag);

    free(mem);
    return result;
}
