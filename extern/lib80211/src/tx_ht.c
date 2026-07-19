/**
 * tx_ht.c — HT-mixed (802.11n) frame generation, MCS 0-7, BCC.
 *
 * Frame structure:
 *   L-STF(160) | L-LTF(160) | L-SIG(80) | HT-SIG(160)
 *   | HT-STF(80) | HT-LTF(80) | HT-DATA(n_sym * symbol_len)
 *
 * DATA path: bytes->bits -> scramble -> conv_encode -> puncture ->
 *            HT-interleave -> modulate -> HT-OFDM symbols (52 subcarriers)
 *
 * TX normalization (IEEE TGn reference, 11-06/1715r0):
 *   Legacy fields (STF, LTF, L-SIG, HT-SIG, HT-STF): NFFT / sqrt(52)
 *   HT fields (HT-LTF, DATA): NFFT / sqrt(56)
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

/* TX normalization factors (per IEEE TGn reference) */
#define HT_NORM_LEGACY  (64.0f / sqrtf(52.0f))   /* ~8.875 */
#define HT_NORM_HT      (64.0f / sqrtf(56.0f))   /* ~8.552 */

/*
 * Maximum PSDU size for stack-based buffer allocation.
 * Matches the legacy single-MPDU maximum (4095 bytes).
 * Worst case at MCS 0 (n_dbps=26): ceil((16+32760+6)/26)=1261 symbols.
 * Peak stack usage: ~130 KB (data_bits + coded buffers).
 */
#define HT_MAX_PSDU      4095
#define HT_MAX_DATA_BITS (16 + HT_MAX_PSDU * 8 + 6 + 312)  /* SERVICE+PSDU+TAIL+maxPAD */
#define HT_MAX_CODED     (HT_MAX_DATA_BITS * 2)

size_t lib80211_tx_ht_samples(const lib80211_tx_ht_params *params) {
    if (params->mcs < 0 || params->mcs > 7) return 0;
    if (params->psdu_len > HT_MAX_PSDU) return 0;

    const lib80211_ht_mcs_info *mcs = &LIB80211_HT_MCS_TABLE[params->mcs];

    int n_sym;
    if (params->ldpc) {
        /* LDPC: no tail bits. Symbol count may increase due to ldpc_extra.
         * Conservative estimate: add 1 for possible extra symbol. */
        int n_bits = 16 + 8 * (int)params->psdu_len;
        n_sym = (n_bits + mcs->n_dbps - 1) / mcs->n_dbps + 1;
    } else {
        /* BCC: ceil((16 + 8*length + 6) / n_dbps) */
        int n_bits = 16 + 8 * (int)params->psdu_len + 6;
        n_sym = (n_bits + mcs->n_dbps - 1) / mcs->n_dbps;
    }

    int sym_len = params->short_gi ? LIB80211_SYMBOL_LEN_SGI : LIB80211_SYMBOL_LEN;

    /* L-STF + L-LTF + L-SIG + HT-SIG + HT-STF + HT-LTF + DATA */
    return 160 + 160 + 80 + 160 + 80 + 80 + (size_t)n_sym * (size_t)sym_len;
}

size_t lib80211_tx_ht_s(lib80211_fft_plan *plan,
                        const lib80211_tx_ht_params *params,
                        lib80211_scratch *scratch,
                        float *out_real, float *out_imag) {
    if (params->mcs < 0 || params->mcs > 7) return 0;
    if (params->psdu_len > HT_MAX_PSDU) return 0;

    lib80211_scratch_reset(scratch);

    const lib80211_ht_mcs_info *mcs = &LIB80211_HT_MCS_TABLE[params->mcs];
    int n_bpsc = mcs->n_bpsc;
    int n_cbps = mcs->n_cbps;
    int n_dbps = mcs->n_dbps;
    int cr_n = mcs->cr_n;
    int cr_d = mcs->cr_d;

    /* Compute frame dimensions */
    int n_bits_min = 16 + 8 * (int)params->psdu_len + 6;
    int n_sym = (n_bits_min + n_dbps - 1) / n_dbps;
    int n_data_bits = n_sym * n_dbps;

    int cp_len = params->short_gi ? LIB80211_NCP_SHORT : LIB80211_NCP;
    int sym_len = cp_len + LIB80211_NFFT;

    size_t out_idx = 0;

    /* === L-Preamble === */
    lib80211_generate_stf(plan, &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], LIB80211_STF_SAMPLES, HT_NORM_LEGACY);
    out_idx += LIB80211_STF_SAMPLES;

    lib80211_generate_ltf(plan, &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], LIB80211_LTF_SAMPLES, HT_NORM_LEGACY);
    out_idx += LIB80211_LTF_SAMPLES;

    /* === L-SIG (rate=6 Mbps, length encodes NAV protection) === */
    int lsig_length;
    if (params->short_gi) {
        lsig_length = 3 * ((int)ceil(n_sym * 0.9) + 4) - 3;
    } else {
        lsig_length = (4 + n_sym) * 3 - 3;
    }
    lib80211_make_lsig_symbol(plan, 6, lsig_length,
                              &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], LIB80211_SYMBOL_LEN, HT_NORM_LEGACY);
    out_idx += LIB80211_SYMBOL_LEN;

    /* === HT-SIG (2 OFDM symbols, 160 samples) === */
    lib80211_make_htsig_symbols(plan, params->mcs, (int)params->psdu_len,
                                params->short_gi, params->ldpc,
                                &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], 160, HT_NORM_LEGACY);
    out_idx += 160;

    /* === HT-STF (80 samples) === */
    lib80211_generate_ht_stf(plan, &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], 80, HT_NORM_LEGACY);
    out_idx += 80;

    /* === HT-LTF (80 samples, 1 spatial stream) === */
    lib80211_generate_ht_ltf(plan, &out_real[out_idx], &out_imag[out_idx]);
    lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], 80, HT_NORM_HT);
    out_idx += 80;

    /* === HT-DATA === */

    /* Allocate working buffers from scratch. */
    size_t alloc_data = (size_t)HT_MAX_DATA_BITS;
    size_t alloc_coded = (size_t)HT_MAX_CODED;
    uint8_t *data_bits = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_data, 0);
    uint8_t *scrambled = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_data, 0);
    uint8_t *coded     = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_coded, 0);
    if (!data_bits || !scrambled || !coded) return 0;

    if (params->ldpc) {
        /* --- LDPC path: no tail bits, no interleaving --- */

        /* Build payload: SERVICE(16) + PSDU bits */
        int n_payload = 16 + 8 * (int)params->psdu_len;
        memset(data_bits, 0, (size_t)n_payload);

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

        /* Scramble */
        lib80211_scramble(data_bits, scrambled, (size_t)scramble_len,
                          params->scrambler_seed);

        /* LDPC encode (handles codeword selection, shortening, puncturing, extra symbol) */
        int n_sym_ldpc = 0, ldpc_extra = 0;
        lib80211_ldpc_encode_data(scrambled, n_payload,
                                  n_dbps, n_cbps, cr_n, cr_d,
                                  coded, &n_sym_ldpc, &ldpc_extra);

        /* Build OFDM symbols — NO interleaving for LDPC */
        for (int s = 0; s < n_sym_ldpc; s++) {
            uint8_t *sym_bits = &coded[s * n_cbps];

            /* No interleaving — map directly to constellation */
            float mod_real[52], mod_imag[52];
            lib80211_modulate(sym_bits, mod_real, mod_imag, 52, n_bpsc);

            /* HT OFDM symbol */
            lib80211_make_ht_ofdm_symbol(plan, mod_real, mod_imag, s, cp_len,
                                         &out_real[out_idx], &out_imag[out_idx]);
            lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], (size_t)sym_len, HT_NORM_HT);
            out_idx += (size_t)sym_len;
        }
    } else {
        /* --- BCC path (original behavior) --- */
        uint8_t *punctured = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_coded, 0);
        if (!punctured) return 0;

        /* Step 1: Build input bit stream: SERVICE(16) + PSDU + TAIL(6) + PAD */
        memset(data_bits, 0, (size_t)n_data_bits);

        /* PSDU: LSB-first per byte */
        for (size_t byte_idx = 0; byte_idx < params->psdu_len; byte_idx++) {
            uint8_t val = params->psdu[byte_idx];
            for (int bit = 0; bit < 8; bit++) {
                data_bits[16 + byte_idx * 8 + bit] = (val >> bit) & 1;
            }
        }

        /* Step 2: Scramble */
        lib80211_scramble(data_bits, scrambled, (size_t)n_data_bits,
                          params->scrambler_seed);

        /* Zero the tail bits after scrambling */
        int tail_start = 16 + 8 * (int)params->psdu_len;
        for (int i = tail_start; i < tail_start + 6 && i < n_data_bits; i++)
            scrambled[i] = 0;

        /* Step 3: Convolutional encode (rate-1/2) */
        lib80211_conv_encode(scrambled, coded, (size_t)n_data_bits, false);

        /* Step 4: Puncture to target rate */
        int total_coded_needed = n_sym * n_cbps;

        if (cr_n == 1 && cr_d == 2) {
            /* Rate 1/2: no puncturing needed, just use coded directly */
            memcpy(punctured, coded, (size_t)total_coded_needed);
        } else {
            /* Determine how many rate-1/2 bits we need to feed the puncturer */
            int pre_punct;
            if (cr_n == 2 && cr_d == 3)
                pre_punct = (total_coded_needed * 4) / 3;
            else if (cr_n == 3 && cr_d == 4)
                pre_punct = (total_coded_needed * 6) / 4;
            else /* cr_n == 5 && cr_d == 6 */
                pre_punct = (total_coded_needed * 10) / 6;

            /* Puncture */
            lib80211_puncture(coded, punctured, (size_t)pre_punct, cr_n, cr_d);
        }

        /* Step 5: Per-symbol: HT-interleave -> modulate -> HT OFDM symbol */
        for (int s = 0; s < n_sym; s++) {
            uint8_t *sym_bits = &punctured[s * n_cbps];

            /* HT Interleave (N_col=13) */
            uint8_t interleaved[416];  /* max n_cbps = 312 (64-QAM) */
            lib80211_interleave_ht(sym_bits, interleaved, n_cbps, n_bpsc);

            /* Modulate (52 subcarriers) */
            float mod_real[52], mod_imag[52];
            lib80211_modulate(interleaved, mod_real, mod_imag, 52, n_bpsc);

            /* HT OFDM symbol */
            lib80211_make_ht_ofdm_symbol(plan, mod_real, mod_imag, s, cp_len,
                                         &out_real[out_idx], &out_imag[out_idx]);
            lib80211_scale_iq(&out_real[out_idx], &out_imag[out_idx], (size_t)sym_len, HT_NORM_HT);
            out_idx += (size_t)sym_len;
        }
    }

    return out_idx;
}

size_t lib80211_tx_ht(lib80211_fft_plan *plan,
                      const lib80211_tx_ht_params *params,
                      float *out_real, float *out_imag)
{
    size_t scratch_size = LIB80211_SCRATCH_TX_SIZE;
    uint8_t *mem = (uint8_t *)malloc(scratch_size);
    if (!mem) return 0;

    lib80211_scratch scratch;
    lib80211_scratch_init(&scratch, mem, scratch_size);

    size_t result = lib80211_tx_ht_s(plan, params, &scratch, out_real, out_imag);

    free(mem);
    return result;
}
