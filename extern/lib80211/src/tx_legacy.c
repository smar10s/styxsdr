/**
 * tx_legacy.c — Legacy 802.11a frame generation (all 8 rates).
 *
 * Assembles: STF + LTF + SIGNAL + DATA symbols.
 * DATA path: bytes->bits -> scramble -> conv_encode -> puncture -> interleave -> modulate -> OFDM
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

#include <string.h>
#include <stdlib.h>


size_t lib80211_tx_legacy_samples(const lib80211_tx_legacy_params *params) {
    const lib80211_rate_info *rate = lib80211_rate_lookup(params->rate_mbps);
    if (!rate) return 0;
    if (params->psdu_len == 0 || params->psdu_len > 4095) return 0;

    /* Number of OFDM symbols: ceil((16 + 8*length + 6) / n_dbps) */
    int n_bits = 16 + 8 * (int)params->psdu_len + 6;
    int n_sym = (n_bits + rate->n_dbps - 1) / rate->n_dbps;

    return LIB80211_STF_SAMPLES + LIB80211_LTF_SAMPLES +
           LIB80211_SYMBOL_LEN + /* SIGNAL */
           (size_t)n_sym * LIB80211_SYMBOL_LEN;  /* DATA */
}

size_t lib80211_tx_legacy_s(lib80211_fft_plan *plan,
                            const lib80211_tx_legacy_params *params,
                            lib80211_scratch *scratch,
                            float *out_real, float *out_imag) {
    const lib80211_rate_info *rate = lib80211_rate_lookup(params->rate_mbps);
    if (!rate) return 0;
    if (params->psdu_len == 0 || params->psdu_len > 4095) return 0;

    lib80211_scratch_reset(scratch);

    /* Compute frame dimensions */
    int n_bits_min = 16 + 8 * (int)params->psdu_len + 6;
    int n_sym = (n_bits_min + rate->n_dbps - 1) / rate->n_dbps;
    int n_data_bits = n_sym * rate->n_dbps;  /* Total bits including pad */

    size_t out_idx = 0;

    /* === Preamble === */
    lib80211_generate_stf(plan, &out_real[out_idx], &out_imag[out_idx]);
    out_idx += LIB80211_STF_SAMPLES;

    lib80211_generate_ltf(plan, &out_real[out_idx], &out_imag[out_idx]);
    out_idx += LIB80211_LTF_SAMPLES;

    /* === SIGNAL field === */
    /* For HT/VHT, L-SIG carries a spoofed length for NAV protection.
     * For legacy, it carries the actual parameters. */
    lib80211_make_lsig_symbol(plan, params->rate_mbps, (int)params->psdu_len,
                              &out_real[out_idx], &out_imag[out_idx]);
    out_idx += LIB80211_SYMBOL_LEN;

    /* === DATA field === */

    /* Allocate working buffers from scratch. */
    size_t alloc_data = (size_t)n_data_bits;
    size_t alloc_coded = (size_t)(n_data_bits * 2);
    uint8_t *data_bits = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_data, 0);
    uint8_t *scrambled = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_data, 0);
    uint8_t *coded     = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_coded, 0);
    uint8_t *punctured = (uint8_t *)lib80211_scratch_alloc(scratch, alloc_coded, 0);
    if (!data_bits || !scrambled || !coded || !punctured) return 0;
    memset(data_bits, 0, alloc_data);

    /* SERVICE: 16 zero bits (already zeroed) */

    /* PSDU: LSB-first per byte */
    for (size_t byte_idx = 0; byte_idx < params->psdu_len; byte_idx++) {
        uint8_t val = params->psdu[byte_idx];
        for (int bit = 0; bit < 8; bit++) {
            data_bits[16 + byte_idx * 8 + bit] = (val >> bit) & 1;
        }
    }
    /* TAIL (6 zeros) and PAD (zeros) are already set */

    /* Step 2: Scramble */
    lib80211_scramble(data_bits, scrambled, (size_t)n_data_bits, params->scrambler_seed);

    /* Zero the tail bits after scrambling (positions SERVICE+PSDU = 16+8*len .. +5) */
    int tail_start = 16 + 8 * (int)params->psdu_len;
    for (int i = tail_start; i < tail_start + 6; i++)
        scrambled[i] = 0;

    /* Step 3: Convolutional encode (no extra tail — tail already in stream) */
    lib80211_conv_encode(scrambled, coded, (size_t)n_data_bits, false);
    /* coded has n_data_bits * 2 bits */

    /* Step 4: Puncture to target rate */
    size_t n_punct = lib80211_puncture(coded, punctured,
                                       (size_t)(n_data_bits * 2),
                                       rate->cr_n, rate->cr_d);
    (void)n_punct;  /* Should equal n_coded_bits */

    /* Step 5: Interleave + modulate + OFDM symbol, one symbol at a time */
    for (int sym = 0; sym < n_sym; sym++) {
        uint8_t *sym_bits = &punctured[sym * rate->n_cbps];

        /* Interleave */
        uint8_t interleaved[288];  /* max n_cbps */
        lib80211_interleave(sym_bits, interleaved, rate->n_cbps, rate->n_bpsc);

        /* Modulate */
        float mod_real[48], mod_imag[48];
        lib80211_modulate(interleaved, mod_real, mod_imag, 48, rate->n_bpsc);

        /* OFDM symbol (symbol_index = sym + 1, since SIGNAL uses index 0) */
        lib80211_make_ofdm_symbol(plan, mod_real, mod_imag, sym + 1,
                                  &out_real[out_idx], &out_imag[out_idx]);
        out_idx += LIB80211_SYMBOL_LEN;
    }

    return out_idx;
}

size_t lib80211_tx_legacy(lib80211_fft_plan *plan,
                          const lib80211_tx_legacy_params *params,
                          float *out_real, float *out_imag)
{
    size_t scratch_size = LIB80211_SCRATCH_TX_SIZE;
    uint8_t *mem = (uint8_t *)malloc(scratch_size);
    if (!mem) return 0;

    lib80211_scratch scratch;
    lib80211_scratch_init(&scratch, mem, scratch_size);

    size_t result = lib80211_tx_legacy_s(plan, params, &scratch, out_real, out_imag);

    free(mem);
    return result;
}
