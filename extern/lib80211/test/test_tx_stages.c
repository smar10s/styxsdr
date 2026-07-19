/**
 * test_tx_stages.c — Validate HT/VHT TX pipeline stages independently
 * against IEEE TGn/TGac golden vectors.
 *
 * Tests:
 *   1. HT Scramble (MCS 0-7): SERVICE+PSDU → scramble → compare to vector
 *   2. HT Encode  (MCS 0-7): scrambled vector → conv_encode+puncture → compare
 *   3. VHT Scramble (MCS 0-8): SERVICE(w/ SIG-B CRC)+PSDU → scramble → compare
 *   4. VHT Encode  (MCS 0-8): scrambled vector → conv_encode+puncture → compare
 */

#include "test_util.h"
#include "lib80211/scrambler.h"
#include "lib80211/fec.h"
#include "lib80211/constants.h"
#include "lib80211/signal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Annex I.1 100-byte PSDU (same as test_tx_ht.c) */
static const uint8_t TEST_PSDU[100] = {
    0x04, 0x02, 0x00, 0x2E, 0x00, 0x60, 0x08, 0xCD, 0x37, 0xA6,
    0x00, 0x20, 0xD6, 0x01, 0x3C, 0xF1, 0x00, 0x60, 0x08, 0xAD,
    0x3B, 0xAF, 0x00, 0x00, 0x4A, 0x6F, 0x79, 0x2C, 0x20, 0x62,
    0x72, 0x69, 0x67, 0x68, 0x74, 0x20, 0x73, 0x70, 0x61, 0x72,
    0x6B, 0x20, 0x6F, 0x66, 0x20, 0x64, 0x69, 0x76, 0x69, 0x6E,
    0x69, 0x74, 0x79, 0x2C, 0x0A, 0x44, 0x61, 0x75, 0x67, 0x68,
    0x74, 0x65, 0x72, 0x20, 0x6F, 0x66, 0x20, 0x45, 0x6C, 0x79,
    0x73, 0x69, 0x75, 0x6D, 0x2C, 0x0A, 0x46, 0x69, 0x72, 0x65,
    0x2D, 0x69, 0x6E, 0x73, 0x69, 0x72, 0x65, 0x64, 0x20, 0x77,
    0x65, 0x20, 0x74, 0x72, 0x65, 0x61, 0x67, 0x33, 0x21, 0xB6,
};

#define PSDU_LEN       100
#define PSDU_BITS      (PSDU_LEN * 8)  /* 800 */
#define SERVICE_BITS   16
#define TAIL_BITS      6
#define SCRAMBLER_SEED 0x5D

/* ========================================================================
 * HT Scramble tests (MCS 0-7)
 *
 * Algorithm (matches Python ht_build_scrambled_bits):
 *   1. SERVICE(16 zeros) + PSDU bits (LSB first) + tail(6 zeros) = "all_bits"
 *   2. n_sym = ceil(len(all_bits) / n_dbps)
 *   3. Pad with zeros to n_sym * n_dbps
 *   4. Scramble entire padded stream with seed 0x5D
 *   5. Zero tail bits at positions [16+800 .. 16+800+5] after scrambling
 * ======================================================================== */

static void test_ht_scramble(int mcs) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "ht_scramble_mcs%d", mcs);
    TEST_BEGIN(test_name);

    /* Load expected vector */
    char vec_name[64];
    snprintf(vec_name, sizeof(vec_name), "ht_mcs%d_scrambled", mcs);
    test_vector *vec = vector_load(vec_name);
    if (!vec || !vec->bits || vec->n_bits == 0) {
        TEST_FAIL("cannot load %s", vec_name);
        if (vec) vector_free(vec);
        return;
    }

    const lib80211_ht_mcs_info *info = &LIB80211_HT_MCS_TABLE[mcs];
    int n_dbps = info->n_dbps;

    /* Compute total length: ceil((SERVICE + PSDU + TAIL) / n_dbps) * n_dbps */
    int all_bits_len = SERVICE_BITS + PSDU_BITS + TAIL_BITS;
    int n_sym = (all_bits_len + n_dbps - 1) / n_dbps;
    int n_total = n_sym * n_dbps;

    /* Build input bit stream */
    uint8_t *input = calloc((size_t)n_total, 1);
    if (!input) {
        TEST_FAIL("malloc failed");
        vector_free(vec);
        return;
    }

    /* SERVICE field: 16 zero bits (already zero from calloc) */
    /* PSDU: bytes to bits, LSB first per byte */
    for (int byte_idx = 0; byte_idx < PSDU_LEN; byte_idx++) {
        for (int bit = 0; bit < 8; bit++) {
            input[SERVICE_BITS + byte_idx * 8 + bit] =
                (TEST_PSDU[byte_idx] >> bit) & 1;
        }
    }
    /* Tail bits + pad: all zeros (already from calloc) */

    /* Scramble */
    uint8_t *scrambled = malloc((size_t)n_total);
    if (!scrambled) {
        TEST_FAIL("malloc failed");
        free(input);
        vector_free(vec);
        return;
    }
    lib80211_scramble(input, scrambled, (size_t)n_total, SCRAMBLER_SEED);

    /* NOTE: The reference vector stores scrambler output BEFORE tail zeroing.
     * Tail zeroing is a separate step applied before encoding. */

    /* Verify length matches vector */
    if ((size_t)n_total != vec->n_bits) {
        TEST_FAIL("length mismatch: computed %d, vector has %zu",
                  n_total, vec->n_bits);
        free(input);
        free(scrambled);
        vector_free(vec);
        return;
    }

    /* Compare bits */
    if (assert_bits_equal(vec->bits, scrambled, vec->n_bits, test_name)) {
        TEST_PASS();
    } else {
        TEST_FAIL("bit mismatch");
    }

    free(input);
    free(scrambled);
    vector_free(vec);
}

/* ========================================================================
 * HT Encode tests (MCS 0-7)
 *
 * Load scrambled vector as input, conv_encode (rate 1/2, no tail),
 * then puncture to the MCS's code rate. Compare to encoded vector.
 * ======================================================================== */

static void test_ht_encode(int mcs) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "ht_encode_mcs%d", mcs);
    TEST_BEGIN(test_name);

    /* Load scrambled input */
    char scr_name[64];
    snprintf(scr_name, sizeof(scr_name), "ht_mcs%d_scrambled", mcs);
    test_vector *scr_vec = vector_load(scr_name);
    if (!scr_vec || !scr_vec->bits || scr_vec->n_bits == 0) {
        TEST_FAIL("cannot load %s", scr_name);
        if (scr_vec) vector_free(scr_vec);
        return;
    }

    /* Load expected encoded output */
    char enc_name[64];
    snprintf(enc_name, sizeof(enc_name), "ht_mcs%d_encoded", mcs);
    test_vector *enc_vec = vector_load(enc_name);
    if (!enc_vec || !enc_vec->bits || enc_vec->n_bits == 0) {
        TEST_FAIL("cannot load %s", enc_name);
        vector_free(scr_vec);
        if (enc_vec) vector_free(enc_vec);
        return;
    }

    const lib80211_ht_mcs_info *info = &LIB80211_HT_MCS_TABLE[mcs];
    size_t n_in = scr_vec->n_bits;

    /* The scrambled vector stores raw scrambler output. Zero tail bits
     * (positions 816-821) before encoding, matching the TX pipeline. */
    int tail_start = SERVICE_BITS + PSDU_BITS; /* 816 */
    for (int i = 0; i < TAIL_BITS && (size_t)(tail_start + i) < n_in; i++) {
        scr_vec->bits[tail_start + i] = 0;
    }

    /* Rate-1/2 encode (no tail — tail bits are already in the scrambled stream) */
    size_t n_coded = n_in * 2;
    uint8_t *coded = malloc(n_coded);
    if (!coded) {
        TEST_FAIL("malloc failed");
        vector_free(scr_vec);
        vector_free(enc_vec);
        return;
    }
    lib80211_conv_encode(scr_vec->bits, coded, n_in, false);

    /* Puncture */
    uint8_t *punctured = malloc(n_coded); /* upper bound */
    if (!punctured) {
        TEST_FAIL("malloc failed");
        free(coded);
        vector_free(scr_vec);
        vector_free(enc_vec);
        return;
    }
    size_t n_punctured = lib80211_puncture(coded, punctured, n_coded,
                                           info->cr_n, info->cr_d);
    if (n_punctured == 0) {
        TEST_FAIL("puncture returned 0 (unsupported rate %d/%d?)",
                  info->cr_n, info->cr_d);
        free(coded);
        free(punctured);
        vector_free(scr_vec);
        vector_free(enc_vec);
        return;
    }

    /* Compare: use minimum of computed and expected lengths */
    size_t compare_len = n_punctured < enc_vec->n_bits ?
                         n_punctured : enc_vec->n_bits;

    if (n_punctured != enc_vec->n_bits) {
        TEST_FAIL("encoded length mismatch: got %zu, expected %zu",
                  n_punctured, enc_vec->n_bits);
    } else if (assert_bits_equal(enc_vec->bits, punctured, compare_len, test_name)) {
        TEST_PASS();
    } else {
        TEST_FAIL("bit mismatch");
    }

    free(coded);
    free(punctured);
    vector_free(scr_vec);
    vector_free(enc_vec);
}

/* ========================================================================
 * VHT Scramble tests (MCS 0-8)
 *
 * VHT scrambling convention (TGac bcc_encoder.m):
 *   1. SERVICE field: bits 0-7 zeros, bits 8-15 = CRC-8 of VHT-SIG-B
 *      (ones-complemented, MSB-first)
 *   2. Scramble SERVICE + PSDU + PAD (no tail in scrambler input)
 *   3. Append 6 zero tail bits after scrambling
 *
 * n_sym = ceil((16 + 8*PSDU_LEN + 6) / n_dbps)
 * n_total = n_sym * n_dbps
 * n_pad = n_total - (16 + 8*PSDU_LEN + 6)
 * scrambler input = SERVICE + PSDU + PAD  (length = n_total - 6)
 * output = scramble(input) + [0]*6
 * ======================================================================== */

static void test_vht_scramble(int mcs) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "vht_scramble_mcs%d", mcs);
    TEST_BEGIN(test_name);

    /* Load expected vector */
    char vec_name[64];
    snprintf(vec_name, sizeof(vec_name), "vht_mcs%d_scrambled", mcs);
    test_vector *vec = vector_load(vec_name);
    if (!vec || !vec->bits || vec->n_bits == 0) {
        TEST_FAIL("cannot load %s", vec_name);
        if (vec) vector_free(vec);
        return;
    }

    const lib80211_ht_mcs_info *info = &LIB80211_VHT_MCS_TABLE[mcs];
    int n_dbps = info->n_dbps;

    /* Compute symbol count and total bits */
    int n_data_bits = SERVICE_BITS + PSDU_BITS;  /* 16 + 800 = 816 */
    int n_sym = (n_data_bits + TAIL_BITS + n_dbps - 1) / n_dbps;
    int n_total = n_sym * n_dbps;
    int n_pad = n_total - n_data_bits - TAIL_BITS;

    /* Scrambler input length = SERVICE + PSDU + PAD (no tail) */
    int scr_input_len = n_data_bits + n_pad;  /* = n_total - 6 */

    /* Build SERVICE field with VHT-SIG-B CRC in bits 8-15 */
    uint8_t *input = calloc((size_t)scr_input_len, 1);
    if (!input) {
        TEST_FAIL("malloc failed");
        vector_free(vec);
        return;
    }

    /* SERVICE bits 0-7: zeros (already from calloc) */
    /* SERVICE bits 8-15: CRC-8 of VHT-SIG-B first 20 bits (ones-comp, MSB-first) */
    uint8_t sigb_bits[26];
    lib80211_make_vhtsigb_bits(PSDU_LEN, sigb_bits);
    uint8_t sigb_crc = lib80211_htsig_crc8(sigb_bits, 20);
    uint8_t sigb_crc_inv = (uint8_t)(~sigb_crc & 0xFF);
    for (int i = 0; i < 8; i++) {
        input[8 + i] = (uint8_t)((sigb_crc_inv >> (7 - i)) & 1);
    }

    /* PSDU: bytes to bits, LSB first per byte */
    for (int byte_idx = 0; byte_idx < PSDU_LEN; byte_idx++) {
        for (int bit = 0; bit < 8; bit++) {
            input[SERVICE_BITS + byte_idx * 8 + bit] =
                (TEST_PSDU[byte_idx] >> bit) & 1;
        }
    }
    /* PAD: zeros (already from calloc) */

    /* Scramble (no tail bits in input) */
    uint8_t *scrambled = malloc((size_t)n_total);
    if (!scrambled) {
        TEST_FAIL("malloc failed");
        free(input);
        vector_free(vec);
        return;
    }
    lib80211_scramble(input, scrambled, (size_t)scr_input_len, SCRAMBLER_SEED);

    /* Append 6 zero tail bits */
    for (int i = 0; i < TAIL_BITS; i++) {
        scrambled[scr_input_len + i] = 0;
    }

    /* Verify length */
    if ((size_t)n_total != vec->n_bits) {
        TEST_FAIL("length mismatch: computed %d, vector has %zu",
                  n_total, vec->n_bits);
        free(input);
        free(scrambled);
        vector_free(vec);
        return;
    }

    /* Compare bits */
    if (assert_bits_equal(vec->bits, scrambled, vec->n_bits, test_name)) {
        TEST_PASS();
    } else {
        TEST_FAIL("bit mismatch");
    }

    free(input);
    free(scrambled);
    vector_free(vec);
}

/* ========================================================================
 * VHT Encode tests (MCS 0-8)
 *
 * Same approach as HT encode: load scrambled vector, encode, puncture,
 * compare to encoded vector.
 * ======================================================================== */

static void test_vht_encode(int mcs) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "vht_encode_mcs%d", mcs);
    TEST_BEGIN(test_name);

    /* Load scrambled input */
    char scr_name[64];
    snprintf(scr_name, sizeof(scr_name), "vht_mcs%d_scrambled", mcs);
    test_vector *scr_vec = vector_load(scr_name);
    if (!scr_vec || !scr_vec->bits || scr_vec->n_bits == 0) {
        TEST_FAIL("cannot load %s", scr_name);
        if (scr_vec) vector_free(scr_vec);
        return;
    }

    /* Load expected encoded output */
    char enc_name[64];
    snprintf(enc_name, sizeof(enc_name), "vht_mcs%d_encoded", mcs);
    test_vector *enc_vec = vector_load(enc_name);
    if (!enc_vec || !enc_vec->bits || enc_vec->n_bits == 0) {
        TEST_FAIL("cannot load %s", enc_name);
        vector_free(scr_vec);
        if (enc_vec) vector_free(enc_vec);
        return;
    }

    const lib80211_ht_mcs_info *info = &LIB80211_VHT_MCS_TABLE[mcs];
    size_t n_in = scr_vec->n_bits;

    /* Rate-1/2 encode (no tail) */
    size_t n_coded = n_in * 2;
    uint8_t *coded = malloc(n_coded);
    if (!coded) {
        TEST_FAIL("malloc failed");
        vector_free(scr_vec);
        vector_free(enc_vec);
        return;
    }
    lib80211_conv_encode(scr_vec->bits, coded, n_in, false);

    /* Puncture */
    uint8_t *punctured = malloc(n_coded);
    if (!punctured) {
        TEST_FAIL("malloc failed");
        free(coded);
        vector_free(scr_vec);
        vector_free(enc_vec);
        return;
    }
    size_t n_punctured = lib80211_puncture(coded, punctured, n_coded,
                                           info->cr_n, info->cr_d);
    if (n_punctured == 0) {
        TEST_FAIL("puncture returned 0 (unsupported rate %d/%d?)",
                  info->cr_n, info->cr_d);
        free(coded);
        free(punctured);
        vector_free(scr_vec);
        vector_free(enc_vec);
        return;
    }

    /* Compare */
    if (n_punctured != enc_vec->n_bits) {
        TEST_FAIL("encoded length mismatch: got %zu, expected %zu",
                  n_punctured, enc_vec->n_bits);
    } else if (assert_bits_equal(enc_vec->bits, punctured, n_punctured, test_name)) {
        TEST_PASS();
    } else {
        TEST_FAIL("bit mismatch");
    }

    free(coded);
    free(punctured);
    vector_free(scr_vec);
    vector_free(enc_vec);
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(void) {
    printf("=== TX Pipeline Stage Validation (HT/VHT) ===\n\n");

    printf("--- HT Scramble (MCS 0-7) ---\n");
    for (int mcs = 0; mcs < 8; mcs++)
        test_ht_scramble(mcs);

    printf("\n--- HT Encode (MCS 0-7) ---\n");
    for (int mcs = 0; mcs < 8; mcs++)
        test_ht_encode(mcs);

    printf("\n--- VHT Scramble (MCS 0-8) ---\n");
    for (int mcs = 0; mcs < 9; mcs++)
        test_vht_scramble(mcs);

    printf("\n--- VHT Encode (MCS 0-8) ---\n");
    for (int mcs = 0; mcs < 9; mcs++)
        test_vht_encode(mcs);

    printf("\n");
    TEST_SUMMARY();
    return TEST_EXIT();
}
