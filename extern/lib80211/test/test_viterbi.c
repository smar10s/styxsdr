/**
 * test_viterbi.c — Validate soft-decision Viterbi decoder (K=7, rate-1/2)
 *
 * Tests:
 *  1. Round-trip: encode known pattern -> soft -> decode -> verify match
 *  2. Multiple lengths (48, 192, 864)
 *  3. Annex I.1: encode annex_i1_data_scrambled at rate 1/2, decode, verify
 */

#include "test_util.h"
#include "lib80211/fec.h"
#include "lib80211/scrambler.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Convert hard bits (0/1) to soft LLRs.
 * Convention: positive = bit 1 more likely.
 * So bit=1 -> +1.0, bit=0 -> -1.0.
 */
static void hard_to_soft(const uint8_t *hard, float *soft, size_t n) {
    for (size_t i = 0; i < n; i++) {
        soft[i] = hard[i] ? 1.0f : -1.0f;
    }
}

/**
 * Generate a deterministic test pattern of n_bits.
 * Uses a simple LFSR-like pattern for variety.
 */
static void generate_test_pattern(uint8_t *bits, size_t n_bits) {
    uint8_t state = 0xA7;  /* arbitrary seed */
    for (size_t i = 0; i < n_bits; i++) {
        bits[i] = (state >> 7) & 1;
        /* x^8 + x^6 + x^5 + x^4 + 1 */
        uint8_t fb = ((state >> 7) ^ (state >> 5) ^ (state >> 4) ^ (state >> 3)) & 1;
        state = (uint8_t)((state << 1) | fb);
    }
}

/**
 * Test: encode known bits -> convert to soft -> decode -> verify exact match.
 */
static void test_roundtrip(size_t n_data_bits) {
    char name[64];
    snprintf(name, sizeof(name), "viterbi_roundtrip_%zu", n_data_bits);
    TEST_BEGIN(name);

    uint8_t *data = malloc(n_data_bits);
    if (!data) { TEST_FAIL("malloc data"); return; }

    generate_test_pattern(data, n_data_bits);

    /* Encode with tail bits */
    size_t n_coded = 2 * (n_data_bits + 6);
    uint8_t *coded = malloc(n_coded);
    if (!coded) { TEST_FAIL("malloc coded"); free(data); return; }

    lib80211_conv_encode(data, coded, n_data_bits, true);

    /* Convert to soft bits */
    float *soft = malloc(n_coded * sizeof(float));
    if (!soft) { TEST_FAIL("malloc soft"); free(coded); free(data); return; }

    hard_to_soft(coded, soft, n_coded);

    /* Decode */
    uint8_t *decoded = malloc(n_data_bits);
    if (!decoded) { TEST_FAIL("malloc decoded"); free(soft); free(coded); free(data); return; }

    lib80211_viterbi_decode(soft, decoded, n_coded, n_data_bits);

    /* Verify */
    if (assert_bits_equal(data, decoded, n_data_bits, name)) {
        TEST_PASS();
    } else {
        TEST_FAIL("decoded bits mismatch");
    }

    free(decoded);
    free(soft);
    free(coded);
    free(data);
}

/**
 * Test: Annex I.1 rate-1/2 encode + decode round-trip.
 *
 * annex_i1_data_scrambled is 864 bits (SERVICE + PSDU + TAIL + PAD).
 * Encoding with add_tail=false gives 1728 coded bits.
 * Decode with n_data_bits=858 (864-6 tail), verify first 858 bits match.
 */
static void test_annex_i1_roundtrip(void) {
    TEST_BEGIN("viterbi_annex_i1_roundtrip");

    /* Build scrambled input exactly as test_conv_encode does */
    test_vector *psdu_vec = vector_load("annex_i1_psdu");
    if (!psdu_vec || !psdu_vec->hex_octets) {
        TEST_FAIL("cannot load annex_i1_psdu");
        if (psdu_vec) vector_free(psdu_vec);
        return;
    }

    uint8_t *input = calloc(864, 1);
    if (!input) { TEST_FAIL("calloc"); vector_free(psdu_vec); return; }

    /* SERVICE(16 zeros) + PSDU(800 bits LSB-first) + tail(6) + pad(42) */
    for (size_t byte_idx = 0; byte_idx < 100 && byte_idx < psdu_vec->n_octets; byte_idx++) {
        uint8_t val = (uint8_t)strtoul(psdu_vec->hex_octets[byte_idx], NULL, 16);
        for (int bit = 0; bit < 8; bit++) {
            input[16 + byte_idx * 8 + bit] = (val >> bit) & 1;
        }
    }

    uint8_t *scrambled = malloc(864);
    if (!scrambled) { TEST_FAIL("malloc"); free(input); vector_free(psdu_vec); return; }

    lib80211_scramble(input, scrambled, 864, 0x5D);

    /* Zero tail bits after scrambling */
    for (int i = 816; i < 822; i++)
        scrambled[i] = 0;

    free(input);
    vector_free(psdu_vec);

    /* Encode at rate 1/2 without adding extra tail (tail embedded in 864) */
    uint8_t *coded = malloc(1728);
    if (!coded) { TEST_FAIL("malloc coded"); free(scrambled); return; }

    lib80211_conv_encode(scrambled, coded, 864, false);

    /* Convert to soft */
    float *soft = malloc(1728 * sizeof(float));
    if (!soft) { TEST_FAIL("malloc soft"); free(coded); free(scrambled); return; }

    hard_to_soft(coded, soft, 1728);

    /* Decode: n_data_bits = 858 (864 - 6 tail bits) */
    size_t n_data_bits = 858;
    uint8_t *decoded = malloc(n_data_bits);
    if (!decoded) { TEST_FAIL("malloc decoded"); free(soft); free(coded); free(scrambled); return; }

    lib80211_viterbi_decode(soft, decoded, 1728, n_data_bits);

    /* Verify first 858 decoded bits match first 858 scrambled bits */
    if (assert_bits_equal(scrambled, decoded, n_data_bits, "annex_i1_decoded")) {
        TEST_PASS();
    } else {
        TEST_FAIL("annex I.1 decoded bits mismatch");
    }

    free(decoded);
    free(soft);
    free(coded);
    free(scrambled);
}

/**
 * Test: Annex I.1 full decode with depuncturing.
 *
 * Load annex_i1_data_encoded (1152 punctured bits at rate 3/4),
 * convert to soft, depuncture to 1728 soft bits, Viterbi decode
 * with n_data_bits=858, compare against first 858 bits of annex_i1_data_scrambled.
 */
static void test_annex_i1_depuncture_decode(void) {
    TEST_BEGIN("viterbi_annex_i1_depuncture_decode");

    /* Load the punctured encoded vector (1152 bits at rate 3/4) */
    test_vector *enc_vec = vector_load("annex_i1_data_encoded");
    if (!enc_vec || !enc_vec->bits || enc_vec->n_bits != 1152) {
        TEST_FAIL("cannot load annex_i1_data_encoded (expected 1152 bits)");
        if (enc_vec) vector_free(enc_vec);
        return;
    }

    /* Load the scrambled reference (864 bits) */
    test_vector *scr_vec = vector_load("annex_i1_data_scrambled");
    if (!scr_vec || !scr_vec->bits || scr_vec->n_bits != 864) {
        TEST_FAIL("cannot load annex_i1_data_scrambled (expected 864 bits)");
        vector_free(enc_vec);
        if (scr_vec) vector_free(scr_vec);
        return;
    }

    /* Convert punctured hard bits to soft: 0 -> -1.0, 1 -> +1.0 */
    float *soft_punctured = malloc(1152 * sizeof(float));
    if (!soft_punctured) {
        TEST_FAIL("malloc");
        vector_free(enc_vec);
        vector_free(scr_vec);
        return;
    }
    hard_to_soft(enc_vec->bits, soft_punctured, 1152);

    /* Depuncture: 1152 soft bits at rate 3/4 -> 1728 soft bits (rate 1/2) */
    float *soft_depunctured = malloc(1728 * sizeof(float));
    if (!soft_depunctured) {
        TEST_FAIL("malloc");
        free(soft_punctured);
        vector_free(enc_vec);
        vector_free(scr_vec);
        return;
    }

    size_t n_depunctured = lib80211_depuncture(soft_punctured, soft_depunctured,
                                               1152, 3, 4);
    if (n_depunctured != 1728) {
        TEST_FAIL("depuncture output: expected 1728, got %zu", n_depunctured);
        free(soft_depunctured);
        free(soft_punctured);
        vector_free(enc_vec);
        vector_free(scr_vec);
        return;
    }

    /*
     * Viterbi decode: The Annex I.1 frame has tail bits at positions 816-821
     * which flush the encoder to state 0. After that are pad bits (822-863)
     * which are scrambled and drive the encoder away from state 0.
     * So we decode with n_data_bits=816 to traceback from state 0 at step 822,
     * which is where the encoder actually reaches state 0.
     */
    size_t n_data_bits = 816;
    uint8_t *decoded = malloc(n_data_bits);
    if (!decoded) {
        TEST_FAIL("malloc");
        free(soft_depunctured);
        free(soft_punctured);
        vector_free(enc_vec);
        vector_free(scr_vec);
        return;
    }

    lib80211_viterbi_decode(soft_depunctured, decoded, 1728, n_data_bits);

    /* Verify first 816 decoded bits match first 816 scrambled bits */
    if (assert_bits_equal(scr_vec->bits, decoded, n_data_bits,
                          "annex_i1_depuncture_decode")) {
        TEST_PASS();
    } else {
        TEST_FAIL("annex I.1 depuncture+decode mismatch");
    }

    free(decoded);
    free(soft_depunctured);
    free(soft_punctured);
    vector_free(enc_vec);
    vector_free(scr_vec);
}

/**
 * Test: encode -> puncture -> depuncture -> decode round-trip at rate 3/4.
 *
 * Encode 192 bits, puncture at 3/4, convert to soft, depuncture, decode,
 * verify match against original.
 */
static void test_depuncture_roundtrip_34(void) {
    TEST_BEGIN("viterbi_depuncture_roundtrip_34");

    size_t n_data_bits = 192;
    uint8_t *data = malloc(n_data_bits);
    if (!data) { TEST_FAIL("malloc"); return; }

    generate_test_pattern(data, n_data_bits);

    /* Encode with tail bits: 192 + 6 = 198 input -> 396 coded bits */
    size_t n_coded = 2 * (n_data_bits + 6);  /* 396 */
    uint8_t *coded = malloc(n_coded);
    if (!coded) { TEST_FAIL("malloc"); free(data); return; }

    lib80211_conv_encode(data, coded, n_data_bits, true);

    /* Puncture at rate 3/4: 396 coded bits -> 264 punctured bits */
    /* Pattern 3/4: {1,1,1,0,0,1}, 6 in -> 4 out. 396/6 = 66 groups -> 264 out */
    uint8_t *punctured = malloc(n_coded);  /* oversize is fine */
    if (!punctured) { TEST_FAIL("malloc"); free(coded); free(data); return; }

    size_t n_punctured = lib80211_puncture(coded, punctured, n_coded, 3, 4);
    if (n_punctured != 264) {
        TEST_FAIL("puncture output: expected 264, got %zu", n_punctured);
        free(punctured); free(coded); free(data);
        return;
    }

    /* Convert to soft */
    float *soft_punctured = malloc(n_punctured * sizeof(float));
    if (!soft_punctured) { TEST_FAIL("malloc"); free(punctured); free(coded); free(data); return; }
    hard_to_soft(punctured, soft_punctured, n_punctured);

    /* Depuncture: 264 -> 396 */
    float *soft_depunctured = malloc(n_coded * sizeof(float));
    if (!soft_depunctured) { TEST_FAIL("malloc"); free(soft_punctured); free(punctured); free(coded); free(data); return; }

    size_t n_depunctured = lib80211_depuncture(soft_punctured, soft_depunctured,
                                               n_punctured, 3, 4);
    if (n_depunctured != n_coded) {
        TEST_FAIL("depuncture output: expected %zu, got %zu", n_coded, n_depunctured);
        free(soft_depunctured); free(soft_punctured); free(punctured); free(coded); free(data);
        return;
    }

    /* Decode */
    uint8_t *decoded = malloc(n_data_bits);
    if (!decoded) { TEST_FAIL("malloc"); free(soft_depunctured); free(soft_punctured); free(punctured); free(coded); free(data); return; }

    lib80211_viterbi_decode(soft_depunctured, decoded, n_coded, n_data_bits);

    /* Verify */
    if (assert_bits_equal(data, decoded, n_data_bits, "depuncture_roundtrip_34")) {
        TEST_PASS();
    } else {
        TEST_FAIL("depuncture round-trip mismatch");
    }

    free(decoded);
    free(soft_depunctured);
    free(soft_punctured);
    free(punctured);
    free(coded);
    free(data);
}

int main(void) {
    printf("test_viterbi\n");

    /* Round-trip tests at multiple lengths */
    test_roundtrip(48);
    test_roundtrip(192);
    test_roundtrip(864);

    /* Annex I.1 test (rate 1/2) */
    test_annex_i1_roundtrip();

    /* Depuncture + decode tests */
    test_annex_i1_depuncture_decode();
    test_depuncture_roundtrip_34();

    TEST_SUMMARY();
    return TEST_EXIT();
}
