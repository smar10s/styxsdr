/**
 * test_conv_encode.c — Validate convolutional encoder and puncturing
 * against Annex I.1 golden vector (rate 36 Mbps = rate 3/4, 16-QAM).
 */

#include "test_util.h"
#include "lib80211/fec.h"
#include "lib80211/scrambler.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ANNEX_I1_SEED 0x5D

/**
 * Build the 864 scrambled input bits for Annex I.1.
 * Returns a heap-allocated array (caller must free).
 */
static uint8_t *build_scrambled_input(void) {
    test_vector *psdu_vec = vector_load("annex_i1_psdu");
    if (!psdu_vec || !psdu_vec->hex_octets) return NULL;

    uint8_t *input = calloc(864, 1);
    if (!input) { vector_free(psdu_vec); return NULL; }

    /* SERVICE(16 zeros) + PSDU(800 bits LSB-first) + tail(6) + pad(42) */
    for (size_t byte_idx = 0; byte_idx < 100 && byte_idx < psdu_vec->n_octets; byte_idx++) {
        uint8_t val = (uint8_t)strtoul(psdu_vec->hex_octets[byte_idx], NULL, 16);
        for (int bit = 0; bit < 8; bit++) {
            input[16 + byte_idx * 8 + bit] = (val >> bit) & 1;
        }
    }

    uint8_t *scrambled = malloc(864);
    if (!scrambled) { free(input); vector_free(psdu_vec); return NULL; }

    lib80211_scramble(input, scrambled, 864, ANNEX_I1_SEED);

    /* Per spec: tail bits (positions 816..821) zeroed AFTER scrambling */
    for (int i = 816; i < 822; i++)
        scrambled[i] = 0;

    free(input);
    vector_free(psdu_vec);
    return scrambled;
}

/**
 * Test: scramble -> conv_encode(no tail) -> puncture(3/4) matches golden vector.
 *
 * The Annex I.1 scrambled data already includes SERVICE + PSDU + TAIL + PAD = 864 bits.
 * The encoder processes all 864 bits without adding its own tail (tail is embedded).
 * Output: 864*2 = 1728 rate-1/2 bits -> puncture 3/4 -> 1152 bits.
 */
static void test_encode_puncture_annex_i1(void) {
    TEST_BEGIN("conv_encode_puncture_annex_i1");

    test_vector *vec = vector_load("annex_i1_data_encoded");
    if (!vec || !vec->bits || vec->n_bits != 1152) {
        TEST_FAIL("cannot load annex_i1_data_encoded.json (expected 1152 bits)");
        if (vec) vector_free(vec);
        return;
    }

    uint8_t *scrambled = build_scrambled_input();
    if (!scrambled) {
        TEST_FAIL("cannot build scrambled input");
        vector_free(vec);
        return;
    }

    /* Conv encode (no tail — tail is already in the 864 bits) */
    uint8_t coded[1728];  /* 864 * 2 */
    lib80211_conv_encode(scrambled, coded, 864, false);

    /* Puncture at rate 3/4 */
    uint8_t punctured[1152];
    size_t n_out = lib80211_puncture(coded, punctured, 1728, 3, 4);

    if (n_out != 1152) {
        TEST_FAIL("puncture output length: expected 1152, got %zu", n_out);
    } else if (assert_bits_equal(vec->bits, punctured, 1152, "encoded+punctured")) {
        TEST_PASS();
    } else {
        TEST_FAIL("encoded+punctured mismatch");
    }

    free(scrambled);
    vector_free(vec);
}

int main(void) {
    printf("test_conv_encode\n");

    test_encode_puncture_annex_i1();

    TEST_SUMMARY();
    return TEST_EXIT();
}
