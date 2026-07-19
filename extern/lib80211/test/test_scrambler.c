/**
 * test_scrambler.c — Validate scrambler against Annex I.1 golden vector.
 */

#include "test_util.h"
#include "lib80211/scrambler.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Annex I.1 scrambler seed = 0x5D (binary 1011101, MSB first) */
#define ANNEX_I1_SEED 0x5D

static void test_scrambler_annex_i1(void) {
    TEST_BEGIN("scrambler_annex_i1");

    test_vector *vec = vector_load("annex_i1_data_scrambled");
    if (!vec || !vec->bits) {
        TEST_FAIL("cannot load annex_i1_data_scrambled.json");
        return;
    }

    if (vec->n_bits != 864) {
        TEST_FAIL("expected 864 bits, got %zu", vec->n_bits);
        vector_free(vec);
        return;
    }

    /* The scrambled vector is the output of scrambling the SERVICE+PSDU+tail+pad.
     * To validate, we need the pre-scrambled input. The scrambler is self-inverse,
     * so scramble(scrambled) = original input.
     * But we can also verify: scramble(zeros) produces the scrambler sequence,
     * and scrambled XOR original = scrambler_seq.
     *
     * For a direct test: we need the input bits. The input is:
     *   SERVICE (16 bits, all zero) + PSDU (100 bytes = 800 bits, LSB first per byte)
     *   + TAIL (6 bits, zero) + PAD (to fill: 864 - 822 = 42 zero bits)
     * Total = 864 bits.
     *
     * Actually simpler: load the PSDU, construct the full input, scramble,
     * and compare to the golden vector.
     */
    test_vector *psdu_vec = vector_load("annex_i1_psdu");
    if (!psdu_vec || !psdu_vec->hex_octets) {
        TEST_FAIL("cannot load annex_i1_psdu.json");
        vector_free(vec);
        return;
    }

    /* Build input: SERVICE(16 zeros) + PSDU(800 bits, LSB first) + tail(6) + pad(42) = 864 */
    uint8_t input[864];
    memset(input, 0, sizeof(input));

    /* SERVICE field: 16 zero bits (already set) */

    /* PSDU: 100 bytes, LSB first per byte */
    for (size_t byte_idx = 0; byte_idx < 100 && byte_idx < psdu_vec->n_octets; byte_idx++) {
        uint8_t val = (uint8_t)strtoul(psdu_vec->hex_octets[byte_idx], NULL, 16);
        for (int bit = 0; bit < 8; bit++) {
            input[16 + byte_idx * 8 + bit] = (val >> bit) & 1;
        }
    }
    /* Tail and pad: already zero */

    /* Scramble */
    uint8_t output[864];
    lib80211_scramble(input, output, 864, ANNEX_I1_SEED);

    /* Per spec: tail bits (positions 816..821) are zeroed AFTER scrambling.
     * SERVICE(16) + PSDU(800) = 816, then 6 tail bits forced to 0. */
    for (int i = 816; i < 822; i++)
        output[i] = 0;

    if (assert_bits_equal(vec->bits, output, 864, "scrambler output")) {
        TEST_PASS();
    } else {
        TEST_FAIL("scrambler output mismatch");
    }

    vector_free(vec);
    vector_free(psdu_vec);
}

/* Test scrambler self-inverse property */
static void test_scrambler_self_inverse(void) {
    TEST_BEGIN("scrambler_self_inverse");

    uint8_t data[128];
    for (int i = 0; i < 128; i++) data[i] = (uint8_t)(i & 1);

    uint8_t scrambled[128];
    uint8_t recovered[128];

    lib80211_scramble(data, scrambled, 128, 0x7F);
    lib80211_scramble(scrambled, recovered, 128, 0x7F);

    if (assert_bits_equal(data, recovered, 128, "self-inverse")) {
        TEST_PASS();
    } else {
        TEST_FAIL("scrambler not self-inverse");
    }
}

int main(void) {
    printf("test_scrambler\n");

    test_scrambler_annex_i1();
    test_scrambler_self_inverse();

    TEST_SUMMARY();
    return TEST_EXIT();
}
