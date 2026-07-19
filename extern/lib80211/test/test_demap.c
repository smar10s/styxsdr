/**
 * test_demap.c — Validate soft demapper against modulator round-trip
 * and Annex I.1 golden vector.
 */

#include "test_util.h"
#include "lib80211/modulation.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Helper: hard-decision on soft bits. positive -> 1, else -> 0.
 */
static void hard_decide(const float *soft, uint8_t *bits, size_t n) {
    for (size_t i = 0; i < n; i++)
        bits[i] = (soft[i] > 0.0f) ? 1 : 0;
}

/**
 * Test: Annex I.1 QAM symbols -> soft demap -> hard decision matches interleaved bits.
 * 48 symbols, 16-QAM (n_bpsc=4), 192 bits.
 */
static void test_demap_annex_i1(void) {
    TEST_BEGIN("demap_16qam_annex_i1");

    test_vector *qam_vec = vector_load("annex_i1_data_qam");
    test_vector *int_vec = vector_load("annex_i1_data_interleaved");

    if (!qam_vec || !qam_vec->real || qam_vec->n_complex != 48) {
        TEST_FAIL("cannot load QAM vector (expected 48 complex)");
        goto cleanup;
    }
    if (!int_vec || !int_vec->bits || int_vec->n_bits < 192) {
        TEST_FAIL("cannot load interleaved vector");
        goto cleanup;
    }

    /* Soft demap: 48 16-QAM symbols -> 192 soft bits */
    float soft[192];
    lib80211_soft_demap(qam_vec->real, qam_vec->imag, soft, 48, 4);

    /* Hard decision */
    uint8_t decoded[192];
    hard_decide(soft, decoded, 192);

    /* Compare against golden interleaved bits */
    if (assert_bits_equal(int_vec->bits, decoded, 192, "16-QAM demap")) {
        TEST_PASS();
    } else {
        TEST_FAIL("16-QAM demap hard decision mismatch");
    }

cleanup:
    if (qam_vec) vector_free(qam_vec);
    if (int_vec) vector_free(int_vec);
}

/**
 * BPSK round-trip: modulate -> demap -> hard decision == original bits.
 */
static void test_demap_bpsk(void) {
    TEST_BEGIN("demap_bpsk_roundtrip");

    uint8_t bits[] = {0, 1, 1, 0, 1, 0, 0, 1};
    const size_t n = 8;

    float mod_r[8], mod_i[8];
    lib80211_modulate(bits, mod_r, mod_i, n, 1);

    float soft[8];
    lib80211_soft_demap(mod_r, mod_i, soft, n, 1);

    uint8_t decoded[8];
    hard_decide(soft, decoded, n);

    if (assert_bits_equal(bits, decoded, n, "BPSK roundtrip")) {
        TEST_PASS();
    } else {
        TEST_FAIL("BPSK roundtrip mismatch");
    }
}

/**
 * QPSK round-trip: modulate -> demap -> hard decision == original bits.
 */
static void test_demap_qpsk(void) {
    TEST_BEGIN("demap_qpsk_roundtrip");

    uint8_t bits[] = {0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1};
    const size_t n_symbols = 6;
    const size_t n_bits = 12;

    float mod_r[6], mod_i[6];
    lib80211_modulate(bits, mod_r, mod_i, n_symbols, 2);

    float soft[12];
    lib80211_soft_demap(mod_r, mod_i, soft, n_symbols, 2);

    uint8_t decoded[12];
    hard_decide(soft, decoded, n_bits);

    if (assert_bits_equal(bits, decoded, n_bits, "QPSK roundtrip")) {
        TEST_PASS();
    } else {
        TEST_FAIL("QPSK roundtrip mismatch");
    }
}

/**
 * 64-QAM round-trip: modulate -> demap -> hard decision == original bits.
 */
static void test_demap_64qam(void) {
    TEST_BEGIN("demap_64qam_roundtrip");

    /* 4 symbols x 6 bits = 24 bits */
    uint8_t bits[] = {
        0, 1, 0, 1, 1, 0,
        1, 0, 1, 0, 0, 1,
        1, 1, 0, 0, 1, 0,
        0, 0, 1, 1, 0, 1
    };
    const size_t n_symbols = 4;
    const size_t n_bits = 24;

    float mod_r[4], mod_i[4];
    lib80211_modulate(bits, mod_r, mod_i, n_symbols, 6);

    float soft[24];
    lib80211_soft_demap(mod_r, mod_i, soft, n_symbols, 6);

    uint8_t decoded[24];
    hard_decide(soft, decoded, n_bits);

    if (assert_bits_equal(bits, decoded, n_bits, "64-QAM roundtrip")) {
        TEST_PASS();
    } else {
        TEST_FAIL("64-QAM roundtrip mismatch");
    }
}

int main(void) {
    printf("test_demap\n");

    test_demap_bpsk();
    test_demap_qpsk();
    test_demap_annex_i1();
    test_demap_64qam();

    TEST_SUMMARY();
    return TEST_EXIT();
}
