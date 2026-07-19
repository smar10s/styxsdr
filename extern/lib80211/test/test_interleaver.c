/**
 * test_interleaver.c — Validate interleaver against golden vectors.
 *
 * Tests legacy (Annex I.1) and HT (MCS 0-7) interleavers,
 * plus soft-bit deinterleavers (legacy and HT).
 */

#include "test_util.h"
#include "lib80211/interleaver.h"
#include "lib80211/constants.h"
#include "lib80211/fec.h"
#include "lib80211/scrambler.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ANNEX_I1_SEED 0x5D

/**
 * Test: encoded bits -> interleave matches golden vector.
 * Annex I.1: rate 36 Mbps = 16-QAM, n_cbps=192, n_bpsc=4.
 */
static void test_interleave_annex_i1(void) {
    TEST_BEGIN("interleave_annex_i1");

    test_vector *enc_vec = vector_load("annex_i1_data_encoded");
    test_vector *int_vec = vector_load("annex_i1_data_interleaved");

    if (!enc_vec || !enc_vec->bits || enc_vec->n_bits < 192) {
        TEST_FAIL("cannot load encoded vector (need at least 192 bits)");
        goto cleanup;
    }
    if (!int_vec || !int_vec->bits || int_vec->n_bits != 192) {
        TEST_FAIL("cannot load interleaved vector (expected 192 bits, got %zu)",
                  int_vec ? int_vec->n_bits : 0);
        goto cleanup;
    }

    /* Interleave first OFDM symbol (n_cbps=192 for 16-QAM) */
    uint8_t interleaved[192];
    int n_cbps = 192;
    int n_bpsc = 4;

    lib80211_interleave(enc_vec->bits, interleaved, n_cbps, n_bpsc);

    if (assert_bits_equal(int_vec->bits, interleaved, 192, "interleaved")) {
        TEST_PASS();
    } else {
        TEST_FAIL("interleaver output mismatch");
    }

cleanup:
    if (enc_vec) vector_free(enc_vec);
    if (int_vec) vector_free(int_vec);
}

/**
 * Test HT interleaver for a given MCS against golden vector.
 * Validates all symbols in the vector (encoded -> interleaved).
 */
static void test_ht_interleave_mcs(int mcs) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "ht_interleave_mcs%d", mcs);
    TEST_BEGIN(test_name);

    char enc_name[64], int_name[64];
    snprintf(enc_name, sizeof(enc_name), "ht_mcs%d_encoded", mcs);
    snprintf(int_name, sizeof(int_name), "ht_mcs%d_interleaved", mcs);

    test_vector *enc_vec = vector_load(enc_name);
    test_vector *int_vec = vector_load(int_name);

    if (!enc_vec || !enc_vec->bits) {
        TEST_FAIL("cannot load %s", enc_name);
        goto cleanup;
    }
    if (!int_vec || !int_vec->bits) {
        TEST_FAIL("cannot load %s", int_name);
        goto cleanup;
    }
    if (enc_vec->n_bits != int_vec->n_bits) {
        TEST_FAIL("size mismatch: encoded=%zu, interleaved=%zu",
                  enc_vec->n_bits, int_vec->n_bits);
        goto cleanup;
    }

    const lib80211_ht_mcs_info *info = &LIB80211_HT_MCS_TABLE[mcs];
    int n_cbps = info->n_cbps;
    int n_bpsc = info->n_bpsc;
    int n_sym = (int)enc_vec->n_bits / n_cbps;

    /* Interleave each symbol and compare */
    uint8_t interleaved[416];  /* max n_cbps for HT (64QAM = 312) */
    for (int s = 0; s < n_sym; s++) {
        lib80211_interleave_ht(&enc_vec->bits[s * n_cbps], interleaved, n_cbps, n_bpsc);

        for (int i = 0; i < n_cbps; i++) {
            if (interleaved[i] != int_vec->bits[s * n_cbps + i]) {
                TEST_FAIL("mismatch at symbol %d, bit %d: expected %u, got %u",
                          s, i, int_vec->bits[s * n_cbps + i], interleaved[i]);
                goto cleanup;
            }
        }
    }

    TEST_PASS();

cleanup:
    if (enc_vec) vector_free(enc_vec);
    if (int_vec) vector_free(int_vec);
}

/**
 * Test: deinterleave(interleaved) recovers original encoded bits.
 * Annex I.1: rate 36 Mbps = 16-QAM, n_cbps=192, n_bpsc=4.
 */
static void test_deinterleave_annex_i1(void) {
    TEST_BEGIN("deinterleave_annex_i1");

    test_vector *enc_vec = vector_load("annex_i1_data_encoded");
    test_vector *int_vec = vector_load("annex_i1_data_interleaved");

    if (!enc_vec || !enc_vec->bits || enc_vec->n_bits < 192) {
        TEST_FAIL("cannot load encoded vector (need at least 192 bits)");
        goto cleanup;
    }
    if (!int_vec || !int_vec->bits || int_vec->n_bits != 192) {
        TEST_FAIL("cannot load interleaved vector (expected 192 bits, got %zu)",
                  int_vec ? int_vec->n_bits : 0);
        goto cleanup;
    }

    int n_cbps = 192;
    int n_bpsc = 4;

    /* Convert interleaved hard bits to soft: 0 -> +1.0, 1 -> -1.0 */
    float soft_in[192];
    for (int i = 0; i < n_cbps; i++) {
        soft_in[i] = int_vec->bits[i] ? -1.0f : 1.0f;
    }

    /* Deinterleave */
    float soft_out[192];
    lib80211_deinterleave(soft_in, soft_out, n_cbps, n_bpsc);

    /* Convert back to hard decisions and compare against first 192 encoded bits */
    for (int i = 0; i < n_cbps; i++) {
        uint8_t hard = (soft_out[i] < 0.0f) ? 1 : 0;
        if (hard != enc_vec->bits[i]) {
            TEST_FAIL("bit %d: expected %u, got %u (soft=%.2f)",
                      i, enc_vec->bits[i], hard, soft_out[i]);
            goto cleanup;
        }
    }

    TEST_PASS();

cleanup:
    if (enc_vec) vector_free(enc_vec);
    if (int_vec) vector_free(int_vec);
}

/**
 * Test: round-trip interleave -> deinterleave recovers original bits.
 * Tests all legacy rate configurations.
 */
static void test_deinterleave_roundtrip(void) {
    TEST_BEGIN("deinterleave_roundtrip");

    /* Test each legacy rate's n_cbps/n_bpsc combination */
    static const int configs[][2] = {
        {48, 1},   /* BPSK */
        {96, 2},   /* QPSK */
        {192, 4},  /* 16-QAM */
        {288, 6},  /* 64-QAM */
    };
    int n_configs = 4;

    for (int c = 0; c < n_configs; c++) {
        int n_cbps = configs[c][0];
        int n_bpsc = configs[c][1];

        /* Generate known pattern */
        uint8_t original[288];
        for (int i = 0; i < n_cbps; i++) {
            original[i] = (uint8_t)((i * 7 + 3) % 2);
        }

        /* Interleave */
        uint8_t interleaved[288];
        lib80211_interleave(original, interleaved, n_cbps, n_bpsc);

        /* Convert to soft bits */
        float soft_in[288];
        for (int i = 0; i < n_cbps; i++) {
            soft_in[i] = interleaved[i] ? -1.0f : 1.0f;
        }

        /* Deinterleave */
        float soft_out[288];
        lib80211_deinterleave(soft_in, soft_out, n_cbps, n_bpsc);

        /* Verify hard decisions match original */
        for (int i = 0; i < n_cbps; i++) {
            uint8_t hard = (soft_out[i] < 0.0f) ? 1 : 0;
            if (hard != original[i]) {
                TEST_FAIL("n_cbps=%d, n_bpsc=%d, bit %d: expected %u, got %u",
                          n_cbps, n_bpsc, i, original[i], hard);
                return;
            }
        }
    }

    TEST_PASS();
}

/**
 * Test: HT deinterleave round-trip for all MCS 0-7.
 */
static void test_deinterleave_ht_roundtrip(void) {
    TEST_BEGIN("deinterleave_ht_roundtrip");

    for (int mcs = 0; mcs < 8; mcs++) {
        const lib80211_ht_mcs_info *info = &LIB80211_HT_MCS_TABLE[mcs];
        int n_cbps = info->n_cbps;
        int n_bpsc = info->n_bpsc;

        /* Generate known pattern */
        uint8_t original[416];
        for (int i = 0; i < n_cbps; i++) {
            original[i] = (uint8_t)((i * 13 + 5) % 2);
        }

        /* Interleave (HT) */
        uint8_t interleaved[416];
        lib80211_interleave_ht(original, interleaved, n_cbps, n_bpsc);

        /* Convert to soft bits */
        float soft_in[416];
        for (int i = 0; i < n_cbps; i++) {
            soft_in[i] = interleaved[i] ? -1.0f : 1.0f;
        }

        /* Deinterleave (HT) */
        float soft_out[416];
        lib80211_deinterleave_ht(soft_in, soft_out, n_cbps, n_bpsc);

        /* Verify hard decisions match original */
        for (int i = 0; i < n_cbps; i++) {
            uint8_t hard = (soft_out[i] < 0.0f) ? 1 : 0;
            if (hard != original[i]) {
                TEST_FAIL("MCS %d, bit %d: expected %u, got %u",
                          mcs, i, original[i], hard);
                return;
            }
        }
    }

    TEST_PASS();
}

int main(void) {
    printf("test_interleaver\n");

    test_interleave_annex_i1();

    /* HT interleaver: all 8 MCS */
    for (int mcs = 0; mcs < 8; mcs++) {
        test_ht_interleave_mcs(mcs);
    }

    /* Deinterleaver tests */
    test_deinterleave_annex_i1();
    test_deinterleave_roundtrip();
    test_deinterleave_ht_roundtrip();

    TEST_SUMMARY();
    return TEST_EXIT();
}
