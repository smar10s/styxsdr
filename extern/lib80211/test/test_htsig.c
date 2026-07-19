/**
 * test_htsig.c — Validate HT-SIG field generation against golden vectors.
 *
 * Tests HT-SIG bits for all 8 MCS values (normal GI) and SGI variants.
 */

#include "test_util.h"
#include "lib80211/signal.h"
#include "lib80211/constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Test HT-SIG bits for a given MCS against golden vector.
 */
static void test_htsig_bits_mcs(int mcs, bool sgi) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "htsig_bits_mcs%d%s", mcs, sgi ? "_sgi" : "");
    TEST_BEGIN(test_name);

    char vec_name[64];
    snprintf(vec_name, sizeof(vec_name), "ht_mcs%d%s_htsig_bits", mcs, sgi ? "_sgi" : "");

    test_vector *vec = vector_load(vec_name);
    if (!vec || !vec->bits || vec->n_bits != 48) {
        TEST_FAIL("cannot load %s (expected 48 bits)", vec_name);
        if (vec) vector_free(vec);
        return;
    }

    /* Generate HT-SIG bits.
     * The vectors use length_bytes=100 (PSDU length as encoded in HT-SIG). */
    uint8_t htsig[48];
    lib80211_make_htsig_bits(mcs, 100, sgi, false, htsig);

    if (assert_bits_equal(vec->bits, htsig, 48, vec_name)) {
        TEST_PASS();
    } else {
        TEST_FAIL("HT-SIG bits mismatch");
    }

    vector_free(vec);
}

int main(void) {
    printf("test_htsig\n");

    /* Normal GI: MCS 0-7 */
    for (int mcs = 0; mcs < 8; mcs++) {
        test_htsig_bits_mcs(mcs, false);
    }

    /* Short GI: MCS 0-7 */
    for (int mcs = 0; mcs < 8; mcs++) {
        test_htsig_bits_mcs(mcs, true);
    }

    TEST_SUMMARY();
    return TEST_EXIT();
}
