/**
 * test_modulation.c — Validate QAM modulator against Annex I.1 golden vector.
 */

#include "test_util.h"
#include "lib80211/modulation.h"
#include "lib80211/interleaver.h"
#include "lib80211/fec.h"
#include "lib80211/scrambler.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Test: interleaved bits -> modulate(16-QAM) matches golden QAM vector.
 * Annex I.1: rate 36 Mbps, n_bpsc=4, first OFDM symbol (48 subcarriers).
 */
static void test_modulate_annex_i1(void) {
    TEST_BEGIN("modulate_16qam_annex_i1");

    test_vector *int_vec = vector_load("annex_i1_data_interleaved");
    test_vector *qam_vec = vector_load("annex_i1_data_qam");

    if (!int_vec || !int_vec->bits || int_vec->n_bits < 192) {
        TEST_FAIL("cannot load interleaved vector");
        goto cleanup;
    }
    if (!qam_vec || !qam_vec->real || qam_vec->n_complex != 48) {
        TEST_FAIL("cannot load QAM vector (expected 48 complex)");
        goto cleanup;
    }

    /* Modulate first symbol: 192 bits -> 48 16-QAM symbols */
    float out_real[48], out_imag[48];
    lib80211_modulate(int_vec->bits, out_real, out_imag, 48, 4);

    /* Golden vector has 3-decimal precision -> tolerance ~1e-3 */
    if (assert_complex_close(qam_vec->real, qam_vec->imag,
                            out_real, out_imag, 48, 1e-3f, "16-QAM")) {
        TEST_PASS();
    } else {
        TEST_FAIL("16-QAM modulation mismatch");
    }

cleanup:
    if (int_vec) vector_free(int_vec);
    if (qam_vec) vector_free(qam_vec);
}

/* Test BPSK normalization: output should be +/-1 */
static void test_modulate_bpsk(void) {
    TEST_BEGIN("modulate_bpsk");

    uint8_t bits[] = {0, 1, 0, 1};
    float out_r[4], out_i[4];
    lib80211_modulate(bits, out_r, out_i, 4, 1);

    int ok = 1;
    float expected_r[] = {-1.0f, 1.0f, -1.0f, 1.0f};  /* bit=0->-1, bit=1->+1 */
    for (int i = 0; i < 4; i++) {
        if (out_r[i] != expected_r[i] || out_i[i] != 0.0f) {
            printf("    BPSK[%d]: expected %.1f+0i, got %.4f+%.4fi\n",
                   i, expected_r[i], out_r[i], out_i[i]);
            ok = 0;
        }
    }

    if (ok) TEST_PASS();
    else TEST_FAIL("BPSK values incorrect");
}

int main(void) {
    printf("test_modulation\n");

    test_modulate_bpsk();
    test_modulate_annex_i1();

    TEST_SUMMARY();
    return TEST_EXIT();
}
