/**
 * test_rx_ht.c -- Validate HT-mixed (802.11n) frame decode pipeline.
 *
 * Tests:
 * 1. TX->RX loopback at all 8 MCS (BCC, normal GI)
 * 2. TX->RX loopback with short GI
 * 3. Decode TGn golden waveform vectors (if available)
 */

#include "test_util.h"
#include "lib80211/rx.h"
#include "lib80211/tx.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Test PSDU: 100 bytes total (96 payload + 4 FCS) */
static const int PSDU_LEN = 100;

/* CRC-32 for generating FCS */
static uint32_t test_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static void fill_psdu(uint8_t *psdu, int seed)
{
    /* Fill 96 payload bytes */
    for (int i = 0; i < PSDU_LEN - 4; i++)
        psdu[i] = (uint8_t)((i * 7 + seed) & 0xFF);
    /* Compute and append FCS */
    uint32_t fcs = test_crc32(psdu, PSDU_LEN - 4);
    psdu[PSDU_LEN - 4] = (uint8_t)(fcs & 0xFF);
    psdu[PSDU_LEN - 3] = (uint8_t)((fcs >> 8) & 0xFF);
    psdu[PSDU_LEN - 2] = (uint8_t)((fcs >> 16) & 0xFF);
    psdu[PSDU_LEN - 1] = (uint8_t)((fcs >> 24) & 0xFF);
}

/**
 * TX->RX loopback test for given MCS and GI.
 */
static void test_loopback(lib80211_fft_plan *plan, int mcs, bool short_gi)
{
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "rx_ht_mcs%d%s",
             mcs, short_gi ? "_sgi" : "");
    TEST_BEGIN(test_name);

    uint8_t psdu[100];
    fill_psdu(psdu, mcs);

    lib80211_tx_ht_params tx_params = {
        .mcs = mcs,
        .psdu = psdu,
        .psdu_len = PSDU_LEN,
        .scrambler_seed = 0x5D,
        .short_gi = short_gi,
        .ldpc = false,
    };

    size_t max_samples = lib80211_tx_ht_samples(&tx_params);
    /* Add some padding for sync detection */
    size_t buf_samples = max_samples + 200;
    float *tx_real = (float *)calloc(buf_samples, sizeof(float));
    float *tx_imag = (float *)calloc(buf_samples, sizeof(float));

    if (!tx_real || !tx_imag) {
        TEST_FAIL("allocation failed");
        free(tx_real); free(tx_imag);
        return;
    }

    /* Generate frame with some leading zeros for sync */
    size_t offset = 100;
    size_t n_tx = lib80211_tx_ht(plan, &tx_params,
                                  tx_real + offset, tx_imag + offset);
    if (n_tx == 0) {
        TEST_FAIL("tx_ht returned 0");
        free(tx_real); free(tx_imag);
        return;
    }

    size_t total_samples = offset + n_tx + 100;
    if (total_samples > buf_samples) total_samples = buf_samples;

    /* Decode */
    lib80211_rx_result result;
    int rc = lib80211_rx_decode(plan, tx_real, tx_imag, total_samples, &result);

    if (rc != 0) {
        TEST_FAIL("rx_decode returned -1 (decode failed)");
        free(tx_real); free(tx_imag);
        return;
    }

    int ok = 1;

    /* Check frame type */
    if (result.type != LIB80211_FRAME_HT) {
        printf("    type: expected HT(%d), got %d\n", LIB80211_FRAME_HT, result.type);
        ok = 0;
    }

    /* Check MCS */
    if (result.mcs != mcs) {
        printf("    mcs: expected %d, got %d\n", mcs, result.mcs);
        ok = 0;
    }

    /* Check SGI */
    if (result.short_gi != short_gi) {
        printf("    sgi: expected %d, got %d\n", short_gi, result.short_gi);
        ok = 0;
    }

    /* Check PSDU length */
    if (result.psdu_len != (size_t)PSDU_LEN) {
        printf("    psdu_len: expected %d, got %zu\n", PSDU_LEN, result.psdu_len);
        ok = 0;
    }

    /* Check FCS */
    if (!result.fcs_valid) {
        printf("    FCS check FAILED\n");
        printf("    PSDU[0:10]:");
        for (int i = 0; i < 10 && i < (int)result.psdu_len; i++)
            printf(" %02x", result.psdu[i]);
        printf("\n");
        ok = 0;
    }

    /* Check PSDU content (first PSDU_LEN-4 bytes, last 4 are FCS) */
    if (ok && result.fcs_valid) {
        int mismatches = 0;
        for (int i = 0; i < PSDU_LEN - 4; i++) {
            if (result.psdu[i] != psdu[i]) {
                if (mismatches == 0)
                    printf("    first mismatch at byte %d: expected 0x%02x got 0x%02x\n",
                           i, psdu[i], result.psdu[i]);
                mismatches++;
            }
        }
        if (mismatches > 0) {
            printf("    total mismatches: %d / %d\n", mismatches, PSDU_LEN - 4);
            ok = 0;
        }
    }

    if (ok) {
        printf("    mcs=%d, sgi=%d, len=%zu, n_sym=%d, fcs=OK\n",
               result.mcs, result.short_gi, result.psdu_len, result.n_symbols);
        TEST_PASS();
    } else {
        TEST_FAIL("decode results incorrect");
    }

    free(tx_real);
    free(tx_imag);
}

/**
 * Decode a TGn golden waveform vector.
 */
static void test_decode_vector(lib80211_fft_plan *plan, int mcs, bool short_gi)
{
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "rx_ht_vec_mcs%d%s",
             mcs, short_gi ? "_sgi" : "");
    TEST_BEGIN(test_name);

    char vec_name[64];
    snprintf(vec_name, sizeof(vec_name), "ht_mcs%d%s_waveform",
             mcs, short_gi ? "_sgi" : "");

    test_vector *vec = vector_load(vec_name);
    if (!vec || !vec->real || !vec->imag || vec->n_complex == 0) {
        /* Vector not available — skip (not a failure) */
        printf("    vector %s not available, skipping\n", vec_name);
        TEST_PASS();
        if (vec) vector_free(vec);
        return;
    }

    lib80211_rx_result result;
    int rc = lib80211_rx_decode(plan, vec->real, vec->imag, vec->n_complex, &result);

    if (rc != 0) {
        TEST_FAIL("rx_decode returned -1 for %s", vec_name);
        vector_free(vec);
        return;
    }

    int ok = 1;

    if (result.type != LIB80211_FRAME_HT) {
        printf("    type: expected HT, got %d\n", result.type);
        ok = 0;
    }

    if (result.mcs != mcs) {
        printf("    mcs: expected %d, got %d\n", mcs, result.mcs);
        ok = 0;
    }

    if (!result.fcs_valid) {
        printf("    FCS FAILED (len=%zu, n_sym=%d)\n", result.psdu_len, result.n_symbols);
        ok = 0;
    }

    if (ok) {
        printf("    mcs=%d, sgi=%d, len=%zu, fcs=OK\n",
               result.mcs, result.short_gi, result.psdu_len);
        TEST_PASS();
    } else {
        TEST_FAIL("golden vector decode incorrect");
    }

    vector_free(vec);
}

/**
 * TX->RX loopback test for LDPC-coded HT frame.
 */
static void test_loopback_ldpc(lib80211_fft_plan *plan, int mcs, bool short_gi)
{
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "rx_ht_ldpc_mcs%d%s",
             mcs, short_gi ? "_sgi" : "");
    TEST_BEGIN(test_name);

    uint8_t psdu[100];
    fill_psdu(psdu, mcs + 10);

    lib80211_tx_ht_params tx_params = {
        .mcs = mcs,
        .psdu = psdu,
        .psdu_len = PSDU_LEN,
        .scrambler_seed = 0x5D,
        .short_gi = short_gi,
        .ldpc = true,
    };

    size_t max_samples = lib80211_tx_ht_samples(&tx_params);
    size_t buf_samples = max_samples + 200;
    float *tx_real = (float *)calloc(buf_samples, sizeof(float));
    float *tx_imag = (float *)calloc(buf_samples, sizeof(float));

    if (!tx_real || !tx_imag) {
        TEST_FAIL("allocation failed");
        free(tx_real); free(tx_imag);
        return;
    }

    size_t offset = 100;
    size_t n_tx = lib80211_tx_ht(plan, &tx_params,
                                  tx_real + offset, tx_imag + offset);
    if (n_tx == 0) {
        TEST_FAIL("tx_ht returned 0");
        free(tx_real); free(tx_imag);
        return;
    }

    size_t total_samples = offset + n_tx + 100;
    if (total_samples > buf_samples) total_samples = buf_samples;

    lib80211_rx_result result;
    int rc = lib80211_rx_decode(plan, tx_real, tx_imag, total_samples, &result);

    if (rc != 0) {
        TEST_FAIL("rx_decode returned -1 (decode failed)");
        free(tx_real); free(tx_imag);
        return;
    }

    int ok = 1;

    if (result.type != LIB80211_FRAME_HT) {
        printf("    type: expected HT(%d), got %d\n", LIB80211_FRAME_HT, result.type);
        ok = 0;
    }
    if (result.mcs != mcs) {
        printf("    mcs: expected %d, got %d\n", mcs, result.mcs);
        ok = 0;
    }
    if (!result.ldpc) {
        printf("    ldpc: expected true, got false\n");
        ok = 0;
    }
    if (result.psdu_len != (size_t)PSDU_LEN) {
        printf("    psdu_len: expected %d, got %zu\n", PSDU_LEN, result.psdu_len);
        ok = 0;
    }
    if (!result.fcs_valid) {
        printf("    FCS check FAILED\n");
        ok = 0;
    }

    if (ok) {
        printf("    mcs=%d, ldpc=1, sgi=%d, len=%zu, n_sym=%d, fcs=OK\n",
               result.mcs, result.short_gi, result.psdu_len, result.n_symbols);
        TEST_PASS();
    } else {
        TEST_FAIL("decode results incorrect");
    }

    free(tx_real);
    free(tx_imag);
}

int main(void)
{
    printf("test_rx_ht: HT-mixed frame decode (MCS 0-7, BCC)\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("  [ FAIL ] could not create FFT plan\n");
        return 1;
    }

    /* TX->RX loopback: all 8 MCS, normal GI */
    for (int mcs = 0; mcs <= 7; mcs++) {
        test_loopback(plan, mcs, false);
    }

    /* TX->RX loopback: selected MCS with SGI */
    test_loopback(plan, 0, true);
    test_loopback(plan, 4, true);
    test_loopback(plan, 7, true);

    /* TGn golden waveform vectors — normal GI */
    for (int mcs = 0; mcs <= 7; mcs++) {
        test_decode_vector(plan, mcs, false);
    }

    /* TGn golden waveform vectors — short GI */
    for (int mcs = 0; mcs <= 7; mcs++) {
        test_decode_vector(plan, mcs, true);
    }

    /* TX->RX loopback: LDPC coded, all 8 MCS */
    printf("\ntest_rx_ht: HT-mixed frame decode (MCS 0-7, LDPC)\n");
    for (int mcs = 0; mcs <= 7; mcs++) {
        test_loopback_ldpc(plan, mcs, false);
    }
    /* LDPC with SGI */
    test_loopback_ldpc(plan, 0, true);
    test_loopback_ldpc(plan, 4, true);
    test_loopback_ldpc(plan, 7, true);

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
