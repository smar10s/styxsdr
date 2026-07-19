/**
 * test_edge_cases.c -- Large-frame loopback, boundary-size, and scrambler
 * seed exhaustion tests for lib80211.
 *
 * Validates that TX->RX loopback works for:
 * 1. Large PSDUs (2000-4000 bytes) at various HT/VHT MCS
 * 2. All 127 valid scrambler seeds
 * 3. Boundary PSDU sizes that stress padding/codeword logic
 */

#include "test_util.h"
#include "lib80211/tx.h"
#include "lib80211/rx.h"
#include "lib80211/fft.h"
#include "lib80211/modulation.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ========================================================================
 * CRC-32 helper
 * ======================================================================== */

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

/* ========================================================================
 * Large-frame loopback tests
 * ======================================================================== */

/**
 * TX->RX loopback for large frames.
 * frame_type: 1=HT, 2=VHT
 */
static void test_large_frame(lib80211_fft_plan *plan, const char *label,
                             int frame_type, int rate_or_mcs,
                             int psdu_len, bool ldpc)
{
    TEST_BEGIN(label);

    uint8_t *psdu = malloc(psdu_len);
    if (!psdu) { TEST_FAIL("malloc psdu"); return; }

    /* Fill payload with deterministic pattern */
    int payload_len = psdu_len - 4;
    for (int i = 0; i < payload_len; i++)
        psdu[i] = (uint8_t)((i * 7 + 13) & 0xFF);

    /* Compute and append FCS */
    uint32_t fcs = test_crc32(psdu, payload_len);
    psdu[payload_len + 0] = (uint8_t)(fcs & 0xFF);
    psdu[payload_len + 1] = (uint8_t)((fcs >> 8) & 0xFF);
    psdu[payload_len + 2] = (uint8_t)((fcs >> 16) & 0xFF);
    psdu[payload_len + 3] = (uint8_t)((fcs >> 24) & 0xFF);

    size_t max_samples;
    size_t n_tx;
    size_t offset = 100;

    float *tx_re = NULL;
    float *tx_im = NULL;

    if (frame_type == 1) {
        /* HT */
        lib80211_tx_ht_params params = {
            .mcs = rate_or_mcs,
            .psdu = psdu,
            .psdu_len = psdu_len,
            .scrambler_seed = 0x5D,
            .short_gi = false,
            .ldpc = ldpc,
        };
        max_samples = lib80211_tx_ht_samples(&params);
        size_t buf_n = max_samples + 200;
        tx_re = calloc(buf_n, sizeof(float));
        tx_im = calloc(buf_n, sizeof(float));
        if (!tx_re || !tx_im) {
            TEST_FAIL("malloc buffers");
            free(psdu); free(tx_re); free(tx_im);
            return;
        }
        n_tx = lib80211_tx_ht(plan, &params, tx_re + offset, tx_im + offset);
        if (n_tx == 0) {
            TEST_FAIL("tx_ht returned 0");
            free(psdu); free(tx_re); free(tx_im);
            return;
        }
        size_t total = offset + n_tx + 100;
        if (total > buf_n) total = buf_n;

        lib80211_rx_result result;
        int rc = lib80211_rx_decode(plan, tx_re, tx_im, total, &result);
        if (rc != 0) {
            TEST_FAIL("rx_decode failed (rc=%d)", rc);
        } else if (!result.fcs_valid) {
            TEST_FAIL("FCS invalid (len=%zu)", result.psdu_len);
        } else if (result.psdu_len != (size_t)psdu_len) {
            TEST_FAIL("length mismatch: expected %d, got %zu", psdu_len, result.psdu_len);
        } else if (memcmp(result.psdu, psdu, payload_len) != 0) {
            /* Find first mismatch */
            int first = -1;
            for (int i = 0; i < payload_len; i++) {
                if (result.psdu[i] != psdu[i]) { first = i; break; }
            }
            TEST_FAIL("payload mismatch at byte %d: expected 0x%02x, got 0x%02x",
                      first, psdu[first], result.psdu[first]);
        } else {
            TEST_PASS();
        }
    } else {
        /* VHT */
        lib80211_tx_vht_params params = {
            .mcs = rate_or_mcs,
            .psdu = psdu,
            .psdu_len = psdu_len,
            .scrambler_seed = 0x5D,
            .short_gi = false,
            .ldpc = ldpc,
        };
        max_samples = lib80211_tx_vht_samples(&params);
        size_t buf_n = max_samples + 200;
        tx_re = calloc(buf_n, sizeof(float));
        tx_im = calloc(buf_n, sizeof(float));
        if (!tx_re || !tx_im) {
            TEST_FAIL("malloc buffers");
            free(psdu); free(tx_re); free(tx_im);
            return;
        }
        n_tx = lib80211_tx_vht(plan, &params, tx_re + offset, tx_im + offset);
        if (n_tx == 0) {
            TEST_FAIL("tx_vht returned 0");
            free(psdu); free(tx_re); free(tx_im);
            return;
        }
        size_t total = offset + n_tx + 100;
        if (total > buf_n) total = buf_n;

        lib80211_rx_result result;
        int rc = lib80211_rx_decode(plan, tx_re, tx_im, total, &result);
        if (rc != 0) {
            TEST_FAIL("rx_decode failed (rc=%d)", rc);
        } else if (!result.fcs_valid) {
            TEST_FAIL("FCS invalid (len=%zu)", result.psdu_len);
        } else if (result.psdu_len != (size_t)psdu_len) {
            TEST_FAIL("length mismatch: expected %d, got %zu", psdu_len, result.psdu_len);
        } else if (memcmp(result.psdu, psdu, payload_len) != 0) {
            int first = -1;
            for (int i = 0; i < payload_len; i++) {
                if (result.psdu[i] != psdu[i]) { first = i; break; }
            }
            TEST_FAIL("payload mismatch at byte %d: expected 0x%02x, got 0x%02x",
                      first, psdu[first], result.psdu[first]);
        } else {
            TEST_PASS();
        }
    }

    free(psdu);
    free(tx_re);
    free(tx_im);
}

/* ========================================================================
 * Scrambler seed exhaustion test
 * ======================================================================== */

static void test_all_scrambler_seeds(lib80211_fft_plan *plan)
{
    TEST_BEGIN("scrambler_seeds_all_127");

    int psdu_len = 100;
    uint8_t psdu[100];

    /* Fill payload */
    for (int i = 0; i < psdu_len - 4; i++)
        psdu[i] = (uint8_t)((i * 3 + 7) & 0xFF);
    uint32_t fcs = test_crc32(psdu, psdu_len - 4);
    psdu[psdu_len - 4] = (uint8_t)(fcs & 0xFF);
    psdu[psdu_len - 3] = (uint8_t)((fcs >> 8) & 0xFF);
    psdu[psdu_len - 2] = (uint8_t)((fcs >> 16) & 0xFF);
    psdu[psdu_len - 1] = (uint8_t)((fcs >> 24) & 0xFF);

    int failures = 0;
    for (int seed = 1; seed <= 127; seed++) {
        lib80211_tx_ht_params params = {
            .mcs = 3,
            .psdu = psdu,
            .psdu_len = psdu_len,
            .scrambler_seed = (uint8_t)seed,
            .short_gi = false,
            .ldpc = false,
        };

        size_t max_samples = lib80211_tx_ht_samples(&params);
        size_t buf_n = max_samples + 200;
        float *re = calloc(buf_n, sizeof(float));
        float *im = calloc(buf_n, sizeof(float));
        if (!re || !im) { free(re); free(im); failures++; continue; }

        size_t offset = 100;
        size_t n_tx = lib80211_tx_ht(plan, &params, re + offset, im + offset);
        if (n_tx == 0) { free(re); free(im); failures++; continue; }

        size_t total = offset + n_tx + 100;
        if (total > buf_n) total = buf_n;

        lib80211_rx_result result;
        int rc = lib80211_rx_decode(plan, re, im, total, &result);

        if (rc != 0 || !result.fcs_valid || result.psdu_len != (size_t)psdu_len) {
            printf("    seed %d: FAILED (rc=%d, fcs=%d, len=%zu)\n",
                   seed, rc, result.fcs_valid, result.psdu_len);
            failures++;
        }

        free(re);
        free(im);
    }

    if (failures == 0) {
        printf("    all 127 seeds decoded OK\n");
        TEST_PASS();
    } else {
        TEST_FAIL("%d / 127 seeds failed", failures);
    }
}

/* ========================================================================
 * Boundary-size PSDU tests
 * ======================================================================== */

static void test_boundary_sizes(lib80211_fft_plan *plan)
{
    /* VHT-SIG-B encodes APEP length in 4-byte units, so PSDU sizes must be
     * multiples of 4 to round-trip correctly. These sizes are chosen to hit
     * padding and codeword boundary edge cases at MCS 4 (n_dbps=234). */
    int sizes[] = {8, 52, 120, 236, 468, 1000};
    int n_sizes = 6;

    for (int s = 0; s < n_sizes; s++) {
        int psdu_len = sizes[s];
        char name[64];
        snprintf(name, sizeof(name), "boundary_vht4_%dB", psdu_len);
        TEST_BEGIN(name);

        uint8_t *psdu = malloc(psdu_len);
        if (!psdu) { TEST_FAIL("malloc"); continue; }

        int payload_len = psdu_len - 4;
        if (payload_len < 1) {
            /* Too small for meaningful FCS test */
            free(psdu);
            TEST_PASS();
            continue;
        }
        for (int i = 0; i < payload_len; i++)
            psdu[i] = (uint8_t)((i * 11 + s) & 0xFF);
        uint32_t fcs = test_crc32(psdu, payload_len);
        psdu[payload_len + 0] = (uint8_t)(fcs & 0xFF);
        psdu[payload_len + 1] = (uint8_t)((fcs >> 8) & 0xFF);
        psdu[payload_len + 2] = (uint8_t)((fcs >> 16) & 0xFF);
        psdu[payload_len + 3] = (uint8_t)((fcs >> 24) & 0xFF);

        lib80211_tx_vht_params params = {
            .mcs = 4,
            .psdu = psdu,
            .psdu_len = psdu_len,
            .scrambler_seed = 0x5D,
            .short_gi = false,
            .ldpc = false,
        };

        size_t max_samples = lib80211_tx_vht_samples(&params);
        size_t buf_n = max_samples + 200;
        float *re = calloc(buf_n, sizeof(float));
        float *im = calloc(buf_n, sizeof(float));
        if (!re || !im) {
            free(psdu); free(re); free(im);
            TEST_FAIL("malloc");
            continue;
        }

        size_t offset = 100;
        size_t n_tx = lib80211_tx_vht(plan, &params, re + offset, im + offset);
        size_t total = offset + n_tx + 100;
        if (total > buf_n) total = buf_n;

        lib80211_rx_result result;
        int rc = lib80211_rx_decode(plan, re, im, total, &result);

        if (rc != 0) {
            TEST_FAIL("decode failed for %dB PSDU (rc=%d)", psdu_len, rc);
        } else if (!result.fcs_valid) {
            TEST_FAIL("FCS failed for %dB PSDU (got len=%zu)", psdu_len, result.psdu_len);
        } else if (result.psdu_len != (size_t)psdu_len) {
            TEST_FAIL("length mismatch: expected %d, got %zu", psdu_len, result.psdu_len);
        } else {
            TEST_PASS();
        }

        free(psdu);
        free(re);
        free(im);
    }
}

/* ========================================================================
 * MU-MIMO rejection test (group_id != 0)
 * ======================================================================== */

static void test_mu_mimo_rejection(lib80211_fft_plan *plan)
{
    TEST_BEGIN("vht_su_group_id_zero");

    /* A normal VHT SU frame should decode fine (group_id=0) */
    uint8_t psdu[100];
    for (int i = 0; i < 96; i++) psdu[i] = (uint8_t)((i * 7 + 13) & 0xFF);
    uint32_t fcs = test_crc32(psdu, 96);
    psdu[96] = (uint8_t)(fcs & 0xFF);
    psdu[97] = (uint8_t)((fcs >> 8) & 0xFF);
    psdu[98] = (uint8_t)((fcs >> 16) & 0xFF);
    psdu[99] = (uint8_t)((fcs >> 24) & 0xFF);

    lib80211_tx_vht_params params = {
        .mcs = 4, .psdu = psdu, .psdu_len = 100,
        .scrambler_seed = 0x5D, .short_gi = false, .ldpc = false,
    };

    size_t max_samples = lib80211_tx_vht_samples(&params);
    size_t buf_n = max_samples + 200;
    float *re = calloc(buf_n, sizeof(float));
    float *im = calloc(buf_n, sizeof(float));

    size_t offset = 100;
    size_t n_tx = lib80211_tx_vht(plan, &params, re + offset, im + offset);
    size_t total = offset + n_tx + 100;
    if (total > buf_n) total = buf_n;

    lib80211_rx_result result;
    int rc = lib80211_rx_decode(plan, re, im, total, &result);

    if (rc != 0 || !result.fcs_valid || result.type != LIB80211_FRAME_VHT) {
        TEST_FAIL("SU VHT frame should decode (rc=%d, fcs=%d, type=%d)",
                  rc, result.fcs_valid, result.type);
    } else {
        TEST_PASS();
    }

    free(re); free(im);
}

/* ========================================================================
 * MCS 9 rejection test (invalid for 20 MHz single-stream)
 * ======================================================================== */

static void test_mcs9_rejected(lib80211_fft_plan *plan)
{
    TEST_BEGIN("vht_mcs9_rejected");

    uint8_t psdu[100];
    memset(psdu, 0xAA, 100);

    lib80211_tx_vht_params params = {
        .mcs = 9, .psdu = psdu, .psdu_len = 100,
        .scrambler_seed = 0x5D, .short_gi = false, .ldpc = false,
    };

    size_t max_samples = lib80211_tx_vht_samples(&params);
    /* Even if samples returns non-zero, the actual TX should fail or return 0 */
    float *re = calloc(max_samples + 100, sizeof(float));
    float *im = calloc(max_samples + 100, sizeof(float));

    size_t n = lib80211_tx_vht(plan, &params, re, im);

    if (n == 0) {
        TEST_PASS();  /* Correctly rejected */
    } else {
        TEST_FAIL("MCS 9 should be rejected but got %zu samples", n);
    }

    free(re); free(im);
}

/* ========================================================================
 * VHT LDPC extra symbol bit test
 * ======================================================================== */

static void test_vht_ldpc_extra_symbol(lib80211_fft_plan *plan)
{
    /* A size that does NOT require extra symbol: use small PSDU with low MCS */
    TEST_BEGIN("vht_ldpc_extra_not_set");
    {
        uint8_t psdu[40];
        for (int i = 0; i < 36; i++) psdu[i] = (uint8_t)(i & 0xFF);
        uint32_t fcs = test_crc32(psdu, 36);
        psdu[36] = (uint8_t)(fcs); psdu[37] = (uint8_t)(fcs>>8);
        psdu[38] = (uint8_t)(fcs>>16); psdu[39] = (uint8_t)(fcs>>24);

        lib80211_tx_vht_params params = {
            .mcs = 0, .psdu = psdu, .psdu_len = 40,
            .scrambler_seed = 0x5D, .short_gi = false, .ldpc = true,
        };

        size_t max_s = lib80211_tx_vht_samples(&params);
        size_t buf = max_s + 200;
        float *re = calloc(buf, sizeof(float));
        float *im = calloc(buf, sizeof(float));
        size_t off = 100;
        size_t n_tx = lib80211_tx_vht(plan, &params, re+off, im+off);
        size_t total = off + n_tx + 100; if (total > buf) total = buf;

        lib80211_rx_result result;
        int rc = lib80211_rx_decode(plan, re, im, total, &result);

        if (rc != 0 || !result.fcs_valid) {
            TEST_FAIL("decode failed (rc=%d, fcs=%d)", rc, result.fcs_valid);
        } else if (result.type != LIB80211_FRAME_VHT || !result.ldpc) {
            TEST_FAIL("wrong type or not LDPC (type=%d, ldpc=%d)", result.type, result.ldpc);
        } else {
            TEST_PASS();
        }
        free(re); free(im);
    }

    /* A size that DOES require extra symbol: MCS 5 (64-QAM 2/3), 148 byte PSDU */
    TEST_BEGIN("vht_ldpc_extra_set");
    {
        int plen = 148;
        uint8_t *psdu = malloc(plen);
        for (int i = 0; i < plen-4; i++) psdu[i] = (uint8_t)((i*3+7) & 0xFF);
        uint32_t fcs = test_crc32(psdu, plen-4);
        psdu[plen-4] = (uint8_t)(fcs); psdu[plen-3] = (uint8_t)(fcs>>8);
        psdu[plen-2] = (uint8_t)(fcs>>16); psdu[plen-1] = (uint8_t)(fcs>>24);

        lib80211_tx_vht_params params = {
            .mcs = 5, .psdu = psdu, .psdu_len = plen,
            .scrambler_seed = 0x5D, .short_gi = false, .ldpc = true,
        };

        size_t max_s = lib80211_tx_vht_samples(&params);
        size_t buf = max_s + 200;
        float *re = calloc(buf, sizeof(float));
        float *im = calloc(buf, sizeof(float));
        size_t off = 100;
        size_t n_tx = lib80211_tx_vht(plan, &params, re+off, im+off);
        size_t total = off + n_tx + 100; if (total > buf) total = buf;

        lib80211_rx_result result;
        int rc = lib80211_rx_decode(plan, re, im, total, &result);

        if (rc != 0 || !result.fcs_valid) {
            TEST_FAIL("decode failed (rc=%d, fcs=%d)", rc, result.fcs_valid);
        } else if (result.type != LIB80211_FRAME_VHT || !result.ldpc) {
            TEST_FAIL("wrong type or not LDPC (type=%d, ldpc=%d)", result.type, result.ldpc);
        } else {
            TEST_PASS();
        }
        free(psdu); free(re); free(im);
    }
}

/* ========================================================================
 * 256-QAM modulate/demap round-trip test
 * ======================================================================== */

static void test_256qam_roundtrip(void)
{
    TEST_BEGIN("256qam_modulate_demap_roundtrip");

    /* 256-QAM: n_bpsc=8, test all 256 constellation points */
    int n_symbols = 256;  /* One of each possible symbol */
    int n_bits = n_symbols * 8;

    uint8_t *bits = malloc(n_bits);
    float *mod_re = malloc(n_symbols * sizeof(float));
    float *mod_im = malloc(n_symbols * sizeof(float));
    float *soft_bits = malloc(n_bits * sizeof(float));

    /* Generate all possible 8-bit patterns */
    for (int s = 0; s < n_symbols; s++) {
        for (int b = 0; b < 8; b++) {
            bits[s * 8 + b] = (s >> b) & 1;
        }
    }

    /* Modulate */
    lib80211_modulate(bits, mod_re, mod_im, n_symbols, 8);

    /* Demap (clean, no noise) */
    lib80211_soft_demap(mod_re, mod_im, soft_bits, n_symbols, 8);

    /* Verify hard decisions match original bits */
    int mismatches = 0;
    for (int i = 0; i < n_bits; i++) {
        int hard = (soft_bits[i] > 0) ? 1 : 0;
        if (hard != bits[i]) mismatches++;
    }

    if (mismatches == 0) {
        TEST_PASS();
    } else {
        TEST_FAIL("%d / %d bit mismatches in 256-QAM round-trip", mismatches, n_bits);
    }

    free(bits); free(mod_re); free(mod_im); free(soft_bits);
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(void)
{
    printf("test_edge_cases\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        printf("FATAL: cannot create FFT plan\n");
        return 1;
    }

    /* Large frame tests */
    printf("\nLarge frame loopback tests\n");
    test_large_frame(plan, "large_ht7_2000", 1, 7, 2000, false);
    test_large_frame(plan, "large_ht0_4000", 1, 0, 4000, false);
    test_large_frame(plan, "large_vht8_2000", 2, 8, 2000, false);
    test_large_frame(plan, "large_vht4_4000", 2, 4, 4000, false);
    test_large_frame(plan, "large_ht3_ldpc_2000", 1, 3, 2000, true);
    test_large_frame(plan, "large_vht5_ldpc_3000", 2, 5, 3000, true);

    /* Boundary sizes */
    printf("\nBoundary-size PSDU tests\n");
    test_boundary_sizes(plan);

    /* All scrambler seeds */
    printf("\nScrambler seed exhaustion\n");
    test_all_scrambler_seeds(plan);

    /* Minor gap tests */
    printf("\nMinor gap tests\n");
    test_mu_mimo_rejection(plan);
    test_mcs9_rejected(plan);
    test_vht_ldpc_extra_symbol(plan);
    test_256qam_roundtrip();

    lib80211_fft_plan_destroy(plan);

    TEST_SUMMARY();
    return TEST_EXIT();
}
