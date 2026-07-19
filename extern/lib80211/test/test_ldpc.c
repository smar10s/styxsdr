/**
 * test_ldpc.c — Validate LDPC encoder against IEEE 802.11 parity check matrices.
 *
 * For each of the 12 (cw_len, rate) combinations:
 *   1. Encode deterministic info bits using lib80211_ldpc_encode()
 *   2. Expand the full H matrix from H_SUB circulant sub-matrices (IEEE spec)
 *   3. Verify syndrome: H * codeword = 0 (mod 2)
 *   4. Verify systematic property: codeword[0:K] == info_bits
 *
 * The H matrices come directly from the IEEE 802.11n LDPC .mat files
 * (scripts/octave/ieee_tx11ac/parity_ck/802_11n_ldpc_pcm.mat).
 */

#include "test_util.h"
#include "ldpc_hsub.h"
#include "lib80211/ldpc.h"
#include "lib80211/tx.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
 * Simple deterministic PRNG for test data (xorshift32).
 */
static uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

/**
 * Check syndrome: H * codeword = 0 (mod 2).
 *
 * H is expanded on-the-fly from H_SUB circulant sub-matrices.
 * H_SUB[i][j] = 0 means zero ZxZ block.
 * H_SUB[i][j] = k > 0 means circulant shift by (k % Z).
 *
 * Full H has (M*Z) rows and (24*Z) = cw_len columns.
 * For row (i*Z + r), column (j*Z + c): H[i*Z+r][j*Z+c] = 1
 *   iff H_SUB[i][j] != 0 and c == (r + (H_SUB[i][j] % Z)) % Z.
 *
 * Syndrome bit s = sum over j of H[row][j*Z + ...] * cw[j*Z + ...]
 */
static int check_syndrome(const uint8_t *codeword, int cw_len,
                          const uint8_t (*hsub)[24], int m, int z) {
    int n_check_rows = m * z;

    for (int bi = 0; bi < m; bi++) {
        for (int r = 0; r < z; r++) {
            /* Compute syndrome bit for row (bi*z + r) */
            uint8_t s = 0;
            for (int bj = 0; bj < 24; bj++) {
                uint8_t val = hsub[bi][bj];
                if (val == 0) continue;  /* zero block */
                int shift = val % z;
                int col = bj * z + (r + shift) % z;
                if (col < cw_len) {
                    s ^= codeword[col];
                }
            }
            if (s != 0) {
                printf("    syndrome fail at check row %d (block_row=%d, r=%d)\n",
                       bi * z + r, bi, r);
                return -1;
            }
        }
    }
    (void)n_check_rows;
    return 0;
}

/**
 * Find the H_SUB entry for a given (cw_len, rate_n, rate_d).
 */
static const ldpc_hsub_entry *find_hsub(int cw_len, int rate_n, int rate_d) {
    for (int i = 0; i < LDPC_HSUB_TABLE_LEN; i++) {
        if (LDPC_HSUB_TABLE[i].cw_len == cw_len &&
            LDPC_HSUB_TABLE[i].rate_n == rate_n &&
            LDPC_HSUB_TABLE[i].rate_d == rate_d) {
            return &LDPC_HSUB_TABLE[i];
        }
    }
    return NULL;
}

/**
 * Test one LDPC code: encode, check syndrome, check systematic property.
 */
static void test_ldpc_code(int cw_len, int rate_n, int rate_d) {
    char name[64];
    snprintf(name, sizeof(name), "ldpc_encode_%d_r%d/%d", cw_len, rate_n, rate_d);
    TEST_BEGIN(name);

    int K = cw_len * rate_n / rate_d;

    /* Generate deterministic info bits */
    uint8_t *info_bits = malloc((size_t)K);
    uint8_t *codeword = malloc((size_t)cw_len);
    if (!info_bits || !codeword) {
        TEST_FAIL("malloc failed");
        free(info_bits);
        free(codeword);
        return;
    }

    /* Fill info bits with PRNG (seed derived from parameters for reproducibility) */
    uint32_t seed = (uint32_t)(cw_len * 100 + rate_n * 10 + rate_d);
    for (int i = 0; i < K; i++) {
        info_bits[i] = (uint8_t)(xorshift32(&seed) & 1);
    }

    /* Encode */
    int rc = lib80211_ldpc_encode(info_bits, codeword, cw_len, rate_n, rate_d);
    if (rc != 0) {
        TEST_FAIL("lib80211_ldpc_encode returned %d", rc);
        free(info_bits);
        free(codeword);
        return;
    }

    /* Verify systematic property: first K bits unchanged */
    if (memcmp(codeword, info_bits, (size_t)K) != 0) {
        TEST_FAIL("systematic property violated");
        free(info_bits);
        free(codeword);
        return;
    }

    /* Verify syndrome H*c = 0 */
    const ldpc_hsub_entry *entry = find_hsub(cw_len, rate_n, rate_d);
    if (!entry) {
        TEST_FAIL("H_SUB table entry not found");
        free(info_bits);
        free(codeword);
        return;
    }

    if (check_syndrome(codeword, cw_len, entry->hsub, entry->m, entry->z) != 0) {
        TEST_FAIL("syndrome check failed (H*c != 0)");
        free(info_bits);
        free(codeword);
        return;
    }

    TEST_PASS();
    free(info_bits);
    free(codeword);
}

/**
 * Test invalid parameter handling.
 */
static void test_ldpc_invalid_params(void) {
    TEST_BEGIN("ldpc_invalid_params");

    uint8_t info[324];
    uint8_t codeword[648];
    memset(info, 0, sizeof(info));

    /* Invalid cw_len */
    if (lib80211_ldpc_encode(info, codeword, 100, 1, 2) != -1) {
        TEST_FAIL("should reject invalid cw_len");
        return;
    }

    /* Invalid rate */
    if (lib80211_ldpc_encode(info, codeword, 648, 4, 5) != -1) {
        TEST_FAIL("should reject invalid rate");
        return;
    }

    TEST_PASS();
}

/**
 * Test all-zeros and all-ones info bits (edge cases).
 */
static void test_ldpc_edge_cases(void) {
    TEST_BEGIN("ldpc_edge_zeros");

    /* All-zeros should produce all-zeros codeword (P_GEN * 0 = 0) */
    int cw_len = 648;
    int K = 324;
    uint8_t *info = calloc((size_t)K, 1);
    uint8_t *cw = malloc((size_t)cw_len);

    lib80211_ldpc_encode(info, cw, cw_len, 1, 2);

    int nonzero = 0;
    for (int i = 0; i < cw_len; i++) {
        if (cw[i] != 0) nonzero++;
    }
    if (nonzero != 0) {
        TEST_FAIL("all-zeros info should give all-zeros codeword, got %d nonzero", nonzero);
    } else {
        TEST_PASS();
    }

    free(info);
    free(cw);

    /* All-ones info: verify syndrome */
    TEST_BEGIN("ldpc_edge_ones");

    cw_len = 1944;
    K = 1944 * 3 / 4;  /* rate 3/4 */
    info = malloc((size_t)K);
    cw = malloc((size_t)cw_len);
    memset(info, 1, (size_t)K);

    lib80211_ldpc_encode(info, cw, cw_len, 3, 4);

    const ldpc_hsub_entry *entry = find_hsub(cw_len, 3, 4);
    if (!entry) {
        TEST_FAIL("H_SUB not found");
    } else if (check_syndrome(cw, cw_len, entry->hsub, entry->m, entry->z) != 0) {
        TEST_FAIL("syndrome check failed for all-ones info");
    } else {
        TEST_PASS();
    }

    free(info);
    free(cw);
}

/**
 * Test HT TX with LDPC coding — verify it produces non-zero output
 * and correct sample count.
 */
static void test_ldpc_ht_tx(void) {
    TEST_BEGIN("ldpc_ht_tx_mcs0");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();

    /* 50-byte PSDU with LDPC coding */
    uint8_t psdu[50];
    for (int i = 0; i < 50; i++) psdu[i] = (uint8_t)(i & 0xFF);

    lib80211_tx_ht_params params = {
        .mcs = 0,
        .psdu = psdu,
        .psdu_len = 50,
        .scrambler_seed = 0x5D,
        .short_gi = false,
        .ldpc = true,
    };

    size_t max_samples = lib80211_tx_ht_samples(&params);
    float *out_real = calloc(max_samples, sizeof(float));
    float *out_imag = calloc(max_samples, sizeof(float));

    size_t n = lib80211_tx_ht(plan, &params, out_real, out_imag);

    if (n == 0) {
        TEST_FAIL("lib80211_tx_ht returned 0");
    } else if (n > max_samples) {
        TEST_FAIL("output %zu exceeds max %zu", n, max_samples);
    } else {
        /* Check that output is non-trivial (not all zeros) */
        float max_mag = 0.0f;
        for (size_t i = 0; i < n; i++) {
            float mag = out_real[i] * out_real[i] + out_imag[i] * out_imag[i];
            if (mag > max_mag) max_mag = mag;
        }
        if (max_mag < 1.0f) {
            TEST_FAIL("output looks empty (max magnitude %.4f)", max_mag);
        } else {
            TEST_PASS();
        }
    }

    free(out_real);
    free(out_imag);
    lib80211_fft_plan_destroy(plan);
}

/**
 * Test VHT TX with LDPC coding.
 */
static void test_ldpc_vht_tx(void) {
    TEST_BEGIN("ldpc_vht_tx_mcs4");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();

    /* 100-byte PSDU with LDPC coding, MCS 4 (16-QAM r=3/4) */
    uint8_t psdu[100];
    for (int i = 0; i < 100; i++) psdu[i] = (uint8_t)((i * 7 + 3) & 0xFF);

    lib80211_tx_vht_params params = {
        .mcs = 4,
        .psdu = psdu,
        .psdu_len = 100,
        .scrambler_seed = 0x5D,
        .short_gi = false,
        .ldpc = true,
    };

    size_t max_samples = lib80211_tx_vht_samples(&params);
    float *out_real = calloc(max_samples, sizeof(float));
    float *out_imag = calloc(max_samples, sizeof(float));

    size_t n = lib80211_tx_vht(plan, &params, out_real, out_imag);

    if (n == 0) {
        TEST_FAIL("lib80211_tx_vht returned 0");
    } else if (n > max_samples) {
        TEST_FAIL("output %zu exceeds max %zu", n, max_samples);
    } else {
        /* Check non-trivial output */
        float max_mag = 0.0f;
        for (size_t i = 0; i < n; i++) {
            float mag = out_real[i] * out_real[i] + out_imag[i] * out_imag[i];
            if (mag > max_mag) max_mag = mag;
        }
        if (max_mag < 1.0f) {
            TEST_FAIL("output looks empty (max magnitude %.4f)", max_mag);
        } else {
            TEST_PASS();
        }
    }

    free(out_real);
    free(out_imag);
    lib80211_fft_plan_destroy(plan);
}

/**
 * Box-Muller AWGN sample using xorshift32 PRNG.
 */
static float box_muller(uint32_t *state) {
    float u1, u2;
    do {
        u1 = (float)(xorshift32(state) & 0xFFFFFF) / 16777216.0f;
    } while (u1 < 1e-10f);
    u2 = (float)(xorshift32(state) & 0xFFFFFF) / 16777216.0f;
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

/**
 * Test LDPC decoder with clean (noiseless) LLR input for all 12 codes.
 */
static void test_ldpc_decode_clean(void) {
    int rates[][2] = { {1,2}, {2,3}, {3,4}, {5,6} };
    int cw_lens[] = { 648, 1296, 1944 };

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 3; c++) {
            int cw_len = cw_lens[c];
            int rate_n = rates[r][0];
            int rate_d = rates[r][1];
            int K = cw_len * rate_n / rate_d;

            char name[64];
            snprintf(name, sizeof(name), "ldpc_decode_clean_%d_r%d/%d",
                     cw_len, rate_n, rate_d);
            TEST_BEGIN(name);

            uint8_t *info_bits = malloc((size_t)K);
            uint8_t *codeword = malloc((size_t)cw_len);
            float *llr = malloc((size_t)cw_len * sizeof(float));
            uint8_t *decoded = malloc((size_t)cw_len);

            if (!info_bits || !codeword || !llr || !decoded) {
                TEST_FAIL("malloc failed");
                free(info_bits); free(codeword); free(llr); free(decoded);
                continue;
            }

            /* Generate random info bits */
            uint32_t seed = (uint32_t)(cw_len * 1000 + rate_n * 100 + rate_d + 7);
            for (int i = 0; i < K; i++)
                info_bits[i] = (uint8_t)(xorshift32(&seed) & 1);

            /* Encode */
            int rc = lib80211_ldpc_encode(info_bits, codeword, cw_len, rate_n, rate_d);
            if (rc != 0) {
                TEST_FAIL("encode returned %d", rc);
                free(info_bits); free(codeword); free(llr); free(decoded);
                continue;
            }

            /* Convert to clean LLR: bit==0 -> +4.0, bit==1 -> -4.0 */
            for (int i = 0; i < cw_len; i++)
                llr[i] = codeword[i] ? -4.0f : 4.0f;

            /* Decode */
            int iters = lib80211_ldpc_decode(llr, decoded, cw_len, rate_n, rate_d, 30);
            if (iters <= 0) {
                TEST_FAIL("decoder did not converge (returned %d)", iters);
                free(info_bits); free(codeword); free(llr); free(decoded);
                continue;
            }

            /* Check first K decoded bits match original info bits */
            int mismatches = 0;
            for (int i = 0; i < K; i++) {
                if (decoded[i] != info_bits[i]) mismatches++;
            }
            if (mismatches != 0) {
                TEST_FAIL("%d bit mismatches in first K=%d bits", mismatches, K);
            } else {
                TEST_PASS();
            }

            free(info_bits);
            free(codeword);
            free(llr);
            free(decoded);
        }
    }
}

/**
 * Test LDPC decoder with noisy (AWGN) channel for 4 representative codes.
 * Uses 1944-length codewords at each rate with marginal Eb/N0.
 */
static void test_ldpc_decode_noisy(void) {
    struct {
        int rate_n, rate_d;
        float eb_n0_db;
    } codes[] = {
        { 1, 2, 2.0f },
        { 2, 3, 3.5f },
        { 3, 4, 4.5f },
        { 5, 6, 5.5f },
    };
    int cw_len = 1944;
    int n_trials = 5;
    int min_pass = 4;

    for (int ci = 0; ci < 4; ci++) {
        int rate_n = codes[ci].rate_n;
        int rate_d = codes[ci].rate_d;
        float eb_n0_db = codes[ci].eb_n0_db;
        int K = cw_len * rate_n / rate_d;
        float code_rate = (float)rate_n / (float)rate_d;

        /* Eb/N0 -> noise sigma */
        float eb_n0_lin = powf(10.0f, eb_n0_db / 10.0f);
        float sigma = sqrtf(1.0f / (2.0f * code_rate * eb_n0_lin));

        char name[64];
        snprintf(name, sizeof(name), "ldpc_decode_noisy_1944_r%d/%d_%.1fdB",
                 rate_n, rate_d, (double)eb_n0_db);
        TEST_BEGIN(name);

        int successes = 0;

        for (int trial = 0; trial < n_trials; trial++) {
            uint8_t *info_bits = malloc((size_t)K);
            uint8_t *codeword = malloc((size_t)cw_len);
            float *llr = malloc((size_t)cw_len * sizeof(float));
            uint8_t *decoded = malloc((size_t)cw_len);

            if (!info_bits || !codeword || !llr || !decoded) {
                free(info_bits); free(codeword); free(llr); free(decoded);
                continue;
            }

            /* Generate random info bits with trial-dependent seed */
            uint32_t seed = (uint32_t)(rate_n * 10000 + rate_d * 1000 + trial * 13 + 42);
            for (int i = 0; i < K; i++)
                info_bits[i] = (uint8_t)(xorshift32(&seed) & 1);

            /* Encode */
            lib80211_ldpc_encode(info_bits, codeword, cw_len, rate_n, rate_d);

            /* BPSK modulate + AWGN noise -> LLR */
            uint32_t noise_seed = (uint32_t)(trial * 7919 + rate_n * 31 + 123);
            float sigma2 = sigma * sigma;
            for (int i = 0; i < cw_len; i++) {
                float x = codeword[i] ? -1.0f : 1.0f;  /* BPSK: 0->+1, 1->-1 */
                float n = box_muller(&noise_seed) * sigma;
                float y = x + n;
                llr[i] = 2.0f * y / sigma2;
            }

            /* Decode */
            int iters = lib80211_ldpc_decode(llr, decoded, cw_len, rate_n, rate_d, 50);

            /* Check convergence and bit-exact match */
            if (iters > 0) {
                int ok = 1;
                for (int i = 0; i < K; i++) {
                    if (decoded[i] != info_bits[i]) { ok = 0; break; }
                }
                if (ok) successes++;
            }

            free(info_bits);
            free(codeword);
            free(llr);
            free(decoded);
        }

        if (successes >= min_pass) {
            TEST_PASS();
        } else {
            TEST_FAIL("%d/%d trials passed (need >= %d)", successes, n_trials, min_pass);
        }
    }
}

/**
 * Test that LDPC decoder converges early on clean input.
 */
static void test_ldpc_decode_early_termination(void) {
    TEST_BEGIN("ldpc_decode_early_termination");

    int cw_len = 648;
    int rate_n = 1, rate_d = 2;
    int K = cw_len * rate_n / rate_d;

    /* All-zeros info -> all-zeros codeword */
    uint8_t *info_bits = calloc((size_t)K, 1);
    uint8_t *codeword = malloc((size_t)cw_len);
    float *llr = malloc((size_t)cw_len * sizeof(float));
    uint8_t *decoded = malloc((size_t)cw_len);

    if (!info_bits || !codeword || !llr || !decoded) {
        TEST_FAIL("malloc failed");
        free(info_bits); free(codeword); free(llr); free(decoded);
        return;
    }

    lib80211_ldpc_encode(info_bits, codeword, cw_len, rate_n, rate_d);

    /* Clean LLR: all bits are 0, so LLR = +4.0 for all */
    for (int i = 0; i < cw_len; i++)
        llr[i] = 4.0f;

    /* Decode with max_iterations=30 */
    int iters = lib80211_ldpc_decode(llr, decoded, cw_len, rate_n, rate_d, 30);

    if (iters <= 0) {
        TEST_FAIL("decoder did not converge (returned %d)", iters);
    } else if (iters > 3) {
        TEST_FAIL("expected <= 3 iterations for clean input, got %d", iters);
    } else {
        TEST_PASS();
    }

    free(info_bits);
    free(codeword);
    free(llr);
    free(decoded);
}

int main(void) {
    printf("test_ldpc\n");

    /* Test all 12 LDPC codes */
    int rates[][2] = { {1,2}, {2,3}, {3,4}, {5,6} };
    int cw_lens[] = { 648, 1296, 1944 };

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 3; c++) {
            test_ldpc_code(cw_lens[c], rates[r][0], rates[r][1]);
        }
    }

    test_ldpc_invalid_params();
    test_ldpc_edge_cases();

    /* Test LDPC TX integration (HT + VHT) */
    test_ldpc_ht_tx();
    test_ldpc_vht_tx();

    /* Test LDPC decoder */
    test_ldpc_decode_clean();
    test_ldpc_decode_noisy();
    test_ldpc_decode_early_termination();

    TEST_SUMMARY();
    return TEST_EXIT();
}
