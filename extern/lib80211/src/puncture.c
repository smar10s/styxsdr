/**
 * puncture.c — Puncturing for convolutional codes
 *
 * IEEE 802.11-2020 Section 17.3.5.6 (rates 2/3, 3/4)
 * and Section 19.3.11.3 (rate 5/6).
 */

#include "lib80211/fec.h"

/* Puncture patterns: 1 = keep, 0 = delete */
static const uint8_t PATTERN_23[] = { 1, 1, 1, 0 };          /* 4 in -> 3 out */
static const uint8_t PATTERN_34[] = { 1, 1, 1, 0, 0, 1 };    /* 6 in -> 4 out */
static const uint8_t PATTERN_56[] = { 1, 1, 1, 0, 0, 1, 1, 0, 0, 1 }; /* 10 in -> 6 out */

size_t lib80211_puncture(const uint8_t *in_bits, uint8_t *out_bits,
                         size_t n_coded_bits, int cr_n, int cr_d) {
    /* Rate 1/2: no puncturing */
    if (cr_n == 1 && cr_d == 2) {
        for (size_t i = 0; i < n_coded_bits; i++)
            out_bits[i] = in_bits[i];
        return n_coded_bits;
    }

    const uint8_t *pattern;
    size_t pat_len;

    if (cr_n == 2 && cr_d == 3) {
        pattern = PATTERN_23;
        pat_len = 4;
    } else if (cr_n == 3 && cr_d == 4) {
        pattern = PATTERN_34;
        pat_len = 6;
    } else if (cr_n == 5 && cr_d == 6) {
        pattern = PATTERN_56;
        pat_len = 10;
    } else {
        return 0;  /* Unsupported rate */
    }

    size_t n_groups = n_coded_bits / pat_len;
    size_t out_idx = 0;

    for (size_t g = 0; g < n_groups; g++) {
        for (size_t p = 0; p < pat_len; p++) {
            if (pattern[p]) {
                out_bits[out_idx++] = in_bits[g * pat_len + p];
            }
        }
    }

    return out_idx;
}

size_t lib80211_depuncture(const float *in_soft, float *out_soft,
                           size_t n_input, int cr_n, int cr_d) {
    /* Rate 1/2: identity */
    if (cr_n == 1 && cr_d == 2) {
        for (size_t i = 0; i < n_input; i++)
            out_soft[i] = in_soft[i];
        return n_input;
    }

    const uint8_t *pattern;
    size_t pat_len;
    size_t kept_per_group;

    if (cr_n == 2 && cr_d == 3) {
        pattern = PATTERN_23; pat_len = 4; kept_per_group = 3;
    } else if (cr_n == 3 && cr_d == 4) {
        pattern = PATTERN_34; pat_len = 6; kept_per_group = 4;
    } else if (cr_n == 5 && cr_d == 6) {
        pattern = PATTERN_56; pat_len = 10; kept_per_group = 6;
    } else {
        return 0;
    }

    size_t n_groups = n_input / kept_per_group;
    size_t in_idx = 0;
    size_t out_idx = 0;

    for (size_t g = 0; g < n_groups; g++) {
        for (size_t p = 0; p < pat_len; p++) {
            if (pattern[p]) {
                out_soft[out_idx++] = in_soft[in_idx++];
            } else {
                out_soft[out_idx++] = 0.0f;  /* erasure */
            }
        }
    }

    return out_idx;
}
