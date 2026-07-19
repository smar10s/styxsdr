#include "test_util.h"
#include "vendor/cJSON.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Global test state */
test_state g_test_state = { .passed = 0, .failed = 0, .current_test = NULL };

/* ========================================================================
 * Path resolution — find vectors/ directory
 * ======================================================================== */

/**
 * Resolve path to vectors directory.
 * Tries: ../vectors (build-cmake/), ../../vectors (build-cmake/test/),
 * and the VECTORS_DIR env var.
 */
static const char *vectors_dir(void) {
    static char path[4096] = {0};
    if (path[0]) return path;

    /* Check environment override */
    const char *env = getenv("LIB80211_VECTORS_DIR");
    if (env) {
        snprintf(path, sizeof(path), "%s", env);
        return path;
    }

    /* Try relative paths from typical build locations */
    const char *candidates[] = {
        "vectors",
        "../vectors",
        "../../vectors",
        "../../../vectors",
    };

    for (size_t i = 0; i < sizeof(candidates)/sizeof(candidates[0]); i++) {
        char test_path[4096];
        snprintf(test_path, sizeof(test_path), "%s/annex_i1_psdu.json", candidates[i]);
        FILE *f = fopen(test_path, "r");
        if (f) {
            fclose(f);
            snprintf(path, sizeof(path), "%s", candidates[i]);
            return path;
        }
    }

    fprintf(stderr, "ERROR: Cannot find vectors/ directory. "
            "Set LIB80211_VECTORS_DIR or run from project root.\n");
    return NULL;
}

/* ========================================================================
 * Vector loading
 * ======================================================================== */

test_vector *vector_load(const char *name) {
    const char *dir = vectors_dir();
    if (!dir) return NULL;

    char filepath[4096];
    snprintf(filepath, sizeof(filepath), "%s/%s.json", dir, name);

    FILE *f = fopen(filepath, "r");
    if (!f) {
        fprintf(stderr, "ERROR: Cannot open vector file: %s\n", filepath);
        return NULL;
    }

    /* Read entire file */
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *json_str = malloc(fsize + 1);
    if (!json_str) { fclose(f); return NULL; }
    fread(json_str, 1, fsize, f);
    json_str[fsize] = '\0';
    fclose(f);

    /* Parse JSON */
    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) {
        fprintf(stderr, "ERROR: JSON parse failed for %s\n", filepath);
        return NULL;
    }

    test_vector *vec = calloc(1, sizeof(test_vector));
    if (!vec) { cJSON_Delete(root); return NULL; }

    /* Helper: extract array of [real, imag] pairs into split complex */
    #define EXTRACT_PAIRS(arr) do { \
        vec->n_complex = (size_t)cJSON_GetArraySize(arr); \
        vec->real = malloc(vec->n_complex * sizeof(float)); \
        vec->imag = malloc(vec->n_complex * sizeof(float)); \
        if (vec->real && vec->imag) { \
            for (size_t i = 0; i < vec->n_complex; i++) { \
                cJSON *pair = cJSON_GetArrayItem(arr, (int)i); \
                if (pair && cJSON_IsArray(pair)) { \
                    cJSON *re = cJSON_GetArrayItem(pair, 0); \
                    cJSON *im = cJSON_GetArrayItem(pair, 1); \
                    vec->real[i] = (float)(re ? re->valuedouble : 0.0); \
                    vec->imag[i] = (float)(im ? im->valuedouble : 0.0); \
                } else { \
                    vec->real[i] = 0.0f; \
                    vec->imag[i] = 0.0f; \
                } \
            } \
        } \
    } while(0)

    /* Extract "data" field — either flat bit array or array of [Re,Im] pairs */
    cJSON *data = cJSON_GetObjectItem(root, "data");
    if (data && cJSON_IsArray(data)) {
        cJSON *first = cJSON_GetArrayItem(data, 0);
        if (first && cJSON_IsArray(first)) {
            /* Array of [real, imag] pairs (e.g. annex_i1_data_qam.json) */
            EXTRACT_PAIRS(data);
        } else {
            /* Flat integer bit array */
            vec->n_bits = (size_t)cJSON_GetArraySize(data);
            vec->bits = malloc(vec->n_bits);
            if (vec->bits) {
                for (size_t i = 0; i < vec->n_bits; i++) {
                    cJSON *item = cJSON_GetArrayItem(data, (int)i);
                    vec->bits[i] = (uint8_t)(item ? item->valueint : 0);
                }
            }
        }
    }

    /* Extract "real"/"imag" parallel arrays (HT/VHT vectors) */
    cJSON *real_arr = cJSON_GetObjectItem(root, "real");
    cJSON *imag_arr = cJSON_GetObjectItem(root, "imag");
    if (real_arr && cJSON_IsArray(real_arr) && !vec->real) {
        vec->n_complex = (size_t)cJSON_GetArraySize(real_arr);
        vec->real = malloc(vec->n_complex * sizeof(float));
        vec->imag = malloc(vec->n_complex * sizeof(float));
        if (vec->real && vec->imag) {
            for (size_t i = 0; i < vec->n_complex; i++) {
                cJSON *r = cJSON_GetArrayItem(real_arr, (int)i);
                vec->real[i] = (float)(r ? r->valuedouble : 0.0);
                cJSON *im = imag_arr ? cJSON_GetArrayItem(imag_arr, (int)i) : NULL;
                vec->imag[i] = (float)(im ? im->valuedouble : 0.0);
            }
        }
    }

    /* Extract "samples" field — array of [real, imag] pairs (time-domain vectors) */
    cJSON *samples = cJSON_GetObjectItem(root, "samples");
    if (samples && cJSON_IsArray(samples) && !vec->real) {
        EXTRACT_PAIRS(samples);
    }

    /* Extract "samples_one_period" field (STF one-period vector) */
    cJSON *samples_1p = cJSON_GetObjectItem(root, "samples_one_period");
    if (samples_1p && cJSON_IsArray(samples_1p) && !vec->real) {
        EXTRACT_PAIRS(samples_1p);
    }

    #undef EXTRACT_PAIRS

    /* Extract "bits" field (alternate name for bit arrays, e.g. signal_bits) */
    cJSON *bits_field = cJSON_GetObjectItem(root, "bits");
    if (bits_field && cJSON_IsArray(bits_field) && !vec->bits) {
        vec->n_bits = (size_t)cJSON_GetArraySize(bits_field);
        vec->bits = malloc(vec->n_bits);
        if (vec->bits) {
            for (size_t i = 0; i < vec->n_bits; i++) {
                cJSON *item = cJSON_GetArrayItem(bits_field, (int)i);
                vec->bits[i] = (uint8_t)(item ? item->valueint : 0);
            }
        }
    }

    /* Extract "subcarriers" field — array of [Re,Im] pairs (freq-domain vectors) */
    cJSON *subcarriers = cJSON_GetObjectItem(root, "subcarriers");
    if (subcarriers && cJSON_IsArray(subcarriers) && !vec->real) {
        size_t n = (size_t)cJSON_GetArraySize(subcarriers);
        vec->n_complex = n;
        vec->real = malloc(n * sizeof(float));
        vec->imag = malloc(n * sizeof(float));
        if (vec->real && vec->imag) {
            for (size_t i = 0; i < n; i++) {
                cJSON *item = cJSON_GetArrayItem(subcarriers, (int)i);
                if (item && cJSON_IsArray(item)) {
                    cJSON *re = cJSON_GetArrayItem(item, 0);
                    cJSON *im = cJSON_GetArrayItem(item, 1);
                    vec->real[i] = (float)(re ? re->valuedouble : 0.0);
                    vec->imag[i] = (float)(im ? im->valuedouble : 0.0);
                } else if (item && cJSON_IsNull(item)) {
                    /* null entries (pilot positions in some vectors) */
                    vec->real[i] = 0.0f;
                    vec->imag[i] = 0.0f;
                } else {
                    vec->real[i] = 0.0f;
                    vec->imag[i] = 0.0f;
                }
            }
        }
    }

    /* Extract "octets_hex" field */
    cJSON *hex = cJSON_GetObjectItem(root, "octets_hex");
    if (hex && cJSON_IsArray(hex)) {
        vec->n_octets = (size_t)cJSON_GetArraySize(hex);
        vec->hex_octets = malloc(vec->n_octets * sizeof(char *));
        if (vec->hex_octets) {
            for (size_t i = 0; i < vec->n_octets; i++) {
                cJSON *item = cJSON_GetArrayItem(hex, (int)i);
                vec->hex_octets[i] = item ? strdup(item->valuestring) : strdup("00");
            }
        }
    }

    cJSON_Delete(root);
    return vec;
}

void vector_free(test_vector *vec) {
    if (!vec) return;
    free(vec->bits);
    free(vec->real);
    free(vec->imag);
    if (vec->hex_octets) {
        for (size_t i = 0; i < vec->n_octets; i++)
            free(vec->hex_octets[i]);
        free(vec->hex_octets);
    }
    free(vec);
}

/* ========================================================================
 * Assertions
 * ======================================================================== */

bool assert_bits_equal(const uint8_t *expected, const uint8_t *actual,
                       size_t n_bits, const char *label) {
    for (size_t i = 0; i < n_bits; i++) {
        if (expected[i] != actual[i]) {
            printf("    %s: mismatch at bit %zu: expected %u, got %u\n",
                   label, i, expected[i], actual[i]);
            return false;
        }
    }
    return true;
}

bool assert_complex_close(const float *exp_real, const float *exp_imag,
                          const float *act_real, const float *act_imag,
                          size_t n, float tol, const char *label) {
    float worst_err = 0.0f;
    size_t worst_idx = 0;

    for (size_t i = 0; i < n; i++) {
        float dr = exp_real[i] - act_real[i];
        float di = exp_imag[i] - act_imag[i];
        float err = sqrtf(dr * dr + di * di);
        if (err > worst_err) {
            worst_err = err;
            worst_idx = i;
        }
    }

    if (worst_err > tol) {
        printf("    %s: worst error %.2e at index %zu "
               "(expected %.6f+%.6fi, got %.6f+%.6fi)\n",
               label, worst_err, worst_idx,
               exp_real[worst_idx], exp_imag[worst_idx],
               act_real[worst_idx], act_imag[worst_idx]);
        return false;
    }
    return true;
}

bool assert_float_close(const float *expected, const float *actual,
                        size_t n, float tol, const char *label) {
    float worst_err = 0.0f;
    size_t worst_idx = 0;

    for (size_t i = 0; i < n; i++) {
        float err = fabsf(expected[i] - actual[i]);
        if (err > worst_err) {
            worst_err = err;
            worst_idx = i;
        }
    }

    if (worst_err > tol) {
        printf("    %s: worst error %.2e at index %zu "
               "(expected %.6f, got %.6f)\n",
               label, worst_err, worst_idx,
               expected[worst_idx], actual[worst_idx]);
        return false;
    }
    return true;
}
