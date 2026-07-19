#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Minimal test framework for lib80211 golden vector validation.
 * Each test binary has a main() that calls test functions.
 * Tests load vectors from JSON files in the vectors/ directory.
 */

/* ========================================================================
 * Test result tracking
 * ======================================================================== */

typedef struct {
    int passed;
    int failed;
    const char *current_test;
} test_state;

extern test_state g_test_state;

#define TEST_BEGIN(name) do { \
    g_test_state.current_test = (name); \
    printf("  [ RUN  ] %s\n", (name)); \
} while(0)

#define TEST_PASS() do { \
    g_test_state.passed++; \
    printf("  [ PASS ] %s\n", g_test_state.current_test); \
} while(0)

#define TEST_FAIL(fmt, ...) do { \
    g_test_state.failed++; \
    printf("  [ FAIL ] %s: " fmt "\n", g_test_state.current_test, ##__VA_ARGS__); \
} while(0)

#define TEST_SUMMARY() do { \
    printf("\n  %d passed, %d failed\n", g_test_state.passed, g_test_state.failed); \
} while(0)

#define TEST_EXIT() (g_test_state.failed > 0 ? 1 : 0)

/* ========================================================================
 * Vector loading
 * ======================================================================== */

/**
 * Loaded golden vector. Caller must free with vector_free().
 */
typedef struct {
    /* Bit arrays (from "data" field) */
    uint8_t *bits;
    size_t n_bits;

    /* Float arrays (from "real"/"imag" fields) */
    float *real;
    float *imag;
    size_t n_complex;

    /* Raw JSON string fields */
    char **hex_octets;
    size_t n_octets;
} test_vector;

/**
 * Load a golden vector JSON file by name (without .json extension).
 * Path is relative to the project vectors/ directory.
 * Returns NULL on failure.
 */
test_vector *vector_load(const char *name);

/**
 * Free a loaded vector.
 */
void vector_free(test_vector *vec);

/* ========================================================================
 * Assertions
 * ======================================================================== */

/**
 * Compare two bit arrays (uint8_t per bit). Returns true if equal.
 * Prints first mismatch on failure.
 */
bool assert_bits_equal(const uint8_t *expected, const uint8_t *actual,
                       size_t n_bits, const char *label);

/**
 * Compare two complex float arrays with tolerance.
 * Returns true if all samples within tolerance. Prints worst mismatch on failure.
 */
bool assert_complex_close(const float *exp_real, const float *exp_imag,
                          const float *act_real, const float *act_imag,
                          size_t n, float tol, const char *label);

/**
 * Compare two float arrays with tolerance.
 * Returns true if all values within tolerance.
 */
bool assert_float_close(const float *expected, const float *actual,
                        size_t n, float tol, const char *label);

#endif /* TEST_UTIL_H */
