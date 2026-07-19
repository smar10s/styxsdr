// SPDX-License-Identifier: MIT
#include "test_util.h"
#include "hal.h"      /* IQ_PACK, IQ_REAL, IQ_IMAG */
#include "convert.h"

#include <string.h>
#include <math.h>

/* --------------------------------------------------------------------------
 * IQ_PACK / IQ_REAL / IQ_IMAG round-trip
 * -------------------------------------------------------------------------- */

static void test_iq_round_trip(void)
{
    TEST_BEGIN("iq_pack_unpack_round_trip");

    struct { int16_t re; int16_t im; } cases[] = {
        {0, 0}, {1, -1}, {2047, -2048}, {-2048, 2047},
        {42, -99}, {-512, 1024}, {0x7FF, -0x800},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint32_t packed = IQ_PACK(cases[i].re, cases[i].im);
        int16_t re_got = IQ_REAL(packed);
        int16_t im_got = IQ_IMAG(packed);

        char label[64];
        snprintf(label, sizeof(label), "round_trip[%zu].re", i);
        if (!assert_i32(label, cases[i].re, re_got)) return;

        snprintf(label, sizeof(label), "round_trip[%zu].im", i);
        if (!assert_i32(label, cases[i].im, im_got)) return;
    }

    TEST_PASS();
}

/* Test that 12-bit sign extension handles the boundary correctly:
 * 0x7FF stays +2047, 0x800 maps to -2048 */
static void test_sign_extension(void)
{
    TEST_BEGIN("iq_sign_extension");

    uint32_t pos_max = IQ_PACK(0x7FF, 0);
    if (!assert_i32("real 0x7FF→+2047", 2047, IQ_REAL(pos_max)))
        return;

    uint32_t neg_max = IQ_PACK(0, 0x800);
    if (!assert_i32("imag 0x800→-2048", -2048, IQ_IMAG(neg_max)))
        return;

    /* Check 13-bit values are masked to 12 bits.
     * IQ_PACK masks {real,imag} to 12 bits before packing.
     * Input 0x1FFF truncated to 0xFFF, which sign-extends to -1. */
    uint32_t overflow_in  = IQ_PACK(0x1FFF, 0x1FFF);
    uint32_t masked_12bit = IQ_PACK(0xFFF,   0xFFF);
    if (!assert_u32("masked overflow == literal 0xFFF", masked_12bit, overflow_in))
        return;
    /* Sign-extended 0xFFF (all bits set) is -1 */
    if (!assert_i32("overflow real → -1", -1, IQ_REAL(overflow_in)))
        return;
    if (!assert_i32("overflow imag → -1", -1, IQ_IMAG(overflow_in)))
        return;

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * convert_rx_to_float
 * -------------------------------------------------------------------------- */

static void test_convert_rx_to_float_zero(void)
{
    TEST_BEGIN("convert_rx_to_float_zero");

    uint32_t ddr[4] = {IQ_PACK(0, 0), IQ_PACK(0, 0), IQ_PACK(0, 0), IQ_PACK(0, 0)};
    float re[4], im[4];

    convert_rx_to_float(ddr, 4, re, im);

    for (int i = 0; i < 4; i++) {
        char label[32];
        snprintf(label, sizeof(label), "zero_re[%d]", i);
        if (!assert_float_close(label, 0.0f, re[i], 1e-6f)) return;
        snprintf(label, sizeof(label), "zero_im[%d]", i);
        if (!assert_float_close(label, 0.0f, im[i], 1e-6f)) return;
    }

    TEST_PASS();
}

static void test_convert_rx_to_float_scale(void)
{
    TEST_BEGIN("convert_rx_to_float_scale");

    uint32_t ddr[2] = {IQ_PACK(2047, -2048), IQ_PACK(1024, -1024)};
    float re[2], im[2];

    convert_rx_to_float(ddr, 2, re, im);

    if (!assert_float_close("pos_max_re", 2047.0f / 2048.0f, re[0], 1e-6f)) return;
    if (!assert_float_close("neg_max_im", -2048.0f / 2048.0f, im[0], 1e-6f)) return;
    if (!assert_float_close("half_re", 1024.0f / 2048.0f, re[1], 1e-6f)) return;
    if (!assert_float_close("half_im", -1024.0f / 2048.0f, im[1], 1e-6f)) return;

    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * convert_float_to_tx
 * -------------------------------------------------------------------------- */

static void test_convert_float_to_tx_clamp_max(void)
{
    TEST_BEGIN("convert_float_to_tx_clamp_positive");

    float re_in[1] = {2.0f};  /* would produce 4096 unscaled */
    float im_in[1] = {-2.0f}; /* would produce -4096 unscaled */
    uint32_t ddr[1] = {0};

    convert_float_to_tx(re_in, im_in, 1, 2047.0f, ddr);

    /* Both should clamp: real to 2047, imag to -2048 */
    if (!assert_i32("clamp_re", 2047, IQ_REAL(ddr[0]))) return;
    if (!assert_i32("clamp_im", -2048, IQ_IMAG(ddr[0]))) return;

    TEST_PASS();
}

static void test_convert_float_to_tx_exact(void)
{
    TEST_BEGIN("convert_float_to_tx_exact_scale");

    float re_in[3] = {0.0f, 0.5f, -0.5f};
    float im_in[3] = {0.0f, 0.25f, -0.25f};
    uint32_t ddr[3] = {0};

    convert_float_to_tx(re_in, im_in, 3, 1024.0f, ddr);

    /* 0.5 * 1024 = 512, -0.5 * 1024 = -512 (roundf) */
    if (!assert_i32("re_0", 0, IQ_REAL(ddr[0]))) return;
    if (!assert_i32("re_1", 512, IQ_REAL(ddr[1]))) return;
    if (!assert_i32("re_2", -512, IQ_REAL(ddr[2]))) return;
    /* 0.0 * 1024 = 0, 0.25 * 1024 = 256, -0.25 * 1024 = -256 */
    if (!assert_i32("im_0", 0, IQ_IMAG(ddr[0]))) return;
    if (!assert_i32("im_1", 256, IQ_IMAG(ddr[1]))) return;
    if (!assert_i32("im_2", -256, IQ_IMAG(ddr[2]))) return;

    TEST_PASS();
}

static void test_convert_float_to_tx_rounding(void)
{
    TEST_BEGIN("convert_float_to_tx_rounding");

    /* 0.499 ≈ 511, 0.501 ≈ 513 (banker's rounding via roundf) */
    float re_in[2] = {0.499f, 0.501f};
    float im_in[2] = {-0.499f, -0.501f};
    uint32_t ddr[2] = {0};

    convert_float_to_tx(re_in, im_in, 2, 1024.0f, ddr);

    if (!assert_i32("round_up", 513, IQ_REAL(ddr[1]))) return;
    if (!assert_i32("round_down", -513, IQ_IMAG(ddr[1]))) return;

    (void)ddr[0]; /* roundf does banker's rounding, exact value depends on FPU */
    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * convert_float_to_tx_auto
 * -------------------------------------------------------------------------- */

static void test_convert_float_to_tx_auto_peak(void)
{
    TEST_BEGIN("convert_float_to_tx_auto_finds_peak");

    float re[4] = {0.1f, 0.5f, 0.2f, -0.1f};
    float im[4] = {0.0f, -0.3f, -0.8f, 0.4f};
    uint32_t ddr[4] = {0};

    /* Peak magnitude = 0.8f, scale = 2047 / 0.8 = 2558.75 */
    float scale = convert_float_to_tx_auto(re, im, 4, ddr);

    if (!assert_float_close("scale", 2047.0f / 0.8f, scale, 0.1f)) return;

    /* Maximum scaled sample (im[2] = -0.8) should map to exactly -2047 */
    if (!assert_i32("peak_im", -2047, IQ_IMAG(ddr[2]))) return;

    TEST_PASS();
}

static void test_convert_float_to_tx_auto_zero(void)
{
    TEST_BEGIN("convert_float_to_tx_auto_all_zero");

    float re[2] = {0.0f, 0.0f};
    float im[2] = {0.0f, 0.0f};
    uint32_t ddr[2] = {0xDEAD, 0xBEEF}; /* non-zero init to verify write */

    float scale = convert_float_to_tx_auto(re, im, 2, ddr);

    /* When all zeros, peak=0, scale should fall back to MAX_DAC (2047).
     * All output should be IQ_PACK(0, 0). */
    if (!assert_float_close("scale_fallback", 2047.0f, scale, 0.1f)) return;
    if (!assert_u32("ddr[0]", IQ_PACK(0, 0), ddr[0])) return;
    if (!assert_u32("ddr[1]", IQ_PACK(0, 0), ddr[1])) return;

    TEST_PASS();
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    test_iq_round_trip();
    test_sign_extension();
    test_convert_rx_to_float_zero();
    test_convert_rx_to_float_scale();
    test_convert_float_to_tx_clamp_max();
    test_convert_float_to_tx_exact();
    test_convert_float_to_tx_rounding();
    test_convert_float_to_tx_auto_peak();
    test_convert_float_to_tx_auto_zero();

    TEST_SUMMARY();
    return TEST_EXIT();
}
