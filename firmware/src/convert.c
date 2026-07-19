// SPDX-License-Identifier: MIT
#include "convert.h"
#include "hal.h"

#include <math.h>

/* Scale factor: 12-bit signed range is [-2048, +2047] */
#define SCALE_RX  (1.0f / 2048.0f)
#define MAX_DAC   2047

void convert_rx_to_float(const volatile uint32_t *ddr_buf, size_t n_samples,
                         float *out_real, float *out_imag)
{
    for (size_t i = 0; i < n_samples; i++) {
        uint32_t word = ddr_buf[i];
        out_real[i] = (float)IQ_REAL(word) * SCALE_RX;
        out_imag[i] = (float)IQ_IMAG(word) * SCALE_RX;
    }
}

void convert_float_to_tx(const float *in_real, const float *in_imag,
                         size_t n_samples, float peak_scale,
                         volatile uint32_t *ddr_buf)
{
    for (size_t i = 0; i < n_samples; i++) {
        /* Scale and round */
        int32_t re = (int32_t)roundf(in_real[i] * peak_scale);
        int32_t im = (int32_t)roundf(in_imag[i] * peak_scale);

        /* Clamp to 12-bit signed range */
        if (re > MAX_DAC)  re = MAX_DAC;
        if (re < -2048)    re = -2048;
        if (im > MAX_DAC)  im = MAX_DAC;
        if (im < -2048)    im = -2048;

        ddr_buf[i] = IQ_PACK(re, im);
    }
}

float convert_float_to_tx_auto(const float *in_real, const float *in_imag,
                               size_t n_samples,
                               volatile uint32_t *ddr_buf)
{
    /* Find peak absolute value across I and Q */
    float peak = 0.0f;
    for (size_t i = 0; i < n_samples; i++) {
        float ar = fabsf(in_real[i]);
        float ai = fabsf(in_imag[i]);
        if (ar > peak) peak = ar;
        if (ai > peak) peak = ai;
    }

    /* Compute scale to map peak to MAX_DAC */
    float scale = (peak > 0.0f) ? (float)MAX_DAC / peak : (float)MAX_DAC;

    convert_float_to_tx(in_real, in_imag, n_samples, scale, ddr_buf);

    return scale;
}
