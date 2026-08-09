// SPDX-License-Identifier: MIT
#include "convert.h"
#include "hal.h"

#include <math.h>
#include <string.h>

/* Scale factor: 12-bit signed range is [-2048, +2047] */
#define SCALE_RX  (1.0f / 2048.0f)
#define MAX_DAC   2047

/*
 * Samples staged per block on the way out of DDR.  4096 words is 16 KB,
 * which fits in the Cortex-A9's 32 KB L1 D-cache, so the unpack pass that
 * follows reads cached memory and costs nothing extra.
 */
#define RX_STAGE_SAMPLES 4096

/*
 * The DDR reserve is mapped through /dev/mem with O_SYNC, so it is uncached
 * and a word-at-a-time read loop pays a memory round trip per sample.
 * Staging through memcpy first lets the C library issue wide multi-word
 * loads, which this loop cannot: the `volatile` qualifier is required at the
 * interface, since the FPGA writes this memory, but it forbids the compiler
 * from widening or merging the accesses.
 *
 * Measured in the daemon on fm_mode.c's real 49,152-sample chunk at
 * 2.1 MSPS, A/B against the plain loop:
 *
 *   volatile word loop   2.360 ms/chunk   10.1% of a core
 *   memcpy-staged        2.001 ms/chunk    8.5% of a core   (1.18x)
 *
 * A modest win, and deliberately recorded as such: an earlier estimate put
 * this path at 30.9% of a core and the staging win at 2.56x, which did not
 * survive measurement in the real path. For scale, the FM demodulator that
 * consumes this output costs 36.1% — 3.6x this whole function — so the
 * convert path is not where FM RX headroom is won.
 *
 * Compiler auto-vectorisation is not an alternative. At -O3 with the
 * volatile removed, GCC 11 emits `ldrd` (two words per access, ~2x on
 * access count) and leaves the arithmetic scalar, because the 12-bit
 * sign-extending unpack de-interleaves into two output arrays. That would
 * cost both a memory-model change and a per-file flag change for less than
 * this.
 *
 * Casting away volatile for the memcpy is deliberate and is the standard
 * idiom for bulk DMA-buffer access. Ordering against the DMA write pointer
 * is the caller's job: dma_rx.c fences after reading WR_PTR and before
 * calling here.
 */
void convert_rx_to_float(const volatile uint32_t *ddr_buf, size_t n_samples,
                         float *out_real, float *out_imag)
{
    uint32_t stage[RX_STAGE_SAMPLES];

    for (size_t done = 0; done < n_samples; ) {
        size_t n = n_samples - done;
        if (n > RX_STAGE_SAMPLES)
            n = RX_STAGE_SAMPLES;

        memcpy(stage, (const uint32_t *)ddr_buf + done, n * sizeof(uint32_t));

        for (size_t i = 0; i < n; i++) {
            uint32_t word = stage[i];
            out_real[done + i] = (float)IQ_REAL(word) * SCALE_RX;
            out_imag[done + i] = (float)IQ_IMAG(word) * SCALE_RX;
        }
        done += n;
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
