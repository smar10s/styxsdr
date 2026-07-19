// SPDX-License-Identifier: MIT
/*
 * hil_inject — HIL golden vector injection + snap readback
 *
 * Writes a waveform to DDR, triggers hil_ctrl playback in test mode,
 * reads back the snap buffer, and optionally verifies against expected
 * input values.
 *
 * This proves two things:
 *   1. hil_ctrl mux switches from live ADC to DDR playback
 *   2. Snap probe captures the played-back IQ stream
 *
 * Input: waveform JSON file ({"real":[...],"imag":[...]})
 *
 * Output: 1024 lines of 32-bit hex (snap buffer contents) to stdout.
 *         With -v: PASS/FAIL comparing snap against quantized input.
 *
 * Register map:
 *   hil_ctrl @ 0x7C500000:
 *     0x00 CONTROL    [0]=test_mode, [1]=trigger(W1S)
 *     0x04 STATUS     [0]=playback_active, [1]=playback_done
 *     0x08 DDR_BASE   Physical base of test waveform
 *     0x0C PLAY_COUNT Number of samples
 *     0x10 PLAY_PTR   Current position (RO)
 *
 *   snap_axi @ 0x7C4E0000:
 *     0x00 CONTROL    [0]=arm, [1]=sw_trigger
 *     0x04 STATUS     [0]=captured, [1]=armed
 *     0x0C RD_ADDR    [9:0]
 *     0x10 RD_DATA    [31:0]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <getopt.h>

#include "hal.h"

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

#define SNAP_CTRL_ARM       (1 << 0)
#define SNAP_CTRL_SWTRIG    (1 << 1)
#define SNAP_DEPTH          1024

/* HIL DDR region (2 MB into TX region, avoids conflict with normal TX) */
#define HIL_DDR_BASE        0x18200000
#define HIL_MAX_SAMPLES     (512 * 1024)

/* watermark: first 3 samples encode ASCII codes in the real component
 * (imag=0). DDR word = {8'b0, im[11:0], re[11:0]}. */
#define WATERMARK_WORD_0    0x00000061
#define WATERMARK_WORD_1    0x00000057
#define WATERMARK_WORD_2    0x00000063

/* --------------------------------------------------------------------------
 * Minimal JSON waveform parser
 *
 * Parses {"real":[f,f,...],"imag":[f,f,...]} with no dependencies.
 * Allocates output arrays. Caller must free.
 * -------------------------------------------------------------------------- */

static int parse_float_array(const char *json, const char *key,
                             float **out, int *out_len)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);

    const char *p = strstr(json, search);
    if (!p) return -1;

    /* Find opening bracket */
    p = strchr(p, '[');
    if (!p) return -1;
    p++; /* skip '[' */

    /* Count elements (count commas + 1) */
    int count = 0;
    const char *scan = p;
    int depth = 1;
    while (*scan && depth > 0) {
        if (*scan == '[') depth++;
        else if (*scan == ']') depth--;
        else if (*scan == ',' && depth == 1) count++;
        scan++;
    }
    if (depth != 0) return -1;
    count++; /* one more element than commas */

    float *arr = malloc(count * sizeof(float));
    if (!arr) return -1;

    int idx = 0;
    const char *cur = p;
    while (idx < count) {
        char *end;
        arr[idx] = strtof(cur, &end);
        if (end == cur) break;
        idx++;
        cur = end;
        while (*cur == ',' || *cur == ' ' || *cur == '\n' || *cur == '\r' || *cur == '\t')
            cur++;
        if (*cur == ']') break;
    }

    *out = arr;
    *out_len = idx;
    return 0;
}

static int load_waveform(const char *path, float **re, float **im, int *n_samples)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "ERROR: Cannot open %s\n", path);
        return -1;
    }

    /* Read entire file */
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *json = malloc(fsize + 1);
    if (!json) { fclose(f); return -1; }

    size_t rd = fread(json, 1, fsize, f);
    fclose(f);
    json[rd] = '\0';

    int re_len = 0, im_len = 0;
    if (parse_float_array(json, "real", re, &re_len) != 0) {
        fprintf(stderr, "ERROR: Cannot parse 'real' array from %s\n", path);
        free(json);
        return -1;
    }
    if (parse_float_array(json, "imag", im, &im_len) != 0) {
        fprintf(stderr, "ERROR: Cannot parse 'imag' array from %s\n", path);
        free(json); free(*re);
        return -1;
    }

    free(json);

    if (re_len != im_len) {
        fprintf(stderr, "ERROR: real/imag length mismatch (%d vs %d)\n", re_len, im_len);
        free(*re); free(*im);
        return -1;
    }

    *n_samples = re_len;
    return 0;
}

/* --------------------------------------------------------------------------
 * Quantize and write to DDR
 *
 * Returns the quantized DDR words in `expected_out` (if non-NULL) for
 * verification against the snap buffer.
 * -------------------------------------------------------------------------- */

static void quantize_and_write(const float *re, const float *im, int n_samples,
                               volatile uint32_t *ddr_buf, uint32_t *expected_out,
                               bool quiet)
{
    /* Find peak for scaling to 12-bit range [-2047, +2047] */
    float peak = 0.0f;
    for (int i = 0; i < n_samples; i++) {
        float ar = fabsf(re[i]);
        float ai = fabsf(im[i]);
        if (ar > peak) peak = ar;
        if (ai > peak) peak = ai;
    }

    float scale = (peak > 0.0f) ? 2047.0f / peak : 1.0f;
    if (!quiet)
        fprintf(stderr, "  Peak: %.6f, scale: %.1f\n", peak, scale);

    for (int i = 0; i < n_samples; i++) {
        int16_t ri = (int16_t)roundf(re[i] * scale);
        int16_t qi = (int16_t)roundf(im[i] * scale);
        /* Clamp to 12-bit */
        if (ri > 2047) ri = 2047;
        if (ri < -2048) ri = -2048;
        if (qi > 2047) qi = 2047;
        if (qi < -2048) qi = -2048;
        /* Pack: {8'b0, im[11:0], re[11:0]} */
        uint32_t word = IQ_PACK(ri, qi);
        ddr_buf[i] = word;
        if (expected_out)
            expected_out[i] = word;
    }
}

/* --------------------------------------------------------------------------
 * HIL inject + snap capture
 * -------------------------------------------------------------------------- */

static int hil_inject_and_capture(int n_samples, uint32_t snap_buf[SNAP_DEPTH],
                                   bool quiet)
{
    /* 1. Enable test mode (mux selects DDR playback) */
    hal_reg_write(REG_HIL_CTRL_CONTROL, HIL_CTRL_TEST_MODE);
    usleep(100);

    /* 2. Configure playback: base address + sample count */
    hal_reg_write(REG_HIL_CTRL_DDR_BASE, HIL_DDR_BASE);
    hal_reg_write(REG_HIL_CTRL_PLAY_COUNT, (uint32_t)n_samples);

    /* 3. Arm snap probe (clear stale state, then arm) */
    hal_reg_write(REG_SNAP_CONTROL, 0);
    usleep(10);
    hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM);
    usleep(10);

    /* Verify armed */
    uint32_t snap_status = hal_reg_read(REG_SNAP_STATUS);
    if (!(snap_status & 0x02)) {
        if (!quiet)
            fprintf(stderr, "WARNING: Snap not armed (status=0x%08x)\n", snap_status);
    }

    /* 4. Trigger playback (W1S on bit 1, keep test_mode=1) */
    hal_reg_write(REG_HIL_CTRL_CONTROL, HIL_CTRL_TEST_MODE | HIL_CTRL_TRIGGER);

    /* 5. Immediately sw-trigger the snap.
     *    Playback emits 1 sample per 5 clocks (20 MSPS in 100 MHz fabric).
     *    Total playback = n_samples * 50ns. For 1024 samples = 51.2 us.
     *    POST_DEPTH = 512, so snap completes after 512 * 50ns = 25.6 us.
     *    We must trigger before sample 512. The AXI register write itself
     *    takes ~10 clocks, so triggering snap immediately after playback
     *    trigger ensures we catch the very first samples. */
    hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM | SNAP_CTRL_SWTRIG);

    /* 6. Wait for playback + snap to complete.
     *    Snap needs 512 sample_valid ticks = 25.6 us.
     *    Playback needs 1024 ticks = 51.2 us for full drain.
     *    Both will be done well within 1 ms. */
    usleep(1000);

    /* 7. Verify snap capture completed.
     *    With 1024 playback samples and POST_DEPTH=512, snap should be done
     *    well before playback ends. */
    usleep(100);
    snap_status = hal_reg_read(REG_SNAP_STATUS);
    if (!(snap_status & 0x01)) {
        fprintf(stderr, "ERROR: Snap capture failed (status=0x%08x)\n", snap_status);
        return -1;
    }

    /* 8. Read snap buffer (1024 entries) */
    for (int i = 0; i < SNAP_DEPTH; i++) {
        hal_reg_write(REG_SNAP_RD_ADDR, (uint32_t)i);
        usleep(1);  /* allow BRAM latency */
        snap_buf[i] = hal_reg_read(REG_SNAP_RD_DATA);
    }

    /* 9. Disable test mode */
    hal_reg_write(REG_HIL_CTRL_CONTROL, 0);

    return 0;
}

/* --------------------------------------------------------------------------
 * Verification: compare snap against expected input
 * -------------------------------------------------------------------------- */

static int verify_snap(const uint32_t snap_buf[SNAP_DEPTH],
                       const uint32_t *expected, int n_expected,
                       bool check_watermark, bool quiet)
{
    int errors = 0;

    /* Check watermark in first 3 samples */
    if (check_watermark) {
        bool wm_ok = (snap_buf[0] == WATERMARK_WORD_0 &&
                      snap_buf[1] == WATERMARK_WORD_1 &&
                      snap_buf[2] == WATERMARK_WORD_2);
        if (!wm_ok) {
            if (!quiet)
                fprintf(stderr, "  Watermark FAIL: got %08x %08x %08x, "
                        "expected %08x %08x %08x\n",
                        snap_buf[0], snap_buf[1], snap_buf[2],
                        WATERMARK_WORD_0, WATERMARK_WORD_1, WATERMARK_WORD_2);
            errors++;
        } else if (!quiet) {
            fprintf(stderr, "  Watermark OK\n");
        }
    }

    /* Compare data (up to min(n_expected, SNAP_DEPTH) samples) */
    int compare_len = n_expected < SNAP_DEPTH ? n_expected : SNAP_DEPTH;
    for (int i = 0; i < compare_len; i++) {
        if (snap_buf[i] != expected[i]) {
            if (errors < 10 && !quiet)
                fprintf(stderr, "  Mismatch @ %d: snap=0x%08x expected=0x%08x\n",
                        i, snap_buf[i], expected[i]);
            errors++;
        }
    }

    if (errors == 0) {
        if (!quiet)
            fprintf(stderr, "PASS: %d samples verified\n", compare_len);
        printf("PASS\n");
    } else {
        fprintf(stderr, "FAIL: %d errors in %d samples\n", errors, compare_len);
        printf("FAIL\n");
    }

    return errors > 0 ? 1 : 0;
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options] <waveform.json>\n"
        "\n"
        "  Inject IQ waveform via HIL controller and read snap buffer.\n"
        "  Proves: hil_ctrl mux works, snap probe captures correctly.\n"
        "\n"
        "Options:\n"
        "  -c <hz>    Apply CFO (frequency offset) before injection\n"
        "  -o <file>  Write snap hex to file instead of stdout\n"
        "  -v         Verify: compare snap against quantized input (PASS/FAIL)\n"
        "  -w         Check watermark in first 3 samples\n"
        "  -q         Quiet: suppress stderr diagnostics\n"
        "  -h         Show this help\n"
        "\n"
        "Output: 1024 lines of 32-bit hex (snap buffer contents)\n",
        prog);
}

int main(int argc, char *argv[])
{
    const char *outfile = NULL;
    bool quiet = false;
    bool verify = false;
    bool check_watermark = false;
    double cfo_hz = 0.0;
    int opt;

    while ((opt = getopt(argc, argv, "c:o:vwqh")) != -1) {
        switch (opt) {
        case 'c': cfo_hz = atof(optarg); break;
        case 'o': outfile = optarg; break;
        case 'v': verify = true; break;
        case 'w': check_watermark = true; break;
        case 'q': quiet = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "ERROR: No waveform specified\n");
        usage(argv[0]);
        return 1;
    }

    const char *waveform_path = argv[optind];

    if (!quiet)
        fprintf(stderr, "Loading waveform: %s\n", waveform_path);

    /* Load and parse waveform */
    float *re = NULL, *im = NULL;
    int n_samples = 0;

    if (load_waveform(waveform_path, &re, &im, &n_samples) != 0)
        return 1;

    if (!quiet)
        fprintf(stderr, "  Samples: %d (%.2f ms at 20 MSPS)\n",
                n_samples, n_samples / 20000.0);

    if (n_samples > HIL_MAX_SAMPLES) {
        fprintf(stderr, "ERROR: Waveform too large (%d > %d samples)\n",
                n_samples, HIL_MAX_SAMPLES);
        free(re); free(im);
        return 1;
    }

    /* Apply CFO if requested (rotate IQ by cfo_hz at 20 MSPS) */
    if (cfo_hz != 0.0) {
        double sample_rate = 20e6;
        double phase_inc = 2.0 * M_PI * cfo_hz / sample_rate;
        if (!quiet)
            fprintf(stderr, "  Applying CFO: %.1f Hz (%.6f rad/sample)\n",
                    cfo_hz, phase_inc);
        for (int i = 0; i < n_samples; i++) {
            double phase = phase_inc * (double)i;
            double c = cos(phase);
            double s = sin(phase);
            float r_new = (float)(re[i] * c - im[i] * s);
            float i_new = (float)(re[i] * s + im[i] * c);
            re[i] = r_new;
            im[i] = i_new;
        }
    }

    /* Initialize HAL */
    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        free(re); free(im);
        return 1;
    }

    /* Get DDR TX buffer pointer and offset to HIL region */
    volatile uint32_t *tx_buf = hal_ddr_tx_buf();
    volatile uint32_t *hil_buf = tx_buf + (HIL_DDR_BASE - DDR_TX_BASE) / 4;

    /* Quantize and write to DDR.
     * Keep a copy of quantized words for verification. */
    uint32_t *expected = NULL;
    if (verify || check_watermark)
        expected = malloc(n_samples * sizeof(uint32_t));

    if (!quiet)
        fprintf(stderr, "Quantizing to 12-bit and writing to DDR @ 0x%08X...\n",
                HIL_DDR_BASE);
    quantize_and_write(re, im, n_samples, hil_buf, expected, quiet);
    free(re); free(im);

    /* Inject and capture */
    if (!quiet)
        fprintf(stderr, "Triggering HIL playback (%d samples)...\n", n_samples);

    uint32_t snap_buf[SNAP_DEPTH];
    if (hil_inject_and_capture(n_samples, snap_buf, quiet) != 0) {
        free(expected);
        hal_cleanup();
        return 1;
    }

    if (!quiet)
        fprintf(stderr, "Snap capture complete.\n");

    /* Verification mode */
    if (verify || check_watermark) {
        int result = verify_snap(snap_buf, expected, n_samples,
                                 check_watermark, quiet);
        free(expected);
        hal_cleanup();
        return result;
    }

    /* Output snap data (hex) */
    FILE *out = stdout;
    if (outfile) {
        out = fopen(outfile, "w");
        if (!out) {
            fprintf(stderr, "ERROR: Cannot open output file %s\n", outfile);
            hal_cleanup();
            return 1;
        }
    }

    for (int i = 0; i < SNAP_DEPTH; i++) {
        fprintf(out, "%08x\n", snap_buf[i]);
    }

    if (outfile) fclose(out);

    hal_cleanup();

    if (!quiet)
        fprintf(stderr, "Done.\n");

    return 0;
}
