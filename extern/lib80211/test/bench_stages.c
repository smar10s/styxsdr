/**
 * bench_stages.c -- Per-stage decode microbenchmark for lib80211.
 *
 * Isolates each decode pipeline stage (FFT, equalize, demap, deinterleave,
 * depuncture, viterbi, LDPC) and measures per-call latency with realistic
 * input data derived from a TX-generated frame.
 *
 * NOT a ctest — run manually for performance measurement.
 */

#include "lib80211/perf.h"
#include "lib80211/fft.h"
#include "lib80211/channel.h"
#include "lib80211/modulation.h"
#include "lib80211/interleaver.h"
#include "lib80211/fec.h"
#include "lib80211/ldpc.h"
#include "lib80211/constants.h"
#include "lib80211/tx.h"
#include "lib80211/rx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* ========================================================================
 * Test data generation
 * ======================================================================== */

#define PSDU_LEN 100

static uint32_t bench_crc32(const uint8_t *data, size_t len)
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

static void make_psdu(uint8_t *psdu)
{
    for (int i = 0; i < PSDU_LEN - 4; i++)
        psdu[i] = (uint8_t)((i * 7 + 42) & 0xFF);
    uint32_t fcs = bench_crc32(psdu, (size_t)(PSDU_LEN - 4));
    psdu[PSDU_LEN - 4] = (uint8_t)(fcs & 0xFF);
    psdu[PSDU_LEN - 3] = (uint8_t)((fcs >> 8) & 0xFF);
    psdu[PSDU_LEN - 2] = (uint8_t)((fcs >> 16) & 0xFF);
    psdu[PSDU_LEN - 1] = (uint8_t)((fcs >> 24) & 0xFF);
}

/* ========================================================================
 * Benchmark harness
 * ======================================================================== */

typedef void (*bench_fn)(void *ctx);

typedef struct {
    const char *name;
    bench_fn fn;
    void *ctx;
} bench_stage;

/**
 * Auto-calibrate iterations and measure a single stage.
 * Runs until total elapsed >= 1 second.
 */
static void run_bench(const bench_stage *stage)
{
    /* Warm up */
    for (int i = 0; i < 10; i++)
        stage->fn(stage->ctx);

    /* Calibrate iteration count */
    int n_iters = 100;
    uint64_t elapsed_ns;
    while (1) {
        uint64_t t0 = perf_now_ns();
        for (int i = 0; i < n_iters; i++)
            stage->fn(stage->ctx);
        elapsed_ns = perf_now_ns() - t0;
        if (elapsed_ns >= 1000000000ULL) break;
        n_iters *= 2;
        if (n_iters > 50000000) break; /* safety cap */
    }

    double total_s = (double)elapsed_ns / 1e9;
    double per_call_ns = (double)elapsed_ns / (double)n_iters;
    double per_call_us = per_call_ns / 1000.0;

    printf("  %-20s  %10d iters  %6.3f s  %8.1f ns/call  %6.3f us/call\n",
           stage->name, n_iters, total_s, per_call_ns, per_call_us);
}

/* ========================================================================
 * Stage contexts and callbacks
 * ======================================================================== */

/* --- FFT --- */
typedef struct {
    lib80211_fft_plan *plan;
    float in_re[64], in_im[64];
    float out_re[64], out_im[64];
} fft_ctx;

static void bench_fft(void *ctx)
{
    fft_ctx *c = (fft_ctx *)ctx;
    lib80211_fft_forward(c->plan, c->in_re, c->in_im, c->out_re, c->out_im);
}

/* --- Equalize --- */
typedef struct {
    float Y_re[64], Y_im[64];
    float H_re[64], H_im[64];
    float noise_var;
    float out_re[64], out_im[64];
} eq_ctx;

static void bench_equalize(void *ctx)
{
    eq_ctx *c = (eq_ctx *)ctx;
    lib80211_equalize(c->Y_re, c->Y_im, c->H_re, c->H_im,
                      c->noise_var, c->out_re, c->out_im);
}

/* --- Soft demap (48 symbols, 64QAM = 6 bpsc) --- */
typedef struct {
    float rx_re[48], rx_im[48];
    float soft_bits[48 * 6];
    int n_bpsc;
} demap_ctx;

static void bench_demap(void *ctx)
{
    demap_ctx *c = (demap_ctx *)ctx;
    lib80211_soft_demap(c->rx_re, c->rx_im, c->soft_bits, 48, c->n_bpsc);
}

/* --- Deinterleave (64QAM: n_cbps=288, n_bpsc=6) --- */
typedef struct {
    float in_soft[288];
    float out_soft[288];
    int n_cbps;
    int n_bpsc;
} deintlv_ctx;

static void bench_deinterleave(void *ctx)
{
    deintlv_ctx *c = (deintlv_ctx *)ctx;
    lib80211_deinterleave(c->in_soft, c->out_soft, c->n_cbps, c->n_bpsc);
}

/* --- Depuncture (rate 3/4, one OFDM symbol worth: 288 punctured -> ~384 depunctured) --- */
typedef struct {
    float *in_soft;
    float *out_soft;
    size_t n_input;
    int cr_n;
    int cr_d;
} depunct_ctx;

static void bench_depuncture(void *ctx)
{
    depunct_ctx *c = (depunct_ctx *)ctx;
    lib80211_depuncture(c->in_soft, c->out_soft, c->n_input, c->cr_n, c->cr_d);
}

/* --- Viterbi (100-byte frame: 800 data bits + 6 tail = 1612 coded bits) --- */
typedef struct {
    float *soft_bits;
    uint8_t *out_bits;
    size_t n_coded_bits;
    size_t n_data_bits;
} viterbi_ctx;

static void bench_viterbi(void *ctx)
{
    viterbi_ctx *c = (viterbi_ctx *)ctx;
    lib80211_viterbi_decode(c->soft_bits, c->out_bits,
                            c->n_coded_bits, c->n_data_bits);
}

/* --- LDPC decode (648-length codeword, rate 1/2) --- */
typedef struct {
    float *llr_in;
    uint8_t *decoded_bits;
    int cw_len;
    int rate_n;
    int rate_d;
    int max_iters;
} ldpc_ctx;

static void bench_ldpc(void *ctx)
{
    ldpc_ctx *c = (ldpc_ctx *)ctx;
    lib80211_ldpc_decode(c->llr_in, c->decoded_bits,
                         c->cw_len, c->rate_n, c->rate_d, c->max_iters);
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(void)
{
    printf("bench_stages: per-stage decode microbenchmark (lib80211)\n");
    printf("==========================================================\n\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "Failed to create FFT plan\n");
        return 1;
    }

    /* ------------------------------------------------------------------
     * Generate a realistic legacy 54 Mbps frame for input data
     * ------------------------------------------------------------------ */
    uint8_t psdu[PSDU_LEN];
    make_psdu(psdu);

    lib80211_tx_legacy_params tx_params = {
        .rate_mbps = 54,
        .psdu = psdu,
        .psdu_len = PSDU_LEN,
        .scrambler_seed = 0x5D,
    };

    size_t max_samples = lib80211_tx_legacy_samples(&tx_params) + 200;
    float *tx_re = (float *)calloc(max_samples, sizeof(float));
    float *tx_im = (float *)calloc(max_samples, sizeof(float));
    if (!tx_re || !tx_im) {
        fprintf(stderr, "Allocation failed\n");
        return 1;
    }

    size_t n_tx = lib80211_tx_legacy(plan, &tx_params, tx_re + 100, tx_im + 100);
    size_t n_samples = 100 + n_tx + 100;
    if (n_samples > max_samples) n_samples = max_samples;

    /* Validate the frame decodes correctly */
    lib80211_rx_result rx_result;
    int rc = lib80211_rx_decode(plan, tx_re, tx_im, n_samples, &rx_result);
    if (rc != 0) {
        fprintf(stderr, "WARNING: full decode failed (rc=%d), inputs may be suboptimal\n", rc);
    } else {
        printf("Validation: decode OK (rate=%d Mbps, %zu bytes)\n\n",
               rx_result.rate_mbps, rx_result.psdu_len);
    }

    /* ------------------------------------------------------------------
     * Derive realistic intermediate buffers from the TX frame
     * ------------------------------------------------------------------ */

    /* Channel estimation from L-LTF (starts at sample 160+32 = 192 in TX output) */
    float H_re[64], H_im[64];
    float noise_var = 0.0f;
    /* L-LTF: preamble has STF (160 samples) + LTF GI2 (32 samples) + 2x64 symbols
     * In our TX output (offset by 100), LTF symbols start at 100 + 160 + 32 = 292 */
    const float *ltf_start_re = tx_re + 100 + 160 + 32;
    const float *ltf_start_im = tx_im + 100 + 160 + 32;
    lib80211_estimate_channel(plan, ltf_start_re, ltf_start_im,
                              H_re, H_im, &noise_var);
    if (noise_var < 1e-10f) noise_var = 0.001f;  /* avoid division by zero */

    /* First data symbol starts after preamble (320) + SIGNAL (80) = sample 400 from TX start */
    const float *data_sym_re = tx_re + 100 + 320 + 80;
    const float *data_sym_im = tx_im + 100 + 320 + 80;

    /* Extract one symbol to get freq-domain & equalized data */
    float eq_re[48], eq_im[48];
    lib80211_pilot_state pilot_state = {.slope = 0, .alpha = 0.3f, .initialized = 0};
    lib80211_extract_legacy_symbol(plan, data_sym_re, data_sym_im,
                                   H_re, H_im, noise_var, 0, &pilot_state,
                                   eq_re, eq_im);

    /* FFT the data symbol to get freq-domain input for equalize bench */
    float fft_out_re[64], fft_out_im[64];
    lib80211_fft_forward(plan, data_sym_re + 16, data_sym_im + 16,
                         fft_out_re, fft_out_im);

    /* Soft demap the equalized data (54 Mbps = 64QAM, 6 bpsc) */
    const int n_bpsc = 6;
    const int n_cbps = 48 * n_bpsc;  /* 288 */
    float soft_bits_interleaved[288];
    lib80211_soft_demap(eq_re, eq_im, soft_bits_interleaved, 48, n_bpsc);

    /* Deinterleave */
    float soft_bits_deintlv[288];
    lib80211_deinterleave(soft_bits_interleaved, soft_bits_deintlv, n_cbps, n_bpsc);

    /* Depuncture (54 Mbps = rate 3/4) */
    /* rate 3/4: 288 input -> 288 * 2 / (3/4*2) = 384 output (rate-1/2 equivalent) */
    float soft_depunct[512];  /* oversized for safety */
    size_t n_depunct = lib80211_depuncture(soft_bits_deintlv, soft_depunct, 288, 3, 4);

    /* For Viterbi: need a full frame's worth of rate-1/2 soft bits.
     * 100-byte PSDU = 800 bits + 6 tail = 806 data bits -> 1612 coded bits at rate 1/2
     * At rate 3/4 with 54 Mbps: n_dbps = 216, n_sym = ceil((16+800+6)/216) = 4 symbols
     * Total coded bits: 4 * 288 = 1152, after depuncture: 4 * 384 = 1536 (close to 1612)
     * We'll use a realistic sized buffer. */
    const size_t viterbi_data_bits = 800;
    const size_t viterbi_coded_bits = 2 * (viterbi_data_bits + 6);  /* 1612 */
    float *viterbi_soft = (float *)calloc(viterbi_coded_bits, sizeof(float));
    uint8_t *viterbi_out = (uint8_t *)calloc(viterbi_data_bits, sizeof(uint8_t));
    if (!viterbi_soft || !viterbi_out) {
        fprintf(stderr, "Allocation failed\n");
        return 1;
    }
    /* Fill with realistic-ish soft bits (copy depunctured data, repeat) */
    for (size_t i = 0; i < viterbi_coded_bits; i++)
        viterbi_soft[i] = soft_depunct[i % n_depunct];

    /* For LDPC: generate realistic LLRs for a 648-length codeword at rate 1/2
     * with AWGN noise at ~2.5 dB Eb/N0 (forces ~10-15 iterations to converge).
     * K = 648/2 = 324 information bits. */
    const int ldpc_cw_len = 648;
    const int ldpc_rate_n = 1;
    const int ldpc_rate_d = 2;
    const int ldpc_k = ldpc_cw_len * ldpc_rate_n / ldpc_rate_d;  /* 324 */
    uint8_t *ldpc_info = (uint8_t *)calloc((size_t)ldpc_cw_len, sizeof(uint8_t));
    uint8_t *ldpc_codeword = (uint8_t *)calloc((size_t)ldpc_cw_len, sizeof(uint8_t));
    float *ldpc_llr = (float *)calloc((size_t)ldpc_cw_len, sizeof(float));
    uint8_t *ldpc_decoded = (uint8_t *)calloc((size_t)ldpc_cw_len, sizeof(uint8_t));
    if (!ldpc_info || !ldpc_codeword || !ldpc_llr || !ldpc_decoded) {
        fprintf(stderr, "Allocation failed\n");
        return 1;
    }
    /* Generate random-ish info bits */
    for (int i = 0; i < ldpc_k; i++)
        ldpc_info[i] = (uint8_t)((i * 13 + 7) & 1);
    lib80211_ldpc_encode(ldpc_info, ldpc_codeword, ldpc_cw_len, ldpc_rate_n, ldpc_rate_d);
    /* Create noisy LLRs at ~1.5 dB Eb/N0 (sigma ≈ 0.95).
     * BPSK: transmitted ±1, received + noise. LLR = 2*rx/sigma^2.
     * Use a simple xorshift32 PRNG + Box-Muller for Gaussian noise. */
    {
        uint32_t rng_state = 0xDEADBEEF;
        const float sigma = 0.95f;  /* ~1.5 dB Eb/N0 for rate 1/2 BPSK */
        const float llr_scale = 2.0f / (sigma * sigma);
        for (int i = 0; i < ldpc_cw_len; i += 2) {
            /* xorshift32 */
            rng_state ^= rng_state << 13;
            rng_state ^= rng_state >> 17;
            rng_state ^= rng_state << 5;
            float u1 = (float)(rng_state & 0xFFFF) / 65536.0f + 1e-6f;
            rng_state ^= rng_state << 13;
            rng_state ^= rng_state >> 17;
            rng_state ^= rng_state << 5;
            float u2 = (float)(rng_state & 0xFFFF) / 65536.0f;
            /* Box-Muller */
            float r = sigma * sqrtf(-2.0f * logf(u1));
            float n1 = r * cosf(6.2831853f * u2);
            float n2 = r * sinf(6.2831853f * u2);
            /* BPSK: bit 0 -> +1, bit 1 -> -1 */
            float tx0 = ldpc_codeword[i] ? -1.0f : 1.0f;
            ldpc_llr[i] = (tx0 + n1) * llr_scale;
            if (i + 1 < ldpc_cw_len) {
                float tx1 = ldpc_codeword[i+1] ? -1.0f : 1.0f;
                ldpc_llr[i+1] = (tx1 + n2) * llr_scale;
            }
        }
    }
    /* Verify LDPC converges with these noisy inputs */
    {
        int iters = lib80211_ldpc_decode(ldpc_llr, ldpc_decoded,
                                          ldpc_cw_len, ldpc_rate_n, ldpc_rate_d, 30);
        printf("LDPC bench: converged in %d iterations (noisy, ~1.5 dB Eb/N0)\n\n", iters);
    }

    /* ------------------------------------------------------------------
     * Set up stage contexts
     * ------------------------------------------------------------------ */

    /* FFT context */
    fft_ctx fctx;
    fctx.plan = plan;
    memcpy(fctx.in_re, data_sym_re + 16, 64 * sizeof(float));  /* skip CP */
    memcpy(fctx.in_im, data_sym_im + 16, 64 * sizeof(float));

    /* Equalize context */
    eq_ctx ectx;
    memcpy(ectx.Y_re, fft_out_re, 64 * sizeof(float));
    memcpy(ectx.Y_im, fft_out_im, 64 * sizeof(float));
    memcpy(ectx.H_re, H_re, 64 * sizeof(float));
    memcpy(ectx.H_im, H_im, 64 * sizeof(float));
    ectx.noise_var = noise_var;

    /* Demap context */
    demap_ctx dctx;
    memcpy(dctx.rx_re, eq_re, 48 * sizeof(float));
    memcpy(dctx.rx_im, eq_im, 48 * sizeof(float));
    dctx.n_bpsc = n_bpsc;

    /* Deinterleave context */
    deintlv_ctx ictx;
    memcpy(ictx.in_soft, soft_bits_interleaved, 288 * sizeof(float));
    ictx.n_cbps = n_cbps;
    ictx.n_bpsc = n_bpsc;

    /* Depuncture context (rate 3/4, 288 input bits = one OFDM symbol) */
    depunct_ctx pctx;
    pctx.in_soft = soft_bits_deintlv;
    pctx.out_soft = (float *)calloc(512, sizeof(float));
    pctx.n_input = 288;
    pctx.cr_n = 3;
    pctx.cr_d = 4;

    /* Viterbi context */
    viterbi_ctx vctx;
    vctx.soft_bits = viterbi_soft;
    vctx.out_bits = viterbi_out;
    vctx.n_coded_bits = viterbi_coded_bits;
    vctx.n_data_bits = viterbi_data_bits;

    /* LDPC context */
    ldpc_ctx lctx;
    lctx.llr_in = ldpc_llr;
    lctx.decoded_bits = ldpc_decoded;
    lctx.cw_len = ldpc_cw_len;
    lctx.rate_n = ldpc_rate_n;
    lctx.rate_d = ldpc_rate_d;
    lctx.max_iters = 30;

    /* ------------------------------------------------------------------
     * Run benchmarks
     * ------------------------------------------------------------------ */
    printf("Stage                     Iterations   Time    Per-call (ns)  Per-call (us)\n");
    printf("--------------------------------------------------------------------------\n");

    bench_stage stages[] = {
        {"FFT (64-pt)",       bench_fft,          &fctx},
        {"Equalize",          bench_equalize,     &ectx},
        {"Soft demap (48sc)", bench_demap,        &dctx},
        {"Deinterleave",      bench_deinterleave, &ictx},
        {"Depuncture (3/4)",  bench_depuncture,   &pctx},
        {"Viterbi (800b)",    bench_viterbi,      &vctx},
        {"LDPC (648, R1/2)",  bench_ldpc,         &lctx},
    };
    int n_stages = (int)(sizeof(stages) / sizeof(stages[0]));

    for (int i = 0; i < n_stages; i++)
        run_bench(&stages[i]);

    printf("\nDone.\n");

    /* Cleanup */
    free(tx_re);
    free(tx_im);
    free(viterbi_soft);
    free(viterbi_out);
    free(ldpc_info);
    free(ldpc_codeword);
    free(ldpc_llr);
    free(ldpc_decoded);
    free(pctx.out_soft);
    lib80211_fft_plan_destroy(plan);

    return 0;
}
