# Tools & RTL Hardening Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Fix confirmed bugs in the validation/test tooling, harden RTL safety, and refactor duplicated scaffolding across 8 firmware tools into a shared library.

**Architecture:** Extract common patterns (ring-buffer reads, AD9361 config, TX/RX loopback capture, signal handling) into `firmware/tools/common/` as a shared static library. Fix individual tool bugs in-place. RTL changes are minimal (comment correction in hal.c; no Verilog changes needed since the DMA gating is already correct).

**Tech Stack:** C (ARM cross-compile), Verilog (no changes), Bash scripts, CMake

---

## Phase 1: Safety & Correctness (no refactor, just fixes)

### Task 1: Fix hal.c accel comment to reflect actual behavior

The comment at `firmware/src/hal.c:146-149` says reads return "0xDEADBEEF or bus error" but on Zynq with no AXI slave, it's always DECERR → SIGBUS. The comment should make SIGBUS the primary expectation and note that PROJECT_ID must be probed first.

**Files:**
- Modify: `firmware/src/hal.c:146-149`

**Step 1: Fix the comment**

```c
    /* Accelerator register regions — may or may not be populated depending
     * on bitstream.  mmap succeeds regardless (it's just address space);
     * accessing an unpopulated region causes AXI DECERR → SIGBUS.
     * The accelerator library MUST probe PROJECT_ID (0x43C00004) before
     * touching these regions.  Only child bitstreams (deimos, lora, etc.)
     * instantiate AXI slaves here. */
```

**Step 2: Commit**

```bash
git add firmware/src/hal.c
git commit -m "fix: hal.c accel mmap comment — SIGBUS, not 0xDEADBEEF"
```

---

### Task 2: Fix adc_capture exit code

`firmware/tools/adc_capture.c` computes clip/RMS/low-signal diagnostics but always returns 0.

**Files:**
- Modify: `firmware/tools/adc_capture.c:~392-465`

**Step 1: Propagate diagnostic results as exit code**

After the diagnostic section (~line 420), set a result variable:
```c
int diag_result = 0;
if (clip_pct > 1.0f) {
    fprintf(stderr, "WARNING: Clipping detected (%.1f%%)\n", clip_pct);
    diag_result = 2;  /* exit 2 = clipping */
} else if (rms < low_signal_threshold) {
    fprintf(stderr, "WARNING: Very low signal (RMS=%.6f)\n", rms);
    diag_result = 3;  /* exit 3 = no signal */
}
```

Change `return 0;` at line 465 to `return diag_result;`.

**Step 2: Commit**

```bash
git add firmware/tools/adc_capture.c
git commit -m "fix: adc_capture propagates clip/low-signal as exit code"
```

---

### Task 3: Fix hil_inject -v comparison length

`firmware/tools/hil_inject.c` compares 1024 snap words but only 512 are filled (POST_DEPTH=512).

**Files:**
- Modify: `firmware/tools/hil_inject.c:~274-318`

**Step 1: Limit comparison to POST_DEPTH**

```c
#define SNAP_POST_DEPTH 512
...
int compare_len = n_expected < SNAP_POST_DEPTH ? n_expected : SNAP_POST_DEPTH;
```

**Step 2: Commit**

```bash
git add firmware/tools/hil_inject.c
git commit -m "fix: hil_inject -v compares only POST_DEPTH (512) valid snap entries"
```

---

### Task 4: Fix TX-left-keyed — add signal handlers to all TX tools

**Files:**
- Modify: `firmware/tools/platform/pluto_stream_test.c`
- Modify: `firmware/tools/platform/pluto_sigladder.c`
- Modify: `firmware/tools/platform/pluto_dma_test.c`
- Modify: `firmware/tools/platform/pluto_tx_beacon.c` (install handler before dma_tx_start)

**Step 1: Add a common shutdown pattern**

Each tool that uses TX DMA needs:
```c
#include <signal.h>

static volatile sig_atomic_t g_shutdown = 0;

static void shutdown_handler(int sig) {
    (void)sig;
    g_shutdown = 1;
}

/* Call early in main(), before any TX start */
static void install_tx_shutdown(void) {
    struct sigaction sa = { .sa_handler = shutdown_handler };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
}
```

At tool exit (normal or via g_shutdown):
```c
dma_tx_stop();  /* or dma_tx_stream_stop() */
hal_cleanup();
```

**Step 2: Fix pluto_tx_beacon ordering**

Move `signal()` call before `dma_tx_start()`.

**Step 3: Add atexit handler as belt-and-suspenders**

```c
static void atexit_tx_stop(void) { dma_tx_stop(); }
...
atexit(atexit_tx_stop);
```

**Step 4: Commit**

```bash
git add firmware/tools/platform/pluto_stream_test.c \
        firmware/tools/platform/pluto_sigladder.c \
        firmware/tools/platform/pluto_dma_test.c \
        firmware/tools/platform/pluto_tx_beacon.c
git commit -m "fix: install signal handlers before TX start in all tools"
```

---

### Task 5: Fix pluto_dma_test — require explicit AD9361 config

**Files:**
- Modify: `firmware/tools/platform/pluto_dma_test.c`

**Step 1: Add minimal AD9361 setup before TX**

```c
hal_ad9361_set_tx_lo(2400000000ULL);    /* 2.4 GHz — ISM band */
hal_ad9361_set_tx_attenuation(89.75);    /* maximum attenuation */
hal_ad9361_set_tx_bw(2000000);
```

The key is maximum attenuation so a test that's just verifying DMA mechanics doesn't accidentally radiate.

**Step 2: Commit**

```bash
git add firmware/tools/platform/pluto_dma_test.c
git commit -m "fix: pluto_dma_test configures AD9361 with max attenuation before TX"
```

---

### Task 6: Fix pluto_stream_test dropout threshold

**Files:**
- Modify: `firmware/tools/platform/pluto_stream_test.c:~496`

**Step 1: Replace self-referential threshold with absolute reference**

The TX signal is a known chirp (unit modulus). Expected RX envelope power per block (after loopback + gain) should be computed from the TX amplitude, not the received mean.

```c
/* TX is unit-modulus chirp → expected RX block power ≈ 1.0 (pre-gain).
 * Use 10% of expected as dropout floor, not 10% of observed mean
 * (which passes with cable unplugged). */
double dropout_threshold = 0.1;  /* absolute: 10% of unit-power TX */
```

If gain varies, a relative threshold can still work but it should require a *minimum* absolute floor:
```c
double dropout_threshold = fmax(env.mean * 0.1, 0.01);
/* Must also fail if env.mean itself is below noise floor */
if (env.mean < 0.05) {
    fprintf(stderr, "FAIL: No signal detected (mean envelope %.4f)\n", env.mean);
    return -1;
}
```

**Step 2: Commit**

```bash
git add firmware/tools/platform/pluto_stream_test.c
git commit -m "fix: stream_test dropout threshold uses absolute floor, fails on no signal"
```

---

### Task 7: Fix pluto_stream_test float32 phase cast

**Files:**
- Modify: `firmware/tools/platform/pluto_stream_test.c:~174`

**Step 1: Keep phase as double through range reduction**

Replace:
```c
phases[j] = (float)cs->phase;
```

With range reduction *before* the float cast:
```c
double reduced = fmod(cs->phase, 2.0 * M_PI);
phases[j] = (float)reduced;
```

Or better, do the entire sincos in double and only cast the I/Q result to float.

**Step 2: Commit**

```bash
git add firmware/tools/platform/pluto_stream_test.c
git commit -m "fix: stream_test range-reduces phase before float cast (prevents precision loss)"
```

---

### Task 8: Fix pluto_sigladder L3 — real DMA integrity check

**Files:**
- Modify: `firmware/tools/platform/pluto_sigladder.c:~292-600`

**Step 1: Replace envelope-only correlation with frequency-aware check**

L3 should verify that the received signal matches the transmitted chirp in *frequency content*, not just power flatness. Options (pick one):

**Option A: Cross-correlate the complex IQ directly (detects drops, reordering, timing)**
```c
/* Compute complex cross-correlation between TX and RX IQ.
 * Peak magnitude indicates match quality; peak position indicates timing offset. */
double dot_re = 0, dot_im = 0;
for (size_t i = 0; i < n; i++) {
    dot_re += tx_re[i] * rx_re[i] + tx_im[i] * rx_im[i];
    dot_im += tx_im[i] * rx_re[i] - tx_re[i] * rx_im[i];
}
double corr_mag = sqrt(dot_re*dot_re + dot_im*dot_im) / n;
```

**Option B: Block-by-block instantaneous frequency comparison**
Compute the derivative of the phase (instantaneous frequency) for both TX and RX and correlate those — this catches drops, reversed chirps, and CW substitution.

**Step 2: Lower threshold appropriately and document what's being tested**

**Step 3: Commit**

```bash
git add firmware/tools/platform/pluto_sigladder.c
git commit -m "fix: sigladder L3 uses complex cross-correlation (detects drops, reversal, CW)"
```

---

### Task 9: Fix pluto_sigladder L1/L2 bin tolerance and swap detection

**Files:**
- Modify: `firmware/tools/platform/pluto_sigladder.c:~396-490`

**Step 1: Reduce bin tolerance to match stated intent**

```c
/* L1: ±2 bins (as documented) */
if (total_energy > 100.0 && bin_error <= 2) {

/* L2: ±1 bin (as documented) */
if (bin_error <= 1 && ratio_db > 20.0) {
```

**Step 2: Fix L2 I/Q swap detection — remove min(), check only positive bin**

```c
/* L2: peak must be at the POSITIVE frequency bin only.
 * If I/Q are swapped, the peak appears at the negative bin (mirrored). */
int bin_error = abs(peak_bin - expected_bin_pos);
if (bin_error > (int)(fft_len / 2)) bin_error = (int)fft_len - bin_error;
```

Remove the `min(err_pos, err_neg)` — only accept the positive frequency.

**Step 3: Commit**

```bash
git add firmware/tools/platform/pluto_sigladder.c
git commit -m "fix: sigladder L1/L2 bin tolerance matches docs, L2 detects I/Q swap"
```

---

### Task 10: Fix burst_loopback.c:379 — 3000 to dB API

**Files:**
- Modify: `firmware/tools/platform/pluto_burst_loopback.c:379`

**Step 1: Fix the value**

```c
hal_ad9361_set_tx_attenuation(3.0);  /* -3 dB (match pluto_loopback) */
```

**Step 2: Commit**

```bash
git add firmware/tools/platform/pluto_burst_loopback.c
git commit -m "fix: burst_loopback TX attenuation 3.0 dB (was 3000, wrong unit)"
```

---

### Task 11: Fix hardware_baseline.sh — record streaming TX results

**Files:**
- Modify: `scripts/hardware_baseline.sh:~226`

**Step 1: Add stream_pass to LOG_ENTRY**

```bash
LOG_ENTRY="{\"timestamp\":\"${TIMESTAMP}\",...,\"stream_pass\":${STREAM_PASS},\"dma_pass\":${DMA_PASSED},..."
```

**Step 2: Commit**

```bash
git add scripts/hardware_baseline.sh
git commit -m "fix: hardware_baseline.sh records streaming TX result in JSON log"
```

---

## Phase 2: Refactoring — Extract shared scaffolding

### Task 12: Create `firmware/tools/common/` shared library

**Files:**
- Create: `firmware/tools/common/styx_tool.h` — unified header
- Create: `firmware/tools/common/styx_tool.c` — implementation
- Modify: `firmware/tools/CMakeLists.txt` — add common lib

**Shared library provides:**

```c
/* styx_tool.h */
#ifndef STYX_TOOL_H
#define STYX_TOOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* --- Signal handling / TX safety --- */
void styx_install_shutdown_handler(void);
bool styx_shutdown_requested(void);
void styx_register_tx_cleanup(void (*fn)(void));

/* --- AD9361 config (returns -1 on any failure) --- */
typedef struct {
    uint64_t rx_lo_hz;
    uint64_t tx_lo_hz;
    uint32_t sample_rate_hz;
    uint32_t rx_bw_hz;
    uint32_t tx_bw_hz;
    double   tx_atten_db;
    const char *rx_gain_mode;  /* "manual" or "slow_attack" */
    double   rx_gain_db;       /* only if gain_mode = "manual" */
} styx_rf_config_t;

int styx_ad9361_configure(const styx_rf_config_t *cfg);

/* --- Ring buffer reads (with lap detection) --- */
typedef struct {
    uint32_t read_ptr;
    uint32_t last_wr_ptr;
    uint64_t wr_total;      /* monotonic, detects laps */
    uint32_t buf_samples;
} styx_ring_t;

void styx_ring_init(styx_ring_t *r, uint32_t buf_samples);
int  styx_ring_available(styx_ring_t *r, uint32_t wr_ptr);
int  styx_ring_read(styx_ring_t *r, uint32_t wr_ptr,
                    size_t n_samples, float *out_re, float *out_im);

/* --- TX-and-capture (loopback pattern) --- */
typedef struct {
    const float *tx_re;
    const float *tx_im;
    size_t tx_samples;
    bool cyclic;
    uint32_t capture_delay_us;  /* extra delay after TX before RX read */
} styx_loopback_cfg_t;

int styx_tx_and_capture(const styx_loopback_cfg_t *cfg,
                        float **out_re, float **out_im, size_t *out_n);

#endif
```

**Step 1: Implement the shared library**

**Step 2: Update CMakeLists.txt to build it as a static lib linked by all tools**

**Step 3: Commit**

```bash
git add firmware/tools/common/
git add firmware/tools/CMakeLists.txt
git commit -m "feat: add firmware/tools/common/ shared scaffolding library"
```

---

### Task 13: Migrate pluto_loopback to use shared library

**Files:**
- Modify: `firmware/tools/platform/pluto_loopback.c`

Replace:
- AD9361 config block (8 calls) → `styx_ad9361_configure(&cfg)`
- `tx_and_capture()` → `styx_tx_and_capture(&cfg, ...)`
- Ring-wrap logic → `styx_ring_read()`
- Signal handling → `styx_install_shutdown_handler()`

**Step 1: Rewrite to use shared lib**

**Step 2: Verify it still compiles**

Run: `cmake --build build/firmware --target pluto_loopback`

**Step 3: Commit**

```bash
git add firmware/tools/platform/pluto_loopback.c
git commit -m "refactor: pluto_loopback uses shared styx_tool library"
```

---

### Tasks 14-19: Migrate remaining tools (one per task)

Same pattern as Task 13 for:
- Task 14: `pluto_burst_loopback.c`
- Task 15: `pluto_sigladder.c`
- Task 16: `pluto_stream_test.c`
- Task 17: `pluto_dma_test.c`
- Task 18: `pluto_tx_beacon.c`
- Task 19: `adc_capture.c` + `hil_inject.c`

Each task: migrate shared patterns, verify compilation, commit.

---

### Task 20: Final — verify all AD9361 return values are checked

After migration, the single `styx_ad9361_configure()` checks all return values and fails loudly. Verify no unchecked `hal_ad9361_*` calls remain across all tools.

Run: `grep -rn 'hal_ad9361_' firmware/tools/ | grep -v '//' | grep -v styx_tool`

Expected: zero hits outside `common/styx_tool.c`.

**Commit:**
```bash
git commit --allow-empty -m "chore: verify all AD9361 calls go through styx_ad9361_configure()"
```

---

## Phase 3: Build & Validate

### Task 21: Build bitstream (picks up iq_dma_tx.v fix)

```bash
make bitstream
```

### Task 22: Build firmware

```bash
make firmware
```

### Task 23: Package + flash + validate

```bash
make package && make flash && make validate
```

### Task 24: Run hardware_baseline.sh and verify all fields recorded

```bash
scripts/hardware_baseline.sh
cat logs/hardware.jsonl | tail -1 | python3 -m json.tool
```

Verify `stream_pass` field is present.
