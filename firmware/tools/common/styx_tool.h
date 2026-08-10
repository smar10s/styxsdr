// SPDX-License-Identifier: MIT
/*
 * styx_tool.h — Shared scaffolding for Styx firmware tools
 *
 * Provides:
 *   - Signal handling + TX cleanup on SIGINT/SIGTERM
 *   - AD9361 radio configuration (struct-based, return-checked)
 *   - Two-phase TX-and-capture (loopback pattern)
 */
#ifndef STYX_TOOL_H
#define STYX_TOOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * Signal handling / TX safety
 *
 * Call styx_install_shutdown_handler() early in main(), before any TX start.
 * Poll styx_shutdown_requested() in loops.  Register cleanup callbacks via
 * styx_register_tx_cleanup() — they run on signal AND via atexit().
 * -------------------------------------------------------------------------- */

void styx_install_shutdown_handler(void);
bool styx_shutdown_requested(void);

/* Register a cleanup function called on signal or atexit.
 * Multiple registrations are supported (max 4). */
void styx_register_tx_cleanup(void (*fn)(void));

/* --------------------------------------------------------------------------
 * AD9361 configuration
 *
 * All fields are applied; set unused fields to 0 to skip them.
 * Returns 0 on success, -1 if any sysfs write fails.
 * -------------------------------------------------------------------------- */

typedef struct {
    uint64_t rx_lo_hz;        /* 0 = skip */
    uint64_t tx_lo_hz;        /* 0 = skip */
    uint32_t sample_rate_hz;  /* 0 = skip */
    uint32_t rx_bw_hz;        /* 0 = skip */
    uint32_t tx_bw_hz;        /* 0 = skip */
    double   tx_atten_db;     /* applied if tx_lo_hz != 0 */
    const char *rx_gain_mode; /* NULL = skip ("manual" or "slow_attack") */
    double   rx_gain_db;      /* applied if rx_gain_mode = "manual" */
    uint32_t settle_us;       /* post-config settle time (0 = default 100ms) */
} styx_rf_config_t;

int styx_rf_configure(const styx_rf_config_t *cfg);

/* --------------------------------------------------------------------------
 * Two-phase TX-and-capture (loopback pattern)
 *
 * 1. Start RX DMA
 * 2. dma_tx_load() — write waveform to DDR (slow)
 * 3. Record t0 = dma_rx_wr_ptr()
 * 4. dma_tx_trigger() — single register write (fast)
 * 5. Wait for TX complete + margin
 * 6. Stop both DMAs
 * 7. Read captured IQ from DDR ring buffer
 * 8. Remove DC offset
 *
 * Caller must free *out_re / *out_im on success.
 * Returns number of captured samples, or -1 on error.
 * -------------------------------------------------------------------------- */

typedef struct {
    const float *tx_re;
    const float *tx_im;
    size_t       tx_samples;
    size_t       capture_samples;   /* how many RX samples to read */
    uint32_t     extra_wait_us;     /* additional wait after TX time (default 2000) */
    bool         remove_dc;         /* true = subtract mean from output (default: true) */
} styx_capture_cfg_t;

int styx_tx_and_capture(const styx_capture_cfg_t *cfg,
                        float **out_re, float **out_im);

#endif /* STYX_TOOL_H */
