// SPDX-License-Identifier: MIT
/*
 * styx_tool.c — Shared scaffolding implementation
 */

#include "styx_tool.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include "hal.h"
#include "dma_tx.h"
#include "dma_rx.h"
#include "convert.h"

/* ==========================================================================
 * Signal handling
 * ========================================================================== */

#define MAX_CLEANUP_FNS 4

static volatile sig_atomic_t g_shutdown = 0;
static void (*g_cleanup_fns[MAX_CLEANUP_FNS])(void);
static int g_n_cleanup = 0;
static volatile sig_atomic_t g_disarmed = 0;

static void run_cleanups(void)
{
    for (int i = 0; i < g_n_cleanup; i++) {
        if (g_cleanup_fns[i]) {
            g_cleanup_fns[i]();
            g_cleanup_fns[i] = NULL;  /* run at most once */
        }
    }
}

static void shutdown_handler(int sig)
{
    (void)sig;
    g_shutdown = 1;
    run_cleanups();
    _exit(128 + sig);
}

static void atexit_handler(void)
{
    if (!g_disarmed)
        run_cleanups();
}

void styx_install_shutdown_handler(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = shutdown_handler;
    sigemptyset(&sa.sa_mask);

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    atexit(atexit_handler);
}

bool styx_shutdown_requested(void)
{
    return g_shutdown != 0;
}

void styx_register_tx_cleanup(void (*fn)(void))
{
    if (g_n_cleanup < MAX_CLEANUP_FNS && fn)
        g_cleanup_fns[g_n_cleanup++] = fn;
}

void styx_disarm_cleanup(void)
{
    g_disarmed = 1;
    for (int i = 0; i < g_n_cleanup; i++)
        g_cleanup_fns[i] = NULL;
}

/* ==========================================================================
 * AD9361 configuration
 * ========================================================================== */

int styx_rf_configure(const styx_rf_config_t *cfg)
{
    if (!cfg) return -1;

    if (cfg->sample_rate_hz) {
        if (hal_ad9361_set_sample_rate(cfg->sample_rate_hz) != 0) {
            fprintf(stderr, "styx_rf_configure: set_sample_rate(%u) failed\n",
                    cfg->sample_rate_hz);
            return -1;
        }
    }

    if (cfg->tx_lo_hz) {
        if (hal_ad9361_set_tx_lo(cfg->tx_lo_hz) != 0) {
            fprintf(stderr, "styx_rf_configure: set_tx_lo(%llu) failed\n",
                    (unsigned long long)cfg->tx_lo_hz);
            return -1;
        }
        if (hal_ad9361_set_tx_attenuation(cfg->tx_atten_db) != 0) {
            fprintf(stderr, "styx_rf_configure: set_tx_attenuation(%.2f) failed\n",
                    cfg->tx_atten_db);
            return -1;
        }
    }

    if (cfg->rx_lo_hz) {
        if (hal_ad9361_set_rx_lo(cfg->rx_lo_hz) != 0) {
            fprintf(stderr, "styx_rf_configure: set_rx_lo(%llu) failed\n",
                    (unsigned long long)cfg->rx_lo_hz);
            return -1;
        }
    }

    if (cfg->tx_bw_hz) {
        if (hal_ad9361_set_tx_bandwidth(cfg->tx_bw_hz) != 0) {
            fprintf(stderr, "styx_rf_configure: set_tx_bandwidth(%u) failed\n",
                    cfg->tx_bw_hz);
            return -1;
        }
    }

    if (cfg->rx_bw_hz) {
        if (hal_ad9361_set_rx_bandwidth(cfg->rx_bw_hz) != 0) {
            fprintf(stderr, "styx_rf_configure: set_rx_bandwidth(%u) failed\n",
                    cfg->rx_bw_hz);
            return -1;
        }
    }

    if (cfg->rx_gain_mode) {
        if (hal_ad9361_set_rx_gain_mode(cfg->rx_gain_mode) != 0) {
            fprintf(stderr, "styx_rf_configure: set_rx_gain_mode(\"%s\") failed\n",
                    cfg->rx_gain_mode);
            return -1;
        }
        if (strcmp(cfg->rx_gain_mode, "manual") == 0) {
            if (hal_ad9361_set_rx_gain(cfg->rx_gain_db) != 0) {
                fprintf(stderr, "styx_rf_configure: set_rx_gain(%.1f) failed\n",
                        cfg->rx_gain_db);
                return -1;
            }
        }
    }

    /* Post-config settle */
    uint32_t settle = cfg->settle_us ? cfg->settle_us : 100000;
    usleep(settle);

    return 0;
}

/* ==========================================================================
 * TX-and-capture (loopback pattern)
 * ========================================================================== */

int styx_tx_and_capture(const styx_capture_cfg_t *cfg,
                        float **out_re, float **out_im)
{
    if (!cfg || !out_re || !out_im) return -1;

    size_t cap_n = cfg->capture_samples;
    *out_re = malloc(cap_n * sizeof(float));
    *out_im = malloc(cap_n * sizeof(float));
    if (!*out_re || !*out_im) {
        free(*out_re); free(*out_im);
        *out_re = NULL; *out_im = NULL;
        return -1;
    }

    /* Start RX DMA — ring buffer fills continuously */
    if (dma_rx_start() != 0) {
        free(*out_re); free(*out_im);
        *out_re = NULL; *out_im = NULL;
        return -1;
    }

    /* Phase 1: Load waveform into DDR and configure DMA (slow) */
    if (dma_tx_load(cfg->tx_re, cfg->tx_im, cfg->tx_samples, false) != 0) {
        dma_rx_stop();
        free(*out_re); free(*out_im);
        *out_re = NULL; *out_im = NULL;
        return -1;
    }

    /* Record t0 — time reference in ring buffer */
    uint32_t t0 = dma_rx_wr_ptr();

    /* Phase 2: Trigger TX — single register write (fast, deterministic) */
    if (dma_tx_trigger() != 0) {
        dma_rx_stop();
        free(*out_re); free(*out_im);
        *out_re = NULL; *out_im = NULL;
        return -1;
    }

    /* Wait for TX to complete + propagation margin */
    uint32_t extra = cfg->extra_wait_us ? cfg->extra_wait_us : 2000;
    unsigned int tx_us = (unsigned int)(cfg->tx_samples / 20);  /* 20 samples/us @ 20 MSPS */
    usleep(tx_us + extra);

    /* Verify enough data captured since t0 */
    uint32_t cur = dma_rx_wr_ptr();
    uint32_t available = (cur >= t0) ? cur - t0
                       : (DMA_RX_BUF_SAMPLES - t0) + cur;
    if (available < (uint32_t)cap_n) {
        usleep(5000);
        cur = dma_rx_wr_ptr();
        available = (cur >= t0) ? cur - t0
                   : (DMA_RX_BUF_SAMPLES - t0) + cur;
    }

    /* Stop both DMAs */
    dma_tx_stop();
    dma_rx_stop();

    if (available < (uint32_t)cap_n) {
        fprintf(stderr, "styx_tx_and_capture: insufficient data (%u/%zu)\n",
                available, cap_n);
        free(*out_re); free(*out_im);
        *out_re = NULL; *out_im = NULL;
        return -1;
    }

    /* Read from DDR ring buffer starting at t0 */
    volatile uint32_t *rx_buf = hal_ddr_rx_buf();
    uint32_t read_ptr = t0;
    if (read_ptr + (uint32_t)cap_n <= DMA_RX_BUF_SAMPLES) {
        convert_rx_to_float(&rx_buf[read_ptr], cap_n,
                            *out_re, *out_im);
    } else {
        size_t first = DMA_RX_BUF_SAMPLES - read_ptr;
        size_t second = cap_n - first;
        convert_rx_to_float(&rx_buf[read_ptr], first,
                            *out_re, *out_im);
        convert_rx_to_float(&rx_buf[0], second,
                            &(*out_re)[first], &(*out_im)[first]);
    }

    /* Remove DC offset if requested */
    bool remove_dc = cfg->remove_dc;
    if (remove_dc) {
        double sum_re = 0.0, sum_im = 0.0;
        for (size_t i = 0; i < cap_n; i++) {
            sum_re += (*out_re)[i];
            sum_im += (*out_im)[i];
        }
        float dc_re = (float)(sum_re / (double)cap_n);
        float dc_im = (float)(sum_im / (double)cap_n);
        for (size_t i = 0; i < cap_n; i++) {
            (*out_re)[i] -= dc_re;
            (*out_im)[i] -= dc_im;
        }
    }

    return (int)cap_n;
}
