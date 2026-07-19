// SPDX-License-Identifier: MIT
#include "hal.h"  /* styx infrastructure HAL */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

#define PAGE_SIZE           4096
#define REG_REGION_SIZE     PAGE_SIZE   /* each register block fits in one page */

#define AD9361_IIO_PATH     "/sys/bus/iio/devices/iio:device1/"

/* --------------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------------- */

static int mem_fd = -1;

/* Register region mappings */
static volatile uint32_t *map_build_id = NULL;
static volatile uint32_t *map_rx_dma   = NULL;
static volatile uint32_t *map_tx_dma   = NULL;
static volatile uint32_t *map_dac_core = NULL;
static volatile uint32_t *map_snap     = NULL;
static volatile uint32_t *map_hil_ctrl = NULL;

/* DDR buffer mappings */
static volatile uint32_t *map_ddr_tx   = NULL;
static volatile uint32_t *map_ddr_rx   = NULL;

static bool hal_initialized = false;

/* --------------------------------------------------------------------------
 * Sysfs helpers
 * -------------------------------------------------------------------------- */

static int sysfs_write_str(const char *path, const char *val)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "hal: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    if (fprintf(f, "%s", val) < 0) {
        fprintf(stderr, "hal: write to %s failed: %s\n", path, strerror(errno));
        fclose(f);
        return -1;
    }
    /* fclose flushes and propagates kernel-side errors (e.g. EOPNOTSUPP
     * from AD9361 driver when writing gain in AGC mode) */
    if (fclose(f) != 0) {
        fprintf(stderr, "hal: write to %s rejected: %s\n", path, strerror(errno));
        return -1;
    }
    return 0;
}

static int sysfs_write_ll(const char *path, long long val)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%lld", val);
    return sysfs_write_str(path, buf);
}

static long long sysfs_read_ll(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    long long val = 0;
    if (fscanf(f, "%lld", &val) != 1)
        val = -1;
    fclose(f);
    return val;
}

/* --------------------------------------------------------------------------
 * HAL init / cleanup
 * -------------------------------------------------------------------------- */

int hal_init(void)
{
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        fprintf(stderr, "hal: cannot open /dev/mem: %s\n", strerror(errno));
        return -1;
    }

    /* Map register regions */
    map_build_id = (volatile uint32_t *)mmap(NULL, REG_REGION_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, REG_BUILD_ID);
    if (map_build_id == MAP_FAILED) {
        fprintf(stderr, "hal: mmap build_id failed: %s\n", strerror(errno));
        goto fail;
    }

    map_rx_dma = (volatile uint32_t *)mmap(NULL, REG_REGION_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, REG_IQ_DMA_RX_BASE);
    if (map_rx_dma == MAP_FAILED) {
        fprintf(stderr, "hal: mmap rx_dma failed: %s\n", strerror(errno));
        goto fail;
    }

    map_tx_dma = (volatile uint32_t *)mmap(NULL, REG_REGION_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, REG_IQ_DMA_TX_BASE);
    if (map_tx_dma == MAP_FAILED) {
        fprintf(stderr, "hal: mmap tx_dma failed: %s\n", strerror(errno));
        goto fail;
    }

    map_dac_core = (volatile uint32_t *)mmap(NULL, REG_REGION_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, REG_AD9361_DAC_BASE);
    if (map_dac_core == MAP_FAILED) {
        fprintf(stderr, "hal: mmap dac_core failed: %s\n", strerror(errno));
        goto fail;
    }

    map_snap = (volatile uint32_t *)mmap(NULL, REG_REGION_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, REG_SNAP_BASE);
    if (map_snap == MAP_FAILED) {
        fprintf(stderr, "hal: mmap snap failed: %s\n", strerror(errno));
        goto fail;
    }

    map_hil_ctrl = (volatile uint32_t *)mmap(NULL, REG_REGION_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, REG_HIL_CTRL_BASE);
    if (map_hil_ctrl == MAP_FAILED) {
        fprintf(stderr, "hal: mmap hil_ctrl failed: %s\n", strerror(errno));
        goto fail;
    }

    /* Map DDR regions */
    map_ddr_tx = (volatile uint32_t *)mmap(NULL, DDR_TX_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, DDR_TX_BASE);
    if (map_ddr_tx == MAP_FAILED) {
        fprintf(stderr, "hal: mmap ddr_tx failed: %s\n", strerror(errno));
        goto fail;
    }

    map_ddr_rx = (volatile uint32_t *)mmap(NULL, DDR_RX_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, DDR_RX_BASE);
    if (map_ddr_rx == MAP_FAILED) {
        fprintf(stderr, "hal: mmap ddr_rx failed: %s\n", strerror(errno));
        goto fail;
    }

    /* Initialize axi_ad9361 DAC data path.
     * The cf-ad9361-dds-core-lpc kernel driver doesn't probe (we removed
     * its DMA dependency). Without this init, DAC channels default to
     * internal DDS mode and ignore dac_data_i0/q0 from our iq_dma_tx. */
    {
        volatile uint32_t *dac = map_dac_core;
        uint32_t dac_off = 0;  /* offsets are relative to DAC base */

        /* Reset DAC core */
        dac[(0x040 - dac_off) / 4] = 0x00000000;
        usleep(1000);
        /* Release reset (resetn=1, mmcm_resetn=1) */
        dac[(0x040 - dac_off) / 4] = 0x00000003;
        usleep(1000);
        /* CNTRL_2: datafmt=1 (2's complement), r1_mode=1 */
        dac[(0x048 - dac_off) / 4] = 0x00000030;
        /* CH0_I data_sel = DMA (external dac_data_i0 port) */
        dac[(0x418 - dac_off) / 4] = 0x00000002;
        /* CH0_Q data_sel = DMA (external dac_data_q0 port) */
        dac[(0x458 - dac_off) / 4] = 0x00000002;
        /* SYNC pulse */
        dac[(0x044 - dac_off) / 4] = 0x00000001;
        usleep(1000);
    }

    hal_initialized = true;
    return 0;

fail:
    hal_cleanup();
    return -1;
}

void hal_cleanup(void)
{
    hal_initialized = false;

    if (map_build_id && map_build_id != MAP_FAILED)
        munmap((void *)map_build_id, REG_REGION_SIZE);
    if (map_rx_dma && map_rx_dma != MAP_FAILED)
        munmap((void *)map_rx_dma, REG_REGION_SIZE);
    if (map_tx_dma && map_tx_dma != MAP_FAILED)
        munmap((void *)map_tx_dma, REG_REGION_SIZE);
    if (map_dac_core && map_dac_core != MAP_FAILED)
        munmap((void *)map_dac_core, REG_REGION_SIZE);
    if (map_snap && map_snap != MAP_FAILED)
        munmap((void *)map_snap, REG_REGION_SIZE);
    if (map_hil_ctrl && map_hil_ctrl != MAP_FAILED)
        munmap((void *)map_hil_ctrl, REG_REGION_SIZE);
    if (map_ddr_tx && map_ddr_tx != MAP_FAILED)
        munmap((void *)map_ddr_tx, DDR_TX_SIZE);
    if (map_ddr_rx && map_ddr_rx != MAP_FAILED)
        munmap((void *)map_ddr_rx, DDR_RX_SIZE);

    map_build_id = NULL;
    map_rx_dma = NULL;
    map_tx_dma = NULL;
    map_dac_core = NULL;
    map_snap = NULL;
    map_hil_ctrl = NULL;
    map_ddr_tx = NULL;
    map_ddr_rx = NULL;

    if (mem_fd >= 0) {
        close(mem_fd);
        mem_fd = -1;
    }
}

/* --------------------------------------------------------------------------
 * Register access
 * -------------------------------------------------------------------------- */

uint32_t hal_reg_read(uint32_t addr)
{
    if (!hal_initialized) {
        fprintf(stderr, "hal: reg access before hal_init()\n");
        return 0xDEADBEEF;
    }

    if (addr >= REG_BUILD_ID && addr < REG_BUILD_ID + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_BUILD_ID) / 4;
        return map_build_id[offset];
    }
    if (addr >= REG_IQ_DMA_RX_BASE && addr < REG_IQ_DMA_RX_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_IQ_DMA_RX_BASE) / 4;
        return map_rx_dma[offset];
    }
    if (addr >= REG_IQ_DMA_TX_BASE && addr < REG_IQ_DMA_TX_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_IQ_DMA_TX_BASE) / 4;
        return map_tx_dma[offset];
    }
    if (addr >= REG_AD9361_DAC_BASE && addr < REG_AD9361_DAC_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_AD9361_DAC_BASE) / 4;
        return map_dac_core[offset];
    }
    if (addr >= REG_SNAP_BASE && addr < REG_SNAP_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_SNAP_BASE) / 4;
        return map_snap[offset];
    }
    if (addr >= REG_HIL_CTRL_BASE && addr < REG_HIL_CTRL_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_HIL_CTRL_BASE) / 4;
        return map_hil_ctrl[offset];
    }
    fprintf(stderr, "hal: reg_read unknown addr 0x%08" PRIX32 "\n", addr);
    return 0xDEADBEEF;
}

void hal_reg_write(uint32_t addr, uint32_t val)
{
    if (!hal_initialized) {
        fprintf(stderr, "hal: reg access before hal_init()\n");
        return;
    }

    if (addr >= REG_BUILD_ID && addr < REG_BUILD_ID + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_BUILD_ID) / 4;
        map_build_id[offset] = val;
        return;
    }
    if (addr >= REG_IQ_DMA_RX_BASE && addr < REG_IQ_DMA_RX_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_IQ_DMA_RX_BASE) / 4;
        map_rx_dma[offset] = val;
        return;
    }
    if (addr >= REG_IQ_DMA_TX_BASE && addr < REG_IQ_DMA_TX_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_IQ_DMA_TX_BASE) / 4;
        map_tx_dma[offset] = val;
        return;
    }
    if (addr >= REG_AD9361_DAC_BASE && addr < REG_AD9361_DAC_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_AD9361_DAC_BASE) / 4;
        map_dac_core[offset] = val;
        return;
    }
    if (addr >= REG_SNAP_BASE && addr < REG_SNAP_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_SNAP_BASE) / 4;
        map_snap[offset] = val;
        return;
    }
    if (addr >= REG_HIL_CTRL_BASE && addr < REG_HIL_CTRL_BASE + REG_REGION_SIZE) {
        uint32_t offset = (addr - REG_HIL_CTRL_BASE) / 4;
        map_hil_ctrl[offset] = val;
        return;
    }
    fprintf(stderr, "hal: reg_write unknown addr 0x%08" PRIX32 "\n", addr);
}

/* --------------------------------------------------------------------------
 * DDR buffer access
 * -------------------------------------------------------------------------- */

volatile uint32_t *hal_ddr_rx_buf(void)
{
    return map_ddr_rx;
}

volatile uint32_t *hal_ddr_tx_buf(void)
{
    return map_ddr_tx;
}

/* --------------------------------------------------------------------------
 * AD9361 sysfs configuration
 * -------------------------------------------------------------------------- */

int hal_ad9361_set_rx_lo(uint64_t freq_hz)
{
    const char *path = AD9361_IIO_PATH "out_altvoltage0_RX_LO_frequency";
    int rc = sysfs_write_ll(path, (long long)freq_hz);
    if (rc == 0) {
        long long actual = sysfs_read_ll(path);
        if (actual >= 0 && llabs(actual - (long long)freq_hz) > 1000) {
            fprintf(stderr, "hal: RX LO mismatch: requested %lld, got %lld\n",
                    (long long)freq_hz, actual);
            return -1;
        }
    }
    return rc;
}

int hal_ad9361_set_tx_lo(uint64_t freq_hz)
{
    const char *path = AD9361_IIO_PATH "out_altvoltage1_TX_LO_frequency";
    int rc = sysfs_write_ll(path, (long long)freq_hz);
    if (rc == 0) {
        long long actual = sysfs_read_ll(path);
        if (actual >= 0 && llabs(actual - (long long)freq_hz) > 1000) {
            fprintf(stderr, "hal: TX LO mismatch: requested %lld, got %lld\n",
                    (long long)freq_hz, actual);
            return -1;
        }
    }
    return rc;
}

int hal_ad9361_set_sample_rate(uint64_t rate_hz)
{
    const char *path = AD9361_IIO_PATH "in_voltage_sampling_frequency";
    int rc = sysfs_write_ll(path, (long long)rate_hz);
    if (rc == 0) {
        long long actual = sysfs_read_ll(path);
        if (actual >= 0 && llabs(actual - (long long)rate_hz) > 100) {
            fprintf(stderr, "hal: sample rate mismatch: requested %lld, got %lld\n",
                    (long long)rate_hz, actual);
            return -1;
        }
    }
    return rc;
}

int hal_ad9361_set_rx_bandwidth(uint64_t bw_hz)
{
    return sysfs_write_ll(AD9361_IIO_PATH "in_voltage_rf_bandwidth",
                          (long long)bw_hz);
}

int hal_ad9361_set_tx_bandwidth(uint64_t bw_hz)
{
    return sysfs_write_ll(AD9361_IIO_PATH "out_voltage_rf_bandwidth",
                          (long long)bw_hz);
}

int hal_ad9361_set_rx_gain(double gain_db)
{
    char buf[64];
    /* Ensure manual gain mode — writing gain in AGC mode silently fails.
     * This is the #1 cause of OTA decode collapse after cold boot.
     * See docs/ad9361-gain-mode-gotcha.md */
    int rc = sysfs_write_str(AD9361_IIO_PATH "in_voltage0_gain_control_mode",
                             "manual");
    if (rc != 0) {
        fprintf(stderr, "hal: WARNING: could not set manual gain mode\n");
    }
    /* AD9361 sysfs expects integer or "N.000000 dB" — just write integer */
    snprintf(buf, sizeof(buf), "%d", (int)gain_db);
    return sysfs_write_str(AD9361_IIO_PATH "in_voltage0_hardwaregain", buf);
}

double hal_ad9361_get_rx_gain(void)
{
    const char *path = AD9361_IIO_PATH "in_voltage0_hardwaregain";
    FILE *f = fopen(path, "r");
    if (!f)
        return -999.0;
    double val = 0.0;
    if (fscanf(f, "%lf", &val) != 1)
        val = -999.0;
    fclose(f);
    return val;
}

int hal_ad9361_set_rx_gain_mode(const char *mode)
{
    return sysfs_write_str(AD9361_IIO_PATH "in_voltage0_gain_control_mode", mode);
}

int hal_ad9361_set_tx_attenuation(double atten_db)
{
    char buf[64];
    /* AD9361 sysfs expects integer or bare number for TX hardwaregain.
     * TX attenuation is negative (e.g. -10 means 10 dB attenuation).
     * The attribute supports 0.25 dB steps, so use one decimal. */
    double val = -(atten_db > 0 ? atten_db : -atten_db);
    snprintf(buf, sizeof(buf), "%.1f", val);
    return sysfs_write_str(AD9361_IIO_PATH "out_voltage0_hardwaregain", buf);
}

int hal_ad9361_run_calibration(void)
{
    int rc = 0;
    /* TX quadrature calibration — corrects TX IQ imbalance */
    rc |= sysfs_write_str(AD9361_IIO_PATH "calib_mode", "manual_tx_quad");
    usleep(100000);  /* 100 ms for calibration to complete */

    /* RF DC offset calibration — corrects LO leakage */
    rc |= sysfs_write_str(AD9361_IIO_PATH "calib_mode", "rf_dc_offs");
    usleep(100000);

    /* Return to auto mode */
    rc |= sysfs_write_str(AD9361_IIO_PATH "calib_mode", "auto");
    return rc;
}
