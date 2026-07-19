// SPDX-License-Identifier: MIT
#ifndef STYX_HAL_H
#define STYX_HAL_H

#include <stdint.h>
#include <stdbool.h>

/* --------------------------------------------------------------------------
 * Register addresses — Styx infrastructure
 * -------------------------------------------------------------------------- */

#define REG_BUILD_ID            0x43C00000

/* axi_ad9361 DAC core registers (for DAC data path init) */
#define REG_AD9361_DAC_BASE     0x79024000
#define REG_AD9361_DAC_RSTN     (REG_AD9361_DAC_BASE + 0x040)
#define REG_AD9361_DAC_SYNC     (REG_AD9361_DAC_BASE + 0x044)
#define REG_AD9361_DAC_CNTRL_2  (REG_AD9361_DAC_BASE + 0x048)
#define REG_AD9361_DAC_I0_SEL   (REG_AD9361_DAC_BASE + 0x418)
#define REG_AD9361_DAC_Q0_SEL   (REG_AD9361_DAC_BASE + 0x458)

/* IQ DMA RX registers */
#define REG_IQ_DMA_RX_BASE     0x7C4B0000
#define REG_IQ_DMA_RX_CONTROL  (REG_IQ_DMA_RX_BASE + 0x00)
#define REG_IQ_DMA_RX_STATUS   (REG_IQ_DMA_RX_BASE + 0x04)
#define REG_IQ_DMA_RX_DDR_BASE (REG_IQ_DMA_RX_BASE + 0x08)
#define REG_IQ_DMA_RX_WR_PTR   (REG_IQ_DMA_RX_BASE + 0x0C)
#define REG_IQ_DMA_RX_SAMPLE_COUNT (REG_IQ_DMA_RX_BASE + 0x10)
#define REG_IQ_DMA_RX_WRAP_COUNT   (REG_IQ_DMA_RX_BASE + 0x14)

/* IQ DMA TX registers */
#define REG_IQ_DMA_TX_BASE     0x7C4D0000
#define REG_IQ_DMA_TX_CONTROL  (REG_IQ_DMA_TX_BASE + 0x00)
#define REG_IQ_DMA_TX_STATUS   (REG_IQ_DMA_TX_BASE + 0x04)
#define REG_IQ_DMA_TX_DDR_BASE (REG_IQ_DMA_TX_BASE + 0x08)
#define REG_IQ_DMA_TX_COUNT    (REG_IQ_DMA_TX_BASE + 0x0C)
#define REG_IQ_DMA_TX_PTR      (REG_IQ_DMA_TX_BASE + 0x10)

/* Debug snap registers */
#define REG_SNAP_BASE          0x7C4E0000
#define REG_SNAP_CONTROL       (REG_SNAP_BASE + 0x00)
#define REG_SNAP_STATUS        (REG_SNAP_BASE + 0x04)
#define REG_SNAP_TRIG_CYCLE    (REG_SNAP_BASE + 0x08)
#define REG_SNAP_RD_ADDR       (REG_SNAP_BASE + 0x0C)
#define REG_SNAP_RD_DATA       (REG_SNAP_BASE + 0x10)

/* HIL controller registers (infrastructure only) */
#define REG_HIL_CTRL_BASE      0x7C500000
#define REG_HIL_CTRL_CONTROL   (REG_HIL_CTRL_BASE + 0x00)
#define REG_HIL_CTRL_STATUS    (REG_HIL_CTRL_BASE + 0x04)
#define REG_HIL_CTRL_DDR_BASE  (REG_HIL_CTRL_BASE + 0x08)
#define REG_HIL_CTRL_PLAY_COUNT (REG_HIL_CTRL_BASE + 0x0C)
#define REG_HIL_CTRL_PLAY_PTR  (REG_HIL_CTRL_BASE + 0x10)
/* 0x14–0x38: reserved for downstream extension registers */
#define REG_HIL_CTRL_ADC_CNT   (REG_HIL_CTRL_BASE + 0x3C)

/* HIL control bits */
#define HIL_CTRL_TEST_MODE     (1 << 0)
#define HIL_CTRL_TRIGGER       (1 << 1)

/* DMA control bits */
#define RX_CTRL_ENABLE         (1 << 0)
#define TX_CTRL_ENABLE         (1 << 0)
#define TX_CTRL_TRIGGER        (1 << 1)
#define TX_CTRL_CYCLIC         (1 << 2)

/* --------------------------------------------------------------------------
 * DDR memory map (within 144 MB reserved region)
 * -------------------------------------------------------------------------- */

#define DDR_RX_BASE            0x10000000
#define DDR_RX_SIZE            0x08000000  /* 128 MB */
#define DDR_TX_BASE            0x18000000
#define DDR_TX_SIZE            0x00400000  /* 4 MB */

/* --------------------------------------------------------------------------
 * IQ sample format: {8'b0, imag[11:0], real[11:0]} in 32-bit words
 * -------------------------------------------------------------------------- */

/* Pack signed 12-bit I and Q into DDR word */
#define IQ_PACK(real12, imag12) \
    ((uint32_t)(((uint32_t)((imag12) & 0xFFF) << 12) | ((uint32_t)((real12) & 0xFFF))))

/* Extract real part with 12-bit sign extension */
static inline int16_t IQ_REAL(uint32_t word) {
    int16_t v = (int16_t)(word & 0xFFF);
    if (v & 0x800) v |= (int16_t)0xF000;
    return v;
}

/* Extract imag part with 12-bit sign extension */
static inline int16_t IQ_IMAG(uint32_t word) {
    int16_t v = (int16_t)((word >> 12) & 0xFFF);
    if (v & 0x800) v |= (int16_t)0xF000;
    return v;
}

/* --------------------------------------------------------------------------
 * HAL lifecycle
 * -------------------------------------------------------------------------- */

int  hal_init(void);
void hal_cleanup(void);

/* --------------------------------------------------------------------------
 * Register access (routes to correct mmap region by address)
 * -------------------------------------------------------------------------- */

uint32_t hal_reg_read(uint32_t addr);
void     hal_reg_write(uint32_t addr, uint32_t val);

/* --------------------------------------------------------------------------
 * DDR buffer access
 * -------------------------------------------------------------------------- */

volatile uint32_t *hal_ddr_rx_buf(void);
volatile uint32_t *hal_ddr_tx_buf(void);

/* --------------------------------------------------------------------------
 * AD9361 configuration via sysfs
 * -------------------------------------------------------------------------- */

int hal_ad9361_set_rx_lo(uint64_t freq_hz);
int hal_ad9361_set_tx_lo(uint64_t freq_hz);
int hal_ad9361_set_sample_rate(uint64_t rate_hz);
int hal_ad9361_set_rx_bandwidth(uint64_t bw_hz);
int hal_ad9361_set_tx_bandwidth(uint64_t bw_hz);
int hal_ad9361_set_rx_gain(double gain_db);
double hal_ad9361_get_rx_gain(void);
int hal_ad9361_set_rx_gain_mode(const char *mode);
int hal_ad9361_set_tx_attenuation(double atten_db);
int hal_ad9361_run_calibration(void);

#endif /* STYX_HAL_H */
