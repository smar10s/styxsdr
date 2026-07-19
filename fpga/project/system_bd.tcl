# SPDX-License-Identifier: MIT
# ==========================================================================
# system_bd.tcl — Styx PlutoSDR block design
#
# Infrastructure platform: PS7 + AD9361 + DMA (RX/TX) + DDR + HIL + Snap
#
# Based on: analogdevicesinc/hdl @ 065c8f186ef87ff (pluto project)
#
# Stripped: RX FIR decimator, TX FIR interpolator, axi_dmac (both),
#           cpack, upack, TDD engine, decode pipeline.
# Retained: axi_ad9361 (PHY control for Linux IIO driver).
# Added:    iq_dma_rx/tx, adc_sync, hil_ctrl, snap_axi, axi_build_id.
#
# Optimization flags: MODE_1R1T=1, DAC_DDS_DISABLE=1,
# ADC/DAC IQCORRECTION_DISABLE=1, ADC_DCFILTER_DISABLE=0.
#
# Downstream projects (e.g. Deimos) source this file, then append their
# processing pipeline connected to hil_ctrl_0/iq_* outputs.
# ==========================================================================

###############################################################################
## Copyright (C) 2014-2024 Analog Devices, Inc. All rights reserved.
### SPDX short identifier: ADIBSD
###############################################################################

# ---- Add custom RTL modules to project ----
set script_dir [file dirname [info script]]
set rtl_dir [file normalize $script_dir/../rtl]

add_files -norecurse -fileset sources_1 [glob $rtl_dir/*.v]
add_files -norecurse -fileset sources_1 $script_dir/system_top.v

# Add memory initialization files (.hex) if any
set hex_files [glob -nocomplain $rtl_dir/*.hex]
if {[llength $hex_files] > 0} {
    add_files -norecurse -fileset sources_1 $hex_files
    set_property FILE_TYPE {Memory Initialization Files} [get_files -of_objects [get_filesets sources_1] *.hex]
}

# create board design
source $ad_hdl_dir/projects/common/xilinx/adi_fir_filter_bd.tcl

# default ports

create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 ddr
create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 fixed_io

create_bd_port -dir O spi0_csn_2_o
create_bd_port -dir O spi0_csn_1_o
create_bd_port -dir O spi0_csn_0_o
create_bd_port -dir I spi0_csn_i
create_bd_port -dir I spi0_clk_i
create_bd_port -dir O spi0_clk_o
create_bd_port -dir I spi0_sdo_i
create_bd_port -dir O spi0_sdo_o
create_bd_port -dir I spi0_sdi_i

create_bd_port -dir I -from 17 -to 0 gpio_i
create_bd_port -dir O -from 17 -to 0 gpio_o
create_bd_port -dir O -from 17 -to 0 gpio_t

create_bd_port -dir O spi_csn_o
create_bd_port -dir I spi_csn_i
create_bd_port -dir I spi_clk_i
create_bd_port -dir O spi_clk_o
create_bd_port -dir I spi_sdo_i
create_bd_port -dir O spi_sdo_o
create_bd_port -dir I spi_sdi_i

create_bd_port -dir O txdata_o
create_bd_port -dir I tdd_ext_sync

# instance: sys_ps7

ad_ip_instance processing_system7 sys_ps7

# ps7 settings

ad_ip_parameter sys_ps7 CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 1.8V}
ad_ip_parameter sys_ps7 CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V}
ad_ip_parameter sys_ps7 CONFIG.PCW_PACKAGE_NAME clg225
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_CLK1_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_RST1_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ 100.0
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ 200.0
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_EMIO_GPIO_IO 18
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI1_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_I2C0_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_UART1_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UART1_UART1_IO {MIO 12 .. 13}
ad_ip_parameter sys_ps7 CONFIG.PCW_I2C1_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_QSPI_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SD0_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI0_SPI0_IO EMIO
ad_ip_parameter sys_ps7 CONFIG.PCW_TTC0_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_FABRIC_INTERRUPT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_MIO_GPIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_MIO_GPIO_IO MIO
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_RESET_IO {MIO 52}
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_RESET_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_IRQ_F2P_INTR 1
ad_ip_parameter sys_ps7 CONFIG.PCW_IRQ_F2P_MODE REVERSE
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_0_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_9_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_10_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_11_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_48_PULLUP {enabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_49_PULLUP {disabled}
ad_ip_parameter sys_ps7 CONFIG.PCW_MIO_53_PULLUP {enabled}

# DDR MT41K256M16 HA-125 (32M, 16bit, 8banks)

ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41K256M16 RE-125}
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH {16 Bit}
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_USE_INTERNAL_VREF 0
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 0.048
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 0.050
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0 0.241
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1 0.240

ad_ip_instance xlconcat sys_concat_intc
ad_ip_parameter sys_concat_intc CONFIG.NUM_PORTS 16

ad_ip_instance proc_sys_reset sys_rstgen
ad_ip_parameter sys_rstgen CONFIG.C_EXT_RST_WIDTH 1

# system reset/clock definitions

# add external spi

ad_ip_instance axi_quad_spi axi_spi
ad_ip_parameter axi_spi CONFIG.C_USE_STARTUP 0
ad_ip_parameter axi_spi CONFIG.C_NUM_SS_BITS 1
ad_ip_parameter axi_spi CONFIG.C_SCK_RATIO 8

ad_connect  sys_cpu_clk sys_ps7/FCLK_CLK0
ad_connect  sys_200m_clk sys_ps7/FCLK_CLK1
ad_connect  sys_cpu_reset sys_rstgen/peripheral_reset
ad_connect  sys_cpu_resetn sys_rstgen/peripheral_aresetn
ad_connect  sys_cpu_clk sys_rstgen/slowest_sync_clk
ad_connect  sys_rstgen/ext_reset_in sys_ps7/FCLK_RESET0_N

# interface connections

ad_connect  ddr sys_ps7/DDR
ad_connect  gpio_i sys_ps7/GPIO_I
ad_connect  gpio_o sys_ps7/GPIO_O
ad_connect  gpio_t sys_ps7/GPIO_T
ad_connect  fixed_io sys_ps7/FIXED_IO

# ps7 spi connections

ad_connect  spi0_csn_2_o sys_ps7/SPI0_SS2_O
ad_connect  spi0_csn_1_o sys_ps7/SPI0_SS1_O
ad_connect  spi0_csn_0_o sys_ps7/SPI0_SS_O
ad_connect  spi0_csn_i sys_ps7/SPI0_SS_I
ad_connect  spi0_clk_i sys_ps7/SPI0_SCLK_I
ad_connect  spi0_clk_o sys_ps7/SPI0_SCLK_O
ad_connect  spi0_sdo_i sys_ps7/SPI0_MOSI_I
ad_connect  spi0_sdo_o sys_ps7/SPI0_MOSI_O
ad_connect  spi0_sdi_i sys_ps7/SPI0_MISO_I

# axi spi connections

ad_connect  sys_cpu_clk  axi_spi/ext_spi_clk
ad_connect  spi_csn_i  axi_spi/ss_i
ad_connect  spi_csn_o  axi_spi/ss_o
ad_connect  spi_clk_i  axi_spi/sck_i
ad_connect  spi_clk_o  axi_spi/sck_o
ad_connect  spi_sdo_i  axi_spi/io0_i
ad_connect  spi_sdo_o  axi_spi/io0_o
ad_connect  spi_sdi_i  axi_spi/io1_i

# interrupts

ad_connect  sys_concat_intc/dout sys_ps7/IRQ_F2P
ad_connect  sys_concat_intc/In15 GND
ad_connect  sys_concat_intc/In14 GND
ad_connect  sys_concat_intc/In13 GND
ad_connect  sys_concat_intc/In12 GND
ad_connect  sys_concat_intc/In11 GND
ad_connect  sys_concat_intc/In10 GND
ad_connect  sys_concat_intc/In9 GND
ad_connect  sys_concat_intc/In8 GND
ad_connect  sys_concat_intc/In7 GND
ad_connect  sys_concat_intc/In6 GND
ad_connect  sys_concat_intc/In5 GND
ad_connect  sys_concat_intc/In4 GND
ad_connect  sys_concat_intc/In3 GND
ad_connect  sys_concat_intc/In2 GND
ad_connect  sys_concat_intc/In1 GND
ad_connect  sys_concat_intc/In0 GND

# iic

create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 iic_main

ad_ip_instance axi_iic axi_iic_main

ad_connect  iic_main axi_iic_main/iic
ad_cpu_interconnect 0x41600000 axi_iic_main
ad_cpu_interrupt ps-15 mb-15 axi_iic_main/iic2intc_irpt

# ==========================================================================
# AD9361 Core
# ==========================================================================

create_bd_port -dir I rx_clk_in
create_bd_port -dir I rx_frame_in
create_bd_port -dir I -from 11 -to 0 rx_data_in

create_bd_port -dir O tx_clk_out
create_bd_port -dir O tx_frame_out
create_bd_port -dir O -from 11 -to 0 tx_data_out

create_bd_port -dir O enable
create_bd_port -dir O txnrx
create_bd_port -dir I up_enable
create_bd_port -dir I up_txnrx

# ad9361 core — optimization flags for minimal resource usage
# MODE_1R1T=1: single antenna (disable ch1 I/Q)
# DAC_DDS_DISABLE=1: no tone generators
# IQCORRECTION_DISABLE=1: not calibrated
# ADC_DCFILTER enabled: removes LO leakage / ADC DC offset
#
# NOTE: The AD9361 has an internal 128-tap programmable FIR with
# decimation by 1/2/4 (RX) and interpolation by 1/2/4 (TX).
# This is configured at runtime via the Linux IIO driver — no BD
# changes needed. When decimation is active, l_clk slows proportionally
# (20 MHz / dec_factor), reducing DDR bandwidth automatically.
# Useful for narrowband protocols (LoRA, BLE) that don't need 20 MSPS.

ad_ip_instance axi_ad9361 axi_ad9361
ad_ip_parameter axi_ad9361 CONFIG.ID 0
ad_ip_parameter axi_ad9361 CONFIG.CMOS_OR_LVDS_N 1
ad_ip_parameter axi_ad9361 CONFIG.MODE_1R1T 1
ad_ip_parameter axi_ad9361 CONFIG.ADC_INIT_DELAY 21
ad_ip_parameter axi_ad9361 CONFIG.TDD_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.DAC_DDS_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.ADC_IQCORRECTION_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.DAC_IQCORRECTION_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.ADC_DCFILTER_DISABLE 0

# ad9361 connections — physical pins

ad_connect  rx_clk_in axi_ad9361/rx_clk_in
ad_connect  rx_frame_in axi_ad9361/rx_frame_in
ad_connect  rx_data_in axi_ad9361/rx_data_in
ad_connect  tx_clk_out axi_ad9361/tx_clk_out
ad_connect  tx_frame_out axi_ad9361/tx_frame_out
ad_connect  tx_data_out axi_ad9361/tx_data_out
ad_connect  enable axi_ad9361/enable
ad_connect  txnrx axi_ad9361/txnrx
ad_connect  up_enable axi_ad9361/up_enable
ad_connect  up_txnrx axi_ad9361/up_txnrx

ad_connect  sys_200m_clk axi_ad9361/delay_clk
ad_connect  axi_ad9361/l_clk axi_ad9361/clk

# No stock DMA — tie overflow/underflow to zero
ad_connect  axi_ad9361/adc_dovf GND
ad_connect  axi_ad9361/dac_dunf GND

# DAC channel 1 — tie to zero (MODE_1R1T but ports still exist)
ad_ip_instance xlconstant dac_zero_i1
ad_ip_parameter dac_zero_i1 CONFIG.CONST_WIDTH 16
ad_ip_parameter dac_zero_i1 CONFIG.CONST_VAL 0
ad_connect dac_zero_i1/dout axi_ad9361/dac_data_i1

ad_ip_instance xlconstant dac_zero_q1
ad_ip_parameter dac_zero_q1 CONFIG.CONST_WIDTH 16
ad_ip_parameter dac_zero_q1 CONFIG.CONST_VAL 0
ad_connect dac_zero_q1/dout axi_ad9361/dac_data_q1

# TDD engine stripped — reconnect signals
ad_connect axi_ad9361/tdd_sync GND
ad_connect txdata_o GND

# ==========================================================================
# Build ID Register
# ==========================================================================

# Minimal AXI-Lite RTL module: one 32-bit read-only register.
# Value is set via BUILD_ID parameter at synthesis time.
# ARM reads with: devmem 0x43C00000

create_bd_cell -type module -reference axi_build_id axi_build_id_0
# BUILD_ID parameter will be set by build script

ad_connect sys_cpu_clk axi_build_id_0/s_axi_aclk
ad_connect sys_cpu_resetn axi_build_id_0/s_axi_aresetn

# ---- AXI interconnects ----

ad_cpu_interconnect 0x79020000 axi_ad9361
ad_cpu_interconnect 0x7C430000 axi_spi
ad_cpu_interconnect 0x43C00000 axi_build_id_0

# ==========================================================================
# IQ DMA — continuous ADC-to-DDR + DDR-to-DAC
#
# iq_dma_rx: ADC samples -> DDR ring buffer via AXI3 HP0 (sys_cpu_clk)
# iq_dma_tx: DDR -> DAC via AXI3 HP2 (l_clk domain)
#
# Clock domains:
#   ADC data arrives on l_clk (~20 MHz). CDC (adc_sync) crosses it
#   to sys_cpu_clk (100 MHz) for iq_dma_rx and downstream processing.
#   iq_dma_tx runs on l_clk (same as DAC) — no output CDC needed.
# ==========================================================================

# Add CDC timing constraints
add_files -fileset constrs_1 -norecurse $script_dir/constraints/dma_timing.xdc

# ---- Slice ADC data to 12 bits ----
# AD9361 ADC outputs 16-bit (12-bit data in lower bits, upper 4 zero).
ad_ip_instance xlslice adc_i_slice
ad_ip_parameter adc_i_slice CONFIG.DIN_WIDTH 16
ad_ip_parameter adc_i_slice CONFIG.DOUT_WIDTH 12
ad_ip_parameter adc_i_slice CONFIG.DIN_FROM 11
ad_ip_parameter adc_i_slice CONFIG.DIN_TO 0

ad_ip_instance xlslice adc_q_slice
ad_ip_parameter adc_q_slice CONFIG.DIN_WIDTH 16
ad_ip_parameter adc_q_slice CONFIG.DOUT_WIDTH 12
ad_ip_parameter adc_q_slice CONFIG.DIN_FROM 11
ad_ip_parameter adc_q_slice CONFIG.DIN_TO 0

ad_connect axi_ad9361/adc_data_i0 adc_i_slice/Din
ad_connect axi_ad9361/adc_data_q0 adc_q_slice/Din

# ---- ADC clock domain crossing (l_clk -> sys_cpu_clk) ----
create_bd_cell -type module -reference adc_sync adc_cdc

ad_connect axi_ad9361/l_clk adc_cdc/adc_clk
ad_connect axi_ad9361/rst adc_cdc/adc_rst
ad_connect adc_i_slice/Dout adc_cdc/re_in
ad_connect adc_q_slice/Dout adc_cdc/im_in
ad_connect axi_ad9361/adc_valid_i0 adc_cdc/valid_in

ad_connect sys_cpu_clk adc_cdc/sys_clk

# Reset for sys_cpu_clk domain (active-high from active-low sys_cpu_resetn)
ad_ip_instance util_vector_logic sys_rst_inv
ad_ip_parameter sys_rst_inv CONFIG.C_SIZE 1
ad_ip_parameter sys_rst_inv CONFIG.C_OPERATION {not}
ad_connect sys_cpu_resetn sys_rst_inv/Op1

ad_connect sys_rst_inv/Res adc_cdc/sys_rst

# ---- Continuous IQ DMA RX (ADC -> DDR via HP0) ----
create_bd_cell -type module -reference iq_dma_rx iq_dma_rx_0

ad_connect sys_cpu_clk iq_dma_rx_0/clk
ad_connect sys_rst_inv/Res iq_dma_rx_0/rst

# IQ data input from CDC output
ad_connect adc_cdc/re_out iq_dma_rx_0/re_in
ad_connect adc_cdc/im_out iq_dma_rx_0/im_in
ad_connect adc_cdc/valid_out iq_dma_rx_0/valid_in

# AXI-Lite control registers
ad_cpu_interconnect 0x7C4B0000 iq_dma_rx_0
delete_bd_objs [get_bd_addr_segs sys_ps7/Data/SEG_data_iq_dma_rx_0]
create_bd_addr_seg -range 0x1000 -offset 0x7C4B0000 \
    [get_bd_addr_spaces sys_ps7/Data] \
    [get_bd_addr_segs iq_dma_rx_0/s_axi/reg0] \
    SEG_data_iq_dma_rx_0

# Enable PS7 HP0 (64-bit, sys_cpu_clk)
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP0 {1}
ad_ip_parameter sys_ps7 CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64}
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP0_ACLK
ad_connect iq_dma_rx_0/m_axi sys_ps7/S_AXI_HP0

create_bd_addr_seg -range 0x20000000 -offset 0x00000000 \
    [get_bd_addr_spaces iq_dma_rx_0/m_axi] \
    [get_bd_addr_segs sys_ps7/S_AXI_HP0/HP0_DDR_LOWOCM] \
    SEG_iq_dma_rx_HP0

# ---- TX DMA: DDR -> DAC via HP2 (l_clk domain) ----
create_bd_cell -type module -reference iq_dma_tx iq_dma_tx_0

# TX FSM + AXI3 master: l_clk domain
ad_connect axi_ad9361/l_clk iq_dma_tx_0/clk
ad_connect axi_ad9361/rst iq_dma_tx_0/rst

# AXI-Lite slave: sys_cpu_clk domain
ad_connect sys_cpu_clk iq_dma_tx_0/s_axi_aclk
ad_connect sys_cpu_resetn iq_dma_tx_0/s_axi_aresetn

# AXI-Lite control registers
ad_cpu_interconnect 0x7C4D0000 iq_dma_tx_0
delete_bd_objs [get_bd_addr_segs sys_ps7/Data/SEG_data_iq_dma_tx_0]
create_bd_addr_seg -range 0x1000 -offset 0x7C4D0000 \
    [get_bd_addr_spaces sys_ps7/Data] \
    [get_bd_addr_segs iq_dma_tx_0/s_axi/reg0] \
    SEG_data_iq_dma_tx_0

# TX DMA AXI3 read master -> HP2 (l_clk domain)
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP2 {1}
ad_ip_parameter sys_ps7 CONFIG.PCW_S_AXI_HP2_DATA_WIDTH {64}
ad_connect axi_ad9361/l_clk sys_ps7/S_AXI_HP2_ACLK
ad_connect iq_dma_tx_0/m_axi sys_ps7/S_AXI_HP2

create_bd_addr_seg -range 0x20000000 -offset 0x00000000 \
    [get_bd_addr_spaces iq_dma_tx_0/m_axi] \
    [get_bd_addr_segs sys_ps7/S_AXI_HP2/HP2_DDR_LOWOCM] \
    SEG_iq_dma_tx_HP2

# DAC handshake
ad_connect axi_ad9361/dac_valid_i0 iq_dma_tx_0/dac_valid

# Wire: iq_dma_tx output -> axi_ad9361 DAC channel 0
ad_connect iq_dma_tx_0/re_out axi_ad9361/dac_data_i0
ad_connect iq_dma_tx_0/im_out axi_ad9361/dac_data_q0

# ==========================================================================
# HIL Controller (test mux + DDR playback)
#
# In normal mode: passes ADC IQ through to downstream.
# In test mode: plays IQ from DDR (golden vectors or test signals).
# ==========================================================================

create_bd_cell -type module -reference hil_ctrl hil_ctrl_0

# Clock + reset (sys_cpu_clk domain, active-high reset)
ad_connect sys_cpu_clk hil_ctrl_0/clk
ad_connect sys_rst_inv/Res hil_ctrl_0/rst

# ADC input from CDC
ad_connect adc_cdc/re_out hil_ctrl_0/adc_re
ad_connect adc_cdc/im_out hil_ctrl_0/adc_im
ad_connect adc_cdc/valid_out hil_ctrl_0/adc_valid

# AXI-Lite control registers at 0x7C500000
ad_cpu_interconnect 0x7C500000 hil_ctrl_0
delete_bd_objs [get_bd_addr_segs sys_ps7/Data/SEG_data_hil_ctrl_0]
create_bd_addr_seg -range 0x1000 -offset 0x7C500000 \
    [get_bd_addr_spaces sys_ps7/Data] \
    [get_bd_addr_segs hil_ctrl_0/s_axi/reg0] \
    SEG_data_hil_ctrl_0

# AXI3 read master -> HP1 (DDR playback, sys_cpu_clk domain)
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP1 {1}
ad_ip_parameter sys_ps7 CONFIG.PCW_S_AXI_HP1_DATA_WIDTH {64}
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP1_ACLK
ad_connect hil_ctrl_0/m_axi sys_ps7/S_AXI_HP1

create_bd_addr_seg -range 0x20000000 -offset 0x00000000 \
    [get_bd_addr_spaces hil_ctrl_0/m_axi] \
    [get_bd_addr_segs sys_ps7/S_AXI_HP1/HP1_DDR_LOWOCM] \
    SEG_hil_ctrl_HP1

# ==========================================================================
# Debug Snap Probe
#
# In standalone styx: observes raw IQ from HIL output.
# Downstream projects reconnect to their observation points.
# ==========================================================================

create_bd_cell -type module -reference snap_axi snap_axi_0

ad_connect sys_cpu_clk snap_axi_0/clk
ad_connect sys_rst_inv/Res snap_axi_0/rst

# Default snap connection: observe HIL output IQ
# Pack 12-bit I + 12-bit Q into 32-bit sample_data: {8'b0, q[11:0], i[11:0]}
ad_ip_instance xlconcat snap_iq_concat
ad_ip_parameter snap_iq_concat CONFIG.NUM_PORTS 3
ad_ip_parameter snap_iq_concat CONFIG.IN0_WIDTH 12
ad_ip_parameter snap_iq_concat CONFIG.IN1_WIDTH 12
ad_ip_parameter snap_iq_concat CONFIG.IN2_WIDTH 8

ad_connect hil_ctrl_0/iq_re snap_iq_concat/In0
ad_connect hil_ctrl_0/iq_im snap_iq_concat/In1

ad_ip_instance xlconstant snap_pad_zero
ad_ip_parameter snap_pad_zero CONFIG.CONST_WIDTH 8
ad_ip_parameter snap_pad_zero CONFIG.CONST_VAL 0
ad_connect snap_pad_zero/dout snap_iq_concat/In2

ad_connect snap_iq_concat/dout snap_axi_0/sample_data
ad_connect hil_ctrl_0/iq_valid snap_axi_0/sample_valid

# No external trigger in standalone — tie low (use sw_trigger via register)
ad_connect GND snap_axi_0/ext_trig

# AXI-Lite control registers at 0x7C4E0000
ad_cpu_interconnect 0x7C4E0000 snap_axi_0
delete_bd_objs [get_bd_addr_segs sys_ps7/Data/SEG_data_snap_axi_0]
create_bd_addr_seg -range 0x1000 -offset 0x7C4E0000 \
    [get_bd_addr_spaces sys_ps7/Data] \
    [get_bd_addr_segs snap_axi_0/s_axi/reg0] \
    SEG_data_snap_axi_0

# ==========================================================================
# Interrupts
# ==========================================================================

ad_cpu_interrupt ps-11 mb-11 axi_spi/ip2intc_irpt
