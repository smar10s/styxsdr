# SPDX-License-Identifier: MIT
"""Cocotb tests for hil_ctrl.v — Hardware-in-the-Loop test controller.

Tests:
    1. Normal mode passthrough — test_mode=0: ADC passes through
    2. Playback basic — load DDR, trigger, verify output
    3. Playback rate matching — 1 sample per 5 clocks
    4. Playback done — after play_count samples, playback_done asserts
    5. Mux switches cleanly — switch test_mode during live ADC
    6. ADC valid counter — ADC_CNT increments on each adc_valid
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from axi_lite_driver import AxiLiteMaster
from axi3_mocks import Axi3ReadSlave, pack_iq_word

# Register offsets
REG_CONTROL    = 0x00
REG_STATUS     = 0x04
REG_DDR_BASE   = 0x08
REG_PLAY_COUNT = 0x0C
REG_PLAY_PTR   = 0x10
# 0x14–0x38: reserved for downstream extension
REG_ADC_CNT    = 0x3C

# DDR base address
DDR_BASE = 0x10000000


async def setup(dut, rvalid_delay=0):
    """Start clock, reset, create AXI-Lite + AXI3 read slave."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.rst.value = 1
    dut.adc_valid.value = 0
    dut.adc_re.value = 0
    dut.adc_im.value = 0
    # AXI-Lite
    dut.s_axi_awaddr.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wdata.value = 0
    dut.s_axi_wstrb.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_araddr.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0
    # AXI3 read slave
    dut.m_axi_arready.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rvalid.value = 0
    dut.m_axi_rid.value = 0

    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    slave = Axi3ReadSlave(dut, prefix="m_axi_", clk=dut.clk,
                          rvalid_delay=rvalid_delay)
    cocotb.start_soon(slave.run())

    return axi, slave


async def trigger_playback(axi, ddr_base, play_count):
    """Configure and trigger a playback."""
    await axi.write(REG_DDR_BASE, ddr_base)
    await axi.write(REG_PLAY_COUNT, play_count)
    # Set test_mode=1, trigger=1 (bit[1] W1S)
    await axi.write(REG_CONTROL, 0x03)


async def collect_iq_output(dut, count, timeout_cycles=5000):
    """Collect IQ output samples. Returns list of (re, im) signed 12-bit."""
    results = []
    cycles = 0
    while len(results) < count and cycles < timeout_cycles:
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.iq_valid.value):
            re = int(dut.iq_re.value)
            im = int(dut.iq_im.value)
            # Sign-extend 12-bit
            if re >= 0x800:
                re -= 0x1000
            if im >= 0x800:
                im -= 0x1000
            results.append((re, im))
        cycles += 1
    return results


def to_signed_12(val):
    """Convert unsigned 12-bit to signed."""
    val = val & 0xFFF
    return val - 0x1000 if val >= 0x800 else val


@cocotb.test()
async def test_normal_mode_passthrough(dut):
    """test_mode=0: adc_valid/re/im pass through to iq_valid/re/im."""
    axi, slave = await setup(dut)

    # Default is test_mode=0 (normal mode)
    # Feed ADC samples and verify they appear on output
    test_data = [(100, -200), (500, 1000), (-1000, 2047)]

    for re, im in test_data:
        dut.adc_valid.value = 1
        dut.adc_re.value = re & 0xFFF
        dut.adc_im.value = im & 0xFFF
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")

        assert int(dut.iq_valid.value) == 1, "iq_valid not asserted in passthrough"
        got_re = to_signed_12(int(dut.iq_re.value))
        got_im = to_signed_12(int(dut.iq_im.value))
        assert got_re == re, f"re passthrough: expected {re}, got {got_re}"
        assert got_im == im, f"im passthrough: expected {im}, got {got_im}"

    dut.adc_valid.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.iq_valid.value) == 0, "iq_valid stuck high"


@cocotb.test()
async def test_playback_basic(dut):
    """Load DDR, trigger playback, verify iq_valid fires with correct data."""
    axi, slave = await setup(dut)

    # Pre-load DDR with known IQ samples
    play_count = 32  # one burst worth
    samples = [(i * 10, -(i * 10)) for i in range(play_count)]
    slave.write_iq_samples(DDR_BASE, samples)

    # Trigger playback
    await trigger_playback(axi, DDR_BASE, play_count)

    # Collect output
    results = await collect_iq_output(dut, play_count)

    assert len(results) == play_count, \
        f"Got {len(results)} samples, expected {play_count}"

    # Verify first few samples
    for i in range(min(10, play_count)):
        exp_re, exp_im = samples[i]
        got_re, got_im = results[i]
        assert got_re == exp_re, f"Sample {i} re: expected {exp_re}, got {got_re}"
        assert got_im == exp_im, f"Sample {i} im: expected {exp_im}, got {got_im}"


@cocotb.test()
async def test_playback_rate_matching(dut):
    """Verify playback outputs 1 sample per 5 clocks (20 MSPS into 100 MHz)."""
    axi, slave = await setup(dut)

    play_count = 32
    samples = [(i, i) for i in range(play_count)]
    slave.write_iq_samples(DDR_BASE, samples)

    await trigger_playback(axi, DDR_BASE, play_count)

    # Measure cycles between valid_out assertions
    gaps = []
    last_valid_cycle = None
    cycle = 0
    count = 0

    while count < 10 and cycle < 2000:
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        cycle += 1
        if int(dut.iq_valid.value):
            if last_valid_cycle is not None:
                gaps.append(cycle - last_valid_cycle)
            last_valid_cycle = cycle
            count += 1

    assert len(gaps) >= 5, f"Not enough valid pulses to measure rate"

    # All gaps should be exactly 5 (modulo-5 divider)
    for i, gap in enumerate(gaps):
        assert gap == 5, f"Gap {i}: expected 5 cycles, got {gap}"


@cocotb.test()
async def test_playback_done(dut):
    """After play_count samples output, playback_done asserts."""
    axi, slave = await setup(dut)

    play_count = 32
    samples = [(i, i) for i in range(play_count)]
    slave.write_iq_samples(DDR_BASE, samples)

    await trigger_playback(axi, DDR_BASE, play_count)

    # Wait for playback to complete
    results = await collect_iq_output(dut, play_count)
    assert len(results) == play_count

    # Wait a few more cycles for done to propagate
    for _ in range(20):
        await RisingEdge(dut.clk)

    # Read STATUS — bit[1] = playback_done
    status, _ = await axi.read(REG_STATUS)
    done = (status >> 1) & 1
    assert done == 1, f"playback_done not set: STATUS=0x{status:08X}"


@cocotb.test()
async def test_mux_switches_cleanly(dut):
    """Switch test_mode during live ADC, verify no glitch/lost sample."""
    axi, slave = await setup(dut)

    # Start with ADC samples flowing (normal mode)
    for i in range(5):
        dut.adc_valid.value = 1
        dut.adc_re.value = (i * 100) & 0xFFF
        dut.adc_im.value = (i * 100) & 0xFFF
        await RisingEdge(dut.clk)
    dut.adc_valid.value = 0

    # Switch to test mode (without triggering playback yet)
    await axi.write(REG_CONTROL, 0x01)  # test_mode=1, no trigger

    # ADC should no longer pass through
    dut.adc_valid.value = 1
    dut.adc_re.value = 0x7FF
    dut.adc_im.value = 0x7FF
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")

    # In test mode without playback active, test_valid should be 0
    # So iq_valid = test_valid = 0 (playback not running)
    assert int(dut.iq_valid.value) == 0, \
        "ADC leaking through in test mode (no playback)"

    # Switch back to normal mode
    await axi.write(REG_CONTROL, 0x00)  # test_mode=0

    # ADC should pass through again
    dut.adc_valid.value = 1
    dut.adc_re.value = 123 & 0xFFF
    dut.adc_im.value = 456 & 0xFFF
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")

    assert int(dut.iq_valid.value) == 1, "ADC not passing through after mode switch"
    got_re = to_signed_12(int(dut.iq_re.value))
    assert got_re == 123, f"After switch back: re = {got_re}, expected 123"


@cocotb.test()
async def test_adc_valid_counter(dut):
    """In live mode, verify ADC_CNT register increments on each adc_valid."""
    axi, slave = await setup(dut)

    # Ensure normal mode (test_mode=0)
    # Read initial count
    cnt_before, _ = await axi.read(REG_ADC_CNT)

    # Pulse adc_valid 20 times
    num_pulses = 20
    for _ in range(num_pulses):
        dut.adc_valid.value = 1
        dut.adc_re.value = 0
        dut.adc_im.value = 0
        await RisingEdge(dut.clk)
    dut.adc_valid.value = 0

    # Wait a couple cycles for register to update
    for _ in range(3):
        await RisingEdge(dut.clk)

    cnt_after, _ = await axi.read(REG_ADC_CNT)

    delta = cnt_after - cnt_before
    assert delta == num_pulses, \
        f"ADC_CNT delta = {delta}, expected {num_pulses}"
