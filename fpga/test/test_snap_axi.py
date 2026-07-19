# SPDX-License-Identifier: MIT
"""Cocotb tests for snap_axi.v — AXI4-Lite wrapper for debug_snap.

Tests:
    1. Arm and capture — full workflow via registers
    2. Software trigger — CONTROL[1] fires capture without ext_trig
    3. Readback — iterate RD_ADDR/RD_DATA to verify buffer contents
    4. Rearm — after capture, write ARM again for new capture
    5. Circular — set circular_en, verify never captures
    6. External trigger — ext_trig input fires capture when armed
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from axi_lite_driver import AxiLiteMaster

# Register offsets
REG_CONTROL    = 0x00
REG_STATUS     = 0x04
REG_TRIG_CYCLE = 0x08
REG_RD_ADDR    = 0x0C
REG_RD_DATA    = 0x10

# CONTROL bits
CTRL_ARM       = (1 << 0)
CTRL_SW_TRIG   = (1 << 1)
CTRL_CIRCULAR  = (1 << 2)

# STATUS bits
STAT_CAPTURED  = (1 << 0)
STAT_ARMED     = (1 << 1)

# Module parameters
POST_DEPTH = 512


async def setup(dut):
    """Start clock, reset, return AXI-Lite driver."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.rst.value = 1
    dut.sample_data.value = 0
    dut.sample_valid.value = 0
    dut.ext_trig.value = 0
    # Deassert AXI inputs
    dut.s_axi_awaddr.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wdata.value = 0
    dut.s_axi_wstrb.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_araddr.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0

    for _ in range(5):
        await RisingEdge(dut.clk)

    dut.rst.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    return axi


async def feed_samples(dut, count, start_val=0):
    """Feed count samples with incrementing data."""
    for i in range(count):
        dut.sample_valid.value = 1
        dut.sample_data.value = (start_val + i) & 0xFFFFFFFF
        await RisingEdge(dut.clk)
    dut.sample_valid.value = 0


async def wait_captured(axi, timeout=50):
    """Poll STATUS until captured flag is set."""
    for _ in range(timeout):
        status, _ = await axi.read(REG_STATUS)
        if status & STAT_CAPTURED:
            return True
    return False


async def read_snap_data(axi, addr):
    """Read one word from snap buffer (set RD_ADDR, wait, read RD_DATA).

    BRAM has 2-cycle read latency. We set the address, then do two
    reads of RD_DATA — the second gives us the correct value.
    """
    await axi.write(REG_RD_ADDR, addr)
    # Two reads to account for BRAM pipeline + AXI latency
    await axi.read(REG_RD_DATA)  # dummy read (pipeline fill)
    val, _ = await axi.read(REG_RD_DATA)
    return val


@cocotb.test()
async def test_arm_and_capture(dut):
    """Arm, feed samples + trigger, verify captured flag in STATUS."""
    axi = await setup(dut)

    # Arm
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Verify armed
    status, _ = await axi.read(REG_STATUS)
    assert status & STAT_ARMED, f"Not armed: STATUS=0x{status:08X}"

    # Feed pre-trigger samples
    await feed_samples(dut, 200, start_val=0x1000)

    # Software trigger
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)

    # Feed post-trigger samples
    await feed_samples(dut, POST_DEPTH, start_val=0x2000)

    # Check captured
    assert await wait_captured(axi), "Capture did not complete"


@cocotb.test()
async def test_sw_trigger(dut):
    """Software trigger (CONTROL[1]) fires capture without ext_trig."""
    axi = await setup(dut)

    # Arm
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Feed some pre-trigger data
    await feed_samples(dut, 50, start_val=0xAA)

    # SW trigger
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)

    # Feed post-trigger
    await feed_samples(dut, POST_DEPTH, start_val=0xBB)

    assert await wait_captured(axi), "SW trigger did not cause capture"


@cocotb.test()
async def test_readback(dut):
    """After capture, iterate RD_ADDR/RD_DATA to verify buffer contents."""
    axi = await setup(dut)

    # Arm
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Write known pattern
    num_samples = 20
    await feed_samples(dut, num_samples, start_val=0x100)

    # Trigger and fill post
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)
    await feed_samples(dut, POST_DEPTH, start_val=0x500)

    assert await wait_captured(axi), "Capture did not complete"

    # Read back first 10 pre-trigger samples
    for i in range(10):
        val = await read_snap_data(axi, i)
        expected = 0x100 + i
        assert val == expected, \
            f"Readback[{i}] = 0x{val:08X}, expected 0x{expected:08X}"


@cocotb.test()
async def test_rearm(dut):
    """After capture, write ARM again, verify new capture works."""
    axi = await setup(dut)

    # First capture
    await axi.write(REG_CONTROL, CTRL_ARM)
    await feed_samples(dut, 50, start_val=0x100)
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)
    await feed_samples(dut, POST_DEPTH, start_val=0x200)
    assert await wait_captured(axi), "First capture failed"

    # Rearm (write ARM without trigger — rearm_pulse fires)
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Verify state is reset (captured should be cleared)
    status, _ = await axi.read(REG_STATUS)
    assert not (status & STAT_CAPTURED), "captured not cleared after rearm"
    assert status & STAT_ARMED, "not armed after rearm"

    # Second capture with new data
    await feed_samples(dut, 30, start_val=0x300)
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_SW_TRIG)
    await feed_samples(dut, POST_DEPTH, start_val=0x400)
    assert await wait_captured(axi), "Second capture failed"

    # Verify new data in buffer
    val = await read_snap_data(axi, 0)
    assert val == 0x300, f"After rearm, BRAM[0] = 0x{val:08X}, expected 0x300"


@cocotb.test()
async def test_circular(dut):
    """Set circular_en, verify captured never asserts."""
    axi = await setup(dut)

    # Arm with circular mode
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_CIRCULAR)

    # Feed samples, trigger, feed more
    await feed_samples(dut, 100, start_val=0)
    await axi.write(REG_CONTROL, CTRL_ARM | CTRL_CIRCULAR | CTRL_SW_TRIG)
    await feed_samples(dut, POST_DEPTH + 100, start_val=0x1000)

    # Should NOT be captured
    status, _ = await axi.read(REG_STATUS)
    assert not (status & STAT_CAPTURED), \
        f"captured asserted in circular mode: STATUS=0x{status:08X}"


@cocotb.test()
async def test_ext_trigger(dut):
    """External trigger port fires capture when armed."""
    axi = await setup(dut)

    # Arm
    await axi.write(REG_CONTROL, CTRL_ARM)

    # Feed pre-trigger
    await feed_samples(dut, 50, start_val=0xEE)

    # Pulse ext_trig
    dut.ext_trig.value = 1
    await RisingEdge(dut.clk)
    dut.ext_trig.value = 0

    # Feed post-trigger
    await feed_samples(dut, POST_DEPTH, start_val=0xFF)

    assert await wait_captured(axi), "ext_trig did not cause capture"

    # Verify trig_pos in STATUS
    status, _ = await axi.read(REG_STATUS)
    trig_pos = (status >> 16) & 0x3FF
    assert trig_pos == 50, f"trig_pos = {trig_pos}, expected 50"
