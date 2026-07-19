# SPDX-License-Identifier: MIT
"""Cocotb tests for debug_snap.v — parametric debug snapshot BRAM.

Tests:
    1. Basic capture — writes + trigger + post-trigger, then readback
    2. Trigger freezes — buffer stops accepting after POST_DEPTH
    3. Trigger position — trig_wr reports correct write address
    4. Rearm — after capture, rearm resets state
    5. Wrap around — write >DEPTH pre-trigger, circular buffer
    6. wr_count saturates — stops at DEPTH
    7. Post-trigger exact — exactly POST_DEPTH then captured=1
    8. Circular never captures — circular_en=1, captured never asserts
    9. Circular rearm — rearm in circular mode resets all
   10. Wide data — full 32-bit values survive BRAM cycle
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

# Module parameters (matching defaults)
DEPTH = 1024
POST_DEPTH = 512


async def setup(dut):
    """Start clock, reset, return after ready."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.rst.value = 1
    dut.sample_valid.value = 0
    dut.sample_data.value = 0
    dut.trig.value = 0
    dut.rearm.value = 0
    dut.circular_en.value = 0
    dut.rd_addr.value = 0

    for _ in range(5):
        await RisingEdge(dut.clk)

    dut.rst.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def write_sample(dut, data):
    """Write one sample."""
    dut.sample_valid.value = 1
    dut.sample_data.value = data
    await RisingEdge(dut.clk)
    dut.sample_valid.value = 0


async def write_n_samples(dut, count, start_val=0):
    """Write count samples with incrementing data starting at start_val."""
    for i in range(count):
        dut.sample_valid.value = 1
        dut.sample_data.value = (start_val + i) & 0xFFFFFFFF
        await RisingEdge(dut.clk)
    dut.sample_valid.value = 0


async def pulse_trig(dut):
    """Assert trigger for one cycle."""
    dut.trig.value = 1
    await RisingEdge(dut.clk)
    dut.trig.value = 0


async def pulse_rearm(dut):
    """Assert rearm for one cycle."""
    dut.rearm.value = 1
    await RisingEdge(dut.clk)
    dut.rearm.value = 0


async def read_bram(dut, addr):
    """Read BRAM at addr. Takes 2 cycles due to pipeline."""
    dut.rd_addr.value = addr
    await RisingEdge(dut.clk)  # addr registered
    await RisingEdge(dut.clk)  # data registered
    await Timer(1, unit="ns")  # settle
    return int(dut.rd_data.value)


@cocotb.test()
async def test_basic_capture(dut):
    """Write samples, trigger, POST_DEPTH more samples, verify readback."""
    await setup(dut)

    # Write 200 pre-trigger samples
    await write_n_samples(dut, 200, start_val=0x1000)

    # Trigger
    await pulse_trig(dut)

    # Write POST_DEPTH samples (post-trigger region)
    await write_n_samples(dut, POST_DEPTH, start_val=0x2000)

    # Wait for captured
    for _ in range(5):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.captured.value) == 1, "captured not asserted"

    # Readback: first sample written should be at address 0
    val = await read_bram(dut, 0)
    assert val == 0x1000, f"BRAM[0] = 0x{val:08X}, expected 0x00001000"


@cocotb.test()
async def test_trigger_freezes(dut):
    """After trigger + POST_DEPTH samples, buffer stops accepting writes."""
    await setup(dut)

    # Write 10 samples
    await write_n_samples(dut, 10, start_val=0xAA)

    # Trigger
    await pulse_trig(dut)

    # Write exactly POST_DEPTH samples
    await write_n_samples(dut, POST_DEPTH, start_val=0xBB)

    # Wait for captured
    for _ in range(5):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.captured.value) == 1

    # Remember last written value at a known address
    val_before = await read_bram(dut, 10)

    # Try to write more — should be rejected
    await write_n_samples(dut, 10, start_val=0xFFFF)

    # Verify data unchanged
    val_after = await read_bram(dut, 10)
    assert val_before == val_after, \
        f"Buffer accepted writes after capture: 0x{val_before:08X} -> 0x{val_after:08X}"


@cocotb.test()
async def test_trigger_position(dut):
    """trig_wr reports correct write address at trigger time."""
    await setup(dut)

    # Write exactly 50 samples (wr_addr will be 50 when trigger fires)
    await write_n_samples(dut, 50, start_val=1)

    # Trigger
    await pulse_trig(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")

    trig_pos = int(dut.trig_wr.value)
    assert trig_pos == 50, f"trig_wr = {trig_pos}, expected 50"


@cocotb.test()
async def test_rearm(dut):
    """After capture, rearm resets state, allows new capture."""
    await setup(dut)

    # First capture
    await write_n_samples(dut, 10, start_val=0x100)
    await pulse_trig(dut)
    await write_n_samples(dut, POST_DEPTH, start_val=0x200)

    for _ in range(5):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.captured.value) == 1

    # Rearm
    await pulse_rearm(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")

    # State should be reset
    assert int(dut.captured.value) == 0, "captured not cleared after rearm"
    assert int(dut.wr_count.value) == 0, "wr_count not cleared after rearm"

    # Second capture should work
    await write_n_samples(dut, 5, start_val=0x300)
    await pulse_trig(dut)
    await write_n_samples(dut, POST_DEPTH, start_val=0x400)

    for _ in range(5):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.captured.value) == 1, "Second capture failed"

    # Verify new data
    val = await read_bram(dut, 0)
    assert val == 0x300, f"After rearm, BRAM[0] = 0x{val:08X}, expected 0x00000300"


@cocotb.test()
async def test_wrap_around(dut):
    """Write >DEPTH pre-trigger, verify circular buffer overwrites."""
    await setup(dut)

    # Write DEPTH + 100 samples — should wrap
    await write_n_samples(dut, DEPTH + 100, start_val=0)

    # Trigger
    await pulse_trig(dut)
    await write_n_samples(dut, POST_DEPTH, start_val=0x5000)

    for _ in range(5):
        await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.captured.value) == 1

    # The first 100 addresses should contain the wrapped data (values 1024..1123)
    val = await read_bram(dut, 0)
    assert val == DEPTH, \
        f"BRAM[0] after wrap = 0x{val:08X}, expected 0x{DEPTH:08X} (sample #{DEPTH})"


@cocotb.test()
async def test_wr_count_saturates(dut):
    """wr_count stops at DEPTH, doesn't overflow."""
    await setup(dut)

    # Write more than DEPTH samples
    await write_n_samples(dut, DEPTH + 50, start_val=0)

    await Timer(1, unit="ns")
    count = int(dut.wr_count.value)
    assert count == DEPTH, f"wr_count = {count}, expected {DEPTH} (saturated)"


@cocotb.test()
async def test_post_trigger_exact(dut):
    """Exactly POST_DEPTH samples after trigger, then captured=1."""
    await setup(dut)

    # Write some pre-trigger
    await write_n_samples(dut, 10, start_val=0)

    # Trigger
    await pulse_trig(dut)

    # Write POST_DEPTH - 1 samples — should NOT be captured yet
    await write_n_samples(dut, POST_DEPTH - 1, start_val=0x100)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.captured.value) == 0, "captured too early"

    # Write one more — NOW it should capture
    await write_sample(dut, 0xF1A1)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.captured.value) == 1, "captured not asserted at exact POST_DEPTH"


@cocotb.test()
async def test_circular_never_captures(dut):
    """circular_en=1: captured never asserts despite trigger."""
    await setup(dut)

    dut.circular_en.value = 1

    # Write, trigger, write more
    await write_n_samples(dut, 100, start_val=0)
    await pulse_trig(dut)
    await write_n_samples(dut, POST_DEPTH + 100, start_val=0x1000)

    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.captured.value) == 0, "captured asserted in circular mode"


@cocotb.test()
async def test_circular_rearm(dut):
    """Rearm in circular mode resets all state."""
    await setup(dut)

    dut.circular_en.value = 1

    # Write and trigger
    await write_n_samples(dut, 100, start_val=0)
    await pulse_trig(dut)
    await write_n_samples(dut, 50, start_val=0x1000)

    # Rearm
    await pulse_rearm(dut)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")

    assert int(dut.wr_count.value) == 0, "wr_count not cleared"
    assert int(dut.trig_wr.value) == 0, "trig_wr not cleared"
    assert int(dut.captured.value) == 0, "captured not cleared"


@cocotb.test()
async def test_wide_data(dut):
    """Full 32-bit data values survive BRAM write/read cycle."""
    await setup(dut)

    # Test with values that exercise all 32 bits
    test_values = [
        0x00000000,
        0xFFFFFFFF,
        0xAAAAAAAA,
        0x55555555,
        0xDEADBEEF,
        0x12345678,
        0x80000001,
        0x7FFFFFFF,
    ]

    for val in test_values:
        await write_sample(dut, val)

    # Trigger and fill post to freeze
    await pulse_trig(dut)
    await write_n_samples(dut, POST_DEPTH, start_val=0)

    for _ in range(5):
        await RisingEdge(dut.clk)

    # Read back and verify all values
    for i, expected in enumerate(test_values):
        got = await read_bram(dut, i)
        assert got == expected, \
            f"BRAM[{i}] = 0x{got:08X}, expected 0x{expected:08X}"
