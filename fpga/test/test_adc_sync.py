# SPDX-License-Identifier: MIT
"""Cocotb tests for adc_sync.v — ADC clock domain crossing (l_clk -> sys_clk).

Tests:
    1. Basic crossing — one sample arrives on output
    2. Continuous stream — 100 samples, all correct
    3. Gapped input — intermittent valid_in, no data loss
    4. Sign extension — negative values cross correctly
    5. Reset recovery — sys_rst mid-stream, clean recovery
    6. High throughput — 1000 back-to-back samples, no drops
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import random

# Clock periods: adc_clk=16.27ns (61.44 MHz), sys_clk=10ns (100 MHz)
ADC_CLK_NS = 16.27
SYS_CLK_NS = 10.0


def to_unsigned_12(val):
    """Convert signed 12-bit to unsigned for DUT input."""
    return val & 0xFFF


def to_signed_12(val):
    """Convert unsigned 12-bit to signed."""
    val = val & 0xFFF
    if val >= 0x800:
        val -= 0x1000
    return val


async def setup(dut):
    """Start clocks, reset, return after ready."""
    cocotb.start_soon(Clock(dut.adc_clk, ADC_CLK_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.sys_clk, SYS_CLK_NS, unit="ns").start())

    # Reset both domains
    dut.adc_rst.value = 1
    dut.sys_rst.value = 1
    dut.valid_in.value = 0
    dut.re_in.value = 0
    dut.im_in.value = 0

    for _ in range(5):
        await RisingEdge(dut.adc_clk)
    for _ in range(5):
        await RisingEdge(dut.sys_clk)

    dut.adc_rst.value = 0
    dut.sys_rst.value = 0

    # Wait for any internal state to settle
    for _ in range(4):
        await RisingEdge(dut.sys_clk)


async def push_sample(dut, re_val, im_val):
    """Push one IQ sample on adc_clk edge."""
    dut.valid_in.value = 1
    dut.re_in.value = to_unsigned_12(re_val)
    dut.im_in.value = to_unsigned_12(im_val)
    await RisingEdge(dut.adc_clk)
    dut.valid_in.value = 0


async def collect_outputs(dut, count, timeout_cycles=500):
    """Collect N output samples from sys_clk domain.

    Returns list of (re, im) tuples (signed 12-bit).
    """
    results = []
    cycles = 0
    while len(results) < count:
        await RisingEdge(dut.sys_clk)
        await Timer(1, unit="ns")  # settle combinational outputs
        if int(dut.valid_out.value):
            re = to_signed_12(int(dut.re_out.value))
            im = to_signed_12(int(dut.im_out.value))
            results.append((re, im))
        cycles += 1
        if cycles >= timeout_cycles:
            break
    return results


@cocotb.test()
async def test_basic_crossing(dut):
    """Push one sample on adc_clk, verify it arrives on sys_clk."""
    await setup(dut)

    await push_sample(dut, 100, -200)

    results = await collect_outputs(dut, 1)
    assert len(results) == 1, f"Expected 1 output, got {len(results)}"
    re, im = results[0]
    assert re == 100, f"re: expected 100, got {re}"
    assert im == -200, f"im: expected -200, got {im}"


@cocotb.test()
async def test_continuous_stream(dut):
    """Stream 100 samples, verify all arrive in order with correct values."""
    await setup(dut)

    num_samples = 100
    rng = random.Random(42)
    input_samples = [(rng.randint(-2000, 2000), rng.randint(-2000, 2000))
                     for _ in range(num_samples)]

    # Feed all samples
    async def feeder():
        for re, im in input_samples:
            dut.valid_in.value = 1
            dut.re_in.value = to_unsigned_12(re)
            dut.im_in.value = to_unsigned_12(im)
            await RisingEdge(dut.adc_clk)
        dut.valid_in.value = 0

    cocotb.start_soon(feeder())

    results = await collect_outputs(dut, num_samples, timeout_cycles=num_samples * 5)

    assert len(results) == num_samples, \
        f"Expected {num_samples} outputs, got {len(results)}"
    for i, ((exp_re, exp_im), (got_re, got_im)) in enumerate(zip(input_samples, results)):
        assert got_re == exp_re, f"Sample {i} re: expected {exp_re}, got {got_re}"
        assert got_im == exp_im, f"Sample {i} im: expected {exp_im}, got {got_im}"


@cocotb.test()
async def test_gapped_input(dut):
    """Intermittent valid_in (every 5th cycle), verify no data loss."""
    await setup(dut)

    num_samples = 30
    input_samples = [(i * 10, -(i * 10)) for i in range(num_samples)]

    # Feed with gaps
    async def feeder():
        for re, im in input_samples:
            # Wait 4 cycles (gap)
            for _ in range(4):
                await RisingEdge(dut.adc_clk)
            # Then push
            dut.valid_in.value = 1
            dut.re_in.value = to_unsigned_12(re)
            dut.im_in.value = to_unsigned_12(im)
            await RisingEdge(dut.adc_clk)
            dut.valid_in.value = 0

    cocotb.start_soon(feeder())

    results = await collect_outputs(dut, num_samples, timeout_cycles=num_samples * 20)

    assert len(results) == num_samples, \
        f"Expected {num_samples} outputs, got {len(results)}"
    for i, ((exp_re, exp_im), (got_re, got_im)) in enumerate(zip(input_samples, results)):
        assert got_re == exp_re, f"Sample {i} re: expected {exp_re}, got {got_re}"
        assert got_im == exp_im, f"Sample {i} im: expected {exp_im}, got {got_im}"


@cocotb.test()
async def test_sign_extension(dut):
    """Negative values (0x800 = -2048) cross correctly."""
    await setup(dut)

    # Test boundary values
    test_values = [
        (-2048, -2048),  # most negative
        (-1, -1),        # all ones
        (2047, 2047),    # most positive
        (0, 0),          # zero
        (-1024, 1024),   # mixed signs
    ]

    async def feeder():
        for re, im in test_values:
            dut.valid_in.value = 1
            dut.re_in.value = to_unsigned_12(re)
            dut.im_in.value = to_unsigned_12(im)
            await RisingEdge(dut.adc_clk)
        dut.valid_in.value = 0

    cocotb.start_soon(feeder())

    results = await collect_outputs(dut, len(test_values))

    assert len(results) == len(test_values), \
        f"Expected {len(test_values)} outputs, got {len(results)}"
    for i, ((exp_re, exp_im), (got_re, got_im)) in enumerate(zip(test_values, results)):
        assert got_re == exp_re, \
            f"Sample {i} re: expected {exp_re}, got {got_re}"
        assert got_im == exp_im, \
            f"Sample {i} im: expected {exp_im}, got {got_im}"


@cocotb.test()
async def test_reset_recovery(dut):
    """Assert both resets mid-stream, verify clean recovery after deassert."""
    await setup(dut)

    # Push some samples
    async def feeder_pre():
        for i in range(5):
            dut.valid_in.value = 1
            dut.re_in.value = to_unsigned_12(i * 100)
            dut.im_in.value = to_unsigned_12(i * 100)
            await RisingEdge(dut.adc_clk)
        dut.valid_in.value = 0

    cocotb.start_soon(feeder_pre())

    # Wait for some outputs
    await collect_outputs(dut, 3, timeout_cycles=50)

    # Assert both resets (must reset both domains to fully clear FIFO)
    dut.sys_rst.value = 1
    dut.adc_rst.value = 1
    for _ in range(5):
        await RisingEdge(dut.sys_clk)
    for _ in range(5):
        await RisingEdge(dut.adc_clk)

    # Verify outputs are suppressed during reset
    await RisingEdge(dut.sys_clk)
    await Timer(1, unit="ns")
    assert not int(dut.valid_out.value), "valid_out should be 0 during reset"

    # Deassert resets
    dut.sys_rst.value = 0
    dut.adc_rst.value = 0
    for _ in range(6):
        await RisingEdge(dut.sys_clk)
    for _ in range(4):
        await RisingEdge(dut.adc_clk)

    # Push new samples — they should arrive correctly
    test_re, test_im = 500, -500

    async def feeder_post():
        dut.valid_in.value = 1
        dut.re_in.value = to_unsigned_12(test_re)
        dut.im_in.value = to_unsigned_12(test_im)
        await RisingEdge(dut.adc_clk)
        dut.valid_in.value = 0

    cocotb.start_soon(feeder_post())

    results = await collect_outputs(dut, 1, timeout_cycles=50)
    assert len(results) >= 1, "No output after reset recovery"
    re, im = results[0]
    assert re == test_re, f"Post-reset re: expected {test_re}, got {re}"
    assert im == test_im, f"Post-reset im: expected {test_im}, got {im}"


@cocotb.test()
async def test_high_throughput(dut):
    """Back-to-back valid_in for 1000 samples, no drops."""
    await setup(dut)

    num_samples = 1000
    rng = random.Random(99)
    input_samples = [(rng.randint(-2000, 2000), rng.randint(-2000, 2000))
                     for _ in range(num_samples)]

    async def feeder():
        for re, im in input_samples:
            dut.valid_in.value = 1
            dut.re_in.value = to_unsigned_12(re)
            dut.im_in.value = to_unsigned_12(im)
            await RisingEdge(dut.adc_clk)
        dut.valid_in.value = 0

    cocotb.start_soon(feeder())

    results = await collect_outputs(dut, num_samples, timeout_cycles=num_samples * 5)

    assert len(results) == num_samples, \
        f"Dropped samples: expected {num_samples}, got {len(results)}"

    mismatches = 0
    for i, ((exp_re, exp_im), (got_re, got_im)) in enumerate(zip(input_samples, results)):
        if got_re != exp_re or got_im != exp_im:
            mismatches += 1
            if mismatches <= 5:
                dut._log.error(
                    f"Mismatch at {i}: expected ({exp_re},{exp_im}), "
                    f"got ({got_re},{got_im})")
    assert mismatches == 0, f"{mismatches} sample mismatches (first 5 logged above)"
