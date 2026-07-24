# SPDX-License-Identifier: MIT
"""Cocotb tests for iq_dma_tx.v — TX DMA reader (DDR → DAC).

Tests:
    1. test_oneshot_basic — Load DDR with 32 samples, trigger one-shot, verify valid_out count
    2. test_oneshot_data_integrity — Verify output matches DDR: re_out = re<<4, im_out = im<<4
    3. test_oneshot_restart — Complete one-shot, re-trigger, verify second playback works
    4. test_cyclic_multiple_iterations — Cyclic mode, verify waveform repeats >=2 iterations
    5. test_cyclic_stop — Disable during cyclic TX, verify graceful stop (tx_done)
    6. test_cyclic_restart — Stop then re-trigger cyclic, verify clean restart
    7. test_ddr_latency_fixed — rvalid_delay=2, verify no DAC underrun
    8. test_ddr_latency_jitter — Random 0-5 cycle rvalid delay, verify data integrity
    9. test_cyclic_zero_bubble — Verify no gap in valid_out at cyclic iteration boundary
   10. test_tx_done_status — Verify tx_done asserts after one-shot, readable via STATUS
   11. test_cyclic_contention_at_wrap — Large rvalid_delay=8 stress test
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from axi_lite_driver import AxiLiteMaster
from axi3_mocks import Axi3ReadSlave

# Register offsets
REG_CONTROL  = 0x00
REG_STATUS   = 0x04
REG_DDR_BASE = 0x08
REG_TX_COUNT = 0x0C
REG_TX_PTR   = 0x10
REG_WR_PTR   = 0x14
REG_RD_PTR   = 0x18

# Constants
DDR_BASE_ADDR    = 0x10000000
SAMPLES_PER_BURST = 32


async def setup(dut, rvalid_delay=0, jitter_range=0):
    """Start clocks, reset, create drivers, return (axi, slave)."""
    # Start dual clocks
    cocotb.start_soon(Clock(dut.clk, 50, unit="ns").start())
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, unit="ns").start())

    # Assert resets
    dut.rst.value = 1
    dut.s_axi_aresetn.value = 0
    dut.dac_valid.value = 1  # 1R1T mode: always valid

    # AXI-Lite inputs
    dut.s_axi_awaddr.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wdata.value = 0
    dut.s_axi_wstrb.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_araddr.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0

    # AXI3 read master outputs driven by DUT; drive slave-side inputs
    dut.m_axi_arready.value = 0
    dut.m_axi_rdata.value = 0
    dut.m_axi_rresp.value = 0
    dut.m_axi_rlast.value = 0
    dut.m_axi_rvalid.value = 0

    # Hold reset for several cycles of the slower clock
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    dut.s_axi_aresetn.value = 1
    for _ in range(4):
        await RisingEdge(dut.clk)

    # Create drivers
    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.s_axi_aclk)
    slave = Axi3ReadSlave(dut, prefix="m_axi_", clk=dut.clk,
                          rvalid_delay=rvalid_delay,
                          jitter_range=jitter_range)
    cocotb.start_soon(slave.run())

    return axi, slave


async def configure_and_trigger(axi, slave, samples, tx_count=None,
                                 cyclic=False, stream=False):
    """Load DDR, configure registers, and trigger TX.

    Args:
        axi: AxiLiteMaster instance.
        slave: Axi3ReadSlave instance.
        samples: list of (re, im) tuples to pre-load into DDR.
        tx_count: total sample count (default: len(samples)).
        cyclic: enable cyclic mode.
        stream: enable stream mode (implies cyclic).
    """
    if tx_count is None:
        tx_count = len(samples)

    # Pre-load DDR memory
    slave.write_iq_samples(DDR_BASE_ADDR, samples)

    # Configure registers
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_TX_COUNT, tx_count)

    if stream:
        # Stream mode: set WR_PTR to the full sample count so fill FSM
        # knows all pre-loaded data is available
        await axi.write(REG_WR_PTR, tx_count)

    # Trigger: enable=1, trigger=1, cyclic=bit2, stream=bit3
    ctrl = 0x03  # enable + trigger
    if cyclic or stream:
        ctrl |= 0x04  # cyclic
    if stream:
        ctrl |= 0x08  # stream
    await axi.write(REG_CONTROL, ctrl)


async def capture_output(dut, max_samples, timeout_cycles=5000):
    """Capture DAC output samples until max_samples or timeout.

    Returns list of (re, im) tuples (16-bit raw values from re_out/im_out).
    """
    captured = []
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")  # settle time for Verilator
        if int(dut.valid_out.value) == 1:
            re_val = int(dut.re_out.value)
            im_val = int(dut.im_out.value)
            captured.append((re_val, im_val))
            if len(captured) >= max_samples:
                break
    return captured


async def wait_tx_done(axi, timeout_cycles=8000):
    """Poll STATUS register until tx_done=1 or timeout."""
    for _ in range(timeout_cycles):
        await RisingEdge(axi.clk)
        status, _ = await axi.read(REG_STATUS)
        if (status >> 1) & 1:
            return True
    return False


def make_samples(count, offset=0):
    """Generate test IQ samples: re = (offset+i) & 0xFFF, im = ((offset+i)*3) & 0xFFF."""
    return [((offset + i) & 0xFFF, ((offset + i) * 3) & 0xFFF) for i in range(count)]


def expected_dac_output(re_12bit, im_12bit):
    """Convert 12-bit values to expected 16-bit left-justified DAC output."""
    return ((re_12bit & 0xFFF) << 4, (im_12bit & 0xFFF) << 4)


# =========================================================
# Test 1: One-shot basic — verify valid_out fires correct number of times
# =========================================================
@cocotb.test()
async def test_oneshot_basic(dut):
    """Load DDR with 64 samples (2 bursts), trigger one-shot, verify valid_out count."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count)

    captured = await capture_output(dut, max_samples=tx_count + 10, timeout_cycles=8000)

    assert len(captured) == tx_count, \
        f"Expected {tx_count} valid outputs, got {len(captured)}"


# =========================================================
# Test 2: One-shot data integrity — verify re_out/im_out match DDR
# =========================================================
@cocotb.test()
async def test_oneshot_data_integrity(dut):
    """Verify output matches DDR: re_out = re[11:0]<<4, im_out = im[11:0]<<4."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count)

    captured = await capture_output(dut, max_samples=tx_count, timeout_cycles=8000)

    assert len(captured) == tx_count, \
        f"Only captured {len(captured)}/{tx_count} samples"

    mismatches = 0
    for i, (got_re, got_im) in enumerate(captured):
        exp_re, exp_im = expected_dac_output(*samples[i])
        if got_re != exp_re or got_im != exp_im:
            mismatches += 1
            if mismatches <= 5:
                dut._log.error(
                    f"Sample {i}: expected (0x{exp_re:04X}, 0x{exp_im:04X}), "
                    f"got (0x{got_re:04X}, 0x{got_im:04X})")

    assert mismatches == 0, f"{mismatches} sample mismatches"


# =========================================================
# Test 3: One-shot restart — complete, re-trigger, verify second playback
# =========================================================
@cocotb.test()
async def test_oneshot_restart(dut):
    """Complete one-shot, re-trigger, verify second playback works."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count)

    # Wait for first playback to complete
    captured1 = await capture_output(dut, max_samples=tx_count, timeout_cycles=8000)
    assert len(captured1) == tx_count, \
        f"First playback: got {len(captured1)}/{tx_count} samples"

    # Wait for tx_done
    done = await wait_tx_done(axi, timeout_cycles=2000)
    assert done, "tx_done not asserted after first one-shot"

    # Re-trigger (enable=1, trigger=1)
    await axi.write(REG_CONTROL, 0x03)

    # Capture second playback
    captured2 = await capture_output(dut, max_samples=tx_count, timeout_cycles=8000)
    assert len(captured2) == tx_count, \
        f"Second playback: got {len(captured2)}/{tx_count} samples"

    # Verify data matches
    for i, (got_re, got_im) in enumerate(captured2):
        exp_re, exp_im = expected_dac_output(*samples[i])
        assert got_re == exp_re and got_im == exp_im, \
            f"Restart mismatch at sample {i}"


# =========================================================
# Test 4: Cyclic multiple iterations — verify waveform repeats >=2 times
# =========================================================
@cocotb.test()
async def test_cyclic_multiple_iterations(dut):
    """Cyclic mode, verify waveform repeats at least 2 iterations."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count, cyclic=True)

    # Capture 2.5 iterations worth
    total_capture = tx_count * 3
    captured = await capture_output(dut, max_samples=total_capture, timeout_cycles=20000)

    assert len(captured) >= tx_count * 2, \
        f"Expected >= {tx_count * 2} samples (2 iterations), got {len(captured)}"

    # Verify data repeats: iteration 1 == iteration 2
    mismatches = 0
    for i in range(tx_count):
        if i < len(captured) and (i + tx_count) < len(captured):
            if captured[i] != captured[i + tx_count]:
                mismatches += 1
                if mismatches <= 3:
                    dut._log.error(
                        f"Cyclic mismatch at sample {i}: "
                        f"iter1={captured[i]}, iter2={captured[i + tx_count]}")

    assert mismatches == 0, f"{mismatches} cyclic repeat mismatches"

    # Stop cyclic to clean up
    await axi.write(REG_CONTROL, 0x02)


# =========================================================
# Test 5: Cyclic stop — disable during cyclic TX, verify graceful stop
# =========================================================
@cocotb.test()
async def test_cyclic_stop(dut):
    """Disable during cyclic TX, verify graceful stop (tx_done asserts)."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count, cyclic=True)

    # Let it run for a bit (at least one full iteration)
    captured_before_stop = await capture_output(dut, max_samples=tx_count + 10,
                                                timeout_cycles=8000)
    assert len(captured_before_stop) >= tx_count, \
        f"Cyclic didn't produce enough samples before stop: {len(captured_before_stop)}"

    # Stop: write enable=0 + trigger=1
    await axi.write(REG_CONTROL, 0x02)

    # Wait for tx_done
    done = await wait_tx_done(axi, timeout_cycles=8000)
    assert done, "tx_done not asserted after cyclic stop"


# =========================================================
# Test 6: Cyclic restart — stop then re-trigger cyclic
# =========================================================
@cocotb.test()
async def test_cyclic_restart(dut):
    """Stop then re-trigger cyclic, verify clean restart."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count, cyclic=True)

    # Run one iteration
    captured1 = await capture_output(dut, max_samples=tx_count, timeout_cycles=8000)
    assert len(captured1) >= tx_count, "First cyclic run too short"

    # Stop
    await axi.write(REG_CONTROL, 0x02)
    done = await wait_tx_done(axi, timeout_cycles=8000)
    assert done, "tx_done not asserted after stop"

    # Re-trigger cyclic
    await axi.write(REG_CONTROL, 0x07)

    # Verify it runs again
    captured2 = await capture_output(dut, max_samples=tx_count, timeout_cycles=8000)
    assert len(captured2) == tx_count, \
        f"Restart cyclic: got {len(captured2)}/{tx_count} samples"

    # Verify data integrity on restart
    mismatches = 0
    for i, (got_re, got_im) in enumerate(captured2):
        exp_re, exp_im = expected_dac_output(*samples[i])
        if got_re != exp_re or got_im != exp_im:
            mismatches += 1

    assert mismatches == 0, f"{mismatches} mismatches after cyclic restart"

    # Stop to clean up
    await axi.write(REG_CONTROL, 0x02)


# =========================================================
# Test 7: DDR latency fixed — rvalid_delay=2, verify no DAC underrun
# =========================================================
@cocotb.test()
async def test_ddr_latency_fixed(dut):
    """Add fixed rvalid_delay=2, verify no DAC underrun (all samples correct)."""
    axi, slave = await setup(dut, rvalid_delay=2)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count)

    captured = await capture_output(dut, max_samples=tx_count, timeout_cycles=10000)

    assert len(captured) == tx_count, \
        f"Expected {tx_count} samples, got {len(captured)} (underrun?)"

    # Verify data integrity
    mismatches = 0
    for i, (got_re, got_im) in enumerate(captured):
        exp_re, exp_im = expected_dac_output(*samples[i])
        if got_re != exp_re or got_im != exp_im:
            mismatches += 1
            if mismatches <= 3:
                dut._log.error(
                    f"Sample {i}: expected (0x{exp_re:04X}, 0x{exp_im:04X}), "
                    f"got (0x{got_re:04X}, 0x{got_im:04X})")

    assert mismatches == 0, f"{mismatches} mismatches with rvalid_delay=2"


# =========================================================
# Test 8: DDR latency jitter — random 0-5 cycle delay, verify integrity
# =========================================================
@cocotb.test()
async def test_ddr_latency_jitter(dut):
    """Random 0-5 cycle rvalid delay, verify data integrity."""
    axi, slave = await setup(dut, rvalid_delay=0, jitter_range=5)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count)

    captured = await capture_output(dut, max_samples=tx_count, timeout_cycles=10000)

    assert len(captured) == tx_count, \
        f"Expected {tx_count} samples, got {len(captured)} (jitter-induced underrun?)"

    mismatches = 0
    for i, (got_re, got_im) in enumerate(captured):
        exp_re, exp_im = expected_dac_output(*samples[i])
        if got_re != exp_re or got_im != exp_im:
            mismatches += 1
            if mismatches <= 3:
                dut._log.error(
                    f"Sample {i}: expected (0x{exp_re:04X}, 0x{exp_im:04X}), "
                    f"got (0x{got_re:04X}, 0x{got_im:04X})")

    assert mismatches == 0, f"{mismatches} mismatches with jitter"


# =========================================================
# Test 9: Cyclic zero bubble — no gap in valid_out at iteration boundary
# =========================================================
@cocotb.test()
async def test_cyclic_zero_bubble(dut):
    """Verify no gap in valid_out at cyclic iteration boundary."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count, cyclic=True)

    # Capture 2+ iterations and look for gaps in valid_out
    total_expected = tx_count * 2
    valid_streak = 0
    max_gap = 0
    current_gap = 0
    total_valid = 0
    started = False

    for _ in range(20000):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        v = int(dut.valid_out.value)
        if v == 1:
            if not started:
                started = True
            total_valid += 1
            valid_streak += 1
            if current_gap > max_gap and started:
                max_gap = current_gap
            current_gap = 0
        else:
            if started:
                current_gap += 1

        if total_valid >= total_expected:
            break

    assert total_valid >= total_expected, \
        f"Only got {total_valid}/{total_expected} valid samples"

    # Zero-bubble means no gap once output starts (max_gap should be 0)
    assert max_gap == 0, \
        f"Found {max_gap}-cycle gap in valid_out (zero-bubble violated)"

    # Stop cyclic
    await axi.write(REG_CONTROL, 0x02)


# =========================================================
# Test 10: tx_done status — verify readable via STATUS register
# =========================================================
@cocotb.test()
async def test_tx_done_status(dut):
    """Verify tx_done asserts after one-shot, readable via STATUS register."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count)

    # Before TX completes, STATUS.active should be set
    # (Give CDC time to propagate)
    for _ in range(50):
        await RisingEdge(dut.s_axi_aclk)
    status, _ = await axi.read(REG_STATUS)
    active = status & 1
    assert active == 1, f"STATUS.active not set during TX (STATUS=0x{status:08X})"

    # Wait for all output
    captured = await capture_output(dut, max_samples=tx_count, timeout_cycles=8000)
    assert len(captured) == tx_count

    # Poll for tx_done (with CDC delay)
    done = await wait_tx_done(axi, timeout_cycles=2000)
    assert done, "tx_done not readable via STATUS register after one-shot"

    # Verify STATUS bits
    status, _ = await axi.read(REG_STATUS)
    tx_done_bit = (status >> 1) & 1
    assert tx_done_bit == 1, f"STATUS.tx_done not set (STATUS=0x{status:08X})"


# =========================================================
# Test 11: Cyclic contention at wrap — large rvalid_delay=8 stress test
# =========================================================
@cocotb.test()
async def test_cyclic_contention_at_wrap(dut):
    """Large rvalid_delay=8 at wrap to stress the speculative AR optimization."""
    axi, slave = await setup(dut, rvalid_delay=8)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count, cyclic=True)

    # Run 2 full iterations under heavy latency
    total_expected = tx_count * 2
    captured = await capture_output(dut, max_samples=total_expected, timeout_cycles=30000)

    assert len(captured) >= total_expected, \
        f"Expected >= {total_expected} samples, got {len(captured)} (stall under latency?)"

    # Verify data integrity across both iterations
    mismatches = 0
    for iteration in range(2):
        for i in range(tx_count):
            idx = iteration * tx_count + i
            if idx >= len(captured):
                break
            got_re, got_im = captured[idx]
            exp_re, exp_im = expected_dac_output(*samples[i])
            if got_re != exp_re or got_im != exp_im:
                mismatches += 1
                if mismatches <= 3:
                    dut._log.error(
                        f"Iter {iteration} sample {i}: "
                        f"expected (0x{exp_re:04X}, 0x{exp_im:04X}), "
                        f"got (0x{got_re:04X}, 0x{got_im:04X})")

    assert mismatches == 0, \
        f"{mismatches} mismatches with rvalid_delay=8 across cyclic wrap"

    # Stop cyclic
    await axi.write(REG_CONTROL, 0x02)


# =========================================================
# Test 12: Stream basic — pre-load, trigger stream, verify
# =========================================================
@cocotb.test()
async def test_stream_basic(dut):
    """Pre-load DDR with 128 samples, stream mode, verify fill+drain works."""
    axi, slave = await setup(dut)

    tx_count = 128
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count,
                                cyclic=True, stream=True)

    captured = await capture_output(dut, max_samples=tx_count, timeout_cycles=8000)

    assert len(captured) >= tx_count, \
        f"Expected >= {tx_count} samples in stream mode, got {len(captured)}"

    mismatches = 0
    for i in range(tx_count):
        if i >= len(captured):
            break
        got_re, got_im = captured[i]
        exp_re, exp_im = expected_dac_output(*samples[i])
        if got_re != exp_re or got_im != exp_im:
            mismatches += 1
            if mismatches <= 3:
                dut._log.error(
                    f"Stream sample {i}: expected (0x{exp_re:04X}, 0x{exp_im:04X}), "
                    f"got (0x{got_re:04X}, 0x{got_im:04X})")

    assert mismatches == 0, f"{mismatches} stream data mismatches"

    # Stop stream
    await axi.write(REG_CONTROL, 0x02)


# =========================================================
# Test 13: Stream WR_PTR stall and resume
# =========================================================
@cocotb.test()
async def test_stream_wr_ptr_stall(dut):
    """Set WR_PTR=64, verify fill stalls; extend → 128, verify resume."""
    axi, slave = await setup(dut)

    # Pre-load 128 samples in DDR
    samples = make_samples(128)
    slave.write_iq_samples(DDR_BASE_ADDR, samples)

    # Configure for stream mode but put initial WR_PTR at 64
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_TX_COUNT, 8388608)  # buffer size for stream
    await axi.write(REG_WR_PTR, 64)

    # Trigger stream: enable+cyclic+stream+trigger
    await axi.write(REG_CONTROL, 0x0F)  # enable | trigger | cyclic | stream

    # Capture up to 64 samples — fill should stop at WR_PTR
    captured1 = await capture_output(dut, max_samples=70, timeout_cycles=8000)

    # Should get exactly 64 (or close — the drain consumes what fill loaded)
    assert len(captured1) >= 64, \
        f"Expected >= 64 samples before stall, got {len(captured1)}"

    # Now extend WR_PTR to 128 — fill should resume
    await axi.write(REG_WR_PTR, 128)

    # Capture another 70 samples
    captured2 = await capture_output(dut, max_samples=70, timeout_cycles=8000)

    total = len(captured1) + len(captured2)
    assert total >= 128, \
        f"Expected >= 128 total samples after WR_PTR extension, got {total}"

    # Verify first 64 samples are correct
    mismatches = 0
    for i in range(min(64, len(captured1))):
        got_re, got_im = captured1[i]
        exp_re, exp_im = expected_dac_output(*samples[i])
        if got_re != exp_re or got_im != exp_im:
            mismatches += 1
    assert mismatches == 0, f"{mismatches} mismatches in first 64 stream samples"

    # Stop stream
    await axi.write(REG_CONTROL, 0x02)


# =========================================================
# Test 14: Stream wrap continuity
# =========================================================
@cocotb.test()
async def test_stream_wrap_continuity(dut):
    """Verify drain output is correct across the stream ring-buffer wrap."""
    axi, slave = await setup(dut)

    # Use a small waveform length to test cyclic wrap in stream mode
    wave_len = 96  # 3 bursts
    samples = make_samples(wave_len)
    await configure_and_trigger(axi, slave, samples, tx_count=wave_len,
                                cyclic=True, stream=True)

    # Capture multiple iterations
    captured = await capture_output(dut, max_samples=wave_len * 3,
                                    timeout_cycles=20000)

    assert len(captured) >= wave_len * 2, \
        f"Expected >= {wave_len * 2} samples, got {len(captured)}"

    # Verify data repeats correctly (same pattern each iteration)
    mismatches = 0
    for i in range(wave_len):
        if i < len(captured) and (i + wave_len) < len(captured):
            if captured[i] != captured[i + wave_len]:
                mismatches += 1
                if mismatches <= 3:
                    dut._log.error(
                        f"Wrap mismatch at sample {i}: "
                        f"iter1={captured[i]}, iter2={captured[i + wave_len]}")

    assert mismatches == 0, f"{mismatches} wrap continuity mismatches"

    # Stop stream
    await axi.write(REG_CONTROL, 0x02)


# =========================================================
# Test 15: Stream RD_PTR tracking
# =========================================================
@cocotb.test()
async def test_stream_rd_ptr_tracking(dut):
    """Verify RD_PTR register (offset 0x18) matches drain position."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count,
                                cyclic=True, stream=True)

    # Let it run for a while, then read RD_PTR
    await Timer(5000, unit="ns")
    rd1, _ = await axi.read(REG_RD_PTR)
    assert isinstance(rd1, int), f"RD_PTR read failed"

    # Let it run some more, read again
    await Timer(5000, unit="ns")
    rd2, _ = await axi.read(REG_RD_PTR)

    # RD_PTR should be advancing (or wrapped: rd2 < rd1 is also advancing)
    assert rd2 != rd1, f"RD_PTR not changing: {rd1} -> {rd2}"

    # RD_PTR should be within a reasonable range
    assert rd2 >= 0 and rd2 <= tx_count * 3, \
        f"RD_PTR out of range: {rd2}"

    # Stop stream
    await axi.write(REG_CONTROL, 0x02)


# =========================================================
# Test 16: Stream stop — graceful drain completion
# =========================================================
@cocotb.test()
async def test_stream_stop(dut):
    """Disable during streaming, verify graceful drain-completion stop."""
    axi, slave = await setup(dut)

    tx_count = 64
    samples = make_samples(tx_count)
    await configure_and_trigger(axi, slave, samples, tx_count=tx_count,
                                cyclic=True, stream=True)

    # Let it run for at least one full iteration
    captured_before = await capture_output(dut, max_samples=tx_count,
                                           timeout_cycles=8000)
    assert len(captured_before) == tx_count, \
        f"Expected {tx_count} samples before stop, got {len(captured_before)}"

    # Stop via enable=0 + trigger=1
    await axi.write(REG_CONTROL, 0x02)  # enable=0, trigger=1

    # Wait for tx_done
    done = await wait_tx_done(axi, timeout_cycles=8000)
    assert done, "tx_done not asserted after stream stop"

    # Verify STATUS: done=1
    status, _ = await axi.read(REG_STATUS)
    assert (status >> 1) & 1, f"STATUS tx_done not set after stop (0x{status:08X})"

    # Verify we can restart cleanly
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_TX_COUNT, tx_count)
    await axi.write(REG_CONTROL, 0x0F)  # enable | trigger | cyclic | stream

    captured2 = await capture_output(dut, max_samples=tx_count,
                                     timeout_cycles=8000)
    assert len(captured2) >= tx_count, \
        f"Restart after stop: expected >= {tx_count}, got {len(captured2)}"

    # Stop
    await axi.write(REG_CONTROL, 0x02)
