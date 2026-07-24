# SPDX-License-Identifier: MIT
"""Cocotb tests for tb_iq_dma_loopback — TX→RX loopback co-simulation.

Instantiaties iq_dma_tx and iq_dma_rx sharing an AXI3 mock DDR.
TX DAC output feeds RX ADC input directly (12-bit truncation).
Both modules compete for AXI bandwidth through Axi3SharedSlave.

Tests:
    1. test_loopback_basic        — TX one-shot, RX captures, verify data
    2. test_loopback_sustained    — TX cyclic, RX captures continuously
    3. test_loopback_dma_stall    — Verify RX sample_count never freezes
    4. test_loopback_contention   — max_outstanding=2 stress, verify no RX overflow
    5. test_loopback_restart      — Stop TX mid-cyclic, restart, verify RX clean
    6. test_loopback_ddr_integrity — Bit-perfect roundtrip: TX DDR → DAC → ADC → RX DDR
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from axi_lite_driver import AxiLiteMaster
from axi3_mocks import Axi3SharedSlave

# --- TX register offsets (iq_dma_tx.v) ---
TX_REG_CONTROL  = 0x00
TX_REG_STATUS   = 0x04
TX_REG_DDR_BASE = 0x08
TX_REG_TX_COUNT = 0x0C
TX_REG_TX_PTR   = 0x10
TX_REG_WR_PTR   = 0x14
TX_REG_RD_PTR   = 0x18

# --- RX register offsets (iq_dma_rx.v) ---
RX_REG_CONTROL      = 0x00
RX_REG_STATUS       = 0x04
RX_REG_DDR_BASE     = 0x08
RX_REG_WR_PTR       = 0x0C
RX_REG_SAMPLE_COUNT = 0x10
RX_REG_WRAP_COUNT   = 0x14

# --- DDR regions (non-overlapping) ---
TX_DDR_BASE = 0x10000000
RX_DDR_BASE = 0x20000000

# --- Constants ---
SAMPLES_PER_BURST = 32
VALID_RATIO = 5       # matches Verilog parameter -GVALID_RATIO


async def setup(dut, max_outstanding=2):
    """Start clocks, reset, create drivers, return (axi_tx, axi_rx, slave)."""
    # Single clock for both domains (TX_CLK_SAME_AS_RX=1)
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    cocotb.start_soon(Clock(dut.s_axi_tx_aclk, 10, unit="ns").start())

    # Tie tx_clk to clk (unused in single-clock mode but still driven)
    dut.tx_clk.value = 0

    # Assert resets
    dut.rst.value = 1
    dut.s_axi_tx_aresetn.value = 0

    # AXI-Lite inputs — tie to inactive
    dut.s_axi_tx_awaddr.value = 0
    dut.s_axi_tx_awvalid.value = 0
    dut.s_axi_tx_wdata.value = 0
    dut.s_axi_tx_wstrb.value = 0
    dut.s_axi_tx_wvalid.value = 0
    dut.s_axi_tx_bready.value = 0
    dut.s_axi_tx_araddr.value = 0
    dut.s_axi_tx_arvalid.value = 0
    dut.s_axi_tx_rready.value = 0

    dut.s_axi_rx_awaddr.value = 0
    dut.s_axi_rx_awvalid.value = 0
    dut.s_axi_rx_wdata.value = 0
    dut.s_axi_rx_wstrb.value = 0
    dut.s_axi_rx_wvalid.value = 0
    dut.s_axi_rx_bready.value = 0
    dut.s_axi_rx_araddr.value = 0
    dut.s_axi_rx_arvalid.value = 0
    dut.s_axi_rx_rready.value = 0

    # Hold reset
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    dut.s_axi_tx_aresetn.value = 1
    for _ in range(4):
        await RisingEdge(dut.clk)

    axi_tx = AxiLiteMaster(dut, prefix="s_axi_tx_", clk=dut.s_axi_tx_aclk)
    axi_rx = AxiLiteMaster(dut, prefix="s_axi_rx_", clk=dut.clk)
    slave = Axi3SharedSlave(dut, read_prefix="m_axi_tx_",
                            write_prefix="m_axi_rx_", clk=dut.clk,
                            max_outstanding=max_outstanding)
    cocotb.start_soon(slave.run())

    return axi_tx, axi_rx, slave


async def rx_enable(axi_rx, slave, ddr_base=RX_DDR_BASE):
    """Configure and enable the RX DMA."""
    await axi_rx.write(RX_REG_DDR_BASE, ddr_base)
    await axi_rx.write(RX_REG_CONTROL, 0x01)  # enable


async def rx_read_sample_count(axi_rx):
    """Read RX SAMPLE_COUNT register."""
    val, _ = await axi_rx.read(RX_REG_SAMPLE_COUNT)
    return val


async def rx_read_overflow(axi_rx):
    """Read RX overflow count from STATUS bits [31:16]."""
    status, _ = await axi_rx.read(RX_REG_STATUS)
    return (status >> 16) & 0xFFFF


async def tx_oneshot_trigger(axi_tx, slave, samples, tx_count=None):
    """Pre-load TX DDR, configure for one-shot, trigger."""
    if tx_count is None:
        tx_count = len(samples)
    slave.write_iq_samples(TX_DDR_BASE, samples)
    await axi_tx.write(TX_REG_DDR_BASE, TX_DDR_BASE)
    await axi_tx.write(TX_REG_TX_COUNT, tx_count)
    await axi_tx.write(TX_REG_CONTROL, 0x03)  # enable + trigger


async def tx_cyclic_trigger(axi_tx, slave, samples, tx_count=None):
    """Pre-load TX DDR, configure for cyclic, trigger."""
    if tx_count is None:
        tx_count = len(samples)
    slave.write_iq_samples(TX_DDR_BASE, samples)
    await axi_tx.write(TX_REG_DDR_BASE, TX_DDR_BASE)
    await axi_tx.write(TX_REG_TX_COUNT, tx_count)
    await axi_tx.write(TX_REG_CONTROL, 0x07)  # enable + trigger + cyclic


async def tx_cyclic_stop(axi_tx):
    """Stop cyclic TX: write enable=0 + trigger=1."""
    await axi_tx.write(TX_REG_CONTROL, 0x02)

    # Poll for tx_done
    for _ in range(2000):
        await RisingEdge(axi_tx.clk)
        status, _ = await axi_tx.read(TX_REG_STATUS)
        if (status >> 1) & 1:
            return True
    return False


async def wait_tx_done(axi_tx, timeout=5000):
    """Poll TX STATUS until tx_done=1 or timeout."""
    for _ in range(timeout):
        await RisingEdge(axi_tx.clk)
        status, _ = await axi_tx.read(TX_REG_STATUS)
        if (status >> 1) & 1:
            return True
    return False


def make_samples(count, offset=0):
    """Generate test IQ samples: (offset + i) & 0xFFF, ((offset + i) * 3) & 0xFFF."""
    return [((offset + i) & 0xFFF, ((offset + i) * 3) & 0xFFF) for i in range(count)]


def iq_eq(a_re, a_im, b_re, b_im):
    """Compare IQ values modulo 12-bit (unpack_iq_beat sign-extends)."""
    return (a_re & 0xFFF) == (b_re & 0xFFF) and (a_im & 0xFFF) == (b_im & 0xFFF)


def rx_captures(tx_count):
    """Expected RX captures for tx_count TX samples at VALID_RATIO decimation."""
    return (tx_count + VALID_RATIO - 1) // VALID_RATIO


# =========================================================
# Test 1: TX one-shot → RX capture → verify data integrity
# =========================================================
@cocotb.test()
async def test_loopback_basic(dut):
    """TX 64-sample one-shot, verify RX captures decimated samples."""
    axi_tx, axi_rx, slave = await setup(dut)

    await rx_enable(axi_rx, slave)

    tx_count = 320  # 320 TX → 64 RX captures = 2 bursts at RATIO=5
    samples = make_samples(tx_count)
    await tx_oneshot_trigger(axi_tx, slave, samples, tx_count=tx_count)

    done = await wait_tx_done(axi_tx, timeout=8000)
    assert done, "tx_done not asserted after one-shot"

    for _ in range(500):
        await RisingEdge(dut.clk)

    # At VALID_RATIO=5, RX captures ~ceil(64/5) = 13 samples
    sc = await rx_read_sample_count(axi_rx)
    expected_captures = (tx_count + VALID_RATIO - 1) // VALID_RATIO
    assert sc >= expected_captures, \
        f"RX sample_count {sc} < expected captures {expected_captures}"

    rx_data = slave.read_iq_samples(RX_DDR_BASE, sc)
    mismatches = 0
    for i in range(expected_captures):
        tx_idx = i * VALID_RATIO
        if tx_idx >= tx_count:
            break
        if i >= len(rx_data):
            break
        got_re, got_im = rx_data[i]
        exp_re, exp_im = samples[tx_idx]
        if not iq_eq(got_re, got_im, exp_re, exp_im):
            mismatches += 1
            if mismatches <= 5:
                dut._log.error(
                    f"RX[{i}]=TX[{tx_idx}]: expected ({exp_re},{exp_im}), "
                    f"got ({got_re},{got_im})")

    assert mismatches == 0, f"{mismatches} sample mismatches in loopback"


# =========================================================
# Test 2: TX cyclic → RX continuous capture
# =========================================================
@cocotb.test()
async def test_loopback_sustained(dut):
    """TX cyclic for ~100 RX captures, verify pattern integrity."""
    axi_tx, axi_rx, slave = await setup(dut)

    await rx_enable(axi_rx, slave)

    wave_len = 64
    n_bursts = 6                     # 6 full RX bursts
    tx_produce = n_bursts * SAMPLES_PER_BURST * VALID_RATIO  # 960 TX → 192 captures
    samples = make_samples(wave_len)
    await tx_cyclic_trigger(axi_tx, slave, samples, tx_count=wave_len)

    target = rx_captures(tx_produce)  # should be burst-aligned
    for _ in range(20000):
        await RisingEdge(dut.clk)
        sc = await rx_read_sample_count(axi_rx)
        if sc >= target:
            break

    sc = await rx_read_sample_count(axi_rx)
    assert sc >= target, \
        f"RX sample_count {sc} < target {target}"

    stopped = await tx_cyclic_stop(axi_tx)
    assert stopped, "Failed to stop cyclic TX"

    rx_data = slave.read_iq_samples(RX_DDR_BASE, sc)
    mismatches = 0
    n_check = min(sc, target)
    for i in range(n_check):
        tx_idx = i * VALID_RATIO
        exp_re, exp_im = samples[tx_idx % wave_len]
        got_re, got_im = rx_data[i]
        if not iq_eq(got_re, got_im, exp_re, exp_im):
            mismatches += 1
            if mismatches <= 5:
                dut._log.error(
                    f"RX[{i}]=TX[{tx_idx}] wave[{tx_idx%wave_len}]: "
                    f"expected ({exp_re},{exp_im}), got ({got_re},{got_im})")

    assert mismatches == 0, f"{mismatches} cyclic repeat mismatches"


# =========================================================
# Test 2b: Diagnostic/guard — credit sensitivity and RX overflow
#
# Maps to STATUS.md Regression 1: RX DMA stall from TX AXI contention.
# The speculative wrap AR holds 2 concurrent read credits, starving
# the RX write channel on a shared AXI interconnect.
#
# After RTL fix, this test must show overflow=0 and first_mismatch=-1
# at max_out=2 and 4 (the PS7-relevant credit levels).
# =========================================================
@cocotb.test()
async def test_loopback_cyclic_diagnostics(dut):
    wave_len = 64
    target = wave_len * 8  # 512
    samples = make_samples(wave_len)

    failures = []

    credit_levels = [1, 2, 4, 8]

    for max_out in credit_levels:
        axi_tx, axi_rx, slave = await setup(dut, max_outstanding=max_out)

        await rx_enable(axi_rx, slave)
        overflow_start = await rx_read_overflow(axi_rx)

        await tx_cyclic_trigger(axi_tx, slave, samples, tx_count=wave_len)

        for _ in range(30000):
            await RisingEdge(dut.clk)
            sc = await rx_read_sample_count(axi_rx)
            if sc >= target:
                break

        overflow_end = await rx_read_overflow(axi_rx)
        sc_final = await rx_read_sample_count(axi_rx)
        overflow = overflow_end - overflow_start

        rx_data = slave.read_iq_samples(RX_DDR_BASE, min(sc_final, target))
        first_mismatch = -1
        mismatch_count = 0
        n_check = min(len(rx_data), target // VALID_RATIO)
        for i in range(n_check):
            tx_idx = i * VALID_RATIO
            exp_re, exp_im = samples[tx_idx % wave_len]
            got_re, got_im = rx_data[i]
            if not iq_eq(got_re, got_im, exp_re, exp_im):
                if first_mismatch < 0:
                    first_mismatch = i
                mismatch_count += 1

        ok = (overflow == 0 and first_mismatch < 0)
        label = "PASS" if ok else "FAIL"
        dut._log.info(
            f"[{label}] max_out={max_out}: overflow={overflow} "
            f"first_mismatch={first_mismatch} mismatches={mismatch_count}/{target}")

        if not ok:
            failures.append(
                f"max_out={max_out}: overflow={overflow}, "
                f"first_mismatch={first_mismatch}")

        await tx_cyclic_stop(axi_tx)

    if failures:
        dut._log.error(
            "RX contention detected — TX AXI read pattern starving RX writes:")
        for f in failures:
            dut._log.error(f"  {f}")
        assert False, f"{len(failures)}/{len(credit_levels)} credit levels had RX overflow/drops"


# =========================================================
# Test 2c: Pinpoint — capture TX output and RX DDR at iteration boundary
# =========================================================
@cocotb.test()
async def test_loopback_boundary_trace(dut):
    axi_tx, axi_rx, slave = await setup(dut)

    await rx_enable(axi_rx, slave)

    wave_len = 64
    samples = make_samples(wave_len)
    await tx_cyclic_trigger(axi_tx, slave, samples, tx_count=wave_len)

    tx_log = []
    n_target = 512
    tx_log = []
    for _ in range(20000):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        if int(dut.tx_valid_out.value) == 1:
            re_val = int(dut.tx_re_out.value) >> 4
            im_val = int(dut.tx_im_out.value) >> 4
            tx_log.append((re_val & 0xFFF, im_val & 0xFFF))
            if len(tx_log) >= n_target:
                break

    assert len(tx_log) >= n_target, f"Only captured {len(tx_log)} TX samples"

    for _ in range(500):
        await RisingEdge(dut.clk)

    sc = await rx_read_sample_count(axi_rx)
    rx_data = slave.read_iq_samples(RX_DDR_BASE, sc)

    # Verify TX output is correct per-sample
    tx_bad = sum(1 for i in range(min(n_target, len(tx_log)))
                 if not iq_eq(tx_log[i][0], tx_log[i][1],
                              samples[i % wave_len][0], samples[i % wave_len][1]))
    assert tx_bad == 0, \
        f"TX output has {tx_bad} mismatches — drain FSM corruption"

    # Verify RX DDR matches decimated TX output
    dut._log.info(f"--- RX data check (TX={len(tx_log)}, RX={sc}) ---")
    rx_bad = 0
    for i in range(min(sc, n_target // VALID_RATIO)):
        tx_idx = i * VALID_RATIO
        got_re, got_im = rx_data[i]
        exp_re, exp_im = tx_log[tx_idx]
        if not iq_eq(got_re, got_im, exp_re, exp_im):
            rx_bad += 1
            if rx_bad <= 5:
                dut._log.error(
                    f"  RX[{i}]=TX[{tx_idx}] w[{tx_idx%wave_len}]: "
                    f"got ({got_re:03X},{got_im:03X}) "
                    f"exp ({exp_re:03X},{exp_im:03X})")

    assert rx_bad == 0, \
        f"RX DDR has {rx_bad} mismatches at VALID_RATIO={VALID_RATIO}"

    await tx_cyclic_stop(axi_tx)


# =========================================================
# Test 3: Verify RX sample_count never freezes (Regression 1: DMA stall)
# =========================================================
@cocotb.test()
async def test_loopback_dma_stall(dut):
    """TX one-shot while RX is always-running; verify sample_count advances
    monotonically while TX is active (Regression 1: DMA stall)."""
    axi_tx, axi_rx, slave = await setup(dut)

    await rx_enable(axi_rx, slave)

    tx_count = 256
    samples = make_samples(tx_count)
    await tx_oneshot_trigger(axi_tx, slave, samples, tx_count=tx_count)

    # Track sample_count while TX is draining.  Only measure while TX active.
    sc_prev = 0
    stuck_cycles = 0
    max_stuck = 0
    tx_done_seen = False
    expected_captures = (tx_count + VALID_RATIO - 1) // VALID_RATIO

    for _ in range(20000):
        await RisingEdge(dut.clk)
        sc = await rx_read_sample_count(axi_rx)

        status, _ = await axi_tx.read(TX_REG_STATUS)
        tx_is_done = (status >> 1) & 1

        if not tx_is_done:
            if sc == sc_prev:
                stuck_cycles += 1
                if stuck_cycles > max_stuck:
                    max_stuck = stuck_cycles
            else:
                stuck_cycles = 0
                sc_prev = sc
        else:
            tx_done_seen = True

        if tx_done_seen and sc >= expected_captures + SAMPLES_PER_BURST:
            break

    sc_final = await rx_read_sample_count(axi_rx)
    assert sc_final >= expected_captures, \
        f"RX only captured {sc_final}/{expected_captures} samples"

    assert max_stuck < 200, \
        f"RX sample_count frozen for {max_stuck} cycles while TX active — DMA stall"


# =========================================================
# Test 4: Contention stress — max_outstanding=2, back-to-back triggers
# =========================================================
@cocotb.test()
async def test_loopback_contention(dut):
    """max_outstanding=2, rapid TX triggers; verify RX overflow=0 and
    sample_count is correct."""
    axi_tx, axi_rx, slave = await setup(dut, max_outstanding=2)

    await rx_enable(axi_rx, slave)

    # Run several back-to-back one-shot triggers
    n_iterations = 5
    tx_count = 64
    total_captures = n_iterations * ((tx_count + VALID_RATIO - 1) // VALID_RATIO)

    for iteration in range(n_iterations):
        samples = make_samples(tx_count, offset=iteration * tx_count)
        await tx_oneshot_trigger(axi_tx, slave, samples, tx_count=tx_count)

        done = await wait_tx_done(axi_tx, timeout=8000)
        assert done, f"TX done not asserted on iteration {iteration}"

        for _ in range(200):
            await RisingEdge(dut.clk)

    sc = await rx_read_sample_count(axi_rx)
    assert sc >= total_captures, \
        f"RX sample_count {sc} < expected {total_captures}"

    overflow = await rx_read_overflow(axi_rx)
    assert overflow == 0, \
        f"RX overflow_count={overflow} with max_outstanding=2 — AXI starvation"


# =========================================================
# Test 5: Stop TX mid-cyclic, restart, verify RX continues cleanly
# =========================================================
@cocotb.test()
async def test_loopback_restart(dut):
    """Stop TX during cyclic playback, restart; verify RX continues without gaps."""
    axi_tx, axi_rx, slave = await setup(dut)

    await rx_enable(axi_rx, slave)

    wave_len = 64
    samples = make_samples(wave_len)
    await tx_cyclic_trigger(axi_tx, slave, samples, tx_count=wave_len)

    target_captures = (wave_len * 2 + VALID_RATIO - 1) // VALID_RATIO

    for _ in range(3000):
        await RisingEdge(dut.clk)
        sc = await rx_read_sample_count(axi_rx)
        if sc >= target_captures:
            break

    sc_before_stop = await rx_read_sample_count(axi_rx)
    assert sc_before_stop >= target_captures, \
        f"RX didn't capture enough before stop: {sc_before_stop}"

    # Stop
    stopped = await tx_cyclic_stop(axi_tx)
    assert stopped, "Failed to stop cyclic TX"

    # Verify RX sample_count is stable after stop (TX drain stopped,
    # no new samples arriving)
    sc_after_stop = await rx_read_sample_count(axi_rx)

    # Restart cyclic
    min_new = (wave_len + VALID_RATIO - 1) // VALID_RATIO
    await tx_cyclic_trigger(axi_tx, slave, samples, tx_count=wave_len)

    for _ in range(3000):
        await RisingEdge(dut.clk)
        sc = await rx_read_sample_count(axi_rx)
        if sc >= sc_after_stop + min_new:
            break

    sc_after_restart = await rx_read_sample_count(axi_rx)
    assert sc_after_restart >= sc_after_stop + min_new, \
        f"RX didn't resume after restart: {sc_after_stop} → {sc_after_restart}"

    # Clean up
    await tx_cyclic_stop(axi_tx)


# =========================================================
# Test 6: Bit-perfect DDR roundtrip
# =========================================================
@cocotb.test()
async def test_loopback_ddr_integrity(dut):
    """Verify bit-perfect roundtrip: TX DDR → DAC → ADC → RX DDR.

    The TX DDR word is {8'b0, im[11:0], re[11:0]}.
    TX outputs: re_out = {re[11:0], 4'b0000}, im_out = {im[11:0], 4'b0000}.
    After truncation to 12-bit: re_in = re_out[15:4] = re, im_in = im_out[15:4] = im.
    RX DDR word: {8'b0, im_in[11:0], re_in[11:0]} = original word.
    """
    axi_tx, axi_rx, slave = await setup(dut)

    await rx_enable(axi_rx, slave)

    # Need enough samples for at least 1 RX burst at RATIO=5
    n_base = 7
    base = [
        (0x000, 0x000),
        (0xFFF, 0xFFF),
        (0x800, 0x7FF),
        (0x7FF, 0x800),
        (0x001, 0xFFE),
        (0xFFE, 0x001),
        (0x555, 0xAAA),
    ]
    full = (base * 50)[:320]  # 320 TX → 64 RX captures = 2 bursts

    await tx_oneshot_trigger(axi_tx, slave, full, tx_count=320)
    done = await wait_tx_done(axi_tx, timeout=8000)
    assert done, "tx_done not asserted"

    for _ in range(500):
        await RisingEdge(dut.clk)

    expected_cap = rx_captures(320)
    sc = await rx_read_sample_count(axi_rx)
    assert sc >= expected_cap, f"RX only captured {sc}/{expected_cap} samples"

    rx_data = slave.read_iq_samples(RX_DDR_BASE, sc)
    mismatches = 0
    n_expected = len(full) // VALID_RATIO + 1
    for i in range(min(sc, n_expected)):
        tx_idx = i * VALID_RATIO
        if tx_idx >= len(full):
            break
        if i >= len(rx_data):
            break
        got_re, got_im = rx_data[i]
        exp_re, exp_im = full[tx_idx]
        if not iq_eq(got_re, got_im, exp_re, exp_im):
            mismatches += 1
            if mismatches <= 5:
                dut._log.error(
                    f"RX[{i}]=TX[{tx_idx}]: expected ({exp_re:03X},{exp_im:03X}), "
                    f"got ({got_re:03X},{got_im:03X})")

    assert mismatches == 0, f"{mismatches} DDR integrity mismatches"
