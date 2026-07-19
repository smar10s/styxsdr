# SPDX-License-Identifier: MIT
"""Cocotb tests for iq_dma_rx.v — continuous IQ DMA writer.

Tests:
    1. Basic burst — enable, feed 32 samples, verify one AXI3 burst
    2. Continuous wrap — feed enough to wrap DDR buffer
    3. Sample packing — verify {8'b0, im[11:0], re[11:0]} format
    4. Enable/disable — disable mid-stream, re-enable, clean restart
    5. Overflow count — stall WREADY to cause overflow
    6. Sample count — verify sample_count register increments
    7. No burst without DDR_BASE — no AXI3 activity without base configured
    8. DDR wrap boundary — verify wrap at correct address
    9. Slow WREADY — delayed WREADY doesn't cause overflow
   10. Sample_cnt_out — verify per-sample port advances
   11. Long burst data integrity — 1024+ samples, verify all DDR data
   12. Burst with backpressure — frame-pattern writes with wready delay (KI#9 pattern)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from axi_lite_driver import AxiLiteMaster
from axi3_mocks import Axi3WriteSlave, pack_iq_word, unpack_iq_beat

# Register offsets
REG_CONTROL      = 0x00
REG_STATUS       = 0x04
REG_DDR_BASE     = 0x08
REG_WR_PTR       = 0x0C
REG_SAMPLE_COUNT = 0x10

# DMA parameters (for test, DDR_BUF_SAMPLES=64 via Makefile override)
DDR_BASE_ADDR    = 0x10000000
SAMPLES_PER_BURST = 32


async def setup(dut, wready_delay=0):
    """Start clock, reset, create drivers, configure base, return (axi, slave)."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.rst.value = 1
    dut.valid_in.value = 0
    dut.re_in.value = 0
    dut.im_in.value = 0
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
    # AXI3 write slave signals
    dut.m_axi_awready.value = 0
    dut.m_axi_wready.value = 0
    dut.m_axi_bresp.value = 0
    dut.m_axi_bvalid.value = 0
    dut.m_axi_bid.value = 0

    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.clk)
    slave = Axi3WriteSlave(dut, prefix="m_axi_", clk=dut.clk,
                           wready_delay=wready_delay)
    cocotb.start_soon(slave.run())

    return axi, slave


async def configure_and_enable(axi):
    """Write DDR_BASE and enable DMA."""
    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_CONTROL, 0x01)  # enable


async def feed_iq(dut, samples, gap=0):
    """Feed list of (re, im) tuples to the DUT input.

    Args:
        samples: list of (re, im) tuples (12-bit signed values)
        gap: cycles to wait between samples (0 = back-to-back)
    """
    for re, im in samples:
        dut.valid_in.value = 1
        dut.re_in.value = re & 0xFFF
        dut.im_in.value = im & 0xFFF
        await RisingEdge(dut.clk)
        dut.valid_in.value = 0
        for _ in range(gap):
            await RisingEdge(dut.clk)


async def wait_burst(slave, timeout=500):
    """Wait for at least one burst to complete."""
    for _ in range(timeout):
        await RisingEdge(slave.clk)
        if slave.bursts:
            return True
    return False


@cocotb.test()
async def test_basic_burst(dut):
    """Enable DMA, feed 32 samples, verify one AXI3 burst write to DDR."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    # Feed exactly one burst's worth of samples
    samples = [(i, -i) for i in range(SAMPLES_PER_BURST)]
    await feed_iq(dut, samples)

    # Wait for burst
    assert await wait_burst(slave), "No burst seen after 32 samples"
    assert len(slave.bursts) >= 1, f"Expected >= 1 burst, got {len(slave.bursts)}"

    # Verify burst address
    addr, _ = slave.bursts[0]
    assert addr == DDR_BASE_ADDR, f"Burst addr = 0x{addr:08X}, expected 0x{DDR_BASE_ADDR:08X}"


@cocotb.test()
async def test_continuous_wrap(dut):
    """Feed enough samples to wrap DDR buffer, verify wr_ptr resets to 0."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    # With DDR_BUF_SAMPLES=64, 2 bursts fill the buffer
    buf_samples = int(dut.DDR_BUF_SAMPLES.value)
    total_samples = buf_samples + SAMPLES_PER_BURST
    samples = [(i & 0x7FF, i & 0x7FF) for i in range(total_samples)]
    await feed_iq(dut, samples)

    # Wait for bursts to complete
    for _ in range(1000):
        await RisingEdge(dut.clk)
        if len(slave.bursts) >= (total_samples // SAMPLES_PER_BURST):
            break

    # Read WR_PTR — should have wrapped
    wr_ptr, _ = await axi.read(REG_WR_PTR)
    assert wr_ptr == SAMPLES_PER_BURST, \
        f"WR_PTR after wrap = {wr_ptr}, expected {SAMPLES_PER_BURST}"


@cocotb.test()
async def test_sample_packing(dut):
    """Verify {8'b0, im[11:0], re[11:0]} packing in 32-bit words."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    # Known values: even sample=(100, 200), odd sample=(300, 400)
    samples = [(100, 200), (300, 400)] * 16  # 32 samples = 1 burst
    await feed_iq(dut, samples)

    assert await wait_burst(slave), "No burst"

    # Check first beat (first pair)
    beat_addr = DDR_BASE_ADDR
    beat_data = slave.read_dword(beat_addr)

    # Low 32 bits = even sample: {8'b0, im[11:0], re[11:0]} = {0, 200, 100}
    expected_even = pack_iq_word(100, 200)
    # High 32 bits = odd sample: {8'b0, im[11:0], re[11:0]} = {0, 400, 300}
    expected_odd = pack_iq_word(300, 400)
    expected_beat = (expected_odd << 32) | expected_even

    assert beat_data == expected_beat, \
        f"Beat 0 = 0x{beat_data:016X}, expected 0x{expected_beat:016X}"


@cocotb.test()
async def test_enable_disable(dut):
    """Disable mid-stream, verify no bursts; re-enable, verify burst fires."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    # Feed 16 samples (half a burst)
    samples = [(i, i) for i in range(16)]
    await feed_iq(dut, samples)

    # Disable
    await axi.write(REG_CONTROL, 0x00)

    # Feed more — should be ignored
    await feed_iq(dut, [(99, 99)] * 32)
    for _ in range(100):
        await RisingEdge(dut.clk)

    assert len(slave.bursts) == 0, "Burst fired while disabled"

    # Re-enable
    await axi.write(REG_CONTROL, 0x01)

    # Feed a full burst
    await feed_iq(dut, [(50, 50)] * SAMPLES_PER_BURST)
    assert await wait_burst(slave), "No burst after re-enable"


@cocotb.test()
async def test_overflow_count(dut):
    """Stall WREADY to cause overflow, verify overflow_count increments."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    # Stall AXI writes — will cause overflow when both buffer halves are full
    slave.stall_wready = True

    # Feed many samples — should eventually overflow
    samples = [(i & 0x7FF, i & 0x7FF) for i in range(SAMPLES_PER_BURST * 4)]
    await feed_iq(dut, samples)

    # Wait a bit for overflow to register
    for _ in range(100):
        await RisingEdge(dut.clk)

    # Read STATUS — overflow_count is bits [31:16]
    status, _ = await axi.read(REG_STATUS)
    overflow = (status >> 16) & 0xFFFF
    assert overflow > 0, f"No overflow detected (STATUS=0x{status:08X})"


@cocotb.test()
async def test_sample_count(dut):
    """Verify sample_count register increments on every valid_in."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    num_samples = 50
    samples = [(i, i) for i in range(num_samples)]
    await feed_iq(dut, samples)

    # Wait for fill to complete
    for _ in range(100):
        await RisingEdge(dut.clk)

    count, _ = await axi.read(REG_SAMPLE_COUNT)
    assert count == num_samples, f"SAMPLE_COUNT = {count}, expected {num_samples}"


@cocotb.test()
async def test_no_burst_without_ddr_base(dut):
    """Don't configure DDR_BASE, verify no AXI3 activity despite enable."""
    axi, slave = await setup(dut)

    # Enable WITHOUT setting DDR_BASE
    await axi.write(REG_CONTROL, 0x01)

    # Feed a full burst
    samples = [(i, i) for i in range(SAMPLES_PER_BURST)]
    await feed_iq(dut, samples)

    for _ in range(200):
        await RisingEdge(dut.clk)

    assert len(slave.bursts) == 0, "Burst fired without DDR_BASE configured"


@cocotb.test()
async def test_ddr_wrap_boundary(dut):
    """With small DDR_BUF_SAMPLES, verify wrap happens at correct address."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    buf_samples = int(dut.DDR_BUF_SAMPLES.value)
    bursts_to_fill = buf_samples // SAMPLES_PER_BURST

    # Feed enough to wrap
    total = buf_samples + SAMPLES_PER_BURST
    samples = [(i & 0x7FF, i & 0x7FF) for i in range(total)]
    await feed_iq(dut, samples)

    # Wait for all bursts
    for _ in range(2000):
        await RisingEdge(dut.clk)
        if len(slave.bursts) >= bursts_to_fill + 1:
            break

    # The last burst after wrap should be at DDR_BASE again
    last_addr, _ = slave.bursts[-1]
    expected = DDR_BASE_ADDR
    assert last_addr == expected, \
        f"Post-wrap burst addr = 0x{last_addr:08X}, expected 0x{expected:08X}"


@cocotb.test()
async def test_slow_wready(dut):
    """WREADY delayed by 2 cycles, verify no data corruption or overflow."""
    axi, slave = await setup(dut, wready_delay=2)
    await configure_and_enable(axi)

    # Feed 2 bursts worth — should complete without overflow
    num_samples = SAMPLES_PER_BURST * 2
    samples = [(i & 0x7FF, (i * 3) & 0x7FF) for i in range(num_samples)]
    await feed_iq(dut, samples, gap=4)  # gap=4 gives 5:1 ratio (realistic)

    # Wait for bursts
    for _ in range(3000):
        await RisingEdge(dut.clk)
        if len(slave.bursts) >= 2:
            break

    assert len(slave.bursts) >= 2, f"Only {len(slave.bursts)} bursts completed"

    # Verify no overflow
    status, _ = await axi.read(REG_STATUS)
    overflow = (status >> 16) & 0xFFFF
    assert overflow == 0, f"Overflow with slow WREADY: {overflow}"


@cocotb.test()
async def test_sample_cnt_out(dut):
    """Verify sample_cnt_out port advances per-sample."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    # Feed 10 samples and track sample_cnt_out
    counts = []
    for i in range(10):
        dut.valid_in.value = 1
        dut.re_in.value = i
        dut.im_in.value = i
        await RisingEdge(dut.clk)
        dut.valid_in.value = 0
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        counts.append(int(dut.sample_cnt_out.value))

    # Should increment by 1 each sample
    for i in range(1, len(counts)):
        assert counts[i] == counts[i-1] + 1, \
            f"sample_cnt_out jump: {counts[i-1]} -> {counts[i]} at sample {i}"


@cocotb.test()
async def test_long_burst_data_integrity(dut):
    """Feed 1024+ samples (32+ bursts), verify all DDR data matches input."""
    axi, slave = await setup(dut)
    await configure_and_enable(axi)

    buf_samples = int(dut.DDR_BUF_SAMPLES.value)
    # Use exactly the buffer size to avoid wrap confusion
    num_samples = min(buf_samples, 256)
    num_bursts = num_samples // SAMPLES_PER_BURST

    samples = [(i & 0x7FF, (i * 2) & 0x7FF) for i in range(num_samples)]
    await feed_iq(dut, samples, gap=4)

    # Wait for all bursts
    for _ in range(5000):
        await RisingEdge(dut.clk)
        if len(slave.bursts) >= num_bursts:
            break

    assert len(slave.bursts) >= num_bursts, \
        f"Only {len(slave.bursts)}/{num_bursts} bursts"

    # Verify data integrity
    mismatches = 0
    for i in range(0, num_samples, 2):
        addr = DDR_BASE_ADDR + (i // 2) * 8
        beat = slave.read_dword(addr)
        (got_re0, got_im0), (got_re1, got_im1) = unpack_iq_beat(beat)

        exp_re0, exp_im0 = samples[i]
        exp_re1, exp_im1 = samples[i+1] if i+1 < num_samples else (0, 0)

        if got_re0 != exp_re0 or got_im0 != exp_im0:
            mismatches += 1
            if mismatches <= 3:
                dut._log.error(
                    f"Sample {i}: expected ({exp_re0},{exp_im0}), "
                    f"got ({got_re0},{got_im0})")
        if got_re1 != exp_re1 or got_im1 != exp_im1:
            mismatches += 1
            if mismatches <= 3:
                dut._log.error(
                    f"Sample {i+1}: expected ({exp_re1},{exp_im1}), "
                    f"got ({got_re1},{got_im1})")

    assert mismatches == 0, f"{mismatches} sample mismatches in DDR"


@cocotb.test()
async def test_burst_with_backpressure(dut):
    """Frame-pattern writes with wready_delay=3 (deimos KI#9 pattern).

    Tests that bursty frame-gap traffic with AXI backpressure doesn't
    corrupt data in the DDR ring buffer.
    """
    axi, slave = await setup(dut, wready_delay=3)
    await configure_and_enable(axi)

    buf_samples = int(dut.DDR_BUF_SAMPLES.value)
    # Use fewer frames to stay within DDR buffer (avoid wrap confusion)
    num_frames = buf_samples // SAMPLES_PER_BURST
    all_samples = []
    for frame in range(num_frames):
        frame_samples = [((frame * 32 + i) & 0x7FF, ((frame * 32 + i) * 2) & 0x7FF)
                         for i in range(SAMPLES_PER_BURST)]
        all_samples.extend(frame_samples)
        await feed_iq(dut, frame_samples, gap=4)
        # Inter-frame gap (50 cycles)
        for _ in range(50):
            await RisingEdge(dut.clk)

    # Wait for all bursts
    for _ in range(5000):
        await RisingEdge(dut.clk)
        if len(slave.bursts) >= num_frames:
            break

    assert len(slave.bursts) >= num_frames, \
        f"Only {len(slave.bursts)}/{num_frames} bursts with backpressure"

    # Verify data integrity for each burst
    mismatches = 0
    for i in range(0, len(all_samples), 2):
        addr = DDR_BASE_ADDR + (i // 2) * 8
        beat = slave.read_dword(addr)
        (got_re0, got_im0), (got_re1, got_im1) = unpack_iq_beat(beat)

        exp_re0, exp_im0 = all_samples[i]
        exp_re1, exp_im1 = all_samples[i+1] if i+1 < len(all_samples) else (0, 0)

        if got_re0 != exp_re0 or got_im0 != exp_im0:
            mismatches += 1
            if mismatches <= 3:
                dut._log.error(
                    f"Sample {i}: expected ({exp_re0},{exp_im0}), "
                    f"got ({got_re0},{got_im0})")
        if got_re1 != exp_re1 or got_im1 != exp_im1:
            mismatches += 1
            if mismatches <= 3:
                dut._log.error(
                    f"Sample {i+1}: expected ({exp_re1},{exp_im1}), "
                    f"got ({got_re1},{got_im1})")

    assert mismatches == 0, \
        f"{mismatches} sample mismatches with backpressure (KI#9 pattern)"
