# SPDX-License-Identifier: MIT
"""Stream-mode ring-buffer wrap tests for iq_dma_tx.v.

These run with a deliberately tiny ring (BUF_SIZE_SAMPLES override) so the
ARM write cursor wraps within a few thousand simulation cycles.  The unit
suite uses the real 8 Mi-sample ring, which cannot wrap in simulation, so
the wrap behaviour was previously untested at any level.

The feeder coroutine mirrors the firmware flow control in dma_tx.c: it
keeps monotonic write and read totals, derives a signed depth, and only
feeds when the ring reports room.
"""

import cocotb
from cocotb.triggers import RisingEdge, Timer

from test_iq_dma_tx import (
    setup, REG_CONTROL, REG_DDR_BASE, REG_TX_COUNT, REG_WR_PTR, REG_RD_PTR,
    DDR_BASE_ADDR,
)

# Must match the -GBUF_SIZE_SAMPLES override in the Makefile target.
BUF = 1024
CHUNK = 128


def make_ramp(n):
    """Distinct 12-bit sample per ring slot, so gaps are identifiable."""
    return [((i * 7) & 0x7FF, (i * 13) & 0x7FF) for i in range(n)]


class Feeder:
    """Mirror of the firmware's stream flow control (dma_tx.c)."""

    def __init__(self, axi, buf_size=BUF):
        self.axi = axi
        self.buf = buf_size
        self.cursor = 0
        self.wr_total = 0
        self.rd_total = 0
        self.rd_last = 0
        self.wraps = 0

    async def prefill(self, n):
        """Advance the cursor before trigger, when the whole ring is free."""
        self.cursor = n % self.buf
        self.wr_total = n
        await self.axi.write(REG_WR_PTR, self.cursor)

    async def refresh_depth(self):
        rd, _ = await self.axi.read(REG_RD_PTR)
        self.rd_total += (rd - self.rd_last) & (self.buf - 1)
        self.rd_last = rd
        return self.wr_total - self.rd_total

    async def feed(self, n):
        prev = self.cursor
        self.cursor = (self.cursor + n) % self.buf
        if self.cursor < prev:
            self.wraps += 1
        self.wr_total += n
        await self.axi.write(REG_WR_PTR, self.cursor)


@cocotb.test()
async def test_stream_survives_wr_ptr_wrap(dut):
    """The fill FSM must keep reading when the ARM write cursor wraps.

    The ring is a modular counter, so a wrapped write cursor is numerically
    smaller than the fill pointer while still being logically ahead of it.
    A plain magnitude comparison reads that as "caught up" and stalls the
    fill FSM, starving the DAC.
    """
    axi, slave = await setup(dut)

    slave.write_iq_samples(DDR_BASE_ADDR, make_ramp(BUF))

    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_TX_COUNT, BUF)   # stream mode: TX_COUNT is ring size

    feeder = Feeder(axi)
    await feeder.prefill(BUF // 2)

    await axi.write(REG_CONTROL, 0x0F)   # enable | trigger | cyclic | stream

    # Interleave feeding with output accounting.  The drain FSM emits one
    # sample per clock unless it is starved, so the emitted-to-elapsed
    # ratio is a direct measure of continuity.
    emitted = 0
    cycles = 0
    stall_run = 0
    worst_stall = 0

    for _ in range(400):
        depth = await feeder.refresh_depth()
        avail = BUF - depth - 1
        if avail >= CHUNK:
            await feeder.feed(CHUNK)

        # Observe the DAC for a while between feeds.
        for _ in range(60):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
            cycles += 1
            if int(dut.valid_out.value) == 1:
                emitted += 1
                stall_run = 0
            else:
                stall_run += 1
                worst_stall = max(worst_stall, stall_run)

        if feeder.wraps >= 2:
            break

    f_state = int(dut.f_state.value)
    fill_ptr = int(dut.fill_ptr.value)
    wr_ptr = int(dut.lcl_wr_ptr.value)

    await axi.write(REG_CONTROL, 0x02)   # stop

    dut._log.info(
        f"wraps={feeder.wraps} emitted={emitted}/{cycles} "
        f"({100.0 * emitted / cycles:.1f}%) worst_stall={worst_stall} cycles")
    dut._log.info(
        f"fill FSM: f_state={f_state} fill_ptr={fill_ptr} lcl_wr_ptr={wr_ptr} "
        f"depth={depth}")

    # Some slack for the initial fill latency, but a wrap-induced stall
    # starves the DAC for thousands of cycles and lands far below this.
    ratio = emitted / cycles
    assert ratio > 0.95, (
        f"DAC starved across ring wrap: emitted {emitted}/{cycles} "
        f"({100.0 * ratio:.1f}%), longest gap {worst_stall} cycles; "
        f"fill FSM f_state={f_state} fill_ptr={fill_ptr} wr_ptr={wr_ptr}")

    assert feeder.wraps >= 2, \
        f"test did not exercise the wrap (only {feeder.wraps} wraps)"


@cocotb.test()
async def test_stream_never_reads_past_wr_ptr(dut):
    """The fill FSM must not read DDR the ARM has not written yet.

    WR_PTR is exclusive: samples below it are valid, samples at or above it
    are not.  Reads are issued a whole burst at a time, so a burst may only
    start when a full burst of valid samples is available.  Here WR_PTR sits
    mid-burst, which is the case a plain "pointers differ" check gets wrong:
    it sees data available and reads 32 samples, half of them stale.
    """
    axi, slave = await setup(dut)

    slave.write_iq_samples(DDR_BASE_ADDR, make_ramp(BUF))

    # 48 valid samples: one full burst, then a half burst.  The fill FSM
    # may read [0,32) and must then stall rather than read [32,64).
    wr_ptr = 48

    await axi.write(REG_DDR_BASE, DDR_BASE_ADDR)
    await axi.write(REG_TX_COUNT, BUF)
    await axi.write(REG_WR_PTR, wr_ptr)

    await axi.write(REG_CONTROL, 0x0F)   # enable | trigger | cyclic | stream

    for _ in range(4000):
        await RisingEdge(dut.clk)

    await axi.write(REG_CONTROL, 0x02)   # stop

    # Translate every burst the DUT requested back into sample indices.
    over_reads = []
    for base_addr, burst_len in slave.reads:
        first = (base_addr - DDR_BASE_ADDR) // 4
        last = first + burst_len * 2 - 1     # 2 samples per 64-bit beat
        if last >= wr_ptr:
            over_reads.append((first, last))

    dut._log.info(f"WR_PTR={wr_ptr}, bursts issued={len(slave.reads)}, "
                  f"over-reads={over_reads}")

    assert not over_reads, (
        f"fill FSM read past WR_PTR={wr_ptr} (only samples 0..{wr_ptr - 1} "
        f"are valid): burst sample ranges {over_reads}")


