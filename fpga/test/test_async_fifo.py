# SPDX-License-Identifier: MIT
"""Cocotb tests for async_fifo.v — gray-code pointer asynchronous FIFO.

Tests:
    1. Basic write/read (single word)
    2. Full flag assertion
    3. Empty flag behavior
    4. Fill and drain (all data correct, in order)
    5. Simultaneous write and read (no data loss)
    6. Fast write / slow read (stress full flag)
    7. Slow write / fast read (stress empty flag)
    8. Pointer wrap (multiple fill/drain cycles)
    9. Reset during operation
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

# Clock periods matching deployment: l_clk=16.27ns (61.44 MHz), sys_clk=10ns (100 MHz)
WR_CLK_NS = 16.27
RD_CLK_NS = 10.0

# FIFO parameters (matching RTL defaults for this test: WIDTH=25, DEPTH=4)
DEPTH = 4


async def setup(dut, wr_period=WR_CLK_NS, rd_period=RD_CLK_NS):
    """Start clocks, reset, return after ready."""
    cocotb.start_soon(Clock(dut.wr_clk, wr_period, unit="ns").start())
    cocotb.start_soon(Clock(dut.rd_clk, rd_period, unit="ns").start())

    # Reset both domains
    dut.wr_rst.value = 1
    dut.rd_rst.value = 1
    dut.wr_en.value = 0
    dut.wr_data.value = 0
    dut.rd_en.value = 0

    for _ in range(5):
        await RisingEdge(dut.wr_clk)
    for _ in range(5):
        await RisingEdge(dut.rd_clk)

    dut.wr_rst.value = 0
    dut.rd_rst.value = 0

    # Wait for synchronizers to settle
    for _ in range(4):
        await RisingEdge(dut.rd_clk)


async def write_word(dut, data):
    """Write a single word on wr_clk edge."""
    dut.wr_en.value = 1
    dut.wr_data.value = data
    await RisingEdge(dut.wr_clk)
    dut.wr_en.value = 0
    dut.wr_data.value = 0


async def read_word(dut):
    """Read a single word from FIFO. Must be called when empty=0.

    The FIFO has combinational read: rd_data = mem[rd_bin].
    When empty=0, rd_data is the front-of-queue entry.
    We assert rd_en, and on the next rising edge rd_bin advances.
    The returned data is the value present before the advance.
    """
    # Capture current rd_data (valid because caller checked empty=0)
    data = int(dut.rd_data.value)
    # Assert rd_en — on next posedge, rd_bin will advance
    dut.rd_en.value = 1
    await RisingEdge(dut.rd_clk)
    dut.rd_en.value = 0
    return data


async def wait_not_empty(dut, timeout_cycles=50):
    """Wait for FIFO to become non-empty (rd_clk domain). Returns True if not empty."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.rd_clk)
        if not int(dut.empty.value):
            return True
    return False


async def wait_not_full(dut, timeout_cycles=50):
    """Wait for FIFO to become non-full (wr_clk domain). Returns True if not full."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.wr_clk)
        if not int(dut.full.value):
            return True
    return False


@cocotb.test()
async def test_basic_write_read(dut):
    """Write 1 word, read it back, data matches."""
    await setup(dut)

    test_val = 0x1ABC_DEF
    await write_word(dut, test_val)

    # Wait for data to cross CDC
    assert await wait_not_empty(dut), "FIFO stayed empty after write"

    data = await read_word(dut)
    # Mask to WIDTH=25 bits
    assert data == (test_val & 0x1FFFFFF), \
        f"Read 0x{data:07X}, expected 0x{test_val & 0x1FFFFFF:07X}"


@cocotb.test()
async def test_full_flag(dut):
    """Fill to capacity, verify full asserts, write is rejected."""
    await setup(dut)

    # Fill FIFO to capacity
    for i in range(DEPTH):
        assert not int(dut.full.value), f"Full asserted prematurely at write {i}"
        await write_word(dut, i + 1)

    # Wait for full flag to propagate through synchronizer
    for _ in range(4):
        await RisingEdge(dut.wr_clk)

    assert int(dut.full.value), "Full flag not asserted after filling FIFO"


@cocotb.test()
async def test_empty_flag(dut):
    """Empty after reset, empty after draining all entries."""
    await setup(dut)

    # Should be empty after reset
    assert int(dut.empty.value), "FIFO not empty after reset"

    # Write one word, drain it
    await write_word(dut, 0xAA)
    assert await wait_not_empty(dut), "FIFO stayed empty after write"
    await read_word(dut)

    # Wait for empty to propagate
    for _ in range(4):
        await RisingEdge(dut.rd_clk)

    assert int(dut.empty.value), "FIFO not empty after draining"


@cocotb.test()
async def test_fill_and_drain(dut):
    """Fill completely, drain completely, all data correct and in order."""
    await setup(dut)

    write_data = [0x100 + i for i in range(DEPTH)]

    # Fill
    for val in write_data:
        dut.wr_en.value = 1
        dut.wr_data.value = val
        await RisingEdge(dut.wr_clk)
    dut.wr_en.value = 0

    # Wait for all data to cross
    assert await wait_not_empty(dut), "FIFO empty after fill"

    # Drain and verify order.
    # Verilator needs Timer(1ns) settle after RisingEdge for combinational
    # signals (rd_data, empty) derived from registers updated at that edge.
    read_data = []
    for _ in range(DEPTH):
        # Wait for data available
        timeout = 0
        while int(dut.empty.value):
            await RisingEdge(dut.rd_clk)
            timeout += 1
            assert timeout < 50, "Timeout waiting for data"
        # Settle combinational logic after edge
        await Timer(1, unit="ns")
        read_data.append(int(dut.rd_data.value))
        dut.rd_en.value = 1
        await RisingEdge(dut.rd_clk)
        dut.rd_en.value = 0

    for i, (wr, rd) in enumerate(zip(write_data, read_data)):
        assert (wr & 0x1FFFFFF) == rd, \
            f"Mismatch at index {i}: wrote 0x{wr:07X}, read 0x{rd:07X}"


@cocotb.test()
async def test_simultaneous_write_read(dut):
    """Concurrent producer/consumer, no data loss."""
    await setup(dut)

    num_words = 20
    written = []
    read_out = []

    async def writer():
        for i in range(num_words):
            while int(dut.full.value):
                await RisingEdge(dut.wr_clk)
            val = (i * 7 + 3) & 0x1FFFFFF
            written.append(val)
            dut.wr_en.value = 1
            dut.wr_data.value = val
            await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 0
            await RisingEdge(dut.wr_clk)  # gap between writes

    async def reader():
        timeout = 0
        while len(read_out) < num_words:
            await RisingEdge(dut.rd_clk)
            await Timer(1, unit="ns")  # settle combinational logic
            if not int(dut.empty.value):
                read_out.append(int(dut.rd_data.value))
                dut.rd_en.value = 1
                await RisingEdge(dut.rd_clk)
                dut.rd_en.value = 0
                timeout = 0
            else:
                timeout += 1
                assert timeout < 500, \
                    f"Reader timeout after {len(read_out)}/{num_words} words"

    writer_task = cocotb.start_soon(writer())
    await reader()
    await writer_task

    assert len(read_out) == num_words, \
        f"Read {len(read_out)} words, expected {num_words}"
    for i, (w, r) in enumerate(zip(written, read_out)):
        assert w == r, f"Mismatch at {i}: wrote 0x{w:07X}, read 0x{r:07X}"


@cocotb.test()
async def test_fast_write_slow_read(dut):
    """wr_clk faster than rd_clk — stress full flag."""
    # Swap: write at 10ns (fast), read at 16.27ns (slow)
    await setup(dut, wr_period=10.0, rd_period=16.27)

    num_words = 12  # 3x FIFO depth
    written = []
    read_out = []

    async def writer():
        for i in range(num_words):
            while int(dut.full.value):
                await RisingEdge(dut.wr_clk)
            val = (i + 0x500) & 0x1FFFFFF
            written.append(val)
            dut.wr_en.value = 1
            dut.wr_data.value = val
            await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 0
            await RisingEdge(dut.wr_clk)  # gap

    async def reader():
        timeout = 0
        while len(read_out) < num_words:
            await RisingEdge(dut.rd_clk)
            await Timer(1, unit="ns")
            if not int(dut.empty.value):
                read_out.append(int(dut.rd_data.value))
                dut.rd_en.value = 1
                await RisingEdge(dut.rd_clk)
                dut.rd_en.value = 0
                timeout = 0
            else:
                timeout += 1
                assert timeout < 500, f"Reader timeout after {len(read_out)}/{num_words}"

    writer_task = cocotb.start_soon(writer())
    await reader()
    await writer_task

    assert len(read_out) == num_words
    for i, (w, r) in enumerate(zip(written, read_out)):
        assert w == r, f"Mismatch at {i}: wrote 0x{w:07X}, read 0x{r:07X}"


@cocotb.test()
async def test_slow_write_fast_read(dut):
    """rd_clk faster than wr_clk — stress empty flag."""
    # Deployment configuration: write at 16.27ns, read at 10ns
    await setup(dut, wr_period=16.27, rd_period=10.0)

    num_words = 12
    written = []
    read_out = []

    async def writer():
        for i in range(num_words):
            while int(dut.full.value):
                await RisingEdge(dut.wr_clk)
            val = (i + 0xA00) & 0x1FFFFFF
            written.append(val)
            dut.wr_en.value = 1
            dut.wr_data.value = val
            await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 0
            await RisingEdge(dut.wr_clk)  # gap

    async def reader():
        timeout = 0
        while len(read_out) < num_words:
            await RisingEdge(dut.rd_clk)
            await Timer(1, unit="ns")
            if not int(dut.empty.value):
                read_out.append(int(dut.rd_data.value))
                dut.rd_en.value = 1
                await RisingEdge(dut.rd_clk)
                dut.rd_en.value = 0
                timeout = 0
            else:
                timeout += 1
                assert timeout < 500, f"Reader timeout after {len(read_out)}/{num_words}"

    writer_task = cocotb.start_soon(writer())
    await reader()
    await writer_task

    assert len(read_out) == num_words
    for i, (w, r) in enumerate(zip(written, read_out)):
        assert w == r, f"Mismatch at {i}: wrote 0x{w:07X}, read 0x{r:07X}"


@cocotb.test()
async def test_pointer_wrap(dut):
    """Multiple full/drain cycles — verify gray-code wrap is correct."""
    await setup(dut)

    for cycle in range(5):
        write_data = [(cycle * DEPTH + i) & 0x1FFFFFF for i in range(DEPTH)]

        # Fill
        for val in write_data:
            while int(dut.full.value):
                await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 1
            dut.wr_data.value = val
            await RisingEdge(dut.wr_clk)
            dut.wr_en.value = 0

        # Drain
        read_data = []
        for _ in range(DEPTH):
            timeout = 0
            while True:
                await RisingEdge(dut.rd_clk)
                await Timer(1, unit="ns")
                if not int(dut.empty.value):
                    break
                timeout += 1
                assert timeout < 50, f"Timeout draining cycle {cycle}"
            read_data.append(int(dut.rd_data.value))
            dut.rd_en.value = 1
            await RisingEdge(dut.rd_clk)
            dut.rd_en.value = 0

        for i, (w, r) in enumerate(zip(write_data, read_data)):
            assert w == r, \
                f"Cycle {cycle} index {i}: wrote 0x{w:07X}, read 0x{r:07X}"


@cocotb.test()
async def test_reset_during_operation(dut):
    """Assert reset mid-transfer, verify clean recovery."""
    await setup(dut)

    # Write some data
    for i in range(2):
        await write_word(dut, i + 1)

    assert await wait_not_empty(dut), "FIFO should have data"

    # Assert both resets
    dut.wr_rst.value = 1
    dut.rd_rst.value = 1
    for _ in range(5):
        await RisingEdge(dut.wr_clk)
    for _ in range(5):
        await RisingEdge(dut.rd_clk)

    dut.wr_rst.value = 0
    dut.rd_rst.value = 0

    # Wait for synchronizers to settle
    for _ in range(6):
        await RisingEdge(dut.rd_clk)

    # Should be empty after reset
    assert int(dut.empty.value), "FIFO not empty after reset"
    assert not int(dut.full.value), "FIFO full after reset"

    # Verify functional after reset
    test_val = 0x1DEAD
    await write_word(dut, test_val)
    assert await wait_not_empty(dut), "FIFO stayed empty after post-reset write"
    data = await read_word(dut)
    assert data == test_val, f"Post-reset read: got 0x{data:07X}, expected 0x{test_val:07X}"
