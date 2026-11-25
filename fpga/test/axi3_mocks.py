# SPDX-License-Identifier: MIT
"""Reusable AXI3 mock slaves for cocotb testbenches.

Provides:
    Axi3WriteSlave — accepts AXI3 burst writes, stores in byte-addressed dict.
    Axi3ReadSlave  — services AXI3 burst reads from a byte-addressed dict.

Both run as background coroutines via cocotb.start_soon(mock.run()).

IQ sample helpers:
    Both classes include pack/unpack utilities matching the styx DDR format:
        32-bit word: {8'b0, im[11:0], re[11:0]}
        64-bit beat: {word_odd, word_even} (even at lower address)
"""

from collections import deque

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge, Timer
import random


def pack_iq_word(re_val, im_val):
    """Pack signed 12-bit I/Q into a 32-bit DDR word."""
    return ((im_val & 0xFFF) << 12) | (re_val & 0xFFF)


def pack_iq_beat(re0, im0, re1, im1):
    """Pack two IQ samples into a 64-bit AXI beat.

    Beat format: {word_odd[63:32], word_even[31:0]}
    word_even = sample 0 (lower address), word_odd = sample 1.
    """
    word_even = pack_iq_word(re0, im0)
    word_odd = pack_iq_word(re1, im1)
    return (word_odd << 32) | (word_even & 0xFFFFFFFF)


def unpack_iq_beat(beat):
    """Unpack a 64-bit AXI beat into two (re, im) tuples (12-bit signed)."""
    word_even = beat & 0xFFFFFFFF
    word_odd = (beat >> 32) & 0xFFFFFFFF

    def extract(word):
        re = word & 0xFFF
        im = (word >> 12) & 0xFFF
        # Sign-extend 12-bit
        if re >= 0x800:
            re -= 0x1000
        if im >= 0x800:
            im -= 0x1000
        return re, im

    return extract(word_even), extract(word_odd)


class Axi3WriteSlave:
    """Mock AXI3 write slave with independent AW/W/B channels.

    Args:
        dut: cocotb DUT handle.
        prefix: Signal name prefix (default "m_axi_").
        clk: Clock signal (default dut.clk).
        wready_delay: Cycles to delay WREADY after WVALID (simulates slow DDR).
        bresp_delay: Cycles between a burst's last W beat and its B response.
        max_outstanding: Cap on unresponded AWs. None means no mock-side cap,
            so the DUT's own NUM_WRITE_OUTSTANDING limit is observable.

    Attributes:
        bursts: [(base_addr, [beat_data, ...]), ...] in W-completion order.
        mem: byte-addressed write memory.
        aw_accepted, w_completed, b_completed, inflight, max_inflight_seen:
            transaction counters for assertions.
        stall_wready: If True, never accept W beats (overflow testing).
        stall_bresp: If True, hold B responses (write-pointer testing).
    """

    def __init__(self, dut, prefix="m_axi_", clk=None, wready_delay=0,
                 bresp_delay=0, max_outstanding=None):
        self.dut = dut
        self.clk = clk if clk is not None else dut.clk
        self._p = prefix
        self.mem = {}  # byte_addr -> byte value
        self.bursts = []  # list of (base_addr, [beat_data, ...]) on W completion
        self.wready_delay = wready_delay
        self.bresp_delay = bresp_delay
        self.max_outstanding = max_outstanding
        self.stall_wready = False  # if True, never assert WREADY
        self.stall_bresp = False   # if True, hold B responses

        self.aw_queue = deque()   # [base_addr, burst_len, [beat_data, ...]]
        self.b_pending = deque()  # (due_cycle, base_addr)
        self.aw_accepted = 0
        self.w_completed = 0
        self.b_completed = 0
        self.inflight = 0
        self.max_inflight_seen = 0
        self.cycle = 0
        self._bvalid_driven = False

    def _sig(self, name):
        return getattr(self.dut, self._p + name)

    def read_word(self, byte_addr):
        """Read a 32-bit word from memory model (little-endian)."""
        val = 0
        for i in range(4):
            val |= self.mem.get(byte_addr + i, 0) << (8 * i)
        return val

    def read_dword(self, byte_addr):
        """Read a 64-bit doubleword from memory model (little-endian)."""
        val = 0
        for i in range(8):
            val |= self.mem.get(byte_addr + i, 0) << (8 * i)
        return val

    def write_iq_samples(self, base_addr, samples):
        """Pre-load IQ samples into memory (for verification comparison).

        Args:
            base_addr: DDR byte address.
            samples: list of (re, im) tuples (12-bit signed values).
        """
        for i in range(0, len(samples), 2):
            re0, im0 = samples[i]
            word_even = pack_iq_word(re0, im0)
            if i + 1 < len(samples):
                re1, im1 = samples[i + 1]
                word_odd = pack_iq_word(re1, im1)
            else:
                word_odd = 0
            beat = (word_odd << 32) | (word_even & 0xFFFFFFFF)
            addr = base_addr + (i // 2) * 8
            for b in range(8):
                self.mem[addr + b] = (beat >> (8 * b)) & 0xFF

    def _set(self, name, value):
        try:
            self._sig(name).value = value
        except AttributeError:
            pass

    def _read(self, name):
        try:
            return int(self._sig(name).value)
        except (AttributeError, ValueError):
            return None

    async def run(self):
        """Main loop — start once with cocotb.start_soon(slave.run())."""
        self._set("awready", 0)
        self._set("wready", 0)
        self._set("bresp", 0)
        self._set("bvalid", 0)
        self._set("bid", 0)

        cocotb.start_soon(self._tick())
        cocotb.start_soon(self._aw_channel())
        cocotb.start_soon(self._w_channel())
        cocotb.start_soon(self._b_channel())

    async def _tick(self):
        while True:
            await RisingEdge(self.clk)
            self.cycle += 1

    async def _aw_channel(self):
        while True:
            await RisingEdge(self.clk)
            if (self.max_outstanding is not None
                    and self.inflight >= self.max_outstanding):
                self._set("awready", 0)
                continue
            if self._read("awvalid") != 1:
                self._set("awready", 0)
                continue

            base_addr = self._read("awaddr")
            awlen = self._read("awlen")
            burst_len = (awlen or 0) + 1

            self.aw_queue.append([base_addr, burst_len, []])
            self.aw_accepted += 1
            self.inflight += 1
            self.max_inflight_seen = max(self.max_inflight_seen, self.inflight)

            self._set("awready", 1)
            await RisingEdge(self.clk)
            self._set("awready", 0)

    async def _w_channel(self):
        while True:
            if not self.aw_queue:
                await RisingEdge(self.clk)
                continue

            base_addr, burst_len, beat_data = self.aw_queue[0]

            while len(beat_data) < burst_len:
                for _ in range(self.wready_delay):
                    await RisingEdge(self.clk)

                while self.stall_wready:
                    await RisingEdge(self.clk)

                # Sample WDATA mid-cycle, when a combinational DUT output
                # is settled, then accept it on the next rising edge.
                await FallingEdge(self.clk)
                while self._read("wvalid") != 1:
                    await FallingEdge(self.clk)

                data = self._read("wdata")
                self._set("wready", 1)
                await RisingEdge(self.clk)
                self._set("wready", 0)

                beat_data.append(data)
                beat_addr = base_addr + (len(beat_data) - 1) * 8
                for b in range(8):
                    self.mem[beat_addr + b] = (data >> (8 * b)) & 0xFF

            self.aw_queue.popleft()
            self.bursts.append((base_addr, beat_data))
            self.w_completed += 1
            self.b_pending.append((self.cycle + self.bresp_delay, base_addr))

    async def _b_channel(self):
        while True:
            await RisingEdge(self.clk)

            if self._bvalid_driven:
                if self.stall_bresp:
                    continue
                if self._read("bready") == 1:
                    self._set("bvalid", 0)
                    self._bvalid_driven = False
                    self.b_pending.popleft()
                    self.inflight -= 1
                    self.b_completed += 1
                continue

            if (self.b_pending and not self.stall_bresp
                    and self.cycle >= self.b_pending[0][0]):
                self._set("bresp", 0)
                self._set("bid", 0)
                self._set("bvalid", 1)
                self._bvalid_driven = True


class Axi3ReadSlave:
    """Mock AXI3 read slave that services burst reads from a dict.

    Args:
        dut: cocotb DUT handle.
        prefix: Signal name prefix (default "m_axi_").
        clk: Clock signal (default dut.clk).
        rvalid_delay: Fixed cycles to delay RVALID per beat (simulates DDR latency).
        jitter_range: If set, adds random 0..jitter_range cycles per beat.
    """

    def __init__(self, dut, prefix="m_axi_", clk=None,
                 rvalid_delay=0, jitter_range=0):
        self.dut = dut
        self.clk = clk if clk is not None else dut.clk
        self._p = prefix
        self.mem = {}  # byte_addr -> byte value (pre-loaded)
        self.rvalid_delay = rvalid_delay
        self.jitter_range = jitter_range
        self.stall_rvalid = False
        self.reads = []  # list of (base_addr, burst_len) for diagnostics

    def _sig(self, name):
        return getattr(self.dut, self._p + name)

    def load_dword(self, byte_addr, value):
        """Load a 64-bit value at byte_addr (little-endian)."""
        for b in range(8):
            self.mem[byte_addr + b] = (value >> (8 * b)) & 0xFF

    def read_dword(self, byte_addr):
        """Read a 64-bit value from byte_addr (little-endian)."""
        val = 0
        for b in range(8):
            val |= self.mem.get(byte_addr + b, 0) << (8 * b)
        return val

    def write_iq_samples(self, base_addr, samples):
        """Pre-load IQ samples into memory for DDR read playback.

        Args:
            base_addr: DDR byte address.
            samples: list of (re, im) tuples (12-bit signed values).
        """
        for i in range(0, len(samples), 2):
            re0, im0 = samples[i]
            word_even = pack_iq_word(re0, im0)
            if i + 1 < len(samples):
                re1, im1 = samples[i + 1]
                word_odd = pack_iq_word(re1, im1)
            else:
                word_odd = 0
            beat = (word_odd << 32) | (word_even & 0xFFFFFFFF)
            addr = base_addr + (i // 2) * 8
            self.load_dword(addr, beat)

    async def run(self):
        """Main loop — run as cocotb.start_soon(slave.run())."""
        self._sig("arready").value = 0
        self._sig("rdata").value = 0
        self._sig("rvalid").value = 0
        self._sig("rlast").value = 0
        self._sig("rresp").value = 0
        # RID if it exists
        try:
            self._sig("rid").value = 0
        except AttributeError:
            pass

        while True:
            # Wait for ARVALID
            while True:
                await RisingEdge(self.clk)
                try:
                    if int(self._sig("arvalid").value) == 1:
                        break
                except ValueError:
                    pass  # X/Z during reset

            # Capture address and burst length
            base_addr = int(self._sig("araddr").value)
            arlen = int(self._sig("arlen").value)
            burst_len = arlen + 1

            # Accept AR
            self._sig("arready").value = 1
            await RisingEdge(self.clk)
            self._sig("arready").value = 0

            self.reads.append((base_addr, burst_len))

            # Send R beats
            for beat_idx in range(burst_len):
                # Latency simulation
                delay = self.rvalid_delay
                if self.jitter_range > 0:
                    delay += random.randint(0, self.jitter_range)
                for _ in range(delay):
                    await RisingEdge(self.clk)

                while self.stall_rvalid:
                    await RisingEdge(self.clk)

                # Read data from memory
                beat_addr = base_addr + beat_idx * 8
                data = self.read_dword(beat_addr)

                self._sig("rdata").value = data
                self._sig("rvalid").value = 1
                self._sig("rlast").value = 1 if (beat_idx == burst_len - 1) else 0
                self._sig("rresp").value = 0  # OKAY

                # Wait for RREADY
                while True:
                    await RisingEdge(self.clk)
                    try:
                        if int(self._sig("rready").value) == 1:
                            break
                    except ValueError:
                        pass

                self._sig("rvalid").value = 0
                self._sig("rlast").value = 0


class Axi3SharedSlave:
    """Mock AXI3 slave servicing both read and write channels on shared memory.

    Models contention: a single credit pool (max_outstanding) is shared
    between read and write transactions.  When credits are exhausted, the
    next channel to request blocks.  This approximates the PS7 HP port
    interconnect where TX (read) and RX (write) AXI3 masters compete for
    DDR controller bandwidth.

    Args:
        dut: cocotb DUT handle.
        read_prefix:  signal name prefix for the read channel  (e.g. "m_axi_tx_").
        write_prefix: signal name prefix for the write channel (e.g. "m_axi_rx_").
        clk: clock signal (default dut.clk).
        max_outstanding: max concurrent transactions across both channels.
        rvalid_delay: fixed extra cycles of latency on each R beat.
    """

    def __init__(self, dut, read_prefix, write_prefix, clk=None,
                 max_outstanding=2, rvalid_delay=0):
        self.dut = dut
        self.clk = clk if clk is not None else dut.clk
        self._rp = read_prefix
        self._wp = write_prefix
        self.mem = {}
        self.max_outstanding = max_outstanding
        self.credits = max_outstanding
        self.rvalid_delay = rvalid_delay
        self.reads = []   # (base_addr, burst_len)
        self.writes = []  # (base_addr, [beat_data, ...])
        self._tasks = []

    def _r_sig(self, name):
        return getattr(self.dut, self._rp + name)

    def _w_sig(self, name):
        return getattr(self.dut, self._wp + name)

    # ---- Memory access ----

    def read_dword(self, byte_addr):
        val = 0
        for i in range(8):
            val |= self.mem.get(byte_addr + i, 0) << (8 * i)
        return val

    def write_dword(self, byte_addr, value):
        for i in range(8):
            self.mem[byte_addr + i] = (value >> (8 * i)) & 0xFF

    def write_iq_samples(self, base_addr, samples):
        for i in range(0, len(samples), 2):
            re0, im0 = samples[i]
            word_even = pack_iq_word(re0, im0)
            if i + 1 < len(samples):
                re1, im1 = samples[i + 1]
                word_odd = pack_iq_word(re1, im1)
            else:
                word_odd = 0
            beat = (word_odd << 32) | (word_even & 0xFFFFFFFF)
            self.write_dword(base_addr + (i // 2) * 8, beat)

    def read_iq_samples(self, base_addr, n_samples):
        result = []
        for i in range(0, n_samples, 2):
            addr = base_addr + (i // 2) * 8
            beat = self.read_dword(addr)
            (re0, im0), (re1, im1) = unpack_iq_beat(beat)
            result.append((re0, im0))
            if i + 1 < n_samples:
                result.append((re1, im1))
        return result[:n_samples]

    # ---- Credit management (cooperative — safe without locks) ----

    async def _acquire_credit(self):
        while True:
            if self.credits > 0:
                self.credits -= 1
                return
            await RisingEdge(self.clk)

    async def _release_credit(self):
        self.credits += 1
        # Yield to other coroutines so the write channel gets a
        # chance to acquire before the read channel re-acquires.
        # Without this yield, tightly-looped read bursts can
        # starve writes in cocotb's cooperative scheduler.
        await RisingEdge(self.clk)

    # ---- Launch ----

    async def run(self):
        self._r_sig("arready").value = 0
        self._r_sig("rdata").value = 0
        self._r_sig("rvalid").value = 0
        self._r_sig("rlast").value = 0
        self._r_sig("rresp").value = 0
        self._w_sig("awready").value = 0
        self._w_sig("wready").value = 0
        self._w_sig("bresp").value = 0
        self._w_sig("bvalid").value = 0

        self._tasks = [
            cocotb.start_soon(self._read_channel()),
            cocotb.start_soon(self._write_channel()),
        ]

    def stop(self):
        """Stop background channel coroutines (for re-setup in one test)."""
        for task in self._tasks:
            task.kill()
        self._tasks = []

    # ---- Read channel (TX DDR reads) ----

    async def _read_channel(self):
        while True:
            while True:
                await RisingEdge(self.clk)
                try:
                    if int(self._r_sig("arvalid").value) == 1:
                        break
                except ValueError:
                    pass

            base_addr = int(self._r_sig("araddr").value)
            arlen = int(self._r_sig("arlen").value)
            burst_len = arlen + 1

            await self._acquire_credit()

            self._r_sig("arready").value = 1
            await RisingEdge(self.clk)
            self._r_sig("arready").value = 0

            self.reads.append((base_addr, burst_len))

            for beat_idx in range(burst_len):
                for _ in range(self.rvalid_delay):
                    await RisingEdge(self.clk)

                beat_addr = base_addr + beat_idx * 8
                data = self.read_dword(beat_addr)

                self._r_sig("rdata").value = data
                self._r_sig("rvalid").value = 1
                self._r_sig("rlast").value = 1 if (beat_idx == burst_len - 1) else 0
                self._r_sig("rresp").value = 0

                while True:
                    await RisingEdge(self.clk)
                    try:
                        if int(self._r_sig("rready").value) == 1:
                            break
                    except ValueError:
                        pass

                self._r_sig("rvalid").value = 0
                self._r_sig("rlast").value = 0

            await self._release_credit()

    # ---- Write channel (RX DDR writes) ----

    async def _write_channel(self):
        while True:
            while True:
                await RisingEdge(self.clk)
                try:
                    if int(self._w_sig("awvalid").value) == 1:
                        break
                except ValueError:
                    pass

            base_addr = int(self._w_sig("awaddr").value)
            awlen = int(self._w_sig("awlen").value)
            burst_len = awlen + 1

            await self._acquire_credit()

            self._w_sig("awready").value = 1
            await RisingEdge(self.clk)
            self._w_sig("awready").value = 0

            beat_data = []
            for beat_idx in range(burst_len):
                # Sample WDATA mid-cycle, then accept on the next rising
                # edge.  The RX W data is a combinational output.
                await FallingEdge(self.clk)
                while True:
                    try:
                        if int(self._w_sig("wvalid").value) == 1:
                            break
                    except ValueError:
                        pass
                    await FallingEdge(self.clk)

                data = int(self._w_sig("wdata").value)
                self._w_sig("wready").value = 1
                await RisingEdge(self.clk)
                self._w_sig("wready").value = 0

                beat_data.append(data)
                beat_addr = base_addr + beat_idx * 8
                for b in range(8):
                    self.mem[beat_addr + b] = (data >> (8 * b)) & 0xFF

            self._w_sig("bresp").value = 0
            self._w_sig("bvalid").value = 1

            while True:
                await RisingEdge(self.clk)
                try:
                    if int(self._w_sig("bready").value) == 1:
                        break
                except ValueError:
                    pass

            self._w_sig("bvalid").value = 0
            self.writes.append((base_addr, beat_data))
            await self._release_credit()
