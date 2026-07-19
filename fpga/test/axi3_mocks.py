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

import cocotb
from cocotb.triggers import RisingEdge, Timer
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
    """Mock AXI3 write slave that captures burst writes into a dict.

    Args:
        dut: cocotb DUT handle.
        prefix: Signal name prefix (default "m_axi_").
        clk: Clock signal (default dut.clk).
        wready_delay: Cycles to delay WREADY after WVALID (simulates slow DDR).
    """

    def __init__(self, dut, prefix="m_axi_", clk=None, wready_delay=0):
        self.dut = dut
        self.clk = clk if clk is not None else dut.clk
        self._p = prefix
        self.mem = {}  # byte_addr -> byte value
        self.bursts = []  # list of (base_addr, [beat_data, ...])
        self.wready_delay = wready_delay
        self.stall_wready = False  # if True, never assert WREADY

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

    async def run(self):
        """Main loop — run as cocotb.start_soon(slave.run())."""
        self._sig("awready").value = 0
        self._sig("wready").value = 0
        self._sig("bresp").value = 0
        self._sig("bvalid").value = 0
        # BID if it exists
        try:
            self._sig("bid").value = 0
        except AttributeError:
            pass

        while True:
            # Wait for AWVALID
            while True:
                await RisingEdge(self.clk)
                try:
                    if int(self._sig("awvalid").value) == 1:
                        break
                except ValueError:
                    pass  # X/Z during reset

            # Capture address
            base_addr = int(self._sig("awaddr").value)
            awlen = int(self._sig("awlen").value)
            burst_len = awlen + 1

            # Accept AW
            self._sig("awready").value = 1
            await RisingEdge(self.clk)
            self._sig("awready").value = 0

            # Receive W beats
            beat_data = []
            for beat_idx in range(burst_len):
                # Optional WREADY delay
                if self.wready_delay > 0:
                    for _ in range(self.wready_delay):
                        await RisingEdge(self.clk)

                if self.stall_wready:
                    # Stall forever (for overflow testing)
                    while self.stall_wready:
                        await RisingEdge(self.clk)

                self._sig("wready").value = 1

                # Wait for WVALID
                while True:
                    await RisingEdge(self.clk)
                    try:
                        if int(self._sig("wvalid").value) == 1:
                            break
                    except ValueError:
                        pass

                data = int(self._sig("wdata").value)
                beat_data.append(data)
                self._sig("wready").value = 0

                # Store in byte-addressed memory
                beat_addr = base_addr + beat_idx * 8
                for b in range(8):
                    self.mem[beat_addr + b] = (data >> (8 * b)) & 0xFF

            # Send B response
            self.bursts.append((base_addr, beat_data))
            self._sig("bresp").value = 0  # OKAY
            self._sig("bvalid").value = 1

            # Wait for BREADY
            while True:
                await RisingEdge(self.clk)
                try:
                    if int(self._sig("bready").value) == 1:
                        break
                except ValueError:
                    pass
            self._sig("bvalid").value = 0


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
