# SPDX-License-Identifier: MIT
"""Cocotb tests for axi_build_id.v — read-only build fingerprint register.

Tests:
    1. Read returns BUILD_ID parameter value
    2. Write accepted without bus hang (BRESP=OKAY)
    3. Write does not modify the read-only value
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from axi_lite_driver import AxiLiteMaster


async def setup(dut):
    """Start clock, reset, return AXI-Lite driver."""
    cocotb.start_soon(Clock(dut.s_axi_aclk, 10, unit="ns").start())

    # Reset (active-low)
    dut.s_axi_aresetn.value = 0
    # Deassert all AXI inputs during reset
    dut.s_axi_araddr.value = 0
    dut.s_axi_arprot.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axi_awaddr.value = 0
    dut.s_axi_awprot.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wdata.value = 0
    dut.s_axi_wstrb.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0

    for _ in range(5):
        await RisingEdge(dut.s_axi_aclk)

    dut.s_axi_aresetn.value = 1
    await RisingEdge(dut.s_axi_aclk)
    await RisingEdge(dut.s_axi_aclk)

    axi = AxiLiteMaster(dut, prefix="s_axi_", clk=dut.s_axi_aclk)
    return axi


@cocotb.test()
async def test_read_returns_build_id(dut):
    """Read offset 0 returns the BUILD_ID parameter (0xDEADBEEF default)."""
    axi = await setup(dut)

    rdata, rresp = await axi.read(0x00)

    assert rresp == 0, f"Expected RRESP=OKAY, got {rresp}"
    assert rdata == 0xDEAD_BEEF, f"Expected BUILD_ID=0xDEADBEEF, got 0x{rdata:08X}"


@cocotb.test()
async def test_write_accepted_no_error(dut):
    """Write completes with BRESP=OKAY (bus doesn't hang)."""
    axi = await setup(dut)

    bresp = await axi.write(0x00, 0x1234_5678)

    assert bresp == 0, f"Expected BRESP=OKAY, got {bresp}"


@cocotb.test()
async def test_read_after_write_unchanged(dut):
    """Write arbitrary value, read back — still returns BUILD_ID (read-only)."""
    axi = await setup(dut)

    # Write something
    await axi.write(0x00, 0xCAFE_BABE)

    # Read back — should still be the BUILD_ID
    rdata, rresp = await axi.read(0x00)

    assert rresp == 0, f"Expected RRESP=OKAY, got {rresp}"
    assert rdata == 0xDEAD_BEEF, \
        f"Write modified read-only register! Got 0x{rdata:08X}, expected 0xDEADBEEF"
