# Goal

Remove the ADI PL expansion-header cores, `axi_spi` (0x7C430000) and
`axi_iic` (0x41600000). They cost ~900 LUTs and no Styx user needs them.
Deimos (~89% LUT) gets the same saving. Merge only after Deimos passes
its gates on this branch.

# Status

The change is complete and passes every Styx hardware gate.
- BD: both cores, their ports, interrupts and interconnect slots are gone.
  `system_top.v` keeps the phaser mux; the freed SPI/I2C pins idle.
- Device tree: the overlay disables `i2c@41600000` in all three DTBs.
  `package.sh` disables `spi@7C430000`, which only the Rev.C DTB has.
- HAL: the AD9361 IIO device is found by name. Removing the ADM1177
  moves it from `iio:device1` to `iio:device0`.

Build `0xcd15e8d0`: timing MET, WNS +2.193 ns. It uses 3489 LUT (was
4369), 5492 FF, 3.5 BRAM and 2 DSP.

Hardware, Rev.B Pluto:
- Boots clean, with no I2C probe. `make validate` passes; `ad9361-phy` is
  now `iio:device0`.
- `pluto_ad9361_readback`: 3/3 pass.
- `pluto_dma_test`: 7/7 pass.
- HIL golden vector: pass at 20 and 30.72 MSPS.
- `hardware_baseline.sh` on cable loopback passes 9/9:
  - signal ladder L6/6
  - 5180 MHz: 80/80 frames
  - 915 MHz: 79/80 frames
  - 12 s streaming TX: 0 dropouts, 0 stalls

An earlier run failed RF on this and the previous bitstream alike. The
Pluto had an antenna on instead of the loopback cable.

# Next Steps

1. Deimos (`trim/pl-header-peripherals`, pinned to `5739a64`):
   - Done: build `0x77280121`, timing MET, WNS +0.290, 14,749 LUT (was
     15,591). A second placement strategy also meets timing.
   - Done: `session_end.sh` passes on cable loopback.
   - Done: OTA gate 80/80 EAPOL. `merge_check.sh` passes.
2. shesha (`trim/pl-header-peripherals`, pinned to `5739a64`):
   - Done: the test no longer hardcodes `iio:device1`.
   - Done: hardware suite 48/48 on Deimos, cable loopback.
   - Done: the manual test with the antenna (24 EAPOL vs 6 disassoc).
3. Push this branch, then Deimos and shesha. Deimos is green, so this
   can merge to `main`. Then repin Neso.

# Known Issues

- The ADM1177 input power and current readings are gone.
- `make sim` needs `.venv/bin` on `PATH` to find `cocotb-config`.
