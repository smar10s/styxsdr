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

1. Deimos session, pinned to this branch:
   - Delete the `iic_*` and `pl_spi_*` wiring in its own `system_top.v`
     (copy this branch's).
   - Add `i2c@41600000` disabled to `deimos-pluto.dtso`, and add the
     Rev.C fdtput to its packaging, if it has its own.
   - Record WNS for at least two placement strategies, before and after.
   - Run its gates: loopback through OTA.
2. shesha: repin Styx to this branch for the new `hal.c`. Drop the
   hardcoded `iio:device1` in `client/tests/hardware/test_fm_tx_stop.py`.
   Then run shesha on Deimos as the final manual test.
3. Merge to `main` once Deimos is green. Then repin Neso.

# Known Issues

- The ADM1177 input power and current readings are gone.
- `make firmware` fails in `build/firmware`. Another account's stale
  cmake files there are not writable. Building with
  `FW_BUILD_DIR=build/firmware-gri` works.
- `make sim` needs `.venv/bin` on `PATH` to find `cocotb-config`.
