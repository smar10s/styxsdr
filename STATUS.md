# Goal

Make HIL playback follow the ADC sample rate, removing the hard-coded
20 MSPS pacing. Validate at 20 and 30.72 MSPS on hardware.

# Status

RTL, tests, tools, and docs are done. HIL advances on `adc_valid` after
a FIFO fill gate. ADC_CNT counts in both modes. A sticky underrun
counter lives at 0x14 (W1C). The mux select is registered on the strobe.

Bitstream fingerprint `0x59c34a0d`, timing MET (WNS +2.071 ns), 4369
LUTs, project ID `STYX`. The FIFO now maps to block RAM instead of 128
LUTRAMs, which frees the Deimos placement margin without changing HIL
behavior. Flashed and `make validate` passes.

Hardware evidence:
- HIL golden vector at 20 MSPS: watermark OK, 512 samples verified.
- HIL golden vector at 30.72 MSPS: watermark OK, 512 samples verified.
- Live ADC capture at 20 MSPS: 119,600 samples, 0 clipped.
- No-strobe path: AD9361 in sleep gives `ADC_CNT +0 in 100 ms`, exit 1.

# Next Steps

1. Human merges this branch to `main`.
2. Deimos repins to `main`, reruns `merge_check.sh`, and merges.
3. Neso repin after the Styx merge to `main`.

# Known Issues

- The underrun counter is proven in sim. It was not forced on StyX or
  Deimos hardware.
- `pluto_stream_test` still fails its ARM feed-rate check at 20 MSPS.
  That is the known host-feed limit, not an ADC or HIL failure.
