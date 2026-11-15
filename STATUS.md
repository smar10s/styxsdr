# Goal

Make HIL playback follow the ADC sample rate, removing the hard-coded
20 MSPS pacing. Validate at 20 and 30.72 MSPS on hardware.

# Status

RTL, tests, tools, and docs are done. HIL advances on `adc_valid` after
a FIFO fill gate. ADC_CNT counts in both modes. A sticky underrun
counter lives at 0x14 (W1C). The mux select is registered on the strobe.

Bitstream fingerprint `0x4299c167`, timing MET (WNS +1.124 ns), project
ID `STYX`. Flashed and `make validate` passes.

Hardware evidence:
- HIL golden vector at 20 MSPS: watermark OK, 512 samples verified.
- HIL golden vector at 30.72 MSPS: watermark OK, 512 samples verified.
- Live ADC capture at 20 MSPS: 119,600 samples, 0 clipped.
- No-strobe path: AD9361 in sleep gives `ADC_CNT +0 in 100 ms`, exit 1.

# Next Steps

1. Deimos `hil/follow-adc-rate`: pin this Styx tip, add the ADC
   preflight tool, update tests, and run the HIL regression.
2. Push this branch when asked.
3. Neso repin after the Styx merge to `main`.

# Known Issues

- The underrun counter is proven in sim, not forced on hardware.
- `pluto_stream_test` still fails its ARM feed-rate check at 20 MSPS.
  That is the known host-feed limit, not an ADC or HIL failure.
- Deimos HIL does not configure the ADC yet; its preflight tool is the
  remaining piece before its regression can run.
