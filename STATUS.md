# Goal

Make HIL playback follow the ADC sample rate, removing the hard-coded
20 MSPS pacing. Validate at 20 and 30.72 MSPS on hardware.

# Status

README and STATUS rewritten. Project ID is now part of the build
fingerprint. RTL, tools, and tests pending.

# Next Steps

1. `hil_ctrl`: pace `test_valid` on `adc_valid`; delete the modulo-5 path.
2. `test_hil_ctrl`: drive `adc_valid`; assert cadence follows the strobe.
3. `hil_inject`: explicit rate, wait for ADC valid, rate-derived time math.
4. `registers.md` and RTL/tool comments: remove all "1 per 5 / 20 MSPS" claims.
5. Build, timing MET, flash, validate fingerprint and project ID.
6. HIL at 20 and 30.72 via snap; live ADC capture sanity at 20.
7. Push branch. Deimos and Neso pins follow separately.

# Known Issues

- HIL stalls if the ADC is not streaming.
- HIL/TX region is 32 MB, ~0.27 s at 30.72; use short vectors.
- Styx has no merge-rule scripts; validation is manual.
