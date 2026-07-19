#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate hil_golden.json -- the styx HIL golden test vector.

First 3 samples: encoded watermark (re=ASCII code, im=0)
Remaining 1021 samples: chirp from 0.5 to 9.5 MHz at 20 MSPS, normalized.
"""
import json
import math

SAMPLE_RATE = 20e6
N_SAMPLES = 1024
CHIRP_F0 = 0.5e6
CHIRP_F1 = 9.5e6

# watermark: encode ASCII in 12-bit real values (scaled to float)
# hex-to-float: /2048 to match lib80211 scale
WATERMARK_CHARS = [0x61, 0x57, 0x63]
WATERMARK_REAL = [c / 2048.0 for c in WATERMARK_CHARS]
WATERMARK_IMAG = [0.0, 0.0, 0.0]

# Chirp: remaining samples
n_chirp = N_SAMPLES - len(WATERMARK_CHARS)
k = (CHIRP_F1 - CHIRP_F0) / (n_chirp / SAMPLE_RATE)

chirp_real = []
chirp_imag = []
for i in range(n_chirp):
    t = i / SAMPLE_RATE
    phase = 2.0 * math.pi * (CHIRP_F0 * t + 0.5 * k * t * t)
    chirp_real.append(math.cos(phase))
    chirp_imag.append(math.sin(phase))

# Combine
real = WATERMARK_REAL + chirp_real
imag = WATERMARK_IMAG + chirp_imag

# Output
waveform = {
    "real": [round(x, 8) for x in real],
    "imag": [round(x, 8) for x in imag],
}
print(json.dumps(waveform))
