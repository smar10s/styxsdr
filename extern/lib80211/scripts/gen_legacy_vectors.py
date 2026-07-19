#!/usr/bin/env python3.11
"""Generate legacy 802.11a waveform golden vectors for all 8 rates.

Produces vectors/legacy_{rate}mbps_waveform.json for rates:
6, 9, 12, 18, 24, 36, 48, 54 Mbps.

Uses the same Annex I.1 PSDU (96 payload bytes → 100 with FCS) and
scrambler seed 0x5D that the HT/VHT golden vectors use.

The Python generate_frame() appends FCS internally, so we pass the
96-byte payload (without FCS). The C API receives the full 100 bytes
(with FCS pre-appended), but both process identical data.
"""

import json
import sys
from pathlib import Path

# Add the Python module path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from py80211.gen_ofdm_frame import generate_frame

# Annex I.1 PSDU — 96 bytes payload (without FCS).
# The C test uses 100 bytes (these 96 + 4-byte FCS = 0x67 0x33 0x21 0xB6).
# Python's generate_frame() appends FCS internally.
ANNEX_I1_PSDU = bytes([
    0x04, 0x02, 0x00, 0x2E, 0x00, 0x60, 0x08, 0xCD, 0x37, 0xA6,
    0x00, 0x20, 0xD6, 0x01, 0x3C, 0xF1, 0x00, 0x60, 0x08, 0xAD,
    0x3B, 0xAF, 0x00, 0x00, 0x4A, 0x6F, 0x79, 0x2C, 0x20, 0x62,
    0x72, 0x69, 0x67, 0x68, 0x74, 0x20, 0x73, 0x70, 0x61, 0x72,
    0x6B, 0x20, 0x6F, 0x66, 0x20, 0x64, 0x69, 0x76, 0x69, 0x6E,
    0x69, 0x74, 0x79, 0x2C, 0x0A, 0x44, 0x61, 0x75, 0x67, 0x68,
    0x74, 0x65, 0x72, 0x20, 0x6F, 0x66, 0x20, 0x45, 0x6C, 0x79,
    0x73, 0x69, 0x75, 0x6D, 0x2C, 0x0A, 0x46, 0x69, 0x72, 0x65,
    0x2D, 0x69, 0x6E, 0x73, 0x69, 0x72, 0x65, 0x64, 0x20, 0x77,
    0x65, 0x20, 0x74, 0x72, 0x65, 0x61,
])

SCRAMBLER_SEED = 0x5D
RATES = [6, 9, 12, 18, 24, 36, 48, 54]

def main():
    vectors_dir = Path(__file__).resolve().parent.parent / "vectors"
    vectors_dir.mkdir(exist_ok=True)

    print(f"PSDU: {len(ANNEX_I1_PSDU)} bytes (without FCS)")
    print(f"PSDU first 10 bytes: {ANNEX_I1_PSDU[:10].hex()}")
    print(f"Scrambler seed: 0x{SCRAMBLER_SEED:02X}")
    print()

    for rate in RATES:
        iq, meta = generate_frame(rate, ANNEX_I1_PSDU, scrambler_seed=SCRAMBLER_SEED)

        # Build JSON matching the HT waveform vector format
        vec = {
            "source": "py80211 gen_ofdm_frame.py (IEEE 802.11-2020 Section 17)",
            "description": f"Complete legacy 802.11a PPDU waveform at {rate} Mbps (time domain)",
            "rate_mbps": rate,
            "length": len(iq),
            "real": [float(x) for x in iq.real],
            "imag": [float(x) for x in iq.imag],
        }

        out_path = vectors_dir / f"legacy_{rate}mbps_waveform.json"
        with open(out_path, "w") as f:
            json.dump(vec, f, indent=None, separators=(",", ":"))

        print(f"  {rate:2d} Mbps: {len(iq):5d} samples → {out_path.name}")

    print()
    print("Done. All 8 legacy waveform vectors generated.")


if __name__ == "__main__":
    main()
