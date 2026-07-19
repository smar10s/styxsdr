#!/usr/bin/env python3
"""Capture IQ samples from PlutoSDR and save to .npy file.

Usage:
    python tools/capture_iq.py --freq 900e6 --duration 0.1 --gain 50
    python tools/capture_iq.py --freq 2412e6 --duration 0.01 --gain 50 --file beacon.npy
"""

import argparse
import time

import numpy as np

from pluto import find_pluto, configure_rx, capture

DEFAULT_FREQ = 900e6
DEFAULT_DURATION = 0.1
DEFAULT_GAIN = 50


def main():
    parser = argparse.ArgumentParser(description="Capture IQ from PlutoSDR")
    parser.add_argument("--freq", type=float, default=DEFAULT_FREQ,
                        help=f"Center frequency in Hz (default: {DEFAULT_FREQ/1e6:.0f} MHz)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help="Capture duration in seconds")
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                        help="RX gain in dB (0-73)")
    parser.add_argument("--rate", type=float, default=20e6,
                        help="Sample rate in Hz")
    parser.add_argument("--file", type=str, default=None,
                        help="Output .npy file (default: auto-generated name)")
    args = parser.parse_args()

    uri, sdr = find_pluto()
    print(f"Found PlutoSDR at: {uri}")

    configure_rx(sdr, args.freq, args.gain, sample_rate=args.rate)
    print(f"  RX LO: {args.freq/1e6:.1f} MHz  Gain: {args.gain} dB  Rate: {args.rate/1e6:.1f} MSPS")

    iq = capture(sdr, args.duration)

    fname = args.file or f"iq_capture_{args.freq/1e6:.0f}MHz_{time.strftime('%Y%m%d_%H%M%S')}.npy"
    np.save(fname, iq)
    print(f"Saved {len(iq)} samples to {fname} ({iq.nbytes / (1024*1024):.1f} MB)")


if __name__ == "__main__":
    main()
