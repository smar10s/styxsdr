#!/usr/bin/env python3
"""Beacon TX via C — generates beacon IQ with C tool, transmits via PlutoSDR.

Usage:
    python tools/hardware/test_tx_beacon_c.py
    python tools/hardware/test_tx_beacon_c.py --ssid "MyNet" --channel 36
    python tools/hardware/test_tx_beacon_c.py --dry-run  # just generate, don't transmit
"""

import argparse
import os
import subprocess
import sys
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))

from frame_utils import CHANNEL_FREQ

# Path to compiled tx_beacon tool (relative to project root)
PROJECT_ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
TX_BEACON = os.path.join(PROJECT_ROOT, "build-cmake", "tools", "tx_beacon")

def main():
    parser = argparse.ArgumentParser(description="Beacon TX via C library")
    parser.add_argument("--ssid", default="lib80211-test", help="SSID to broadcast")
    parser.add_argument("--channel", type=int, default=36, help="Channel number")
    parser.add_argument("--interval", type=int, default=20, help="Beacon interval (TU)")
    parser.add_argument("--count", type=int, default=100, help="Number of beacons")
    parser.add_argument("--tx-atten", type=int, default=0, help="TX attenuation (dB)")
    parser.add_argument("--dry-run", action="store_true", help="Generate only, don't transmit")
    args = parser.parse_args()

    if args.channel not in CHANNEL_FREQ:
        print(f"Error: unsupported channel {args.channel}", file=sys.stderr)
        sys.exit(1)

    # Generate ONE beacon period (beacon + silence gap) for cyclic DMA.
    # The Pluto repeats the cyclic buffer automatically — no need for multiple copies.
    with tempfile.NamedTemporaryFile(suffix=".cf32", delete=False) as f:
        cf32_path = f.name

    cmd = [TX_BEACON, "-s", args.ssid, "-c", str(args.channel),
           "-i", str(args.interval), "-n", "1", "-o", cf32_path]
    print(f"Generating: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error: {result.stderr}", file=sys.stderr)
        sys.exit(1)
    print(result.stdout.strip())

    # Load as complex64
    iq = np.fromfile(cf32_path, dtype=np.float32).view(np.complex64)
    print(f"Cyclic buffer: {len(iq)} samples ({len(iq)/20e6*1e3:.1f} ms, "
          f"{iq.nbytes/1024/1024:.1f} MB)")

    if args.dry_run:
        print("Dry run — not transmitting")
        os.unlink(cf32_path)
        return

    # Transmit via PlutoSDR
    try:
        from pluto import find_pluto, configure_tx, transmit, stop_tx
    except ImportError:
        print("Error: pluto.py not found (run from tools/hardware/)", file=sys.stderr)
        sys.exit(1)

    uri, sdr = find_pluto()
    freq = CHANNEL_FREQ[args.channel]
    configure_tx(sdr, freq, args.tx_atten)

    print(f"\nTransmitting SSID '{args.ssid}' on ch{args.channel} ({freq/1e6:.0f} MHz)")
    print("Press Ctrl+C to stop\n")
    print("Verify with:")
    print("  macOS: Option-click WiFi icon, look for SSID")
    print(f"  Linux: iw dev wlan0 scan | grep -A5 '{args.ssid}'")

    try:
        transmit(sdr, iq)
        import signal
        signal.pause()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop_tx(sdr)
        os.unlink(cf32_path)

if __name__ == "__main__":
    main()
