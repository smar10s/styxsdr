#!/usr/bin/env python3
"""Passive RX test using C decoder — validates C decode pipeline on real traffic.

Captures IQ from PlutoSDR (or loads from file), saves as cf32, invokes the
C rx_file tool for decoding, and validates results.

Pass criteria:
  1. At least 1 frame with FCS OK
  2. At least 1 beacon decoded
  3. SSID extracted from beacon

Usage:
    # Live capture (requires PlutoSDR)
    python tools/hardware/test_passive_rx_c.py

    # Save capture for offline replay
    python tools/hardware/test_passive_rx_c.py --save capture.cf32

    # Offline analysis from saved file
    python tools/hardware/test_passive_rx_c.py --load capture.cf32

    # Custom parameters
    python tools/hardware/test_passive_rx_c.py --freq 5200000000 --gain 60 --duration 10
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))

# --- Defaults ---
DEFAULT_FREQ = 5_180_000_000  # ch36
DEFAULT_GAIN = 30             # moderate — avoid ADC clipping from nearby APs
DEFAULT_DURATION = 5.0        # seconds
SAMPLE_RATE = 20_000_000      # 20 MSPS — matches lib80211 and pluto.py


def find_rx_file():
    """Find rx_file binary.

    Checks RX_FILE environment variable first, then looks in standard
    build directories relative to the project root.
    """
    env = os.environ.get("RX_FILE")
    if env and os.path.isfile(env):
        return env
    project_root = os.path.join(os.path.dirname(__file__), "..", "..")
    candidates = [
        os.path.join(project_root, "build", "tools", "rx_file"),
        os.path.join(project_root, "build-arm", "tools", "rx_file"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)
    sys.exit(
        "Error: rx_file not found.\n"
        "  Set RX_FILE env var, or build with: cmake --build build\n"
        "  Searched: build/tools/rx_file, build-arm/tools/rx_file"
    )


def capture_iq(freq, gain, duration, uri=None):
    """Capture IQ from PlutoSDR.

    Returns numpy complex64 array.
    """
    from pluto import find_pluto, configure_rx, capture, SAMPLE_RATE

    print(f"Passive RX test (C decoder) — ch{int((freq - 5e9) / 5e6)}, "
          f"gain={gain} dB, duration={duration}s")
    print()

    found_uri, sdr = find_pluto(uri)
    print(f"  PlutoSDR: {found_uri}")

    configure_rx(sdr, freq, gain)
    print(f"  Configured: {freq/1e6:.0f} MHz, {gain} dB gain")
    print(f"  Capturing {duration}s...")

    import time
    t0 = time.time()
    iq = capture(sdr, duration)
    elapsed = time.time() - t0
    print(f"  Captured {len(iq)} samples in {elapsed:.2f}s")

    return iq


def run_decoder(cf32_path, rx_file_path):
    """Run rx_file on a cf32 file and return parsed JSON output.

    Returns parsed dict on success, None on failure.
    """
    result = subprocess.run(
        [rx_file_path, "-q", cf32_path],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print(f"rx_file error (exit {result.returncode}): {result.stderr}",
              file=sys.stderr)
        return None
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as e:
        print(f"rx_file JSON parse error: {e}", file=sys.stderr)
        print(f"stdout was: {result.stdout[:500]}", file=sys.stderr)
        return None


def print_results(data, duration):
    """Print formatted test results from rx_file JSON output."""
    summary = data["summary"]
    frames = data.get("frames", [])
    n_samples = data.get("n_samples", 0)

    print()
    print("=" * 60)
    print("PASSIVE RX TEST (C decoder) RESULTS")
    print("=" * 60)
    print(f"Capture: {n_samples} samples ({duration:.3f} sec)")
    print(f"Decoder: rx_file (C library)")
    print()

    stf = summary["stf_detections"]
    decoded = summary["frames_decoded"]
    fcs_ok = summary["fcs_ok"]
    fcs_fail = summary["fcs_fail"]

    print(f"STF detections:    {stf}")
    print(f"Frames decoded:    {decoded}")
    print(f"FCS OK:            {fcs_ok}")
    print(f"FCS fail:          {fcs_fail}")
    print()

    beacons = summary["beacons"]
    ssids = summary.get("ssids", [])
    print(f"Beacons:           {beacons}")
    print(f"SSIDs found:       {', '.join(ssids) if ssids else '(none)'}")

    # Collect unique BSSIDs from frames
    bssids = set()
    for frame in frames:
        bssid = frame.get("bssid")
        if bssid:
            bssids.add(bssid)
    print(f"BSSIDs:            {', '.join(sorted(bssids)) if bssids else '(none)'}")
    print()

    # Frame type distribution
    frame_types = summary.get("frame_types", {})
    if frame_types:
        total_frames = sum(frame_types.values())
        print("Frame types:")
        for name, count in sorted(frame_types.items(), key=lambda x: -x[1]):
            pct = 100.0 * count / total_frames if total_frames > 0 else 0
            print(f"  {name:<16} {count:>3}  ({pct:.1f}%)")
        print()

    # Rate distribution
    rate_dist = summary.get("rate_dist", {})
    if rate_dist:
        total_rates = sum(rate_dist.values())
        print("Rate distribution:")
        for rate, count in sorted(rate_dist.items(), key=lambda x: -x[1]):
            pct = 100.0 * count / total_rates if total_rates > 0 else 0
            print(f"  {rate:<16} {count:>3}  ({pct:.1f}%)")
        print()


def check_pass_criteria(summary):
    """Evaluate pass criteria. Returns (overall_pass, criteria_list)."""
    criteria = []

    # Criterion 1: at least 1 frame with FCS OK
    fcs_count = summary["fcs_ok"]
    passed = fcs_count >= 1
    criteria.append({
        "name": "At least 1 frame with FCS OK",
        "passed": passed,
        "detail": f"got {fcs_count}",
    })

    # Criterion 2: at least 1 beacon decoded
    beacon_count = summary["beacons"]
    passed = beacon_count >= 1
    criteria.append({
        "name": "At least 1 beacon decoded",
        "passed": passed,
        "detail": f"got {beacon_count}",
    })

    # Criterion 3: SSID extracted from beacon
    ssids = summary.get("ssids", [])
    passed = len(ssids) >= 1
    criteria.append({
        "name": "SSID extracted from beacon",
        "passed": passed,
        "detail": f"got {ssids}",
    })

    overall = all(c["passed"] for c in criteria)
    return overall, criteria


def print_pass_criteria(overall, criteria, ssids):
    """Print pass/fail criteria report."""
    print("=" * 60)
    if overall:
        ssid_str = ssids[0] if ssids else ""
        print(f"PASS: Beacon decoded, SSID extracted: {ssid_str}")
    else:
        print("FAIL: Pass criteria not met")
        for c in criteria:
            status = "PASS" if c["passed"] else "FAIL"
            print(f"  [{status}] {c['name']} ({c['detail']})")
    print("=" * 60)
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Passive RX test (C decoder) — capture AP traffic and "
                    "validate C decode pipeline"
    )
    parser.add_argument("--freq", type=int, default=DEFAULT_FREQ,
                        help=f"Center frequency in Hz (default: {DEFAULT_FREQ})")
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                        help=f"RX gain in dB (default: {DEFAULT_GAIN})")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help=f"Capture duration in seconds (default: {DEFAULT_DURATION})")
    parser.add_argument("--uri", type=str, default=None,
                        help="PlutoSDR URI (e.g. usb:1.4.5 or ip:192.168.2.1)")
    parser.add_argument("--save", type=str, default=None,
                        help="Save raw IQ to .cf32 file")
    parser.add_argument("--load", type=str, default=None,
                        help="Load IQ from .cf32 file (offline analysis)")
    args = parser.parse_args()

    rx_file = find_rx_file()
    duration = args.duration
    cleanup = False

    if args.load:
        # Offline mode: use provided cf32 file directly
        cf32_path = args.load
        if not os.path.isfile(cf32_path):
            print(f"Error: file not found: {cf32_path}", file=sys.stderr)
            sys.exit(1)
        file_size = os.path.getsize(cf32_path)
        n_samples = file_size // 8  # 4 bytes I + 4 bytes Q
        duration = n_samples / SAMPLE_RATE
        print(f"Loading IQ from: {cf32_path}")
        print(f"  Samples: {n_samples} ({duration:.3f}s at "
              f"{SAMPLE_RATE/1e6:.0f} MSPS)")
    else:
        # Live capture mode
        iq = capture_iq(args.freq, args.gain, args.duration, args.uri)

        # Save as cf32 (numpy complex64 is interleaved float32 = cf32)
        if args.save:
            cf32_path = args.save
        else:
            tmp = tempfile.NamedTemporaryFile(suffix=".cf32", delete=False)
            cf32_path = tmp.name
            tmp.close()
            cleanup = True

        iq.tofile(cf32_path)
        print(f"  Saved cf32: {cf32_path} ({os.path.getsize(cf32_path) / (1024*1024):.1f} MB)")

    # Run C decoder
    print()
    print(f"Running decoder: {os.path.basename(rx_file)}")
    data = run_decoder(cf32_path, rx_file)

    # Cleanup temp file
    if cleanup:
        os.unlink(cf32_path)

    if data is None:
        print("FAIL: decoder returned no output", file=sys.stderr)
        sys.exit(1)

    # Print results
    print_results(data, duration)

    # Pass/fail evaluation
    summary = data["summary"]
    overall, criteria = check_pass_criteria(summary)
    ssids = summary.get("ssids", [])
    print_pass_criteria(overall, criteria, ssids)

    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()
