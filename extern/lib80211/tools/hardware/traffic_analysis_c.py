#!/usr/bin/env python3
"""Traffic Analysis using C decoder — comprehensive WiFi traffic breakdown.

Captures extended IQ and produces detailed traffic composition analysis
using the compiled rx_file tool (C library) for decoding.

Usage:
    # Live capture (requires PlutoSDR)
    python tools/hardware/traffic_analysis_c.py

    # Save capture for offline replay
    python tools/hardware/traffic_analysis_c.py --save capture.cf32

    # Offline analysis from saved file
    python tools/hardware/traffic_analysis_c.py --load capture.cf32

    # Custom parameters
    python tools/hardware/traffic_analysis_c.py --freq 5200000000 --gain 60 --duration 30

    # JSON output
    python tools/hardware/traffic_analysis_c.py --load capture.cf32 --json results.json
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))

SAMPLE_RATE = 20_000_000  # 20 MSPS — matches 802.11 20 MHz channel
DEFAULT_FREQ = 5_180_000_000  # ch36
DEFAULT_GAIN = 30             # moderate — avoid ADC clipping from nearby APs
DEFAULT_DURATION = 15.0       # seconds

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
    """Capture IQ from PlutoSDR, return as complex64 array."""
    from pluto import find_pluto, configure_rx, capture

    ch_num = int((freq - 5e9) / 5e6)
    print(f"Traffic Analysis (C decoder) — ch{ch_num} ({freq/1e6:.0f} MHz), "
          f"gain={gain} dB, duration={duration}s")
    print()

    found_uri, sdr = find_pluto(uri)
    print(f"  PlutoSDR: {found_uri}")

    configure_rx(sdr, freq, gain)
    print(f"  Configured: {freq/1e6:.0f} MHz, {gain} dB gain")
    print(f"  Capturing {duration}s...")

    t0 = time.time()
    iq = capture(sdr, duration)
    elapsed = time.time() - t0
    print(f"  Captured {len(iq)} samples in {elapsed:.2f}s")
    return iq


def save_cf32(iq, path):
    """Save complex64 IQ as interleaved float32 (.cf32)."""
    iq = np.asarray(iq, dtype=np.complex64)
    iq.view(np.float32).tofile(path)
    return path


def run_decoder(cf32_path, rx_file_path):
    """Invoke rx_file on cf32 file, return parsed JSON."""
    cmd = [rx_file_path, "-q", cf32_path]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"rx_file failed (exit {result.returncode}):", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        sys.exit(1)
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as e:
        print(f"Failed to parse rx_file JSON output: {e}", file=sys.stderr)
        print(f"Output was: {result.stdout[:500]}", file=sys.stderr)
        sys.exit(1)


def analyze_networks(frames):
    """Group beacons by BSSID, count per network."""
    networks = defaultdict(lambda: {"ssid": None, "beacons": 0})
    for f in frames:
        if f.get("frame_name") == "Beacon" and f.get("fcs_ok"):
            bssid = f.get("bssid", "unknown")
            networks[bssid]["beacons"] += 1
            if f.get("ssid"):
                networks[bssid]["ssid"] = f["ssid"]
    return dict(networks)


def print_report(data, duration):
    """Print formatted traffic analysis report."""
    summary = data.get("summary", {})
    frames = data.get("frames", [])

    stf_detections = summary.get("stf_detections", 0)
    frames_decoded = summary.get("frames_decoded", 0)
    fcs_ok = summary.get("fcs_ok", 0)
    fcs_fail = summary.get("fcs_fail", 0)

    n_samples = data.get("n_samples", int(duration * SAMPLE_RATE))
    decode_rate = (frames_decoded / stf_detections * 100) if stf_detections > 0 else 0
    fcs_pass_rate = (fcs_ok / frames_decoded * 100) if frames_decoded > 0 else 0

    print()
    print("=" * 64)
    print("TRAFFIC ANALYSIS (C decoder)")
    print("=" * 64)
    print(f"Capture: {n_samples} samples ({duration:.3f} sec)")
    print(f"Decoder: rx_file (C library)")

    # --- Pipeline Statistics ---
    print()
    print("--- Pipeline Statistics ---")
    print(f"STF detections:     {stf_detections}")
    print(f"Frames decoded:     {frames_decoded}")
    print(f"FCS OK:             {fcs_ok}")
    print(f"FCS fail:           {fcs_fail}")
    print(f"Decode rate:        {decode_rate:.1f}%")
    print(f"FCS pass rate:      {fcs_pass_rate:.1f}%")

    # --- Frame Type Breakdown ---
    frame_types = summary.get("frame_types", {})
    total_frames = sum(frame_types.values())
    if frame_types:
        print()
        print("--- Frame Type Breakdown ---")
        for name, count in sorted(frame_types.items(), key=lambda x: -x[1]):
            pct = count / total_frames * 100 if total_frames > 0 else 0
            print(f"  {name:<20}{count:>4}  ({pct:.1f}%)")

    # --- Category Breakdown ---
    categories = summary.get("categories", {})
    total_cat = sum(categories.values())
    if categories:
        print()
        print("--- Category Breakdown ---")
        for cat, count in sorted(categories.items(), key=lambda x: -x[1]):
            pct = count / total_cat * 100 if total_cat > 0 else 0
            print(f"  {cat:<20}{count:>4}  ({pct:.1f}%)")

    # --- Rate/MCS Distribution ---
    rate_dist = summary.get("rate_dist", {})
    total_rate = sum(rate_dist.values())
    if rate_dist:
        print()
        print("--- Rate/MCS Distribution ---")
        for rate, count in sorted(rate_dist.items(), key=lambda x: -x[1]):
            pct = count / total_rate * 100 if total_rate > 0 else 0
            print(f"  {rate:<20}{count:>4}  ({pct:.1f}%)")

    # --- Networks Discovered ---
    networks = analyze_networks(frames)
    if networks:
        print()
        print("--- Networks Discovered ---")
        for bssid, info in sorted(networks.items(), key=lambda x: -x[1]["beacons"]):
            ssid = info["ssid"] or "(hidden)"
            print(f"  {bssid}  {ssid} ({info['beacons']} beacons)")

    # --- Timing & Utilization ---
    fps = fcs_ok / duration if duration > 0 else 0
    print()
    print("--- Timing & Utilization ---")
    print(f"  Frames per second:  {fps:.1f}")
    print(f"  Channel time:       {duration:.3f} sec")
    print("=" * 64)


def build_json_output(data, duration):
    """Build JSON-serializable results dict."""
    summary = data.get("summary", {})
    frames = data.get("frames", [])
    networks = analyze_networks(frames)

    return {
        "test": "traffic_analysis_c",
        "decoder": "rx_file",
        "n_samples": data.get("n_samples", 0),
        "duration_sec": duration,
        "pipeline": {
            "stf_detections": summary.get("stf_detections", 0),
            "frames_decoded": summary.get("frames_decoded", 0),
            "fcs_ok": summary.get("fcs_ok", 0),
            "fcs_fail": summary.get("fcs_fail", 0),
        },
        "frame_types": summary.get("frame_types", {}),
        "categories": summary.get("categories", {}),
        "rate_distribution": summary.get("rate_dist", {}),
        "networks": networks,
        "ssids": summary.get("ssids", []),
        "frames_per_second": summary.get("fcs_ok", 0) / duration if duration > 0 else 0,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Traffic Analysis (C decoder) — comprehensive WiFi traffic breakdown"
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
                        help="Save raw IQ capture to .cf32 file")
    parser.add_argument("--load", type=str, default=None,
                        help="Load IQ from .cf32 file (offline analysis)")
    parser.add_argument("--json", type=str, default=None,
                        help="Write JSON results to file")
    args = parser.parse_args()

    rx_file_path = find_rx_file()

    if args.load:
        # Offline mode: use provided cf32 directly
        cf32_path = args.load
        if not os.path.isfile(cf32_path):
            sys.exit(f"File not found: {cf32_path}")
        # Determine duration from file size
        file_bytes = os.path.getsize(cf32_path)
        n_samples = file_bytes // 8  # 4 bytes real + 4 bytes imag per sample
        duration = n_samples / SAMPLE_RATE
        print(f"Loading: {cf32_path}")
        print(f"  Samples: {n_samples} ({duration:.3f}s at {SAMPLE_RATE/1e6:.0f} MSPS)")
        cleanup_cf32 = False
    else:
        # Live capture mode
        iq = capture_iq(args.freq, args.gain, args.duration, args.uri)
        duration = len(iq) / SAMPLE_RATE

        if args.save:
            cf32_path = args.save
        else:
            fd, cf32_path = tempfile.mkstemp(suffix=".cf32")
            os.close(fd)
        save_cf32(iq, cf32_path)
        print(f"  Saved cf32: {cf32_path} ({os.path.getsize(cf32_path) / (1024*1024):.1f} MB)")
        n_samples = len(iq)
        cleanup_cf32 = not args.save

    # Run C decoder
    print()
    print("Decoding...")
    t0 = time.time()
    data = run_decoder(cf32_path, rx_file_path)
    elapsed = time.time() - t0
    print(f"  rx_file completed in {elapsed:.2f}s")

    # Use duration from JSON if available
    if "duration_sec" in data:
        duration = data["duration_sec"]

    # Print report
    print_report(data, duration)

    # JSON output
    if args.json:
        results = build_json_output(data, duration)
        with open(args.json, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nJSON results saved to: {args.json}")

    # Cleanup temp file
    if cleanup_cf32:
        os.unlink(cf32_path)


if __name__ == "__main__":
    main()
