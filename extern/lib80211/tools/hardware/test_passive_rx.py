#!/usr/bin/env python3
"""Passive RX test — capture real AP traffic and validate decode pipeline.

Captures IQ from the air on ch36 (5180 MHz), runs STF detection and full
frame decode on every detected frame, and reports pass/fail against minimum
criteria.

Pass criteria:
  1. At least 1 frame with FCS OK
  2. At least 1 beacon decoded
  3. SSID extracted from beacon

Usage:
    # Live capture (requires PlutoSDR)
    python tools/hardware/test_passive_rx.py

    # Save capture for offline replay
    python tools/hardware/test_passive_rx.py --save capture.npy

    # Offline analysis from saved file
    python tools/hardware/test_passive_rx.py --load capture.npy

    # Custom parameters
    python tools/hardware/test_passive_rx.py --freq 5200000000 --gain 60 --duration 10
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
sys.path.insert(0, os.path.dirname(__file__))

from py80211.decode_frame import detect_stf, DecodeContext
from frame_utils import classify_frame, extract_bssid, extract_ssid

# --- Defaults ---
DEFAULT_FREQ = 5_180_000_000  # ch36
DEFAULT_GAIN = 30             # moderate — avoid ADC clipping from nearby APs
DEFAULT_DURATION = 5.0        # seconds
MIN_GAP = 320                 # minimum samples between detections (one preamble)


def analyze(iq):
    """Run full decode pipeline on IQ capture. Returns results dict."""
    ctx = DecodeContext()
    offset = 0

    stats = {
        "stf_detections": 0,
        "lsig_ok": 0,
        "fcs_ok": 0,
        "beacons": 0,
        "ssids": set(),
        "bssids": set(),
        "rate_dist": {},     # rate_mbps -> count (FCS OK only)
        "frame_types": {},   # name -> count (FCS OK only)
    }

    while offset < len(iq) - 1000:
        stf = detect_stf(iq[offset:], threshold=0.5, min_periods=6)
        if stf < 0:
            break

        abs_offset = offset + stf
        stats["stf_detections"] += 1

        result = ctx.decode_frame(iq, abs_offset)

        if result is None or "error" in result:
            offset = abs_offset + MIN_GAP
            continue

        stats["lsig_ok"] += 1
        fcs_ok = result.get("fcs_ok", False)

        if fcs_ok:
            stats["fcs_ok"] += 1
            psdu = result.get("psdu", b"")

            # Rate distribution
            sig = result.get("signal", {})
            rate = sig.get("rate_mbps", 0)
            if result.get("frame_type") == "ht":
                ht_sig = result.get("ht_sig", {})
                mcs = ht_sig.get("mcs", 0)
                rate_label = f"HT MCS{mcs}"
            elif result.get("frame_type") == "vht":
                vht_sig = result.get("vht_sig_a", {})
                mcs = vht_sig.get("mcs", 0)
                rate_label = f"VHT MCS{mcs}"
            else:
                rate_label = f"{rate} Mbps"
            stats["rate_dist"][rate_label] = stats["rate_dist"].get(rate_label, 0) + 1

            # Frame classification
            frame_type, subtype, name = classify_frame(psdu)
            stats["frame_types"][name] = stats["frame_types"].get(name, 0) + 1

            # BSSID extraction (management frames)
            bssid = extract_bssid(psdu)
            if bssid:
                stats["bssids"].add(bssid)

            # Beacon detection and SSID extraction
            if frame_type == 0 and subtype == 8:  # Beacon
                stats["beacons"] += 1
                ssid = extract_ssid(psdu)
                if ssid:
                    stats["ssids"].add(ssid)

            # Also extract SSID from probe responses
            if frame_type == 0 and subtype == 5:  # Probe Resp
                ssid = extract_ssid(psdu)
                if ssid:
                    stats["ssids"].add(ssid)

        # Advance past frame
        sig = result.get("signal", {})
        n_sym = sig.get("n_symbols", 1)
        frame_samples = 320 + n_sym * 80
        offset = abs_offset + max(frame_samples, MIN_GAP)

    return stats


def print_results(stats, duration):
    """Print formatted test results."""
    print()
    print("=" * 60)
    print("PASSIVE RX TEST RESULTS")
    print("=" * 60)
    print(f"  Duration:        {duration:.1f}s")
    print(f"  STF detections:  {stats['stf_detections']}")
    print(f"  L-SIG OK:        {stats['lsig_ok']}")
    print(f"  FCS OK:          {stats['fcs_ok']}")
    print(f"  Beacons:         {stats['beacons']}")
    print(f"  SSIDs:           {stats['ssids'] or set()}")
    print(f"  BSSIDs:          {stats['bssids'] or set()}")
    print()

    if stats["rate_dist"]:
        print("  Rate distribution (FCS OK):")
        for rate, count in sorted(stats["rate_dist"].items()):
            print(f"    {rate}: {count}")
        print()

    if stats["frame_types"]:
        print("  Frame types (FCS OK):")
        for name, count in sorted(stats["frame_types"].items(), key=lambda x: -x[1]):
            print(f"    {name:<16}: {count}")
        print()


def check_pass_criteria(stats):
    """Evaluate pass criteria. Returns (overall_pass, criteria_list)."""
    criteria = []

    # Criterion 1: at least 1 frame with FCS OK
    fcs_count = stats["fcs_ok"]
    passed = fcs_count >= 1
    criteria.append({
        "name": "At least 1 frame with FCS OK",
        "passed": passed,
        "detail": f"got {fcs_count}",
    })

    # Criterion 2: at least 1 beacon decoded
    beacon_count = stats["beacons"]
    passed = beacon_count >= 1
    criteria.append({
        "name": "At least 1 beacon decoded",
        "passed": passed,
        "detail": f"got {beacon_count}",
    })

    # Criterion 3: SSID extracted from beacon
    ssids = stats["ssids"]
    passed = len(ssids) >= 1
    criteria.append({
        "name": "SSID extracted from beacon",
        "passed": passed,
        "detail": f"got {ssids or set()}",
    })

    overall = all(c["passed"] for c in criteria)
    return overall, criteria


def print_pass_criteria(overall, criteria):
    """Print pass/fail criteria report."""
    print("PASS CRITERIA")
    for c in criteria:
        status = "PASS" if c["passed"] else "FAIL"
        print(f"  [{status}] {c['name']} ({c['detail']})")
    print()
    result = "PASS" if overall else "FAIL"
    print(f"  OVERALL: {result}")
    print()


def build_json_output(stats, duration, overall, criteria):
    """Build JSON-serializable results dict."""
    return {
        "test": "passive_rx",
        "duration_sec": duration,
        "stf_detections": stats["stf_detections"],
        "lsig_ok": stats["lsig_ok"],
        "fcs_ok": stats["fcs_ok"],
        "beacons": stats["beacons"],
        "ssids": sorted(stats["ssids"]),
        "bssids": sorted(stats["bssids"]),
        "rate_distribution": stats["rate_dist"],
        "frame_types": stats["frame_types"],
        "criteria": [
            {"name": c["name"], "passed": c["passed"], "detail": c["detail"]}
            for c in criteria
        ],
        "overall_pass": overall,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Passive RX test — capture AP traffic and validate decode pipeline"
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
                        help="Save raw IQ to .npy file")
    parser.add_argument("--load", type=str, default=None,
                        help="Load IQ from .npy file (offline analysis)")
    args = parser.parse_args()

    duration = args.duration

    if args.load:
        # Offline mode: load from file
        print(f"Loading IQ from: {args.load}")
        iq = np.load(args.load)
        iq = iq.astype(np.complex64)
        from pluto import SAMPLE_RATE
        duration = len(iq) / SAMPLE_RATE
        print(f"  Samples: {len(iq)} ({duration:.2f}s at {SAMPLE_RATE/1e6:.0f} MSPS)")
    else:
        # Live capture mode
        from pluto import find_pluto, configure_rx, capture, SAMPLE_RATE

        print(f"Passive RX test — ch{int((args.freq - 5e9) / 5e6)}, "
              f"gain={args.gain} dB, duration={duration}s")
        print()

        uri, sdr = find_pluto(args.uri)
        print(f"  PlutoSDR: {uri}")

        configure_rx(sdr, args.freq, args.gain)
        print(f"  Configured: {args.freq/1e6:.0f} MHz, {args.gain} dB gain")
        print(f"  Capturing {duration}s...")

        t0 = time.time()
        iq = capture(sdr, duration)
        elapsed = time.time() - t0
        print(f"  Captured {len(iq)} samples in {elapsed:.2f}s")

    if args.save:
        np.save(args.save, iq)
        print(f"  Saved IQ to: {args.save} ({iq.nbytes / (1024*1024):.1f} MB)")

    # --- Analysis ---
    print()
    print("Analyzing...")
    t0 = time.time()
    stats = analyze(iq)
    elapsed = time.time() - t0
    print(f"  Analysis complete in {elapsed:.2f}s")

    # --- Results ---
    print_results(stats, duration)

    overall, criteria = check_pass_criteria(stats)
    print_pass_criteria(overall, criteria)

    # --- JSON output ---
    results = build_json_output(stats, duration, overall, criteria)
    print("JSON: " + json.dumps(results, indent=2))

    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()
