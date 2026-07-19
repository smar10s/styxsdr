#!/usr/bin/env python3
"""Traffic Analysis — capture and decode all WiFi traffic on a channel.

Captures ~30s of real airtime from PlutoSDR, runs full decode on every
detected frame, and produces a comprehensive breakdown of WiFi traffic
composition including frame types, rates, networks, timing, and utilization.

Usage:
    # Live capture (requires PlutoSDR)
    python tools/hardware/traffic_analysis.py

    # Save capture for offline replay
    python tools/hardware/traffic_analysis.py --save capture.npy

    # Offline analysis from saved file
    python tools/hardware/traffic_analysis.py --load capture.npy

    # Custom parameters
    python tools/hardware/traffic_analysis.py --freq 5200000000 --gain 60 --duration 15
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

SAMPLE_RATE = 20_000_000  # 20 MSPS — matches 802.11 20 MHz channel

# --- Defaults ---
DEFAULT_FREQ = 5_180_000_000  # ch36
DEFAULT_GAIN = 30             # moderate — avoid ADC clipping from nearby APs
DEFAULT_DURATION = 30.0       # seconds
MIN_GAP = 320                 # minimum samples between detections (one preamble)

TYPE_CATEGORIES = {0: "Management", 1: "Control", 2: "Data", 3: "Extension"}


def analyze(iq, duration):
    """Run full decode pipeline on IQ capture with progress reporting.

    Returns results dict with pipeline stats, frame info, and timing.
    """
    ctx = DecodeContext()
    offset = 0
    total_samples = len(iq)

    # Progress reporting
    progress_interval = int(5.0 * SAMPLE_RATE)  # every 5s of data
    next_progress = progress_interval

    # Pipeline counters
    stf_detections = 0
    lsig_ok = 0
    fcs_ok_count = 0
    fcs_fail_count = 0
    sig_fail_count = 0

    # Frame details (FCS OK only)
    frame_types = {}       # name -> count
    category_counts = {}   # category name -> count
    rate_dist = {}         # rate label -> count
    phy_format_counts = {"Legacy": 0, "HT": 0, "VHT": 0}
    bssid_ssid_map = {}    # bssid -> ssid or None
    ssids = set()

    # Timing and airtime (for sig_ok frames)
    frame_offsets_fcs_ok = []   # absolute sample offsets of fcs_ok frames
    total_airtime_samples = 0   # sum of frame durations (sig_ok)

    while offset < total_samples - 1000:
        # Progress reporting
        if offset >= next_progress:
            processed_sec = offset / SAMPLE_RATE
            print(f"  Progress: {processed_sec:.1f}s / {duration:.1f}s "
                  f"({stf_detections} STF, {fcs_ok_count} FCS OK)")
            next_progress += progress_interval

        stf = detect_stf(iq[offset:], threshold=0.5, min_periods=6)
        if stf < 0:
            break

        abs_offset = offset + stf
        stf_detections += 1

        result = ctx.decode_frame(iq, abs_offset)

        if result is None or result.get("error") == "SIGNAL parity fail":
            sig_fail_count += 1
            offset = abs_offset + MIN_GAP
            continue

        if result is not None and "error" in result and "SIGNAL" not in result.get("error", ""):
            # L-SIG was OK but something else failed
            lsig_ok += 1
            sig = result.get("signal", {})
            n_sym = sig.get("n_symbols", 1)
            frame_samples = 320 + n_sym * 80
            total_airtime_samples += frame_samples
            # Count as FCS fail
            fcs_fail_count += 1
            offset = abs_offset + max(frame_samples, MIN_GAP)
            continue

        # L-SIG decoded successfully
        lsig_ok += 1
        sig = result.get("signal", {})
        n_sym = sig.get("n_symbols", 1)
        frame_samples = 320 + n_sym * 80
        total_airtime_samples += frame_samples

        fcs_ok = result.get("fcs_ok", False)

        if fcs_ok:
            fcs_ok_count += 1
            frame_offsets_fcs_ok.append(abs_offset)
            psdu = result.get("psdu", b"")

            # PHY format
            ft = result.get("frame_type", "legacy")
            if ft == "ht":
                phy_format_counts["HT"] += 1
            elif ft == "vht":
                phy_format_counts["VHT"] += 1
            else:
                phy_format_counts["Legacy"] += 1

            # Rate distribution
            if ft == "ht":
                ht_sig = result.get("ht_sig", {})
                mcs = ht_sig.get("mcs", 0)
                rate_label = f"HT MCS{mcs}"
            elif ft == "vht":
                vht_sig = result.get("vht_sig_a", {})
                mcs = vht_sig.get("mcs", 0)
                rate_label = f"VHT MCS{mcs}"
            else:
                rate_mbps = sig.get("rate_mbps", 0)
                rate_label = f"{rate_mbps} Mbps"
            rate_dist[rate_label] = rate_dist.get(rate_label, 0) + 1

            # Frame classification
            frame_type, subtype, name = classify_frame(psdu)
            frame_types[name] = frame_types.get(name, 0) + 1

            # Category
            if frame_type is not None:
                cat = TYPE_CATEGORIES.get(frame_type, "Unknown")
                category_counts[cat] = category_counts.get(cat, 0) + 1

            # BSSID extraction (management frames)
            bssid = extract_bssid(psdu)
            if bssid:
                if bssid not in bssid_ssid_map:
                    bssid_ssid_map[bssid] = None

            # SSID from beacons and probe responses
            if frame_type == 0 and subtype in (5, 8):
                ssid = extract_ssid(psdu)
                if ssid:
                    ssids.add(ssid)
                    if bssid:
                        bssid_ssid_map[bssid] = ssid
        else:
            fcs_fail_count += 1

        # Advance past frame
        offset = abs_offset + max(frame_samples, MIN_GAP)

    # Final progress
    processed_sec = min(offset / SAMPLE_RATE, duration)
    print(f"  Done: {processed_sec:.1f}s processed "
          f"({stf_detections} STF, {fcs_ok_count} FCS OK)")

    # Inter-frame timing (FCS OK frames)
    timing_stats = {}
    if len(frame_offsets_fcs_ok) > 1:
        times_us = np.array(frame_offsets_fcs_ok, dtype=np.float64) / SAMPLE_RATE * 1e6
        gaps_us = np.diff(np.sort(times_us))
        timing_stats = {
            "median_gap_us": float(np.median(gaps_us)),
            "mean_gap_us": float(np.mean(gaps_us)),
            "min_gap_us": float(np.min(gaps_us)),
            "max_gap_us": float(np.max(gaps_us)),
        }

    # Channel utilization
    total_airtime_sec = total_airtime_samples / SAMPLE_RATE
    utilization = total_airtime_sec / duration if duration > 0 else 0.0

    return {
        "duration": duration,
        "stf_detections": stf_detections,
        "lsig_ok": lsig_ok,
        "fcs_ok": fcs_ok_count,
        "fcs_fail": fcs_fail_count,
        "sig_fail": sig_fail_count,
        "frame_types": frame_types,
        "category_counts": category_counts,
        "rate_dist": rate_dist,
        "phy_format_counts": phy_format_counts,
        "bssid_ssid_map": bssid_ssid_map,
        "ssids": sorted(ssids),
        "timing_stats": timing_stats,
        "total_airtime_sec": total_airtime_sec,
        "utilization": utilization,
    }


def print_report(stats):
    """Print formatted traffic analysis report."""
    duration = stats["duration"]
    stf = stats["stf_detections"]
    lsig = stats["lsig_ok"]
    fcs_ok = stats["fcs_ok"]
    fcs_fail = stats["fcs_fail"]
    sig_fail = stats["sig_fail"]

    # Percentages relative to STF
    lsig_pct = (lsig / stf * 100) if stf > 0 else 0
    fcs_pct = (fcs_ok / stf * 100) if stf > 0 else 0

    print()
    print("TRAFFIC ANALYSIS REPORT")
    print("=" * 72)

    # --- Decode Pipeline ---
    print()
    print("DECODE PIPELINE")
    print(f"  Duration:         {duration:.1f}s")
    print(f"  STF detections:   {stf}")
    print(f"  L-SIG OK:         {lsig} ({lsig_pct:.0f}%)")
    print(f"  FCS OK:           {fcs_ok} ({fcs_pct:.0f}%)")
    print(f"  FCS FAIL:         {fcs_fail}")
    print(f"  SIG FAIL:         {sig_fail}")
    print(f"  Frames/sec:       {stf / duration:.1f} (STF)" if duration > 0 else "")
    print(f"  Decoded/sec:      {fcs_ok / duration:.1f} (FCS OK)" if duration > 0 else "")

    # --- Frame Type Composition ---
    print()
    print("FRAME TYPE COMPOSITION (FCS OK only)")
    cat_counts = stats["category_counts"]
    total_frames = sum(cat_counts.values())

    print(f"  {'Category':<16}{'Count':>6}    {'Pct':>5}")
    print(f"  {'-' * 30}")
    for cat in ["Management", "Control", "Data", "Extension"]:
        count = cat_counts.get(cat, 0)
        if count > 0:
            pct = count / total_frames * 100 if total_frames > 0 else 0
            print(f"  {cat:<16}{count:>6}   {pct:>5.1f}%")

    print()
    print(f"  {'Frame Type':<16}{'Count':>6}    {'Pct':>5}")
    print(f"  {'-' * 30}")
    frame_types = stats["frame_types"]
    for name, count in sorted(frame_types.items(), key=lambda x: -x[1]):
        pct = count / total_frames * 100 if total_frames > 0 else 0
        print(f"  {name:<16}{count:>6}   {pct:>5.1f}%")

    # --- Rate/MCS Distribution ---
    print()
    print("RATE/MCS DISTRIBUTION (FCS OK)")
    phy = stats["phy_format_counts"]
    print(f"  PHY format: Legacy={phy['Legacy']} HT={phy['HT']} VHT={phy['VHT']}")
    print()
    rate_dist = stats["rate_dist"]
    if rate_dist:
        print(f"  {'Rate/MCS':<16}{'Count':>6}")
        for rate_label, count in sorted(rate_dist.items()):
            print(f"  {rate_label:<16}{count:>6}")

    # --- Networks ---
    print()
    print("NETWORKS SEEN")
    bssid_map = stats["bssid_ssid_map"]
    print(f"  BSSIDs: {len(bssid_map)}")
    for bssid, ssid in sorted(bssid_map.items()):
        ssid_str = ssid if ssid else "(unknown)"
        print(f"    {bssid}  {ssid_str}")
    ssids = stats["ssids"]
    print(f"  SSIDs: {{{', '.join(ssids)}}}" if ssids else "  SSIDs: (none)")

    # --- Inter-frame Timing ---
    print()
    print("INTER-FRAME TIMING (FCS OK frames)")
    timing = stats["timing_stats"]
    if timing:
        median_us = timing["median_gap_us"]
        mean_us = timing["mean_gap_us"]
        min_us = timing["min_gap_us"]
        max_us = timing["max_gap_us"]

        def fmt_time(us):
            if us >= 1000:
                return f"{us / 1000:.2f} ms"
            return f"{us:.0f} us"

        print(f"  Median gap:  {fmt_time(median_us)}")
        print(f"  Mean gap:    {fmt_time(mean_us)}")
        print(f"  Min gap:     {fmt_time(min_us)}")
        print(f"  Max gap:     {fmt_time(max_us)}")
    else:
        print("  (insufficient frames for timing analysis)")

    # --- Channel Utilization ---
    print()
    print("CHANNEL UTILIZATION (rough estimate)")
    airtime_ms = stats["total_airtime_sec"] * 1000
    duration_ms = duration * 1000
    util_pct = stats["utilization"] * 100
    print(f"  Decoded frame airtime: {airtime_ms:.1f} ms / {duration_ms:.0f} ms = {util_pct:.1f}%")
    print(f"  (This undercounts: misses frames we can't decode)")


def build_json_output(stats):
    """Build JSON-serializable results dict."""
    return {
        "test": "traffic_analysis",
        "duration_sec": stats["duration"],
        "pipeline": {
            "stf_detections": stats["stf_detections"],
            "lsig_ok": stats["lsig_ok"],
            "fcs_ok": stats["fcs_ok"],
            "fcs_fail": stats["fcs_fail"],
            "sig_fail": stats["sig_fail"],
        },
        "frame_types": stats["frame_types"],
        "category_counts": stats["category_counts"],
        "rate_distribution": stats["rate_dist"],
        "phy_formats": stats["phy_format_counts"],
        "networks": {
            "bssids": stats["bssid_ssid_map"],
            "ssids": stats["ssids"],
        },
        "timing": stats["timing_stats"],
        "utilization": {
            "airtime_sec": stats["total_airtime_sec"],
            "duration_sec": stats["duration"],
            "fraction": stats["utilization"],
        },
    }


def main():
    parser = argparse.ArgumentParser(
        description="Traffic Analysis — capture and analyze all WiFi traffic on a channel"
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
        duration = len(iq) / SAMPLE_RATE
        print(f"  Samples: {len(iq)} ({duration:.2f}s at {SAMPLE_RATE/1e6:.0f} MSPS)")
    else:
        # Live capture mode
        from pluto import find_pluto, configure_rx, capture

        ch_num = int((args.freq - 5e9) / 5e6)
        print(f"Traffic Analysis — ch{ch_num} ({args.freq/1e6:.0f} MHz), "
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
    stats = analyze(iq, duration)
    elapsed = time.time() - t0
    print(f"  Analysis complete in {elapsed:.2f}s")

    # --- Report ---
    print_report(stats)

    # --- JSON output ---
    results = build_json_output(stats)
    output_file = "traffic_analysis_results.json"
    with open(output_file, "w") as f:
        json.dump(results, f, indent=2)
    print()
    print(f"Results saved to {output_file}")


if __name__ == "__main__":
    main()
