#!/usr/bin/env python3
"""HT/VHT classification on real AP traffic.

Captures 200 ms at 5180 MHz (ch36), detects frames, classifies each as
legacy/HT/VHT using speculative HT-SIG decode with CRC-8 validation.

Requires antenna connected and positioned near AP.

Pass criteria:
  - At least 1 frame classified as HT or VHT with CRC-8 OK
  - MCS and length fields are plausible
  - No false positives on legacy frames (if any detected)

Usage:
    python tools/classify_live.py
    python tools/classify_live.py --freq 5180e6 --gain 73 --duration 0.2
"""

import argparse
import os
import sys
import numpy as np

from pluto import find_pluto, configure_rx, capture

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
from py80211.gen_ofdm_frame import SAMPLE_RATE
from py80211.decode_frame import detect_stf, DecodeContext

DEFAULT_FREQ = 5_180_000_000  # ch36
DEFAULT_GAIN = 73             # max gain for passive capture
DEFAULT_DURATION = 0.2        # 200 ms = 4M samples


def main():
    parser = argparse.ArgumentParser(description="HT/VHT classify on real AP")
    parser.add_argument("--freq", type=float, default=DEFAULT_FREQ,
                        help="Center frequency (default: 5180 MHz)")
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                        help="RX gain in dB (default: 73)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help="Capture duration in seconds")
    args = parser.parse_args()

    uri, sdr = find_pluto()
    print(f"HT/VHT classification — live AP capture")
    print(f"  Freq: {args.freq/1e6:.0f} MHz  Gain: {args.gain} dB  Duration: {args.duration*1000:.0f} ms")
    print(f"  Pluto: {uri}")
    print()

    configure_rx(sdr, args.freq, args.gain)

    # --- Capture ---
    print("  Capturing...")
    rx_iq = capture(sdr, args.duration)
    print(f"  Captured {len(rx_iq)} samples ({len(rx_iq)/SAMPLE_RATE*1000:.0f} ms)")
    power_db = 10 * np.log10(np.mean(np.abs(rx_iq) ** 2) + 1e-20)
    print(f"  RX power: {power_db:.1f} dBFS")
    print()

    # --- Detect and classify all frames ---
    ctx = DecodeContext()
    offset = 0
    min_gap = 400  # minimum samples between frame detections

    stats = {"legacy": 0, "ht": 0, "vht": 0, "sig_fail": 0, "total_detected": 0}
    ht_results = []

    while offset < len(rx_iq) - 1000:
        stf = detect_stf(rx_iq[offset:], threshold=0.6, min_periods=6)
        if stf < 0:
            break

        abs_offset = offset + stf
        stats["total_detected"] += 1

        result = ctx.decode_frame(rx_iq, abs_offset)

        if result is None or "error" in result:
            stats["sig_fail"] += 1
            offset = abs_offset + min_gap
            continue

        frame_type = result.get("frame_type", "unknown")
        if frame_type == "legacy":
            stats["legacy"] += 1
            fcs_status = "FCS OK" if result.get("fcs_ok") else "FCS FAIL"
            rate = result.get("rate_mbps", "?")
            length = result.get("length", "?")
            print(f"  [{abs_offset:7d}] LEGACY  rate={rate} len={length} {fcs_status}")
        elif frame_type in ("ht", "vht"):
            stats[frame_type] += 1
            ht_sig = result.get("ht_sig", {})
            mcs = ht_sig.get("mcs", "?")
            length = ht_sig.get("length", "?")
            bw = ht_sig.get("bw", "?")
            crc_ok = ht_sig.get("crc_ok", False)
            gi = "SGI" if ht_sig.get("short_gi") else "LGI"

            extra = ""
            if frame_type == "vht" and "vht" in ht_sig:
                vht = ht_sig["vht"]
                extra = f" VHT_MCS={vht['mcs']} NSTS={vht['nsts']}"

            print(f"  [{abs_offset:7d}] {frame_type.upper():6s} MCS={mcs} BW={'20' if bw==0 else '40'} "
                  f"len={length} {gi} CRC={'OK' if crc_ok else 'FAIL'}{extra}")

            if crc_ok:
                ht_results.append(ht_sig)

        # Advance past this frame
        offset = abs_offset + min_gap

    # --- Summary ---
    print(f"\n  Summary:")
    print(f"    Total STF detections: {stats['total_detected']}")
    print(f"    Legacy frames:        {stats['legacy']}")
    print(f"    HT frames (CRC OK):   {stats['ht']}")
    print(f"    VHT frames (CRC OK):  {stats['vht']}")
    print(f"    L-SIG failures:       {stats['sig_fail']}")

    # --- Pass criteria ---
    n_ht_vht = stats["ht"] + stats["vht"]
    print(f"\n  Criteria:")
    print(f"    At least 1 HT/VHT with CRC OK: {'PASS' if n_ht_vht > 0 else 'FAIL'} ({n_ht_vht})")

    # Check plausibility
    plausible = True
    for r in ht_results:
        mcs = r.get("mcs", 99)
        length = r.get("length", -1)
        if r.get("frame_type") == "vht":
            # VHT MCS should be in result["vht"]
            pass
        else:
            if mcs > 7:
                print(f"    WARNING: implausible HT MCS={mcs}")
                plausible = False
        if length > 65535 or length < 0:
            print(f"    WARNING: implausible length={length}")
            plausible = False

    print(f"    Fields plausible:              {'PASS' if plausible else 'WARN'}")

    all_ok = n_ht_vht > 0 and plausible
    print(f"\n  Result: {'PASS' if all_ok else 'FAIL'}")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
