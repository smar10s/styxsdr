#!/usr/bin/env python3
"""Check PMF (802.11w) status of nearby APs by parsing RSN IE from beacons.

Captures beacons on ch36, parses the RSN Information Element (tag 48),
and reports whether Management Frame Protection is advertised. This
determines whether deauth injection will work against a target network.

RSN Capabilities field (2 bytes, offset varies):
  Bit 6: MFPR (Management Frame Protection Required)
  Bit 7: MFPC (Management Frame Protection Capable)

Usage:
    # Live capture (requires PlutoSDR)
    python tools/hardware/check_pmf.py

    # Target a specific BSSID
    python tools/hardware/check_pmf.py --bssid aa:bb:cc:dd:ee:ff

    # Offline analysis from saved capture
    python tools/hardware/check_pmf.py --load captures/some_capture.npy

    # Custom parameters
    python tools/hardware/check_pmf.py --freq 5180000000 --gain 30 --duration 5
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
sys.path.insert(0, os.path.dirname(__file__))

from py80211.decode_frame import detect_stf, DecodeContext
from frame_utils import extract_bssid, extract_ssid

# --- Defaults ---
DEFAULT_FREQ = 5_180_000_000  # ch36
DEFAULT_GAIN = 30
DEFAULT_DURATION = 5.0
MIN_GAP = 320


def parse_rsn_ie(rsn_body: bytes) -> dict:
    """Parse RSN IE body (after tag ID and length bytes).

    Returns dict with cipher/AKM info and PMF capability bits.
    Reference: IEEE 802.11-2020 Section 9.4.2.25
    """
    result = {}

    if len(rsn_body) < 2:
        return {"error": "RSN IE too short"}

    # Version (must be 1)
    offset = 0
    version = int.from_bytes(rsn_body[offset:offset + 2], 'little')
    result["version"] = version
    offset += 2

    if version != 1:
        return {"error": f"Unknown RSN version {version}"}

    # Group Data Cipher Suite (4 bytes: OUI + type)
    if offset + 4 > len(rsn_body):
        result["mfpc"] = False
        result["mfpr"] = False
        return result
    result["group_cipher"] = _cipher_name(rsn_body[offset:offset + 4])
    offset += 4

    # Pairwise Cipher Suite Count + List
    if offset + 2 > len(rsn_body):
        result["mfpc"] = False
        result["mfpr"] = False
        return result
    pw_count = int.from_bytes(rsn_body[offset:offset + 2], 'little')
    offset += 2
    result["pairwise_ciphers"] = []
    for _ in range(pw_count):
        if offset + 4 > len(rsn_body):
            break
        result["pairwise_ciphers"].append(_cipher_name(rsn_body[offset:offset + 4]))
        offset += 4

    # AKM Suite Count + List
    if offset + 2 > len(rsn_body):
        result["mfpc"] = False
        result["mfpr"] = False
        return result
    akm_count = int.from_bytes(rsn_body[offset:offset + 2], 'little')
    offset += 2
    result["akm_suites"] = []
    for _ in range(akm_count):
        if offset + 4 > len(rsn_body):
            break
        result["akm_suites"].append(_akm_name(rsn_body[offset:offset + 4]))
        offset += 4

    # RSN Capabilities (2 bytes)
    if offset + 2 > len(rsn_body):
        # No capabilities field — PMF not advertised
        result["mfpc"] = False
        result["mfpr"] = False
        result["note"] = "RSN Capabilities field absent"
        return result

    caps = int.from_bytes(rsn_body[offset:offset + 2], 'little')
    result["rsn_caps_raw"] = f"0x{caps:04x}"
    result["mfpc"] = bool(caps & (1 << 7))  # bit 7
    result["mfpr"] = bool(caps & (1 << 6))  # bit 6
    result["pre_auth"] = bool(caps & (1 << 0))
    result["no_pairwise"] = bool(caps & (1 << 1))
    # PTKSA replay counter: bits 2-3
    # GTKSA replay counter: bits 4-5

    return result


def _cipher_name(suite: bytes) -> str:
    """Human-readable cipher suite name."""
    # OUI 00-0F-AC (IEEE)
    if suite[:3] == b'\x00\x0f\xac':
        cipher_type = suite[3]
        names = {
            0: "Use Group",
            1: "WEP-40",
            2: "TKIP",
            4: "CCMP-128",
            5: "WEP-104",
            6: "BIP-CMAC-128",
            8: "GCMP-128",
            9: "GCMP-256",
            10: "CCMP-256",
            11: "BIP-GMAC-128",
            12: "BIP-GMAC-256",
            13: "BIP-CMAC-256",
        }
        return names.get(cipher_type, f"00-0F-AC:{cipher_type}")
    return suite.hex(":")


def _akm_name(suite: bytes) -> str:
    """Human-readable AKM suite name."""
    if suite[:3] == b'\x00\x0f\xac':
        akm_type = suite[3]
        names = {
            1: "802.1X (RSN)",
            2: "PSK (RSN)",
            3: "FT-802.1X",
            4: "FT-PSK",
            5: "802.1X-SHA256",
            6: "PSK-SHA256",
            8: "SAE",
            9: "FT-SAE",
            12: "802.1X-SHA384",
            13: "FT-802.1X-SHA384",
            18: "OWE",
        }
        return names.get(akm_type, f"00-0F-AC:{akm_type}")
    return suite.hex(":")


def find_rsn_ie(psdu: bytes) -> bytes | None:
    """Find RSN IE (tag 48) in beacon/probe response tagged parameters.

    Beacon body starts at offset 36 (24B MAC header + 12B fixed fields).
    Returns the IE body (after tag+length) or None.
    """
    if len(psdu) < 38:
        return None
    offset = 36
    # Walk IEs (don't include FCS in the search — last 4 bytes)
    end = len(psdu) - 4
    while offset + 2 <= end:
        tag_id = psdu[offset]
        tag_len = psdu[offset + 1]
        if offset + 2 + tag_len > end:
            break
        if tag_id == 48:  # RSN IE
            return bytes(psdu[offset + 2:offset + 2 + tag_len])
        offset += 2 + tag_len
    return None



def scan_beacons(iq, early_stop=True):
    """Decode beacons from IQ capture. Returns list of beacon info dicts.

    Processes the capture in chunks to avoid O(N^2) behavior from
    calling detect_stf on the full remaining buffer each iteration.
    The _bulk_autocorr_metric inside detect_stf allocates arrays the
    size of its input — with 100M samples that's ~800MB per call.

    Args:
        iq: Full IQ capture (complex64/128 array).
        early_stop: If True, stop after finding one beacon per unique BSSID
                    (faster for the PMF check use case).
    """
    ctx = DecodeContext()
    offset = 0
    beacons = []
    seen_bssids = set()
    total = len(iq)

    # Process in 2M-sample chunks (100ms). Large enough to contain any
    # single frame (~2400 samples for a beacon). We overlap by one max
    # frame length to avoid missing frames at chunk boundaries.
    CHUNK = 2_000_000
    OVERLAP = 10_000  # generous overlap for frame spanning boundary
    last_progress = 0

    while offset < total - 1000:
        # Progress reporting (every 20M samples = 1s)
        if offset - last_progress >= 20_000_000:
            elapsed_s = offset / 20e6
            print(f"    ... {elapsed_s:.1f}s scanned, "
                  f"{len(beacons)} beacon(s) found")
            last_progress = offset

        # Extract chunk from current offset
        chunk_end = min(offset + CHUNK, total)
        chunk = iq[offset:chunk_end]

        stf = detect_stf(chunk, threshold=0.5, min_periods=6)
        if stf < 0:
            # No frame in this chunk — advance past it (minus overlap)
            offset += max(CHUNK - OVERLAP, MIN_GAP)
            continue

        abs_offset = offset + stf
        result = ctx.decode_frame(iq, abs_offset)

        if result is not None and "error" not in result and result.get("fcs_ok"):
            psdu = result.get("psdu", b"")
            if len(psdu) >= 2:
                fc = psdu[0] | (psdu[1] << 8)
                frame_type = (fc >> 2) & 0x03
                subtype = (fc >> 4) & 0x0F
                if frame_type == 0 and subtype == 8:  # Beacon
                    bssid = extract_bssid(psdu)
                    ssid = extract_ssid(psdu)
                    rsn_body = find_rsn_ie(psdu)
                    beacons.append({
                        "bssid": bssid,
                        "ssid": ssid,
                        "rsn_body": rsn_body,
                        "offset": abs_offset,
                    })
                    if bssid:
                        seen_bssids.add(bssid)
                    # For PMF check we only need one beacon per BSSID
                    if early_stop and bssid and len(beacons) >= 1:
                        # Keep scanning briefly in case there are multiple APs
                        # but don't need dozens of beacons from same BSSID
                        pass

        # Advance past this frame
        if result and "error" not in result:
            sig = result.get("signal", {})
            n_sym = sig.get("n_symbols", 1)
            frame_samples = 320 + n_sym * 80
            offset = abs_offset + max(frame_samples, MIN_GAP)
        else:
            offset = abs_offset + MIN_GAP

    return beacons


def main():
    parser = argparse.ArgumentParser(
        description="Check PMF (802.11w) status of nearby APs")
    parser.add_argument("--freq", type=float, default=DEFAULT_FREQ,
                        help="Center frequency in Hz (default: 5180 MHz / ch36)")
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                        help="RX gain in dB (default: 30)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help="Capture duration in seconds (default: 5)")
    parser.add_argument("--uri", type=str, default=None,
                        help="PlutoSDR URI (auto-detect if omitted)")
    parser.add_argument("--bssid", type=str, default=None,
                        help="Filter to specific BSSID (e.g., aa:bb:cc:dd:ee:ff)")
    parser.add_argument("--load", type=str, default=None,
                        help="Load IQ from .npy file instead of live capture")
    args = parser.parse_args()

    # Acquire IQ
    if args.load:
        print(f"Loading capture from {args.load}...")
        iq = np.load(args.load)
        print(f"  {len(iq):,} samples ({len(iq)/20e6:.1f}s)")
    else:
        from pluto import find_pluto, configure_rx, capture
        uri, sdr = find_pluto(args.uri)
        print(f"PlutoSDR: {uri}")
        print(f"Freq: {args.freq/1e6:.0f} MHz, Gain: {args.gain} dB, "
              f"Duration: {args.duration}s")
        configure_rx(sdr, args.freq, gain_db=args.gain)
        print("Capturing...")
        iq = capture(sdr, args.duration)
        print(f"  Got {len(iq):,} samples")

    # Find beacons
    print("\nScanning for beacons...")
    beacons = scan_beacons(iq)
    print(f"  Found {len(beacons)} beacon(s)")

    if not beacons:
        print("\nNo beacons found. Try increasing duration or gain.")
        sys.exit(1)

    # Group by BSSID
    by_bssid = {}
    for b in beacons:
        bssid = b["bssid"]
        if bssid not in by_bssid:
            by_bssid[bssid] = b

    # Filter if requested
    if args.bssid:
        target = args.bssid.lower()
        by_bssid = {k: v for k, v in by_bssid.items() if k == target}
        if not by_bssid:
            print(f"\nBSSID {args.bssid} not found in capture.")
            print("Available BSSIDs:")
            for b in beacons:
                if b["bssid"]:
                    print(f"  {b['bssid']}  SSID={b['ssid'] or '(hidden)'}")
            sys.exit(1)

    # Report
    print()
    print("=" * 65)
    print("PMF (802.11w) STATUS REPORT")
    print("=" * 65)

    for bssid, info in sorted(by_bssid.items()):
        ssid = info["ssid"] or "(hidden)"
        print(f"\n  BSSID: {bssid}")
        print(f"  SSID:  {ssid}")

        rsn_body = info["rsn_body"]
        if rsn_body is None:
            print(f"  RSN IE: NOT PRESENT (open network or WEP)")
            print(f"  PMF:   N/A (no RSN)")
            print(f"  Deauth: VIABLE (no protection)")
            continue

        rsn = parse_rsn_ie(rsn_body)
        if "error" in rsn:
            print(f"  RSN IE: PARSE ERROR: {rsn['error']}")
            continue

        print(f"  RSN IE:")
        print(f"    Group cipher:    {rsn.get('group_cipher', '?')}")
        print(f"    Pairwise:        {', '.join(rsn.get('pairwise_ciphers', []))}")
        print(f"    AKM:             {', '.join(rsn.get('akm_suites', []))}")
        print(f"    Capabilities:    {rsn.get('rsn_caps_raw', '?')}")
        print(f"  PMF:")
        mfpc = rsn.get("mfpc", False)
        mfpr = rsn.get("mfpr", False)
        print(f"    MFPC (capable):  {mfpc}")
        print(f"    MFPR (required): {mfpr}")

        if mfpr:
            verdict = "REQUIRED — deauth will NOT work"
        elif mfpc:
            verdict = ("OPTIONAL — deauth may fail against clients that "
                       "negotiate PMF")
        else:
            verdict = "DISABLED — deauth is viable"

        print(f"  Verdict: {verdict}")

    print()


if __name__ == "__main__":
    main()
