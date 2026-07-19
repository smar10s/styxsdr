#!/usr/bin/env python3
"""Diagnostic capture and analysis for beacon hunting.

Captures a large IQ buffer at 5180 MHz (ch36 primary 20 MHz), then runs
the full decode pipeline on every detected frame. Reports:
  - STF detections with metrics
  - L-SIG decode results (rate, length, parity)
  - HT/VHT classification attempts
  - Frame type identification (beacon, probe resp, data, etc.)
  - SSID extraction from any beacon/probe response with FCS OK
  - Power spectral density (quick FFT to confirm energy on channel)

Also supports a quick RF scan mode across the ch36 80 MHz block to
find which 20 MHz chunk has beacons.

Usage:
    # Default: 2 second capture at 5180 MHz, max gain
    python tools/diag_capture.py

    # Longer capture
    python tools/diag_capture.py --duration 5

    # Scan all 4 ch36 sub-channels (5180/5200/5220/5240)
    python tools/diag_capture.py --scan

    # Save raw IQ for offline analysis
    python tools/diag_capture.py --save capture.npy
"""

import argparse
import os
import sys
import time
import numpy as np

from pluto import find_pluto, configure_rx, capture

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
from py80211.gen_ofdm_frame import SAMPLE_RATE
from py80211.decode_frame import detect_stf, DecodeContext

DEFAULT_FREQ = 5_180_000_000  # ch36 primary
DEFAULT_GAIN = 73
DEFAULT_DURATION = 2.0        # 2 seconds = 40M samples

# 802.11 frame control subtypes (first 2 bytes of MAC header)
FRAME_TYPES = {
    0x0080: "Beacon",
    0x0040: "Probe Req",
    0x0050: "Probe Resp",
    0x00d4: "Ack",
    0x00b4: "RTS",
    0x00c4: "CTS",
    0x0048: "Null Data",
    0x0008: "Data",
    0x0088: "QoS Data",
    0x00a0: "Disassoc",
    0x00c0: "Deauth",
    0x0000: "Assoc Req",
    0x0010: "Assoc Resp",
    0x0020: "Reassoc Req",
    0x0030: "Reassoc Resp",
    0x00b0: "Auth",
}


def psd_summary(iq, n_fft=1024):
    """Quick PSD to confirm energy on channel."""
    n_segs = min(len(iq) // n_fft, 100)
    if n_segs == 0:
        return None
    psd = np.zeros(n_fft)
    for i in range(n_segs):
        seg = iq[i * n_fft:(i + 1) * n_fft]
        psd += np.abs(np.fft.fft(seg)) ** 2
    psd /= n_segs
    psd_db = 10 * np.log10(psd + 1e-20)
    # Shift so DC is center
    psd_db = np.fft.fftshift(psd_db)
    return psd_db


def identify_frame(psdu):
    """Parse MAC header from PSDU bytes, return frame info dict."""
    if len(psdu) < 4:
        return {"type": "too_short"}

    fc = psdu[0] | (psdu[1] << 8)
    # Frame type/subtype is in fc bits [3:2] (type) and [7:4] (subtype)
    frame_type = (fc >> 2) & 0x03   # 0=mgmt, 1=ctrl, 2=data
    subtype = (fc >> 4) & 0x0F

    info = {
        "fc": fc,
        "frame_type_code": frame_type,
        "subtype": subtype,
    }

    # Look up human-readable name
    name = FRAME_TYPES.get(fc & 0xF0FF, None)
    if name is None:
        if frame_type == 0:
            name = f"Mgmt(sub={subtype})"
        elif frame_type == 1:
            name = f"Ctrl(sub={subtype})"
        elif frame_type == 2:
            name = f"Data(sub={subtype})"
        else:
            name = f"Unknown({fc:#06x})"
    info["name"] = name

    # For management frames, extract addresses
    if frame_type == 0 and len(psdu) >= 24:
        info["da"] = ":".join(f"{b:02x}" for b in psdu[4:10])
        info["sa"] = ":".join(f"{b:02x}" for b in psdu[10:16])
        info["bssid"] = ":".join(f"{b:02x}" for b in psdu[16:22])

    # For control frames (type=1), shorter headers
    if frame_type == 1 and len(psdu) >= 10:
        info["ra"] = ":".join(f"{b:02x}" for b in psdu[4:10])
        if len(psdu) >= 16:
            info["ta"] = ":".join(f"{b:02x}" for b in psdu[10:16])

    return info


def extract_ssid(psdu):
    """Extract SSID from a beacon or probe response frame body.

    Beacon frame body starts at byte 24 (after MAC header) + 12 bytes
    of fixed fields (timestamp 8B, interval 2B, capability 2B).
    Then tagged parameters: tag_id(1B) + tag_len(1B) + value.
    SSID is tag 0.
    """
    if len(psdu) < 38:  # 24 header + 12 fixed + 2 min tag
        return None

    # Tagged params start at offset 36 (24 + 12)
    offset = 36
    while offset + 2 <= len(psdu) - 4:  # -4 for FCS
        tag_id = psdu[offset]
        tag_len = psdu[offset + 1]
        if offset + 2 + tag_len > len(psdu) - 4:
            break
        if tag_id == 0:  # SSID
            ssid_bytes = bytes(psdu[offset + 2:offset + 2 + tag_len])
            try:
                return ssid_bytes.decode('utf-8', errors='replace')
            except Exception:
                return repr(ssid_bytes)
        offset += 2 + tag_len
    return None


def extract_eapol(psdu):
    """Check if frame carries EAPOL (EtherType 0x888E)."""
    if len(psdu) < 30:
        return False

    fc = psdu[0] | (psdu[1] << 8)
    frame_type = (fc >> 2) & 0x03
    if frame_type != 2:  # not data
        return False

    subtype = (fc >> 4) & 0x0F
    # Determine header size
    to_ds = (fc >> 8) & 1
    from_ds = (fc >> 9) & 1
    hdr_len = 24
    if to_ds and from_ds:
        hdr_len = 30  # 4-address
    if subtype & 0x08:  # QoS
        hdr_len += 2

    # Check LLC/SNAP + EtherType
    snap_start = hdr_len
    if snap_start + 8 > len(psdu) - 4:
        return False

    # LLC: AA AA 03, OUI: 00 00 00
    if (psdu[snap_start] == 0xAA and psdu[snap_start + 1] == 0xAA and
            psdu[snap_start + 2] == 0x03 and
            psdu[snap_start + 3] == 0x00 and psdu[snap_start + 4] == 0x00 and
            psdu[snap_start + 5] == 0x00):
        ethertype = (psdu[snap_start + 6] << 8) | psdu[snap_start + 7]
        if ethertype == 0x888E:
            return True
    return False


def analyze_capture(iq, verbose=True):
    """Run full decode pipeline on captured IQ. Returns stats dict."""
    ctx = DecodeContext()
    offset = 0
    min_gap = 320  # minimum gap between detections (one preamble)

    stats = {
        "total_stf": 0,
        "sig_ok": 0,
        "sig_fail": 0,
        "fcs_ok": 0,
        "fcs_fail": 0,
        "legacy": 0,
        "ht": 0,
        "vht": 0,
        "beacons": 0,
        "eapols": 0,
        "ssids": set(),
        "bssids": set(),
        "lsig_rates": {},
        "frame_names": {},
    }

    frames = []

    while offset < len(iq) - 1000:
        stf = detect_stf(iq[offset:], threshold=0.5, min_periods=6)
        if stf < 0:
            break

        abs_offset = offset + stf
        stats["total_stf"] += 1

        result = ctx.decode_frame(iq, abs_offset)

        if result is None or "error" in result:
            stats["sig_fail"] += 1
            if verbose and result:
                err = result.get("error", "unknown")
                sig = result.get("signal", {})
                rate = sig.get("rate_mbps", "?")
                parity = sig.get("parity_ok", "?")
                if stats["total_stf"] <= 30:  # limit noise
                    print(f"  [{abs_offset:8d}] SIG_FAIL: {err} "
                          f"(rate={rate}, parity={parity})")
            offset = abs_offset + min_gap
            continue

        stats["sig_ok"] += 1
        frame_type = result.get("frame_type", "unknown")
        fcs_ok = result.get("fcs_ok", False)

        # Track L-SIG rates
        sig = result.get("signal", {})
        rate = sig.get("rate_mbps", 0)
        stats["lsig_rates"][rate] = stats["lsig_rates"].get(rate, 0) + 1

        if frame_type == "legacy":
            stats["legacy"] += 1
        elif frame_type == "ht":
            stats["ht"] += 1
        elif frame_type == "vht":
            stats["vht"] += 1

        if fcs_ok:
            stats["fcs_ok"] += 1
            psdu = result.get("psdu", b"")
            frame_info = identify_frame(psdu)
            fname = frame_info.get("name", "?")
            stats["frame_names"][fname] = stats["frame_names"].get(fname, 0) + 1

            # Beacon / Probe Resp -> extract SSID
            fc = frame_info.get("fc", 0)
            if (fc & 0x00FF) in (0x80, 0x50):  # Beacon or Probe Resp
                stats["beacons"] += 1
                ssid = extract_ssid(psdu)
                if ssid:
                    stats["ssids"].add(ssid)
                bssid = frame_info.get("bssid")
                if bssid:
                    stats["bssids"].add(bssid)

            # EAPOL check
            if extract_eapol(psdu):
                stats["eapols"] += 1

            if verbose:
                extra = ""
                if frame_info.get("bssid"):
                    extra += f" bssid={frame_info['bssid']}"
                if (fc & 0x00FF) in (0x80, 0x50):
                    ssid = extract_ssid(psdu)
                    if ssid:
                        extra += f" SSID=\"{ssid}\""
                ht_info = ""
                if frame_type in ("ht", "vht"):
                    ht_sig = result.get("ht_sig", {})
                    mcs = ht_sig.get("mcs", "?")
                    ht_info = f" MCS={mcs}"

                print(f"  [{abs_offset:8d}] {frame_type:6s} rate={rate:2} "
                      f"len={sig.get('length', '?'):4} FCS_OK "
                      f"{fname}{ht_info}{extra}")
        else:
            stats["fcs_fail"] += 1
            if verbose and stats["fcs_fail"] <= 20:
                length = sig.get("length", "?")
                print(f"  [{abs_offset:8d}] {frame_type:6s} rate={rate:2} "
                      f"len={length:4} FCS_FAIL")

        frames.append(result)

        # Advance past frame (use L-SIG length to skip data symbols)
        n_sym = sig.get("n_symbols", 1)
        frame_samples = 320 + n_sym * 80  # preamble + data
        offset = abs_offset + max(frame_samples, min_gap)

    return stats, frames


def run_scan(gain, duration_per_ch=1.0):
    """Scan all 4 sub-channels of ch36 80 MHz block."""
    channels = [
        (5180e6, "ch36 (primary)"),
        (5200e6, "ch40"),
        (5220e6, "ch44"),
        (5240e6, "ch48"),
    ]

    print("=" * 60)
    print("RF SCAN — ch36 80 MHz block (5180–5240 MHz)")
    print("=" * 60)

    for freq, label in channels:
        print(f"\n--- {label}: {freq/1e6:.0f} MHz ---")
        _, sdr = find_pluto()
        configure_rx(sdr, freq, gain)
        iq = capture(sdr, duration_per_ch)

        # Power
        power_db = 10 * np.log10(np.mean(np.abs(iq) ** 2) + 1e-20)
        peak_db = 10 * np.log10(np.max(np.abs(iq) ** 2) + 1e-20)
        print(f"  Mean power: {power_db:.1f} dBFS, Peak: {peak_db:.1f} dBFS")

        # Quick STF count
        n_stf = 0
        off = 0
        while off < len(iq) - 1000:
            s = detect_stf(iq[off:], threshold=0.5, min_periods=6)
            if s < 0:
                break
            n_stf += 1
            off += s + 320
        print(f"  STF detections: {n_stf}")

        # Quick decode summary
        if n_stf > 0:
            stats, _ = analyze_capture(iq, verbose=False)
            print(f"  L-SIG OK: {stats['sig_ok']}  FCS OK: {stats['fcs_ok']}  "
                  f"Beacons: {stats['beacons']}  HT: {stats['ht']}  VHT: {stats['vht']}")
            if stats["ssids"]:
                print(f"  SSIDs: {stats['ssids']}")
            if stats["lsig_rates"]:
                print(f"  L-SIG rates: {dict(stats['lsig_rates'])}")

        del sdr
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="Diagnostic capture + analysis")
    parser.add_argument("--freq", type=float, default=DEFAULT_FREQ,
                        help="Center frequency (default: 5180 MHz)")
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                        help="RX gain in dB (default: 73)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help="Capture duration in seconds (default: 2)")
    parser.add_argument("--scan", action="store_true",
                        help="Scan all 4 sub-channels of ch36 80 MHz block")
    parser.add_argument("--save", type=str, default=None,
                        help="Save raw IQ to .npy file")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress per-frame output")
    args = parser.parse_args()

    uri, sdr = find_pluto()
    print(f"PlutoSDR: {uri}")
    print()

    if args.scan:
        run_scan(args.gain)
        return

    # --- Single frequency capture ---
    print(f"Capture: {args.freq/1e6:.0f} MHz, gain={args.gain} dB, "
          f"duration={args.duration}s ({int(args.duration * SAMPLE_RATE / 1e6)}M samples)")
    print()

    configure_rx(sdr, args.freq, args.gain)

    print("Capturing...")
    t0 = time.time()
    iq = capture(sdr, args.duration)
    t1 = time.time()
    print(f"  Captured {len(iq)} samples in {t1-t0:.2f}s")

    # Basic RF stats
    power_db = 10 * np.log10(np.mean(np.abs(iq) ** 2) + 1e-20)
    peak_db = 10 * np.log10(np.max(np.abs(iq) ** 2) + 1e-20)
    print(f"  Mean power: {power_db:.1f} dBFS")
    print(f"  Peak power: {peak_db:.1f} dBFS")
    print(f"  Dynamic range: {peak_db - power_db:.1f} dB")
    print()

    # PSD
    psd = psd_summary(iq)
    if psd is not None:
        psd_peak = np.max(psd)
        psd_floor = np.median(psd)
        print(f"  PSD peak: {psd_peak:.1f} dB, floor: {psd_floor:.1f} dB, "
              f"contrast: {psd_peak - psd_floor:.1f} dB")
        # Check if energy is concentrated (WiFi signal present)
        above_floor = np.sum(psd > psd_floor + 10)
        print(f"  Bins >10dB above floor: {above_floor}/{len(psd)} "
              f"({100*above_floor/len(psd):.0f}%)")
        print()

    # Save if requested
    if args.save:
        np.save(args.save, iq)
        print(f"  Saved to {args.save} ({iq.nbytes / (1024*1024):.1f} MB)")
        print()

    # --- Decode analysis ---
    print("=" * 60)
    print("DECODE ANALYSIS")
    print("=" * 60)
    print()

    stats, frames = analyze_capture(iq, verbose=not args.quiet)

    # --- Summary ---
    print()
    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"  Duration:          {args.duration}s")
    print(f"  STF detections:    {stats['total_stf']}")
    print(f"  L-SIG OK:          {stats['sig_ok']}")
    print(f"  L-SIG FAIL:        {stats['sig_fail']}")
    print(f"  FCS OK:            {stats['fcs_ok']}")
    print(f"  FCS FAIL:          {stats['fcs_fail']}")
    print(f"  Legacy frames:     {stats['legacy']}")
    print(f"  HT frames:         {stats['ht']}")
    print(f"  VHT frames:        {stats['vht']}")
    print(f"  Beacons:           {stats['beacons']}")
    print(f"  EAPOLs:            {stats['eapols']}")
    print()
    if stats["lsig_rates"]:
        print(f"  L-SIG rate distribution:")
        for rate, count in sorted(stats["lsig_rates"].items()):
            print(f"    {rate:2} Mbps: {count}")
    print()
    if stats["frame_names"]:
        print(f"  Frame types (FCS OK):")
        for name, count in sorted(stats["frame_names"].items(),
                                    key=lambda x: -x[1]):
            print(f"    {name}: {count}")
    print()
    if stats["ssids"]:
        print(f"  SSIDs found: {stats['ssids']}")
    else:
        print(f"  SSIDs found: (none)")
    if stats["bssids"]:
        print(f"  BSSIDs: {stats['bssids']}")
    print()

    # Diagnosis
    print("=" * 60)
    print("DIAGNOSIS")
    print("=" * 60)
    if stats["total_stf"] == 0:
        print("  NO STF detections. Possible causes:")
        print("    - No signal on this frequency (try --scan)")
        print("    - Signal too weak (check antenna/gain)")
        print("    - Wrong center frequency")
    elif stats["sig_ok"] == 0:
        print("  STF detected but NO valid L-SIG. Possible causes:")
        print("    - SNR too low for BPSK demod (channel estimation failing)")
        print("    - CFO too large (>±312 kHz, outside estimator range)")
        print("    - Timing offset (LTF correlation failing)")
        print("    - Signals from adjacent channel (not 20 MHz OFDM)")
    elif stats["fcs_ok"] == 0:
        print("  L-SIG OK but NO FCS pass. Possible causes:")
        print("    - Moderate SNR: L-SIG (BPSK) works but DATA fails")
        print("    - Multipath: channel estimate insufficient for data rates")
        print("    - Wrong sample rate or bandwidth mismatch")
    elif stats["beacons"] == 0 and stats["fcs_ok"] > 0:
        print("  FCS OK frames found but NO beacons. The AP may be on a")
        print("  different 20 MHz sub-channel. Try --scan to check all 4.")
        print(f"  Decoded frame types: {list(stats['frame_names'].keys())}")
    else:
        print("  Pipeline working. Beacons decoded.")
    print()


if __name__ == "__main__":
    main()
