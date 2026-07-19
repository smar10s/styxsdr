#!/usr/bin/env python3
"""Digital loopback with impairments.

Sweeps SNR, CFO, and multipath to establish the Python decoder's
operating envelope.  Each test: generate frame -> apply impairment ->
decode -> verify FCS OK and payload match.

Usage:
    python tools/impairment_sweep.py               # full sweep
    python tools/impairment_sweep.py --rate 6       # single rate
    python tools/impairment_sweep.py --snr 20       # fixed SNR, sweep CFO
    python tools/impairment_sweep.py --report-only  # print known baseline
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from py80211.gen_ofdm_frame import generate_frame, RATE_TABLE
from py80211.impairments import add_awgn, add_cfo, apply_multipath, MULTIPATH_PRESETS
from py80211.decode_frame import detect_stf, DecodeContext

SAMPLE_RATE = 20_000_000
FRAME_BYTES = 100  # reasonable PSDU size


def run_trial(rate_mbps: int, impairment_fn, payload: bytes,
              snr_db: float = None,
              cfo_hz: float = 0.0,
              multipath: str = None) -> bool:
    """Run one decode trial with specified impairments.

    Returns True if FCS OK and payload matches.
    """
    iq, meta = generate_frame(rate_mbps, payload)

    if multipath:
        iq = apply_multipath(iq, MULTIPATH_PRESETS[multipath])
    if cfo_hz != 0.0:
        iq = add_cfo(iq, cfo_hz, SAMPLE_RATE)
    if snr_db is not None:
        iq = add_awgn(iq, snr_db)

    stf_start = detect_stf(iq, threshold=0.5, min_periods=6)
    if stf_start < 0:
        return False

    ctx = DecodeContext()
    result = ctx.decode_frame(iq, stf_start)
    if result is None:
        return False

    if not result.get("fcs_ok"):
        return False

    # Verify payload matches (first FRAME_BYTES of PSDU)
    decoded = result.get("psdu", b"")
    expected_fcs_len = len(payload) + 4
    if len(decoded) < expected_fcs_len:
        return False
    return decoded[:len(payload)] == payload


def sweep_snr(rate_mbps: int, payload: bytes,
              snr_range: range, trials: int = 10) -> list[tuple[float, float]]:
    """Sweep SNR at a given rate.

    Returns list of (snr_db, success_rate) pairs.
    """
    results = []
    for snr in snr_range:
        ok = 0
        for _ in range(trials):
            if run_trial(rate_mbps, None, payload, snr_db=snr):
                ok += 1
        results.append((snr, ok / trials))
        fwd = ">" if ok > trials / 2 else " "
        print(f"    SNR={snr:3d} dB: {ok}/{trials} {fwd}", flush=True)
    return results


def sweep_cfo(rate_mbps: int, payload: bytes,
              cfo_range_hz: list[int],
              snr_db: float = 25,
              trials: int = 10) -> list[tuple[int, float]]:
    """Sweep CFO at a given rate and fixed SNR."""
    results = []
    for cfo in cfo_range_hz:
        ok = 0
        for _ in range(trials):
            if run_trial(rate_mbps, None, payload,
                         snr_db=snr_db, cfo_hz=cfo):
                ok += 1
        results.append((cfo, ok / trials))
        print(f"    CFO={cfo:+6d} Hz: {ok}/{trials}", flush=True)
    return results


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Digital impairment sweep")
    parser.add_argument("--rate", type=int, default=6,
                        choices=sorted(RATE_TABLE.keys()),
                        help="Rate to test (default: 6)")
    parser.add_argument("--snr", type=int, default=None,
                        help="Fixed SNR for CFO sweep")
    parser.add_argument("--cfo", type=int, default=None,
                        help="Fixed CFO (Hz) for SNR sweep")
    parser.add_argument("--multipath", choices=list(MULTIPATH_PRESETS.keys()),
                        default=None, help="Multipath preset")
    parser.add_argument("--trials", type=int, default=5,
                        help="Trials per condition")
    parser.add_argument("--report-only", action="store_true",
                        help="Print baseline and exit")
    args = parser.parse_args()

    payload = bytes([(i * 7 + 13) % 256 for i in range(FRAME_BYTES)])

    if args.report_only:
        print("Baseline (from prior run):")
        for rate in [6, 24, 54]:
            print(f"  Rate {rate} Mbps clean: PASS")
        print("  Rate 6, SNR 10 dB: >90%")
        print("  Rate 6, CFO ±10 kHz: >90%")
        print("  Rate 6, mild multipath: PASS")
        return

    # If specific SNR or CFO given
    if args.snr is not None:
        cfo_range = [args.cfo] if args.cfo is not None else [0, 5000, 10000, 15000, 20000,
                                                             -5000, -10000, -15000, -20000]
        print(f"CFO sweep: rate={args.rate} Mbps, SNR={args.snr} dB")
        sweep_cfo(args.rate, payload, cfo_range, snr_db=args.snr,
                  trials=args.trials)
        return

    if args.cfo is not None:
        print(f"SNR sweep: rate={args.rate} Mbps, CFO={args.cfo} Hz")
        results = sweep_snr(args.rate, payload,
                            range(30, -5, -5), trials=args.trials)
        return

    # Default: full sweep at rate 6
    print(f"=== SNR sweep: rate={args.rate} Mbps ===")
    results_snr = sweep_snr(args.rate, payload,
                            range(30, 0, -5), trials=args.trials)
    best_snr = min((s for s, r in results_snr if r >= 0.8), default=999)
    print(f"  80% success at SNR >= {best_snr} dB")

    print(f"\n=== CFO sweep: rate={args.rate} Mbps, SNR=25 dB ===")
    cfo_list = [0, 2000, 5000, 10000, 20000,
                -2000, -5000, -10000, -20000, 30000, -30000]
    results_cfo = sweep_cfo(args.rate, payload, cfo_list,
                            snr_db=25, trials=args.trials)

    print(f"\n=== Multipath test: rate={args.rate} Mbps ===")
    for mp_name in ["mild", "moderate", "severe"]:
        ok = 0
        for _ in range(args.trials):
            if run_trial(args.rate, None, payload,
                         snr_db=20, multipath=mp_name):
                ok += 1
        print(f"    {mp_name:10s}: {ok}/{args.trials}")


if __name__ == "__main__":
    main()
