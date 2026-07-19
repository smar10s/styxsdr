#!/usr/bin/env python3
"""Sensitivity sweep: measure PER vs TX attenuation via PlutoSDR.

Sweeps TX attenuation from low (strong signal) to high (weak) and measures
packet error rate at each step. Produces PER-vs-attenuation data as both
a formatted table and JSON output.

Usage:
    python test_sensitivity.py [options]

Requires: pyadi-iio, numpy, py80211 (from this repo)
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
sys.path.insert(0, os.path.dirname(__file__))

from pluto import find_pluto, configure_rx, configure_tx, capture, transmit, stop_tx, SAMPLE_RATE
from py80211.gen_ofdm_frame import generate_frame, generate_ht_frame
from py80211.decode_frame import detect_stf, decode_frame

# ============================================================================
# Defaults
# ============================================================================
DEFAULT_FREQ = 5_180_000_000  # Channel 36
DEFAULT_RX_GAIN = 50  # dB
DEFAULT_ATTEN_START = 0
DEFAULT_ATTEN_STOP = 60
DEFAULT_ATTEN_STEP = 5
DEFAULT_TRIALS = 20
PAYLOAD_LENGTH = 80
ZERO_PAD = 20000
CAPTURE_DURATION = 0.05  # 50 ms
TX_SETTLE_TIME = 0.015  # 15 ms


def make_payload(tx_atten, trial, length=PAYLOAD_LENGTH):
    """Generate deterministic payload from attenuation and trial index."""
    rng = np.random.default_rng(int(tx_atten * 1000) + trial)
    return bytes(rng.integers(0, 256, size=length, dtype=np.uint8))


def run_trial(sdr_tx, sdr_rx, freq, tx_atten, rx_gain, rate_mbps=None, mcs=None, trial=0):
    """Run a single sensitivity trial.

    Returns True if frame decoded and payload matched, False otherwise.
    """
    payload = make_payload(tx_atten, trial)

    # Generate frame
    if mcs is not None:
        iq, _meta = generate_ht_frame(mcs, payload)
    else:
        iq, _meta = generate_frame(rate_mbps, payload)

    # Pad with zeros
    padded = np.concatenate([
        np.zeros(ZERO_PAD, dtype=np.complex64),
        iq.astype(np.complex64),
        np.zeros(ZERO_PAD, dtype=np.complex64),
    ])

    # Configure TX and transmit
    configure_tx(sdr_tx, freq, attenuation_db=tx_atten)
    transmit(sdr_tx, padded)

    # Let TX settle
    time.sleep(TX_SETTLE_TIME)

    # Configure RX and capture
    configure_rx(sdr_rx, freq, gain_db=rx_gain)
    rx_iq = capture(sdr_rx, CAPTURE_DURATION)

    # Stop TX
    stop_tx(sdr_tx)

    # Detect STF
    stf_start = detect_stf(rx_iq)
    if stf_start < 0:
        return False

    # Decode frame
    result = decode_frame(rx_iq, stf_start)
    if result is None or "error" in result:
        return False

    # Verify FCS
    if not result.get("fcs_ok", False):
        return False

    # Verify payload
    psdu = result.get("psdu", b"")
    if isinstance(psdu, list):
        psdu = bytes(psdu)
    rx_payload = psdu[:-4] if len(psdu) >= 4 else psdu

    return rx_payload == payload


def mode_label(rate_mbps, mcs):
    """Return human-readable mode label."""
    if mcs is not None:
        return f"HT MCS {mcs}"
    return f"Legacy {rate_mbps} Mbps"


def output_filename(rate_mbps, mcs):
    """Return JSON output filename based on mode."""
    if mcs is not None:
        return f"sensitivity_ht_mcs{mcs}.json"
    return f"sensitivity_legacy_{rate_mbps}_mbps.json"


def main():
    parser = argparse.ArgumentParser(
        description="Sensitivity sweep: PER vs TX attenuation via PlutoSDR")
    parser.add_argument("--freq", type=int, default=DEFAULT_FREQ,
                        help=f"Center frequency in Hz (default: {DEFAULT_FREQ})")
    parser.add_argument("--rx-gain", type=int, default=DEFAULT_RX_GAIN,
                        help=f"RX gain in dB (default: {DEFAULT_RX_GAIN})")
    parser.add_argument("--atten-start", type=int, default=DEFAULT_ATTEN_START,
                        help=f"Starting TX attenuation in dB (default: {DEFAULT_ATTEN_START})")
    parser.add_argument("--atten-stop", type=int, default=DEFAULT_ATTEN_STOP,
                        help=f"Ending TX attenuation in dB (default: {DEFAULT_ATTEN_STOP})")
    parser.add_argument("--atten-step", type=int, default=DEFAULT_ATTEN_STEP,
                        help=f"Attenuation step in dB (default: {DEFAULT_ATTEN_STEP})")
    parser.add_argument("--rate", type=int, default=6,
                        help="Legacy rate in Mbps (default: 6)")
    parser.add_argument("--mcs", type=int, default=None,
                        help="HT MCS index (overrides --rate if specified)")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                        help=f"Trials per attenuation point (default: {DEFAULT_TRIALS})")
    parser.add_argument("--tx-uri", type=str, default=None,
                        help="TX PlutoSDR URI (default: auto-detect)")
    parser.add_argument("--rx-uri", type=str, default=None,
                        help="RX PlutoSDR URI (default: same as TX for full-duplex)")

    args = parser.parse_args()

    rate_mbps = args.rate if args.mcs is None else None
    mcs = args.mcs
    label = mode_label(rate_mbps, mcs)

    # Connect to PlutoSDR(s)
    print("Connecting to PlutoSDR...")
    tx_uri, sdr_tx = find_pluto(args.tx_uri)
    print(f"  TX: {tx_uri}")

    if args.rx_uri is not None and args.rx_uri != args.tx_uri:
        rx_uri, sdr_rx = find_pluto(args.rx_uri)
        print(f"  RX: {rx_uri}")
    else:
        sdr_rx = sdr_tx
        rx_uri = tx_uri
        print(f"  RX: {rx_uri} (same device, full-duplex)")

    # Print header
    print()
    print(f"Mode: {label}")
    print(f"Freq: {args.freq / 1e6:.0f} MHz, RX gain: {args.rx_gain} dB")
    print(f"Sweep: atten {args.atten_start}\u2013{args.atten_stop} dB, step {args.atten_step}")
    print(f"Trials per point: {args.trials}")
    print()
    print(f" {'Atten(dB)':>9}   {'Pass':>4}  {'Total':>5}      {'PER':>4}")
    print("-" * 35)

    # Sweep
    sweep_results = []
    consecutive_full_per = 0
    early_stopped = False

    atten = args.atten_start
    while atten <= args.atten_stop:
        passes = 0
        for trial in range(args.trials):
            ok = run_trial(sdr_tx, sdr_rx, args.freq, atten, args.rx_gain,
                           rate_mbps=rate_mbps, mcs=mcs, trial=trial)
            if ok:
                passes += 1

        per = 1.0 - (passes / args.trials) if args.trials > 0 else 1.0
        sweep_results.append({
            "atten_db": atten,
            "passes": passes,
            "total": args.trials,
            "per": round(per, 4),
        })

        print(f" {atten:>9}   {passes:>4}  {args.trials:>5}   {per:>6.2f}")

        # Early stop: 100% PER for 2 consecutive points
        if per >= 1.0:
            consecutive_full_per += 1
        else:
            consecutive_full_per = 0

        if consecutive_full_per >= 2:
            early_stopped = True
            break

        atten += args.atten_step

    if early_stopped:
        print("(early stop: 100% PER for 2 consecutive points)")

    # Build JSON output
    output = {
        "test": "sensitivity_sweep",
        "mode": label,
        "freq_hz": args.freq,
        "rx_gain_db": args.rx_gain,
        "trials_per_point": args.trials,
        "sweep": sweep_results,
    }

    save_path = output_filename(rate_mbps, mcs)
    with open(save_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\nResults saved to {save_path}")


if __name__ == "__main__":
    main()
