#!/usr/bin/env python3
"""Cable loopback test: TX → attenuator → RX via PlutoSDR full-duplex.

Transmits known 802.11 frames through Pluto TX → SMA cable + attenuator → Pluto RX,
verifies the full analog chain at the target frequency.

Usage:
    python test_cable_loopback.py [options]

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

from pluto import find_pluto, configure_rx, configure_tx, capture, transmit, stop_tx
from py80211.gen_ofdm_frame import generate_frame, generate_ht_frame, SAMPLE_RATE
from py80211.decode_frame import detect_stf, decode_frame

# ============================================================================
# Constants
# ============================================================================
LEGACY_RATES = [6, 9, 12, 18, 24, 36, 48, 54]
HT_MCS_RANGE = list(range(8))

DEFAULT_FREQ = 5_180_000_000  # Channel 36
DEFAULT_TX_ATTEN = 20  # dB (Pluto internal, assumes external 20 dB attenuator)
DEFAULT_RX_GAIN = 30  # dB (moderate — attenuator limits signal)
DEFAULT_TRIALS = 10
DEFAULT_CAPTURE_DURATION = 0.05  # 50 ms per trial
PAYLOAD_LENGTH = 80  # bytes
ZERO_PAD = 10000  # zeros on each side of frame


# ============================================================================
# Status codes
# ============================================================================
STATUS_PASS = "PASS"
STATUS_NO_STF = "NO_STF"
STATUS_DECODE_FAIL = "DECODE_FAIL"
STATUS_FCS_FAIL = "FCS_FAIL"
STATUS_PAYLOAD_MISMATCH = "PAYLOAD_MISMATCH"


def make_payload(seed: int, length: int = PAYLOAD_LENGTH) -> bytes:
    """Generate deterministic payload from seed."""
    rng = np.random.default_rng(seed)
    return bytes(rng.integers(0, 256, size=length, dtype=np.uint8))


def run_trial(sdr, rate_mbps=None, mcs=None, trial=0,
              freq=DEFAULT_FREQ, tx_atten=DEFAULT_TX_ATTEN,
              rx_gain=DEFAULT_RX_GAIN, capture_duration=DEFAULT_CAPTURE_DURATION):
    """Run a single loopback trial.

    Exactly one of rate_mbps or mcs must be specified.

    Returns:
        dict with keys: status, rate_mbps/mcs, trial, and optional decode info.
    """
    # Deterministic payload seed
    if rate_mbps is not None:
        seed = rate_mbps * 100 + trial
        label = f"{rate_mbps} Mbps"
    else:
        seed = (mcs + 100) * 100 + trial
        label = f"MCS {mcs}"

    payload = make_payload(seed)

    # Generate frame
    if rate_mbps is not None:
        iq, meta = generate_frame(rate_mbps, payload)
    else:
        iq, meta = generate_ht_frame(mcs, payload)

    # Pad with zeros for clean capture
    padded = np.concatenate([
        np.zeros(ZERO_PAD, dtype=np.complex64),
        iq.astype(np.complex64),
        np.zeros(ZERO_PAD, dtype=np.complex64),
    ])

    # Configure TX and transmit (cyclic)
    configure_tx(sdr, freq, attenuation_db=tx_atten)
    transmit(sdr, padded)

    # Let TX settle
    time.sleep(0.01)

    # Configure RX and capture
    configure_rx(sdr, freq, rx_gain)
    rx_iq = capture(sdr, capture_duration)

    # Stop TX
    stop_tx(sdr)

    # Detect STF
    stf_start = detect_stf(rx_iq)
    if stf_start < 0:
        return {"status": STATUS_NO_STF, "trial": trial}

    # Decode frame
    result = decode_frame(rx_iq, stf_start)
    if result is None or "error" in result:
        error_msg = result.get("error", "unknown") if result else "decode returned None"
        return {"status": STATUS_DECODE_FAIL, "trial": trial, "error": error_msg}

    # Verify FCS
    if not result.get("fcs_ok", False):
        return {"status": STATUS_FCS_FAIL, "trial": trial}

    # Verify payload match
    # psdu includes FCS (last 4 bytes); payload is the PSDU body without FCS
    psdu = result.get("psdu", b"")
    if isinstance(psdu, list):
        psdu = bytes(psdu)
    # Strip FCS (last 4 bytes) to get original payload
    rx_payload = psdu[:-4] if len(psdu) >= 4 else psdu

    if rx_payload != payload:
        return {"status": STATUS_PAYLOAD_MISMATCH, "trial": trial,
                "expected_len": len(payload), "got_len": len(rx_payload)}

    return {"status": STATUS_PASS, "trial": trial,
            "cfo_hz": result.get("cfo_hz", 0.0)}


def parse_rate_list(s):
    """Parse comma-separated rate list (e.g. '6,12,24')."""
    return [int(x.strip()) for x in s.split(",")]


def parse_mcs_list(s):
    """Parse comma-separated MCS list (e.g. '0,1,2,3')."""
    return [int(x.strip()) for x in s.split(",")]


def main():
    parser = argparse.ArgumentParser(
        description="Cable loopback test: verify 802.11 TX/RX via PlutoSDR")
    parser.add_argument("--freq", type=int, default=DEFAULT_FREQ,
                        help=f"Center frequency in Hz (default: {DEFAULT_FREQ})")
    parser.add_argument("--tx-atten", type=int, default=DEFAULT_TX_ATTEN,
                        help=f"TX attenuation in dB (default: {DEFAULT_TX_ATTEN})")
    parser.add_argument("--rx-gain", type=int, default=DEFAULT_RX_GAIN,
                        help=f"RX gain in dB (default: {DEFAULT_RX_GAIN})")
    parser.add_argument("--rates", type=str, default=None,
                        help="Comma-separated legacy rates to test (default: all 8)")
    parser.add_argument("--mcs", type=str, default=None,
                        help="Comma-separated HT MCS indices to test (default: 0-7)")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                        help=f"Trials per rate/MCS (default: {DEFAULT_TRIALS})")
    parser.add_argument("--uri", type=str, default=None,
                        help="PlutoSDR URI (default: auto-detect)")
    parser.add_argument("--save", type=str, default="cable_loopback_results.json",
                        help="Output JSON file (default: cable_loopback_results.json)")
    parser.add_argument("--no-legacy", action="store_true",
                        help="Skip legacy rate tests")
    parser.add_argument("--no-ht", action="store_true",
                        help="Skip HT MCS tests")

    args = parser.parse_args()

    # Determine which rates/MCS to test
    if args.rates is not None:
        legacy_rates = parse_rate_list(args.rates)
    elif args.no_legacy:
        legacy_rates = []
    else:
        legacy_rates = LEGACY_RATES

    if args.mcs is not None:
        ht_mcs = parse_mcs_list(args.mcs)
    elif args.no_ht:
        ht_mcs = []
    else:
        ht_mcs = HT_MCS_RANGE

    # Find Pluto
    print(f"Connecting to PlutoSDR...")
    uri, sdr = find_pluto(args.uri)
    print(f"  Found: {uri}")
    print(f"  Freq: {args.freq / 1e6:.0f} MHz, TX atten: {args.tx_atten} dB, "
          f"RX gain: {args.rx_gain} dB")
    print(f"  Trials per rate: {args.trials}")
    print()

    all_results = []
    total_pass = 0
    total_trials = 0

    # Legacy rates
    if legacy_rates:
        print("=== Legacy OFDM Rates ===")
        for rate in legacy_rates:
            pass_count = 0
            rate_results = []
            for t in range(args.trials):
                result = run_trial(sdr, rate_mbps=rate, trial=t,
                                   freq=args.freq, tx_atten=args.tx_atten,
                                   rx_gain=args.rx_gain)
                rate_results.append(result)
                if result["status"] == STATUS_PASS:
                    pass_count += 1

            pct = 100 * pass_count / args.trials
            status_tag = STATUS_PASS if pass_count == args.trials else "PARTIAL"
            if pass_count == 0:
                status_tag = "FAIL"
            print(f"  Rate {rate:>2} Mbps: {pass_count}/{args.trials} "
                  f"({pct:.0f}%) [{status_tag}]")

            total_pass += pass_count
            total_trials += args.trials
            all_results.append({
                "type": "legacy",
                "rate_mbps": rate,
                "pass": pass_count,
                "trials": args.trials,
                "pct": pct,
                "details": rate_results,
            })
        print()

    # HT MCS
    if ht_mcs:
        print("=== HT (802.11n) MCS ===")
        for mcs_idx in ht_mcs:
            pass_count = 0
            mcs_results = []
            for t in range(args.trials):
                result = run_trial(sdr, mcs=mcs_idx, trial=t,
                                   freq=args.freq, tx_atten=args.tx_atten,
                                   rx_gain=args.rx_gain)
                mcs_results.append(result)
                if result["status"] == STATUS_PASS:
                    pass_count += 1

            pct = 100 * pass_count / args.trials
            status_tag = STATUS_PASS if pass_count == args.trials else "PARTIAL"
            if pass_count == 0:
                status_tag = "FAIL"
            print(f"  MCS {mcs_idx}: {pass_count}/{args.trials} "
                  f"({pct:.0f}%) [{status_tag}]")

            total_pass += pass_count
            total_trials += args.trials
            all_results.append({
                "type": "ht",
                "mcs": mcs_idx,
                "pass": pass_count,
                "trials": args.trials,
                "pct": pct,
                "details": mcs_results,
            })
        print()

    # Summary
    if total_trials > 0:
        total_pct = 100 * total_pass / total_trials
        print(f"TOTAL: {total_pass}/{total_trials} ({total_pct:.0f}%)")
    else:
        print("No tests run.")

    # Save JSON results
    output = {
        "freq_hz": args.freq,
        "tx_atten_db": args.tx_atten,
        "rx_gain_db": args.rx_gain,
        "trials_per_rate": args.trials,
        "total_pass": total_pass,
        "total_trials": total_trials,
        "results": all_results,
    }

    with open(args.save, "w") as f:
        json.dump(output, f, indent=2, default=str)
    print(f"\nResults saved to: {args.save}")

    # Exit code: 0 if all passed, 1 otherwise
    sys.exit(0 if total_pass == total_trials else 1)


if __name__ == "__main__":
    main()
