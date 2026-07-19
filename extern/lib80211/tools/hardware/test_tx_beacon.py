#!/usr/bin/env python3
"""Beacon broadcast — TX interop sea trial.

Transmits continuous 802.11 beacons on a specified channel so that
nearby STAs (MacBook, RPi, phones) list the SSID in their scan results.

This validates that our TX PHY waveform is spec-conformant enough for
real 802.11 receivers to decode. Pass criteria: SSID appears in scan
results on at least one target device.

HARDWARE SETUP:
  PlutoSDR TX → 5 GHz antenna (or 2.4 GHz if --channel specifies)

Usage:
    python tools/hardware/test_tx_beacon.py
    python tools/hardware/test_tx_beacon.py --ssid "MyTestNet" --channel 36
    python tools/hardware/test_tx_beacon.py --interval 20 --tx-atten 0

Verification (run on another device while this script transmits):
    macOS (Sequoia+):
        Option-click WiFi icon → look for SSID
        Or: system_profiler SPAirPortDataType | grep -A2 lib80211
        Or: python3 -c "
            import objc
            objc.loadBundle('CoreWLAN', globals(),
                '/System/Library/Frameworks/CoreWLAN.framework')
            iface = CWInterface.interface()
            nets = iface.scanForNetworksWithName_error_(None, None)
            for n in sorted(nets, key=lambda x: x.ssid() or ''):
                print(f'{n.ssid():32s} {n.bssid()} ch{n.wlanChannel().channelNumber()}')"

    Linux (RPi):
        nmcli dev wifi rescan && nmcli dev wifi list | grep lib80211
        Or: iw dev wlan0 scan | grep -A5 lib80211
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
from py80211.mac_frames import build_beacon
from frame_utils import CHANNEL_FREQ

# --- Defaults ---
DEFAULT_SSID = "lib80211-test"
DEFAULT_CHANNEL = 36
DEFAULT_INTERVAL_TU = 20       # TU (1 TU = 1.024 ms). 20 TU ≈ 20.48 ms
DEFAULT_TX_ATTEN = 0           # max power for OTA
DEFAULT_RATE = 6               # beacons always at lowest mandatory rate

# Locally-administered BSSID (bit 1 of first octet = 1)
DEFAULT_BSSID = b'\x02\x00\x00\x80\x21\x01'

# Capabilities: ESS, Short Slot Time (standard for 5 GHz)
DEFAULT_CAPABILITIES = 0x0431


def build_beacon_waveform(ssid, bssid, channel, beacon_interval_tu,
                          capabilities, rate_mbps):
    """Build a single beacon frame as baseband IQ.

    Returns (iq, meta, psdu_len) where iq is complex64 samples.
    """
    psdu = build_beacon(
        ssid=ssid,
        bssid=bssid,
        channel=channel,
        beacon_interval=beacon_interval_tu,
        capabilities=capabilities,
    )

    # Generate PHY frame at the specified rate (beacons use 6 Mbps)
    iq, meta = generate_frame(rate_mbps, psdu)
    return iq, meta, len(psdu)


def build_cyclic_buffer(iq_frame, interval_tu):
    """Build a cyclic TX buffer: [beacon] [silence gap].

    The silence gap duration corresponds to the beacon interval so that
    the Pluto's cyclic DMA produces beacons at the correct rate.

    Args:
        iq_frame: Complex64 baseband samples for one beacon frame.
        interval_tu: Beacon interval in TU (1 TU = 1024 us = 20480 samples).

    Returns:
        Complex64 array for the cyclic buffer.
    """
    # 1 TU = 1024 us = 1024e-6 * 20e6 = 20480 samples
    samples_per_tu = int(1024e-6 * SAMPLE_RATE)
    total_samples = interval_tu * samples_per_tu

    # Frame occupies the first portion, rest is silence
    frame_len = len(iq_frame)
    if frame_len >= total_samples:
        # Frame is longer than interval — just use the frame (unlikely)
        return iq_frame.astype(np.complex64)

    buf = np.zeros(total_samples, dtype=np.complex64)
    buf[:frame_len] = iq_frame
    return buf


def main():
    parser = argparse.ArgumentParser(
        description="Beacon broadcast — TX interop test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                           # default: lib80211-test on ch36
  %(prog)s --ssid "FakeAP" --channel 44
  %(prog)s --interval 100            # standard 100 TU interval
  %(prog)s --tx-atten 10             # reduce power by 10 dB
""")
    parser.add_argument("--ssid", type=str, default=DEFAULT_SSID,
                        help=f"SSID to broadcast (default: {DEFAULT_SSID})")
    parser.add_argument("--channel", type=int, default=DEFAULT_CHANNEL,
                        help=f"WiFi channel (default: {DEFAULT_CHANNEL})")
    parser.add_argument("--interval", type=int, default=DEFAULT_INTERVAL_TU,
                        help="Beacon interval in TU (default: 20, standard: 100)")
    parser.add_argument("--tx-atten", type=float, default=DEFAULT_TX_ATTEN,
                        help="TX attenuation in dB (0=max power, default: 0)")
    parser.add_argument("--rate", type=int, default=DEFAULT_RATE,
                        help="PHY rate in Mbps (default: 6)")
    parser.add_argument("--bssid", type=str, default=None,
                        help="Custom BSSID (hex, e.g., 02:00:00:80:21:01)")
    parser.add_argument("--uri", type=str, default=None,
                        help="PlutoSDR URI (auto-detect if omitted)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Build waveform and print stats without transmitting")
    args = parser.parse_args()

    # Resolve channel to frequency
    if args.channel not in CHANNEL_FREQ:
        sys.exit(f"Unknown channel {args.channel}. "
                 f"Supported: {sorted(CHANNEL_FREQ.keys())}")
    freq = CHANNEL_FREQ[args.channel]

    # Resolve BSSID
    if args.bssid:
        bssid = bytes(int(x, 16) for x in args.bssid.split(":"))
        if len(bssid) != 6:
            sys.exit("BSSID must be 6 bytes (e.g., 02:00:00:80:21:01)")
    else:
        bssid = DEFAULT_BSSID

    bssid_str = ":".join(f"{b:02x}" for b in bssid)

    print("=" * 60)
    print("BEACON BROADCAST — TX INTEROP TEST")
    print("=" * 60)
    print(f"  SSID:          {args.ssid}")
    print(f"  BSSID:         {bssid_str}")
    print(f"  Channel:       {args.channel} ({freq/1e6:.0f} MHz)")
    print(f"  Interval:      {args.interval} TU ({args.interval * 1.024:.1f} ms)")
    print(f"  Rate:          {args.rate} Mbps")
    print(f"  TX atten:      {args.tx_atten} dB")
    print()

    # Build beacon waveform
    print("Building beacon waveform...")
    iq_frame, meta, psdu_len = build_beacon_waveform(
        ssid=args.ssid,
        bssid=bssid,
        channel=args.channel,
        beacon_interval_tu=args.interval,
        capabilities=DEFAULT_CAPABILITIES,
        rate_mbps=args.rate,
    )
    print(f"  PSDU:          {psdu_len} bytes (+ 4B FCS = {psdu_len + 4}B over air)")
    print(f"  PHY symbols:   {meta['n_data_symbols']} data + 1 SIGNAL")
    print(f"  Frame IQ:      {len(iq_frame)} samples "
          f"({len(iq_frame)/SAMPLE_RATE*1e6:.0f} us)")

    # Build cyclic buffer
    buf = build_cyclic_buffer(iq_frame, args.interval)
    buf_ms = len(buf) / SAMPLE_RATE * 1000
    buf_mb = buf.nbytes / (1024 * 1024)
    print(f"  Cyclic buffer: {len(buf):,} samples ({buf_ms:.1f} ms, {buf_mb:.1f} MB)")
    print(f"  Effective rate: {1000/buf_ms:.1f} beacons/sec")
    print()

    if args.dry_run:
        print("DRY RUN — not transmitting.")
        print(f"  Peak |I|: {np.max(np.abs(iq_frame.real)):.6f}")
        print(f"  Peak |Q|: {np.max(np.abs(iq_frame.imag)):.6f}")
        print(f"  Peak |IQ|: {np.max(np.abs(iq_frame)):.6f}")
        return

    # Connect to PlutoSDR
    from pluto import find_pluto, configure_tx, transmit, stop_tx

    uri, sdr = find_pluto(args.uri)
    print(f"PlutoSDR: {uri}")
    print("Configuring TX...")
    configure_tx(sdr, freq, attenuation_db=args.tx_atten)

    # Transmit (cyclic — repeats until stopped)
    print("Starting beacon transmission...")
    transmit(sdr, buf)
    print()
    print(">>> TRANSMITTING BEACONS <<<")
    print(f">>> SSID: {args.ssid}")
    print(f">>> Channel: {args.channel} ({freq/1e6:.0f} MHz)")
    print()
    print("Press Ctrl-C to stop.")
    print()
    print("Verification commands:")
    print("  macOS:  Option-click WiFi icon, or:")
    print("          python3 -c \"import objc; "
          "objc.loadBundle('CoreWLAN', globals(), "
          "'/System/Library/Frameworks/CoreWLAN.framework'); "
          "iface = CWInterface.interface(); "
          "nets = iface.scanForNetworksWithName_error_(None, None); "
          "[print(f'{n.ssid()} ch{n.wlanChannel().channelNumber()}') "
          "for n in nets if 'lib80211' in (n.ssid() or '')]\"")
    print(f"  Linux:  nmcli dev wifi rescan && "
          f"nmcli dev wifi list | grep {args.ssid}")
    print()

    # Wait until Ctrl-C
    start_time = time.time()

    def handle_sigint(sig, frame):
        elapsed = time.time() - start_time
        print(f"\n\nStopping TX after {elapsed:.1f}s...")
        stop_tx(sdr)
        print("Done.")
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    # Keep alive with status updates
    try:
        while True:
            time.sleep(10)
            elapsed = time.time() - start_time
            print(f"  ... transmitting ({elapsed:.0f}s elapsed)")
    except KeyboardInterrupt:
        handle_sigint(None, None)


if __name__ == "__main__":
    main()
