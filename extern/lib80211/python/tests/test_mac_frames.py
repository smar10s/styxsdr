"""Tests for MAC frame construction helpers.

Verifies frame structure, field values, and PHY roundtrip integrity.
"""

import struct

import numpy as np
import pytest

from py80211.mac_frames import (
    build_beacon,
    build_deauth,
    build_probe_response,
    build_ack,
    build_rts,
    build_cts,
    FC_BEACON,
    FC_DEAUTH,
    FC_PROBE_RESPONSE,
    FC_ACK,
    FC_RTS,
    FC_CTS,
    BROADCAST,
    DEFAULT_BSSID,
)
from py80211.gen_ofdm_frame import generate_frame, compute_fcs, verify_fcs
from py80211.decode_frame import decode_frame


# ============================================================================
# Frame control value tests
# ============================================================================

class TestFrameControl:
    """Verify frame control field is correct for each frame type."""

    def test_beacon_fc(self):
        frame = build_beacon(ssid='test', timestamp=0)
        fc = struct.unpack_from('<H', frame, 0)[0]
        assert fc == FC_BEACON == 0x0080

    def test_deauth_fc(self):
        frame = build_deauth()
        fc = struct.unpack_from('<H', frame, 0)[0]
        assert fc == FC_DEAUTH == 0x00C0

    def test_probe_response_fc(self):
        frame = build_probe_response(da=BROADCAST, ssid='test', timestamp=0)
        fc = struct.unpack_from('<H', frame, 0)[0]
        assert fc == FC_PROBE_RESPONSE == 0x0050

    def test_ack_fc(self):
        frame = build_ack(ra=BROADCAST)
        fc = struct.unpack_from('<H', frame, 0)[0]
        assert fc == FC_ACK == 0x00D4

    def test_rts_fc(self):
        frame = build_rts(ra=BROADCAST, ta=DEFAULT_BSSID)
        fc = struct.unpack_from('<H', frame, 0)[0]
        assert fc == FC_RTS == 0x00B4

    def test_cts_fc(self):
        frame = build_cts(ra=BROADCAST)
        fc = struct.unpack_from('<H', frame, 0)[0]
        assert fc == FC_CTS == 0x00C4


# ============================================================================
# Frame length tests
# ============================================================================

class TestFrameLengths:
    """Verify correct frame sizes."""

    def test_deauth_length(self):
        frame = build_deauth()
        assert len(frame) == 26  # 24-byte header + 2-byte reason

    def test_ack_length(self):
        frame = build_ack(ra=BROADCAST)
        assert len(frame) == 10

    def test_rts_length(self):
        frame = build_rts(ra=BROADCAST, ta=DEFAULT_BSSID)
        assert len(frame) == 16

    def test_cts_length(self):
        frame = build_cts(ra=BROADCAST)
        assert len(frame) == 10


# ============================================================================
# Content verification tests
# ============================================================================

class TestBeaconContent:
    """Verify beacon frame structure and IE content."""

    def test_broadcast_da(self):
        frame = build_beacon(ssid='test', timestamp=0)
        # DA is at offset 4 (after FC+Duration)
        da = frame[4:10]
        assert da == BROADCAST

    def test_ssid_ie_present(self):
        ssid = 'MyNetwork'
        frame = build_beacon(ssid=ssid, timestamp=0)
        # SSID IE: tag=0, len=N, value=ssid bytes
        ssid_ie = bytes([0, len(ssid)]) + ssid.encode('utf-8')
        assert ssid_ie in frame

    def test_ds_channel_ie(self):
        channel = 44
        frame = build_beacon(ssid='test', channel=channel, timestamp=0)
        # DS Parameter Set IE: tag=3, len=1, value=channel
        ds_ie = bytes([3, 1, channel])
        assert ds_ie in frame

    def test_empty_ssid(self):
        frame = build_beacon(ssid='', timestamp=0)
        # SSID IE with empty string: tag=0, len=0
        assert bytes([0, 0]) in frame


class TestDeauthContent:
    """Verify deauth frame fields."""

    def test_reason_code_default(self):
        frame = build_deauth()
        # Reason code is last 2 bytes of the 26-byte frame
        reason = struct.unpack_from('<H', frame, 24)[0]
        assert reason == 7

    def test_reason_code_custom(self):
        frame = build_deauth(reason=1)
        reason = struct.unpack_from('<H', frame, 24)[0]
        assert reason == 1


class TestProbeResponseContent:
    """Verify probe response structure."""

    def test_ssid_ie_present(self):
        ssid = 'ProbeNet'
        frame = build_probe_response(da=BROADCAST, ssid=ssid, timestamp=0)
        ssid_ie = bytes([0, len(ssid)]) + ssid.encode('utf-8')
        assert ssid_ie in frame

    def test_ds_channel_ie(self):
        channel = 149
        frame = build_probe_response(
            da=BROADCAST, ssid='test', channel=channel, timestamp=0
        )
        ds_ie = bytes([3, 1, channel])
        assert ds_ie in frame


# ============================================================================
# PHY roundtrip tests
# ============================================================================

class TestPhyRoundtrip:
    """Verify frames survive PHY TX → RX loopback."""

    def test_beacon_phy_roundtrip(self):
        """Build beacon → generate_frame at 6 Mbps → verify_fcs passes."""
        psdu = build_beacon(ssid='roundtrip', timestamp=1000000)
        iq, meta = generate_frame(6, psdu)

        # The PSDU with FCS should pass verification
        psdu_with_fcs = meta['psdu_with_fcs']
        assert verify_fcs(psdu_with_fcs)

        # Also verify via decode
        result = decode_frame(iq, 0)
        assert result is not None
        assert 'error' not in result
        assert result['fcs_ok'] is True

    def test_deauth_phy_roundtrip(self):
        """Deauth survives PHY TX/RX at 6 Mbps."""
        psdu = build_deauth()
        iq, meta = generate_frame(6, psdu)
        result = decode_frame(iq, 0)
        assert result is not None
        assert 'error' not in result
        assert result['fcs_ok'] is True
        # Decoded PSDU (with FCS) minus FCS should match original
        assert result['psdu'][:-4] == psdu

    def test_ack_phy_roundtrip(self):
        """ACK frame survives PHY TX/RX loopback."""
        ra = b'\x00\x11\x22\x33\x44\x55'
        psdu = build_ack(ra)
        iq, meta = generate_frame(6, psdu)
        result = decode_frame(iq, 0)
        assert result is not None
        assert 'error' not in result
        assert result['fcs_ok'] is True
        assert result['psdu'][:-4] == psdu

    def test_rts_phy_roundtrip(self):
        """RTS frame survives PHY TX/RX loopback."""
        ra = b'\x00\x11\x22\x33\x44\x55'
        ta = b'\x02\x00\x00\x00\x00\x01'
        psdu = build_rts(ra, ta, duration=44)
        iq, meta = generate_frame(6, psdu)
        result = decode_frame(iq, 0)
        assert result is not None
        assert 'error' not in result
        assert result['fcs_ok'] is True
        assert result['psdu'][:-4] == psdu

    def test_cts_phy_roundtrip(self):
        """CTS frame survives PHY TX/RX loopback."""
        ra = b'\x00\x11\x22\x33\x44\x55'
        psdu = build_cts(ra, duration=40)
        iq, meta = generate_frame(6, psdu)
        result = decode_frame(iq, 0)
        assert result is not None
        assert 'error' not in result
        assert result['fcs_ok'] is True
        assert result['psdu'][:-4] == psdu

    def test_probe_response_phy_roundtrip(self):
        """Probe response survives PHY TX/RX at 24 Mbps."""
        psdu = build_probe_response(
            da=b'\x00\xaa\xbb\xcc\xdd\xee',
            ssid='FastNet',
            channel=36,
            timestamp=5000000,
        )
        iq, meta = generate_frame(24, psdu)
        result = decode_frame(iq, 0)
        assert result is not None
        assert 'error' not in result
        assert result['fcs_ok'] is True
        assert result['psdu'][:-4] == psdu
