"""MAC frame construction helpers for TX interop testing.

Builds raw PSDU byte sequences for common 802.11 frame types.
All functions return bytes WITHOUT FCS — the PHY layer (gen_ofdm_frame)
appends FCS before transmission.

Reference: IEEE 802.11-2020 §9.3 (Frame formats).
"""

import struct
import time

# ============================================================================
# Frame Control values (little-endian as transmitted)
# ============================================================================
FC_BEACON = 0x0080          # Type=Management, Subtype=Beacon
FC_DEAUTH = 0x00C0          # Type=Management, Subtype=Deauthentication
FC_PROBE_RESPONSE = 0x0050  # Type=Management, Subtype=Probe Response
FC_ACK = 0x00D4             # Type=Control, Subtype=ACK
FC_RTS = 0x00B4             # Type=Control, Subtype=RTS
FC_CTS = 0x00C4             # Type=Control, Subtype=CTS

# ============================================================================
# Defaults
# ============================================================================
BROADCAST = b'\xff\xff\xff\xff\xff\xff'
DEFAULT_BSSID = b'\x02\x00\x00\x00\x00\x01'  # Locally administered
DEFAULT_BEACON_INTERVAL = 100   # TU (1.024 ms each)
DEFAULT_CAPABILITIES = 0x0431
DEFAULT_CHANNEL = 36
DEFAULT_DEAUTH_REASON = 7      # Class 3 frame from nonassociated STA
DEFAULT_RTS_DURATION = 44
DEFAULT_CTS_DURATION = 40

# Supported Rates IE payload: 6*, 9, 12*, 18, 24*, 36, 48, 54 Mbps
# Mandatory rates are marked with bit 7 set (0x80 | rate_in_0.5Mbps)
_SUPPORTED_RATES = bytes([
    0x80 | 12,  # 6 Mbps (mandatory)
    18,         # 9 Mbps
    0x80 | 48,  # 24 Mbps (mandatory)
    72,         # 36 Mbps
    96,         # 48 Mbps
    108,        # 54 Mbps
])


# ============================================================================
# IE (Information Element) builders
# ============================================================================

def _ie(tag: int, data: bytes) -> bytes:
    """Build a single TLV information element."""
    return bytes([tag, len(data)]) + data


def _ie_ssid(ssid: str) -> bytes:
    """SSID IE (tag 0)."""
    return _ie(0, ssid.encode('utf-8'))


def _ie_supported_rates() -> bytes:
    """Supported Rates IE (tag 1)."""
    return _ie(1, _SUPPORTED_RATES)


def _ie_ds_parameter(channel: int) -> bytes:
    """DS Parameter Set IE (tag 3)."""
    return _ie(3, bytes([channel]))


def _ie_tim() -> bytes:
    """Minimal TIM IE (tag 5): DTIM count=0, period=1, no partial bitmap."""
    # Fields: DTIM Count, DTIM Period, Bitmap Control, Partial Virtual Bitmap
    return _ie(5, bytes([0, 1, 0, 0]))


# ============================================================================
# Management frame header
# ============================================================================

def _mgmt_header(fc: int, da: bytes, sa: bytes, bssid: bytes,
                 seq_ctrl: int = 0, duration: int = 0) -> bytes:
    """Build 24-byte management frame header.

    Layout: FC(2) + Duration(2) + DA(6) + SA(6) + BSSID(6) + SeqCtrl(2)
    """
    return struct.pack('<HH', fc, duration) + da + sa + bssid + struct.pack('<H', seq_ctrl)


# ============================================================================
# Frame builders
# ============================================================================

def build_beacon(
    ssid: str = 'lib80211',
    bssid: bytes = DEFAULT_BSSID,
    channel: int = DEFAULT_CHANNEL,
    beacon_interval: int = DEFAULT_BEACON_INTERVAL,
    capabilities: int = DEFAULT_CAPABILITIES,
    timestamp: int = None,
) -> bytes:
    """Build a Beacon frame PSDU (without FCS).

    Returns raw bytes ready for generate_frame().
    """
    if timestamp is None:
        timestamp = int(time.time() * 1_000_000)

    header = _mgmt_header(FC_BEACON, BROADCAST, bssid, bssid)

    # Fixed fields: Timestamp(8) + Beacon Interval(2) + Capability(2)
    fixed = struct.pack('<QHH', timestamp, beacon_interval, capabilities)

    # Information Elements
    ies = (
        _ie_ssid(ssid)
        + _ie_supported_rates()
        + _ie_ds_parameter(channel)
        + _ie_tim()
    )

    return header + fixed + ies


def build_deauth(
    da: bytes = BROADCAST,
    sa: bytes = DEFAULT_BSSID,
    bssid: bytes = DEFAULT_BSSID,
    reason: int = DEFAULT_DEAUTH_REASON,
) -> bytes:
    """Build a Deauthentication frame PSDU (without FCS).

    Returns 26 bytes: 24-byte header + 2-byte reason code.
    """
    header = _mgmt_header(FC_DEAUTH, da, sa, bssid)
    body = struct.pack('<H', reason)
    return header + body


def build_probe_response(
    da: bytes = BROADCAST,
    ssid: str = 'lib80211',
    bssid: bytes = DEFAULT_BSSID,
    channel: int = DEFAULT_CHANNEL,
    beacon_interval: int = DEFAULT_BEACON_INTERVAL,
    capabilities: int = DEFAULT_CAPABILITIES,
    timestamp: int = None,
) -> bytes:
    """Build a Probe Response frame PSDU (without FCS).

    Returns raw bytes ready for generate_frame().
    """
    if timestamp is None:
        timestamp = int(time.time() * 1_000_000)

    header = _mgmt_header(FC_PROBE_RESPONSE, da, bssid, bssid)

    # Fixed fields: Timestamp(8) + Beacon Interval(2) + Capability(2)
    fixed = struct.pack('<QHH', timestamp, beacon_interval, capabilities)

    # Information Elements (same as beacon minus TIM)
    ies = (
        _ie_ssid(ssid)
        + _ie_supported_rates()
        + _ie_ds_parameter(channel)
    )

    return header + fixed + ies


def build_ack(ra: bytes) -> bytes:
    """Build an ACK frame PSDU (without FCS).

    Returns 10 bytes: FC(2) + Duration(2) + RA(6).
    """
    return struct.pack('<HH', FC_ACK, 0) + ra


def build_rts(
    ra: bytes,
    ta: bytes,
    duration: int = DEFAULT_RTS_DURATION,
) -> bytes:
    """Build an RTS frame PSDU (without FCS).

    Returns 16 bytes: FC(2) + Duration(2) + RA(6) + TA(6).
    """
    return struct.pack('<HH', FC_RTS, duration) + ra + ta


def build_cts(
    ra: bytes,
    duration: int = DEFAULT_CTS_DURATION,
) -> bytes:
    """Build a CTS frame PSDU (without FCS).

    Returns 10 bytes: FC(2) + Duration(2) + RA(6).
    """
    return struct.pack('<HH', FC_CTS, duration) + ra
