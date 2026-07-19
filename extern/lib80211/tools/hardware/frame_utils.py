"""Shared 802.11 frame parsing utilities for hardware test tools."""

# --- Frame type lookup table: (type, subtype) → human-readable name ---
FRAME_TYPE_NAMES = {
    # Management (type 0)
    (0, 0): "Assoc Req",
    (0, 1): "Assoc Resp",
    (0, 2): "Reassoc Req",
    (0, 3): "Reassoc Resp",
    (0, 4): "Probe Req",
    (0, 5): "Probe Resp",
    (0, 8): "Beacon",
    (0, 10): "Disassoc",
    (0, 11): "Auth",
    (0, 12): "Deauth",
    (0, 13): "Action",
    # Control (type 1)
    (1, 8): "Block Ack Req",
    (1, 9): "Block Ack",
    (1, 10): "PS-Poll",
    (1, 11): "RTS",
    (1, 12): "CTS",
    (1, 13): "Ack",
    # Data (type 2)
    (2, 0): "Data",
    (2, 4): "Null Data",
    (2, 8): "QoS Data",
    (2, 12): "QoS Null",
}

# --- Channel-to-frequency mapping (Hz) ---
CHANNEL_FREQ = {
    # 2.4 GHz
    1: 2_412_000_000,
    6: 2_437_000_000,
    11: 2_462_000_000,
    # 5 GHz UNII-1
    36: 5_180_000_000,
    40: 5_200_000_000,
    44: 5_220_000_000,
    48: 5_240_000_000,
    # 5 GHz UNII-2
    52: 5_260_000_000,
    56: 5_280_000_000,
    60: 5_300_000_000,
    64: 5_320_000_000,
    # 5 GHz UNII-3
    149: 5_745_000_000,
    153: 5_765_000_000,
    157: 5_785_000_000,
    161: 5_805_000_000,
}


def classify_frame(psdu):
    """Parse FC bytes, return (frame_type, subtype, name)."""
    if len(psdu) < 2:
        return None, None, "too_short"
    fc = psdu[0] | (psdu[1] << 8)
    frame_type = (fc >> 2) & 0x03
    subtype = (fc >> 4) & 0x0F
    name = FRAME_TYPE_NAMES.get((frame_type, subtype), f"Type{frame_type}/Sub{subtype}")
    return frame_type, subtype, name


def extract_bssid(psdu):
    """Extract BSSID from management frame (bytes 16-22).

    Returns colon-separated hex string or None.
    """
    if len(psdu) < 22:
        return None
    fc = psdu[0] | (psdu[1] << 8)
    frame_type = (fc >> 2) & 0x03
    if frame_type != 0:  # management only
        return None
    return ":".join(f"{b:02x}" for b in psdu[16:22])


def extract_ssid(psdu):
    """Extract SSID from beacon/probe response tagged parameters.

    Beacon body: MAC header (24B) + fixed fields (12B: timestamp 8B,
    interval 2B, capability 2B) + tagged params starting at offset 36.
    SSID is tag_id=0.
    """
    if len(psdu) < 38:  # 24 + 12 + 2 min tag
        return None
    offset = 36
    end = len(psdu) - 4  # exclude FCS
    while offset + 2 <= end:
        tag_id = psdu[offset]
        tag_len = psdu[offset + 1]
        if offset + 2 + tag_len > end:
            break
        if tag_id == 0:  # SSID
            ssid_bytes = bytes(psdu[offset + 2:offset + 2 + tag_len])
            try:
                return ssid_bytes.decode("utf-8", errors="replace")
            except Exception:
                return repr(ssid_bytes)
        offset += 2 + tag_len
    return None
