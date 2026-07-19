"""Channel model tests — validates composable multi-frame stream generation.

Tests the channel model's ability to:
1. Generate correct multi-frame IQ streams
2. Decode individual frames through channel presets
3. Exercise scenarios not covered by single-frame impairment tests:
   - Multiple frames in one buffer (back-to-back decode)
   - Near-far power transitions
   - Traffic pattern helpers (EAPOL, beacon/data mix)
   - Channel-wide vs per-frame impairment application order

Each test uses deterministic seeds. Thresholds are conservative — these
are regression gates, not sensitivity measurements.
"""

import numpy as np
import pytest

from conftest import PROJECT_ROOT

import sys
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from py80211.channel import (
    ChannelConfig,
    FrameSpec,
    generate_stream,
    eapol_handshake,
    beacon_data_mix,
    power_sweep_pair,
    SAMPLE_RATE,
    SIFS_SAMPLES,
    DIFS_SAMPLES,
)
from py80211.decode_frame import decode_frame, detect_stf
from py80211.gen_ofdm_frame import generate_frame


# ============================================================================
# Helpers
# ============================================================================

def decode_at(iq: np.ndarray, offset: int) -> dict | None:
    """Decode frame starting search from offset."""
    if offset >= len(iq):
        return None
    stf = detect_stf(iq[offset:], threshold=0.5, min_periods=6)
    if stf < 0:
        return None
    return decode_frame(iq[offset:], stf)


def decode_all_frames(iq: np.ndarray, metadata: list[dict]) -> list[dict | None]:
    """Attempt decode at each frame's expected STF offset."""
    results = []
    for m in metadata:
        # Search from slightly before expected offset to handle SFO drift
        search_start = max(0, m["stf_offset"] - 16)
        result = decode_at(iq, search_start)
        results.append(result)
    return results


def count_successful(results: list[dict | None]) -> int:
    """Count frames that decoded with valid FCS."""
    return sum(1 for r in results if r is not None and r.get("fcs_ok"))


# ============================================================================
# ChannelConfig presets — smoke tests
# ============================================================================

class TestChannelPresets:
    """Verify channel presets produce valid streams and frames decode."""

    def test_clean_single_frame(self):
        """Clean channel, single frame — must decode perfectly."""
        config = ChannelConfig.clean()
        frames = [FrameSpec(rate_mbps=6, psdu_len=100)]
        iq, meta = generate_stream(frames, config, seed=0)

        assert len(meta) == 1
        assert meta[0]["stf_offset"] > 0  # gap before frame
        assert meta[0]["rate"] == 6

        result = decode_at(iq, 0)
        assert result is not None
        assert result["fcs_ok"]

    def test_clean_preserves_payload(self):
        """Clean channel with known PSDU — payload survives roundtrip."""
        payload = bytes(range(50))
        config = ChannelConfig.clean()
        frames = [FrameSpec(rate_mbps=6, psdu_len=50, psdu=payload)]
        iq, meta = generate_stream(frames, config, seed=0)

        result = decode_at(iq, 0)
        assert result is not None
        assert result["fcs_ok"]
        assert result["psdu"][:50] == payload

    def test_cable_loopback_single_frame(self):
        """Cable loopback preset — rate 6 should decode reliably."""
        config = ChannelConfig.cable_loopback()
        successes = 0
        for seed in range(20):
            frames = [FrameSpec(rate_mbps=6, psdu_len=100)]
            iq, meta = generate_stream(frames, config, seed=seed)
            result = decode_at(iq, 0)
            if result is not None and result.get("fcs_ok"):
                successes += 1
        assert successes >= 16, f"Cable loopback: {successes}/20 < 80%"

    def test_indoor_office_rate6(self):
        """Indoor office preset — rate 6 (most robust) should mostly decode."""
        config = ChannelConfig.indoor_office()
        successes = 0
        for seed in range(20):
            frames = [FrameSpec(rate_mbps=6, psdu_len=100)]
            iq, meta = generate_stream(frames, config, seed=seed)
            result = decode_at(iq, 0)
            if result is not None and result.get("fcs_ok"):
                successes += 1
        assert successes >= 14, f"Indoor office rate 6: {successes}/20 < 70%"


# ============================================================================
# Multi-frame decode — the key gap this fills
# ============================================================================

class TestMultiFrame:
    """Verify decode of multiple frames within a single IQ buffer."""

    def test_two_frames_clean(self):
        """Two frames in clean channel — both must decode."""
        config = ChannelConfig.clean()
        frames = [
            FrameSpec(rate_mbps=6, psdu_len=80, gap_samples=DIFS_SAMPLES),
            FrameSpec(rate_mbps=6, psdu_len=80, gap_samples=DIFS_SAMPLES),
        ]
        iq, meta = generate_stream(frames, config, seed=0)

        assert len(meta) == 2
        assert meta[1]["stf_offset"] > meta[0]["stf_offset"]

        results = decode_all_frames(iq, meta)
        assert count_successful(results) == 2

    def test_three_frames_mixed_rates(self):
        """Three frames at different rates — all decode in clean channel."""
        config = ChannelConfig.clean()
        frames = [
            FrameSpec(rate_mbps=6, psdu_len=100, gap_samples=DIFS_SAMPLES),
            FrameSpec(rate_mbps=24, psdu_len=80, gap_samples=SIFS_SAMPLES),
            FrameSpec(rate_mbps=54, psdu_len=50, gap_samples=SIFS_SAMPLES),
        ]
        iq, meta = generate_stream(frames, config, seed=0)
        results = decode_all_frames(iq, meta)
        assert count_successful(results) == 3

    def test_two_frames_cable_loopback(self):
        """Two frames through cable loopback — both should decode."""
        config = ChannelConfig.cable_loopback()
        successes = 0
        for seed in range(20):
            frames = [
                FrameSpec(rate_mbps=6, psdu_len=80, gap_samples=DIFS_SAMPLES),
                FrameSpec(rate_mbps=6, psdu_len=80, gap_samples=DIFS_SAMPLES),
            ]
            iq, meta = generate_stream(frames, config, seed=seed)
            results = decode_all_frames(iq, meta)
            if count_successful(results) == 2:
                successes += 1
        # Both frames decoding is harder — accept 70%
        assert successes >= 14, f"Two-frame cable loopback: {successes}/20 < 70%"

    def test_sifs_gap_decode(self):
        """Frames separated by SIFS (16 us) — tight spacing stress test."""
        config = ChannelConfig.clean()
        frames = [
            FrameSpec(rate_mbps=6, psdu_len=50, gap_samples=SIFS_SAMPLES),
            FrameSpec(rate_mbps=6, psdu_len=50, gap_samples=SIFS_SAMPLES),
        ]
        iq, meta = generate_stream(frames, config, seed=42)
        results = decode_all_frames(iq, meta)
        assert count_successful(results) == 2


# ============================================================================
# Traffic pattern helpers
# ============================================================================

class TestTrafficPatterns:
    """Verify traffic pattern helpers produce valid multi-frame streams."""

    def test_eapol_handshake_clean(self):
        """4-way EAPOL handshake in clean channel — all 4 frames decode."""
        config = ChannelConfig.clean()
        frames = eapol_handshake()
        assert len(frames) == 4

        iq, meta = generate_stream(frames, config, seed=0)
        assert len(meta) == 4

        results = decode_all_frames(iq, meta)
        assert count_successful(results) == 4

    def test_eapol_near_far(self):
        """EAPOL near-far — AP loud, STA quiet, both should decode at good SNR."""
        config = ChannelConfig(snr_db=35.0)
        frames = eapol_handshake(ap_amplitude=1.0, sta_amplitude=0.3)

        successes = 0
        for seed in range(20):
            iq, meta = generate_stream(frames, config, seed=seed)
            results = decode_all_frames(iq, meta)
            if count_successful(results) == 4:
                successes += 1
        assert successes >= 14, f"EAPOL near-far: {successes}/20 < 70%"

    def test_beacon_data_mix_clean(self):
        """Beacon + data mix in clean channel — all frames decode."""
        config = ChannelConfig.clean()
        frames = beacon_data_mix(n_beacons=2, n_data=3)
        assert len(frames) == 5

        iq, meta = generate_stream(frames, config, seed=0)
        results = decode_all_frames(iq, meta)
        assert count_successful(results) == 5

    def test_power_sweep_pair_clean(self):
        """Power sweep: loud then quiet frame, both decode in clean channel."""
        config = ChannelConfig.clean()
        frames = power_sweep_pair(rate_mbps=6, psdu_len=80, ratio_db=10.0)
        assert len(frames) == 2

        iq, meta = generate_stream(frames, config, seed=0)
        results = decode_all_frames(iq, meta)
        assert count_successful(results) == 2

    def test_power_sweep_with_noise(self):
        """Power sweep at 30 dB SNR — loud frame easy, quiet frame harder."""
        config = ChannelConfig(snr_db=30.0)
        frames = power_sweep_pair(rate_mbps=6, psdu_len=80, ratio_db=10.0)

        loud_ok = 0
        quiet_ok = 0
        for seed in range(20):
            iq, meta = generate_stream(frames, config, seed=seed)
            results = decode_all_frames(iq, meta)
            if results[0] is not None and results[0].get("fcs_ok"):
                loud_ok += 1
            if results[1] is not None and results[1].get("fcs_ok"):
                quiet_ok += 1

        # Loud frame should almost always decode
        assert loud_ok >= 18, f"Loud frame: {loud_ok}/20"
        # Quiet frame (-10 dB → effective 20 dB SNR) should mostly decode
        assert quiet_ok >= 14, f"Quiet frame: {quiet_ok}/20"


# ============================================================================
# FrameSpec API tests
# ============================================================================

class TestFrameSpec:
    """Verify FrameSpec dataclass behavior."""

    def test_custom_psdu(self):
        """FrameSpec with explicit PSDU uses it verbatim."""
        payload = b"Hello, WiFi!"
        spec = FrameSpec(rate_mbps=6, psdu_len=12, psdu=payload)
        rng = np.random.default_rng(0)
        assert spec.get_psdu(rng) == payload

    def test_random_psdu_deterministic(self):
        """Random PSDU generation is deterministic given same RNG state."""
        spec = FrameSpec(rate_mbps=6, psdu_len=50)
        rng1 = np.random.default_rng(42)
        rng2 = np.random.default_rng(42)
        assert spec.get_psdu(rng1) == spec.get_psdu(rng2)

    def test_per_frame_cfo(self):
        """Per-frame CFO shifts carrier within that frame only."""
        config = ChannelConfig.clean()
        frames = [
            FrameSpec(rate_mbps=6, psdu_len=50, cfo_hz=5000.0),
            FrameSpec(rate_mbps=6, psdu_len=50, cfo_hz=-5000.0),
        ]
        iq, meta = generate_stream(frames, config, seed=0)
        # Both should still decode (decoder handles CFO)
        results = decode_all_frames(iq, meta)
        assert count_successful(results) == 2


# ============================================================================
# Stream metadata correctness
# ============================================================================

class TestStreamMetadata:
    """Verify generate_stream metadata is accurate."""

    def test_offsets_monotonic(self):
        """STF offsets are strictly increasing."""
        config = ChannelConfig.clean()
        frames = [FrameSpec(rate_mbps=6, psdu_len=50) for _ in range(5)]
        _, meta = generate_stream(frames, config, seed=0)

        offsets = [m["stf_offset"] for m in meta]
        for i in range(1, len(offsets)):
            assert offsets[i] > offsets[i - 1]

    def test_offset_gap_consistent(self):
        """Gap between frames matches specified gap_samples."""
        config = ChannelConfig.clean()
        gap = 1000
        frames = [
            FrameSpec(rate_mbps=6, psdu_len=50, gap_samples=gap),
            FrameSpec(rate_mbps=6, psdu_len=50, gap_samples=gap),
        ]
        _, meta = generate_stream(frames, config, seed=0)

        # Second frame starts at: first_frame_offset + first_frame_samples + gap
        expected_offset = meta[0]["stf_offset"] + meta[0]["n_samples"] + gap
        assert meta[1]["stf_offset"] == expected_offset

    def test_sfo_adjusts_offsets(self):
        """SFO stretches the stream and adjusts metadata offsets."""
        frames = [
            FrameSpec(rate_mbps=6, psdu_len=50, gap_samples=1000),
            FrameSpec(rate_mbps=6, psdu_len=50, gap_samples=1000),
        ]

        # Generate without SFO
        config_clean = ChannelConfig.clean()
        _, meta_clean = generate_stream(frames, config_clean, seed=0)

        # Generate with +20 ppm SFO
        config_sfo = ChannelConfig(snr_db=100.0, sfo_ppm=20.0)
        iq_sfo, meta_sfo = generate_stream(frames, config_sfo, seed=0)

        # Offsets should be scaled by ratio
        ratio = 1.0 + 20.0e-6
        for mc, ms in zip(meta_clean, meta_sfo):
            assert ms["stf_offset"] == int(mc["stf_offset"] * ratio)
