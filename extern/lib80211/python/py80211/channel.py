"""Composable channel model for multi-frame IQ stream generation.

Generates realistic multi-frame WiFi baseband captures with coherent
impairments applied in physically correct order:
  Per-frame: amplitude scaling, per-frame CFO, AGC settling
  Channel-wide: multipath → CFO → SFO → phase_noise → DC offset → AWGN

Inter-frame gaps are filled with thermal noise at noise_floor_dbfs.
"""

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .gen_ofdm_frame import generate_frame
from .impairments import (
    add_agc_ramp,
    add_awgn,
    add_cfo,
    add_dc_offset,
    add_phase_noise,
    add_sfo,
    apply_multipath,
)

# ============================================================================
# Constants
# ============================================================================
SAMPLE_RATE = 20_000_000
SIFS_SAMPLES = 320   # 16 us at 20 MSPS
DIFS_SAMPLES = 680   # 34 us at 20 MSPS


# ============================================================================
# ChannelConfig dataclass
# ============================================================================
@dataclass
class ChannelConfig:
    """All channel parameters for a multi-frame stream."""

    # AWGN
    snr_db: float = 30.0

    # Carrier frequency offset (channel-wide, Hz)
    cfo_hz: float = 0.0

    # Sample frequency offset (ppm)
    sfo_ppm: float = 0.0

    # Multipath taps: list of (delay_samples, complex_gain)
    multipath_taps: list = field(default_factory=lambda: [(0, 1.0 + 0j)])

    # Phase noise
    phase_noise_strength: float = 0.0

    # DC offset (I, Q relative to signal RMS)
    dc_offset_i: float = 0.0
    dc_offset_q: float = 0.0

    # Noise floor for inter-frame gaps (dBFS)
    noise_floor_dbfs: float = -60.0

    # AGC settling per frame
    agc_settle_samples: int = 0
    agc_initial_gain_db: float = -20.0

    @classmethod
    def clean(cls) -> "ChannelConfig":
        """No impairments — ideal channel."""
        return cls(
            snr_db=100.0,
            cfo_hz=0.0,
            sfo_ppm=0.0,
            multipath_taps=[(0, 1.0 + 0j)],
            phase_noise_strength=0.0,
            dc_offset_i=0.0,
            dc_offset_q=0.0,
            noise_floor_dbfs=-80.0,
            agc_settle_samples=0,
        )

    @classmethod
    def cable_loopback(cls) -> "ChannelConfig":
        """RF cable loopback — mild CFO, no multipath, good SNR."""
        return cls(
            snr_db=40.0,
            cfo_hz=1200.0,
            sfo_ppm=2.0,
            multipath_taps=[(0, 1.0 + 0j)],
            phase_noise_strength=0.005,
            dc_offset_i=0.01,
            dc_offset_q=0.01,
            noise_floor_dbfs=-55.0,
            agc_settle_samples=128,
            agc_initial_gain_db=-15.0,
        )

    @classmethod
    def indoor_office(cls) -> "ChannelConfig":
        """Indoor office — moderate multipath, medium SNR."""
        return cls(
            snr_db=25.0,
            cfo_hz=3500.0,
            sfo_ppm=5.0,
            multipath_taps=[
                (0, 1.0 + 0j),
                (5, -0.4 + 0.3j),
                (12, 0.2 - 0.1j),
            ],
            phase_noise_strength=0.01,
            dc_offset_i=0.02,
            dc_offset_q=0.015,
            noise_floor_dbfs=-50.0,
            agc_settle_samples=200,
            agc_initial_gain_db=-20.0,
        )

    @classmethod
    def near_far(cls) -> "ChannelConfig":
        """Near-far scenario — frames at very different power levels."""
        return cls(
            snr_db=30.0,
            cfo_hz=2000.0,
            sfo_ppm=3.0,
            multipath_taps=[(0, 1.0 + 0j), (3, -0.2 + 0.1j)],
            phase_noise_strength=0.008,
            dc_offset_i=0.01,
            dc_offset_q=0.01,
            noise_floor_dbfs=-55.0,
            agc_settle_samples=160,
            agc_initial_gain_db=-25.0,
        )


# ============================================================================
# FrameSpec dataclass
# ============================================================================
@dataclass
class FrameSpec:
    """Per-frame metadata for stream generation."""

    rate_mbps: int = 6
    psdu_len: int = 100
    amplitude: float = 1.0
    cfo_hz: float = 0.0
    gap_samples: int = DIFS_SAMPLES
    psdu: Optional[bytes] = None

    def get_psdu(self, rng: np.random.Generator) -> bytes:
        """Return PSDU bytes — use provided or generate random."""
        if self.psdu is not None:
            return self.psdu
        return bytes(rng.integers(0, 256, size=self.psdu_len, dtype=np.uint8))


# ============================================================================
# Traffic pattern helpers
# ============================================================================
def eapol_handshake(
    ap_amplitude: float = 1.0,
    sta_amplitude: float = 0.5,
    gap: int = SIFS_SAMPLES,
) -> list[FrameSpec]:
    """4-way EAPOL handshake: M1(AP), M2(STA), M3(AP), M4(STA), all rate 6."""
    return [
        FrameSpec(rate_mbps=6, psdu_len=133, amplitude=ap_amplitude, gap_samples=gap),
        FrameSpec(rate_mbps=6, psdu_len=155, amplitude=sta_amplitude, gap_samples=gap),
        FrameSpec(rate_mbps=6, psdu_len=189, amplitude=ap_amplitude, gap_samples=gap),
        FrameSpec(rate_mbps=6, psdu_len=133, amplitude=sta_amplitude, gap_samples=gap),
    ]


def beacon_data_mix(
    n_beacons: int = 3,
    n_data: int = 5,
    beacon_amplitude: float = 1.0,
    data_amplitude: float = 0.7,
) -> list[FrameSpec]:
    """Mix of beacons (rate 6, 350B) and data frames (rate 24, 200B)."""
    frames = []
    for _ in range(n_beacons):
        frames.append(FrameSpec(
            rate_mbps=6, psdu_len=350, amplitude=beacon_amplitude,
            gap_samples=DIFS_SAMPLES,
        ))
    for _ in range(n_data):
        frames.append(FrameSpec(
            rate_mbps=24, psdu_len=200, amplitude=data_amplitude,
            gap_samples=SIFS_SAMPLES,
        ))
    return frames


def power_sweep_pair(
    rate_mbps: int = 6,
    psdu_len: int = 100,
    ratio_db: float = 10.0,
    gap: int = DIFS_SAMPLES,
) -> list[FrameSpec]:
    """Two frames: loud (amplitude=1.0) then quiet (amplitude reduced by ratio_db)."""
    quiet_amplitude = 10.0 ** (-ratio_db / 20.0)
    return [
        FrameSpec(rate_mbps=rate_mbps, psdu_len=psdu_len, amplitude=1.0, gap_samples=gap),
        FrameSpec(rate_mbps=rate_mbps, psdu_len=psdu_len, amplitude=quiet_amplitude, gap_samples=gap),
    ]


# ============================================================================
# Stream generation
# ============================================================================
def generate_stream(
    frames: list[FrameSpec],
    config: ChannelConfig,
    seed: int = 42,
) -> tuple[np.ndarray, list[dict]]:
    """Generate a multi-frame IQ stream through the channel model.

    Args:
        frames: List of FrameSpec describing each frame.
        config: ChannelConfig with channel-wide impairment parameters.
        seed: RNG seed for reproducibility.

    Returns:
        (iq_stream, metadata) where metadata is a list of per-frame dicts
        containing rate, amplitude, stf_offset, n_samples, etc.
    """
    rng = np.random.default_rng(seed)

    # Phase 1: Generate individual frames and concatenate with gaps
    segments = []
    metadata = []
    current_offset = 0

    for i, fspec in enumerate(frames):
        # Inter-frame gap (before each frame, including the first)
        gap_len = fspec.gap_samples
        if gap_len > 0:
            # Thermal noise at noise floor level
            noise_amplitude = 10.0 ** (config.noise_floor_dbfs / 20.0)
            gap_noise = noise_amplitude * (
                rng.standard_normal(gap_len) + 1j * rng.standard_normal(gap_len)
            ) / np.sqrt(2.0)
            segments.append(gap_noise.astype(np.complex64))
            current_offset += gap_len

        # Generate baseband frame
        psdu = fspec.get_psdu(rng)
        scrambler_seed = int(rng.integers(1, 128))
        frame_iq, frame_meta = generate_frame(fspec.rate_mbps, psdu, scrambler_seed)

        # Per-frame amplitude scaling
        frame_iq = frame_iq * fspec.amplitude

        # Per-frame CFO (models different transmitter oscillators)
        if fspec.cfo_hz != 0.0:
            frame_iq = add_cfo(frame_iq, fspec.cfo_hz, SAMPLE_RATE)

        # Per-frame AGC settling
        if config.agc_settle_samples > 0:
            frame_iq = add_agc_ramp(
                frame_iq,
                settle_samples=config.agc_settle_samples,
                initial_gain_db=config.agc_initial_gain_db,
            )

        # Record metadata
        metadata.append({
            "rate": fspec.rate_mbps,
            "amplitude": fspec.amplitude,
            "stf_offset": current_offset,
            "n_samples": len(frame_iq),
            "psdu_len": len(psdu),
            "cfo_hz": fspec.cfo_hz,
        })

        segments.append(frame_iq)
        current_offset += len(frame_iq)

    # Concatenate all segments
    stream = np.concatenate(segments).astype(np.complex64)

    # Phase 2: Apply channel-wide effects in physical order
    # 1. Multipath
    if config.multipath_taps != [(0, 1.0 + 0j)]:
        stream = apply_multipath(stream, config.multipath_taps)

    # 2. Channel-wide CFO
    if config.cfo_hz != 0.0:
        stream = add_cfo(stream, config.cfo_hz, SAMPLE_RATE)

    # 3. SFO (resamples — changes length)
    if config.sfo_ppm != 0.0:
        ratio = 1.0 + config.sfo_ppm * 1e-6
        stream = add_sfo(stream, config.sfo_ppm, SAMPLE_RATE)
        # Adjust stf_offsets for resampling stretch
        for m in metadata:
            m["stf_offset"] = int(m["stf_offset"] * ratio)

    # 4. Phase noise
    if config.phase_noise_strength > 0.0:
        pn_seed = int(rng.integers(0, 2**31))
        stream = add_phase_noise(
            stream, strength=config.phase_noise_strength,
            sample_rate=SAMPLE_RATE, seed=pn_seed,
        )

    # 5. DC offset
    if config.dc_offset_i != 0.0 or config.dc_offset_q != 0.0:
        stream = add_dc_offset(stream, config.dc_offset_i, config.dc_offset_q)

    # 6. AWGN
    if config.snr_db < 100.0:
        awgn_seed = int(rng.integers(0, 2**31))
        stream = add_awgn(stream, config.snr_db, seed=awgn_seed)

    return stream.astype(np.complex64), metadata
