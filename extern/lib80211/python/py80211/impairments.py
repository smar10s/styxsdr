"""802.11 channel impairment models.

AWGN, CFO, SFO, multipath, and IQ imbalance models for testing the
decoder's tolerance.
"""

import math
import numpy as np

# ============================================================================
# AWGN
# ============================================================================


def add_awgn(iq: np.ndarray, snr_db: float, seed: int = None) -> np.ndarray:
    """Add AWGN at a given SNR (dB).

    SNR is signal power / noise power (linear, not per-sample).
    """
    rng = np.random.default_rng(seed) if seed is not None else np.random.default_rng()
    sig_pwr = np.mean(np.abs(iq) ** 2)
    if sig_pwr < 1e-20:
        return iq.copy()
    noise_pwr = sig_pwr / (10.0 ** (snr_db / 10.0))
    noise = np.sqrt(noise_pwr / 2) * (
        rng.standard_normal(len(iq)) + 1j * rng.standard_normal(len(iq))
    )
    return iq + noise


# ============================================================================
# CFO (Carrier Frequency Offset)
# ============================================================================


def add_cfo(iq: np.ndarray, cfo_hz: float,
            sample_rate: float = 20_000_000,
            phi0: float = 0.0) -> np.ndarray:
    """Apply CFO by rotating samples at the given frequency offset.

    Args:
        iq: Complex IQ samples.
        cfo_hz: Carrier frequency offset in Hz.
        sample_rate: Sample rate in Hz.
        phi0: Initial phase offset in radians (models unknown oscillator phase).
    """
    t = np.arange(len(iq)) / sample_rate
    return iq * np.exp(1j * (2.0 * np.pi * cfo_hz * t + phi0))


# ============================================================================
# SFO (Sample Frequency Offset)
# ============================================================================


def add_sfo(iq: np.ndarray, ppm: float,
            sample_rate: float = 20_000_000) -> np.ndarray:
    """Apply SFO via linear interpolation resampling.

    A receiver sampling at (1 + ppm*1e-6) times the TX rate.
    Positive ppm means the receiver clock is faster (more samples).
    """
    ratio = 1.0 + ppm * 1e-6
    n_out = int(len(iq) * ratio)
    in_idx = np.arange(n_out) / ratio
    lo = np.floor(in_idx).astype(int)
    hi = np.minimum(lo + 1, len(iq) - 1)
    frac = in_idx - lo
    return iq[lo] * (1 - frac) + iq[hi] * frac


# ============================================================================
# Multipath (tapped delay line)
# ============================================================================


def apply_multipath(iq: np.ndarray,
                    taps: list[tuple[int, complex]]) -> np.ndarray:
    """Apply a tapped-delay-line multipath channel.

    Args:
        iq: Complex IQ samples
        taps: List of (delay_samples, complex_gain) tuples.
              The first tap should be (0, 1.0+0j) for line-of-sight.

    Returns:
        iq with multipath applied.
    """
    result = np.zeros_like(iq)
    for delay, gain in taps:
        if delay == 0:
            result += iq * gain
        else:
            result[delay:] += iq[:-delay] * gain
    return result


MULTIPATH_PRESETS = {
    "mild": [
        (0, 1.0 + 0j),
        (3, -0.3 + 0.1j),
    ],
    "moderate": [
        (0, 1.0 + 0j),
        (5, -0.4 + 0.3j),
        (12, 0.2 - 0.1j),
    ],
    "severe": [
        (0, 1.0 + 0j),
        (8, 0.8 + 0.0j),
        (15, -0.5 + 0.0j),
    ],
}


# ============================================================================
# IQ imbalance
# ============================================================================


def add_iq_imbalance(iq: np.ndarray, gain_db: float = 1.0,
                     phase_deg: float = 5.0) -> np.ndarray:
    """Apply I/Q gain and phase imbalance.

    Args:
        iq: Complex IQ samples
        gain_db: I/Q gain imbalance in dB (positive = I channel gain excess)
        phase_deg: Phase imbalance in degrees

    Model: I' = I * (1 + gain_err), Q' = Q * (1 - gain_err) + I * sin(phase_err)
    """
    gain_lin = 10.0 ** (gain_db / 20.0)
    g_i = (1.0 + (gain_lin - 1.0) / 2.0)  # excess I gain
    g_q = (1.0 - (gain_lin - 1.0) / 2.0)  # reduced Q gain
    phi = np.deg2rad(phase_deg)
    i_out = iq.real * g_i
    q_out = iq.imag * g_q + iq.real * np.sin(phi)
    return i_out + 1j * q_out


# ============================================================================
# ADC quantization (AGC + finite-resolution digitizer)
# ============================================================================


def add_quantization(iq: np.ndarray, bits: int = 12) -> np.ndarray:
    """Simulate ADC quantization with AGC (full-scale normalization).

    Models: AGC sets peak to full-scale, then quantize to `bits` resolution.

    Args:
        iq: Complex IQ samples.
        bits: ADC resolution in bits (e.g., 12 for PlutoSDR).

    Returns:
        Quantized IQ samples at original scale.
    """
    peak = max(np.max(np.abs(iq.real)), np.max(np.abs(iq.imag)))
    if peak == 0:
        return iq
    normalized = iq / peak
    levels = 2 ** (bits - 1)
    i_q = np.round(normalized.real * levels) / levels
    q_q = np.round(normalized.imag * levels) / levels
    return (i_q + 1j * q_q) * peak


# ============================================================================
# Phase noise (simplified)
# ============================================================================


def add_phase_noise(iq: np.ndarray, strength: float = 0.01,
                    sample_rate: float = 20_000_000,
                    corner_hz: float = 100_000,
                    seed: int = None) -> np.ndarray:
    """Add simplified phase noise via AR(1) filtered random walk.

    The output phase process has steady-state RMS approximately
    strength / sqrt(1 - alpha^2) where alpha = exp(-2*pi*corner_hz/sample_rate).
    For default parameters (corner_hz=100kHz, sample_rate=20MHz):
      alpha ~ 0.969, RMS ~ strength * 4.05

    Args:
        iq: Complex IQ samples.
        strength: Per-sample innovation std-dev in radians (NOT the output
                  RMS — see formula above). Default 0.01 yields ~0.04 rad RMS.
        sample_rate: Sample rate in Hz.
        corner_hz: AR(1) corner frequency controlling correlation time.
        seed: RNG seed for reproducibility.
    """
    rng = np.random.default_rng(seed) if seed is not None else np.random.default_rng()
    # First-order autoregressive process: phi[n] = alpha*phi[n-1] + strength*w[n]
    alpha = np.exp(-2 * np.pi * corner_hz / sample_rate)
    innovations = strength * rng.standard_normal(len(iq))
    # IIR filter: H(z) = 1 / (1 - alpha*z^{-1})
    from scipy.signal import lfilter
    noise = lfilter([1.0], [1.0, -alpha], innovations)
    return iq * np.exp(1j * noise)


# ============================================================================
# DC offset (direct-conversion receiver leakage)
# ============================================================================


def add_dc_offset(iq: np.ndarray, dc_i: float = 0.0, dc_q: float = 0.0) -> np.ndarray:
    """Add a fixed DC offset to I and Q channels.

    Models LO leakage in direct-conversion receivers (e.g., PlutoSDR AD9361).
    The offset appears as a spike at subcarrier 0 (which 802.11 nulls, but
    adjacent bins can be affected by windowing/timing).

    Args:
        iq: Complex IQ samples.
        dc_i: DC offset on I channel (relative to signal RMS).
        dc_q: DC offset on Q channel (relative to signal RMS).

    Returns:
        IQ with DC offset added, scaled relative to signal RMS.
    """
    rms = np.sqrt(np.mean(np.abs(iq) ** 2))
    if rms < 1e-20:
        return iq.copy()
    return iq + rms * complex(dc_i, dc_q)


# ============================================================================
# AGC settling (power ramp during STF)
# ============================================================================


def add_agc_ramp(iq: np.ndarray, settle_samples: int = 128,
                 initial_gain_db: float = -20.0) -> np.ndarray:
    """Simulate AGC settling by ramping gain during the first N samples.

    PlutoSDR AGC takes ~10-15 us to settle (200-300 samples at 20 MSPS).
    During settling, the receiver gain ramps from an initial (low) value
    to the final (correct) value. This models the exponential convergence.

    Args:
        iq: Complex IQ samples (packet starts at sample 0).
        settle_samples: Number of samples for AGC to settle (~128-300 for PlutoSDR).
        initial_gain_db: Initial gain deficit in dB (negative = attenuated start).

    Returns:
        IQ with gain ramp applied to first settle_samples.
    """
    out = iq.copy()
    initial_lin = 10.0 ** (initial_gain_db / 20.0)
    # Exponential convergence: gain(n) = 1 - (1-initial) * exp(-n/tau)
    # Choose tau so that gain reaches ~95% of final at settle_samples
    tau = settle_samples / 3.0  # 3 time constants ≈ 95% settled
    n = np.arange(min(settle_samples, len(iq)))
    gain = 1.0 - (1.0 - initial_lin) * np.exp(-n / tau)
    out[:len(n)] = iq[:len(n)] * gain
    return out
