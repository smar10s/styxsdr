"""Impairment robustness regression tests.

Verifies the decoder maintains acceptable performance under:
- AWGN at known SNR thresholds
- Carrier frequency offset (CFO)
- Multipath channels
- Combined impairments

Each test uses deterministic seeds so failures are reproducible.
Thresholds are set conservatively: if a test fails, it means a code
change degraded decode performance.
"""

import numpy as np
import pytest

from conftest import PROJECT_ROOT

import sys
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from py80211.gen_ofdm_frame import generate_frame, generate_ht_frame, generate_vht_frame, RATE_TABLE
from py80211.decode_frame import decode_frame, detect_stf
from py80211.impairments import (
    add_awgn,
    add_cfo,
    add_sfo,
    apply_multipath,
    add_iq_imbalance,
    add_phase_noise,
    add_dc_offset,
    add_agc_ramp,
    MULTIPATH_PRESETS,
)


# ============================================================================
# Helpers
# ============================================================================

PAYLOAD = bytes([(i * 7 + 13) % 256 for i in range(80)])


def decode_with_detection(iq: np.ndarray) -> dict:
    """Run STF detection + decode. Returns result dict or None."""
    stf = detect_stf(iq, threshold=0.5, min_periods=6)
    if stf < 0:
        return None
    return decode_frame(iq, stf)


def trial_legacy(rate: int, impairment_fn, seed: int) -> bool:
    """Single legacy trial. Returns True if FCS OK and payload matches."""
    iq, meta = generate_frame(rate, PAYLOAD)
    iq = impairment_fn(iq, seed)
    result = decode_with_detection(iq)
    if result is None or not result.get("fcs_ok"):
        return False
    return result["psdu"][:len(PAYLOAD)] == PAYLOAD


def trial_ht(mcs: int, impairment_fn, seed: int) -> bool:
    """Single HT trial. Returns True if FCS OK and payload matches."""
    iq, meta = generate_ht_frame(mcs, PAYLOAD)
    iq = impairment_fn(iq, seed)
    result = decode_with_detection(iq)
    if result is None or not result.get("fcs_ok"):
        return False
    return result["psdu"][:len(PAYLOAD)] == PAYLOAD


def trial_vht(mcs: int, impairment_fn, seed: int) -> bool:
    """Single VHT trial. Returns True if FCS OK and payload matches."""
    iq, meta = generate_vht_frame(mcs, PAYLOAD)
    iq = impairment_fn(iq, seed)
    result = decode_with_detection(iq)
    if result is None or not result.get("fcs_ok"):
        return False
    return result["psdu"][:len(PAYLOAD)] == PAYLOAD


def run_trials(trial_fn, n_trials: int = 20) -> float:
    """Run n_trials with seeds 0..n-1, return success rate.

    Default of 20 trials gives ~95% confidence that a decoder with true
    success rate >=90% won't spuriously fail (binomial: P(>=16/20 | p=0.9) > 0.99).
    """
    successes = sum(trial_fn(seed) for seed in range(n_trials))
    return successes / n_trials


# ============================================================================
# AWGN tests — verify decode at known SNR thresholds
# ============================================================================

class TestAWGN:
    """Verify decode succeeds at conservative SNR levels."""

    @pytest.mark.parametrize("rate,min_snr", [
        (6, 10),    # BPSK 1/2
        (9, 10),    # BPSK 3/4
        (12, 12),   # QPSK 1/2
        (18, 14),   # QPSK 3/4
        (24, 18),   # 16QAM 1/2
        (36, 20),   # 16QAM 3/4
        (48, 24),   # 64QAM 2/3
        (54, 26),   # 64QAM 3/4
    ])
    def test_legacy_awgn(self, rate, min_snr):
        """Legacy rate must decode at threshold SNR with >=80% success."""
        def impairment(iq, seed):
            return add_awgn(iq, snr_db=min_snr, seed=seed)

        success_rate = run_trials(lambda s: trial_legacy(rate, impairment, s))
        assert success_rate >= 0.8, (
            f"Rate {rate} at SNR {min_snr} dB: {success_rate:.0%} < 80%"
        )

    @pytest.mark.parametrize("mcs,min_snr", [
        (0, 8),     # BPSK 1/2
        (1, 10),    # QPSK 1/2
        (2, 12),    # QPSK 3/4
        (3, 14),    # 16QAM 1/2
        (4, 18),    # 16QAM 3/4
        (5, 22),    # 64QAM 2/3
        (6, 24),    # 64QAM 3/4
        (7, 26),    # 64QAM 5/6
    ])
    def test_ht_awgn(self, mcs, min_snr):
        """HT MCS must decode at threshold SNR with >=80% success."""
        def impairment(iq, seed):
            return add_awgn(iq, snr_db=min_snr, seed=seed)

        success_rate = run_trials(lambda s: trial_ht(mcs, impairment, s))
        assert success_rate >= 0.8, (
            f"MCS {mcs} at SNR {min_snr} dB: {success_rate:.0%} < 80%"
        )

    @pytest.mark.parametrize("mcs,snr_db", [
        (0, 10), (1, 11), (2, 13), (3, 15),
        (4, 18), (5, 21), (6, 23), (7, 25), (8, 28),
    ])
    def test_vht_awgn(self, mcs, snr_db):
        """VHT decode at conservative SNR (should get >=80% success)."""
        rate = run_trials(
            lambda seed: trial_vht(mcs, lambda iq, s: add_awgn(iq, snr_db, s), seed))
        assert rate >= 0.8, f"VHT MCS {mcs} at {snr_db} dB: {rate:.0%} < 80%"


# ============================================================================
# CFO tests — verify decoder's CFO correction range
# ============================================================================

class TestCFO:
    """Verify decode succeeds under carrier frequency offset."""

    @pytest.mark.parametrize("cfo_hz", [5000, 10000, 15000, -5000, -10000, -15000])
    def test_legacy_cfo(self, cfo_hz):
        """Rate-6 must tolerate CFO at high SNR."""
        def impairment(iq, seed):
            iq = add_cfo(iq, cfo_hz)
            return add_awgn(iq, snr_db=25, seed=seed)

        success_rate = run_trials(lambda s: trial_legacy(6, impairment, s))
        assert success_rate >= 0.8, (
            f"Rate 6 at CFO {cfo_hz} Hz: {success_rate:.0%} < 80%"
        )

    @pytest.mark.parametrize("cfo_hz", [5000, 10000, -5000, -10000])
    def test_ht_cfo(self, cfo_hz):
        """HT MCS 0 must tolerate CFO at high SNR."""
        def impairment(iq, seed):
            iq = add_cfo(iq, cfo_hz)
            return add_awgn(iq, snr_db=25, seed=seed)

        success_rate = run_trials(lambda s: trial_ht(0, impairment, s))
        assert success_rate >= 0.8, (
            f"HT MCS 0 at CFO {cfo_hz} Hz: {success_rate:.0%} < 80%"
        )


class TestVHTCFO:
    """Verify VHT decode under CFO at PlutoSDR crystal boundaries.

    PlutoSDR TCXO: ±25 ppm. At 5.18 GHz (ch36): ±129.5 kHz.
    At 5.8 GHz (ch149): ±145 kHz.
    Coarse CFO estimator unambiguous range: ±625 kHz (STF period=16, 20 MSPS).
    These tests verify the decoder handles realistic offsets.
    """

    @pytest.mark.parametrize("cfo_hz", [25000, 50000, -25000, -50000])
    def test_vht_cfo_mcs0(self, cfo_hz):
        """VHT MCS 0 (BPSK 1/2) must decode at ±50 kHz CFO."""
        def impairment(iq, seed):
            iq = add_cfo(iq, cfo_hz)
            return add_awgn(iq, snr_db=15, seed=seed)

        rate = run_trials(lambda s: trial_vht(0, impairment, s))
        assert rate >= 0.8, f"VHT MCS 0 at CFO {cfo_hz} Hz: {rate:.0%} < 80%"

    @pytest.mark.parametrize("cfo_hz", [25000, 50000, -25000, -50000])
    def test_vht_cfo_mcs5(self, cfo_hz):
        """VHT MCS 5 (64QAM 2/3) must decode at ±50 kHz CFO."""
        def impairment(iq, seed):
            iq = add_cfo(iq, cfo_hz)
            return add_awgn(iq, snr_db=28, seed=seed)

        rate = run_trials(lambda s: trial_vht(5, impairment, s))
        assert rate >= 0.8, f"VHT MCS 5 at CFO {cfo_hz} Hz: {rate:.0%} < 80%"

    @pytest.mark.parametrize("cfo_hz", [25000, -25000])
    def test_vht_cfo_mcs8(self, cfo_hz):
        """VHT MCS 8 (256QAM 3/4) at ±25 kHz CFO (tighter budget)."""
        def impairment(iq, seed):
            iq = add_cfo(iq, cfo_hz)
            return add_awgn(iq, snr_db=35, seed=seed)

        rate = run_trials(lambda s: trial_vht(8, impairment, s))
        assert rate >= 0.8, f"VHT MCS 8 at CFO {cfo_hz} Hz: {rate:.0%} < 80%"


# ============================================================================
# Multipath tests
# ============================================================================

class TestMultipath:
    """Verify decode survives multipath channels."""

    @pytest.mark.parametrize("preset", ["mild", "moderate"])
    def test_legacy_multipath(self, preset):
        """Rate-6 must decode through mild/moderate multipath."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS[preset])
            return add_awgn(iq, snr_db=20, seed=seed)

        success_rate = run_trials(lambda s: trial_legacy(6, impairment, s))
        assert success_rate >= 0.8, (
            f"Rate 6 multipath {preset}: {success_rate:.0%} < 80%"
        )

    @pytest.mark.parametrize("preset", ["mild", "moderate"])
    def test_ht_multipath(self, preset):
        """HT MCS 0 must decode through mild/moderate multipath."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS[preset])
            return add_awgn(iq, snr_db=20, seed=seed)

        success_rate = run_trials(lambda s: trial_ht(0, impairment, s))
        assert success_rate >= 0.8, (
            f"HT MCS 0 multipath {preset}: {success_rate:.0%} < 80%"
        )

    @pytest.mark.parametrize("mcs,preset", [
        (0, "mild"), (3, "mild"), (5, "mild"), (7, "mild"), (8, "mild"),
        (0, "moderate"), (3, "moderate"), (5, "moderate"),
    ])
    def test_vht_multipath(self, mcs, preset):
        """VHT decode under multipath with per-trial tap variation.

        Randomizes tap complex gains per seed to model different reflective
        environments, rather than testing a single deterministic channel.
        """
        base_taps = MULTIPATH_PRESETS[preset]

        def impairment(iq, seed):
            rng = np.random.default_rng(seed + 1000)
            # Perturb tap gains: keep delays fixed, vary magnitude ±30%
            # and phase ±45° around the preset values
            varied_taps = []
            for delay, gain in base_taps:
                if delay == 0:
                    varied_taps.append((delay, gain))  # keep LOS tap stable
                else:
                    mag = abs(gain) * (1.0 + 0.3 * (2 * rng.random() - 1))
                    phase = np.angle(gain) + rng.uniform(-np.pi / 4, np.pi / 4)
                    varied_taps.append((delay, mag * np.exp(1j * phase)))
            return apply_multipath(iq, varied_taps)

        rate = run_trials(lambda seed: trial_vht(mcs, impairment, seed))
        assert rate >= 0.8, f"VHT MCS {mcs} multipath {preset}: {rate:.0%} < 80%"


# ============================================================================
# Combined impairments — realistic conditions
# ============================================================================

class TestCombined:
    """Verify decode under combined realistic impairments."""

    def test_legacy_realistic(self):
        """Rate 6 with AWGN + mild multipath + small CFO."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            iq = add_cfo(iq, 3000)
            return add_awgn(iq, snr_db=18, seed=seed)

        success_rate = run_trials(lambda s: trial_legacy(6, impairment, s))
        assert success_rate >= 0.8, (
            f"Rate 6 realistic: {success_rate:.0%} < 80%"
        )

    def test_ht_realistic(self):
        """HT MCS 0 with AWGN + mild multipath + small CFO."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            iq = add_cfo(iq, 3000)
            return add_awgn(iq, snr_db=18, seed=seed)

        success_rate = run_trials(lambda s: trial_ht(0, impairment, s))
        assert success_rate >= 0.8, (
            f"HT MCS 0 realistic: {success_rate:.0%} < 80%"
        )

    def test_legacy_high_rate_generous_snr(self):
        """Rate 54 at generous SNR with mild multipath must still decode."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            return add_awgn(iq, snr_db=30, seed=seed)

        success_rate = run_trials(lambda s: trial_legacy(54, impairment, s))
        assert success_rate >= 0.8, (
            f"Rate 54 mild multipath+30dB: {success_rate:.0%} < 80%"
        )

    def test_ht_high_mcs_generous_snr(self):
        """HT MCS 7 at generous SNR with mild multipath must still decode."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            return add_awgn(iq, snr_db=30, seed=seed)

        success_rate = run_trials(lambda s: trial_ht(7, impairment, s))
        assert success_rate >= 0.8, (
            f"HT MCS 7 mild multipath+30dB: {success_rate:.0%} < 80%"
        )

    def test_vht_realistic(self):
        """VHT MCS 5 under moderate CFO + mild multipath + AWGN."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            iq = add_cfo(iq, 3000.0)
            iq = add_awgn(iq, 25.0, seed)
            return iq
        rate = run_trials(lambda seed: trial_vht(5, impairment, seed))
        assert rate >= 0.7, f"VHT realistic: {rate:.0%} < 70%"

    def test_vht_high_mcs_generous_snr(self):
        """VHT MCS 8 under mild multipath + high SNR (tests refinement)."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            iq = add_awgn(iq, 35.0, seed)
            return iq
        rate = run_trials(lambda seed: trial_vht(8, impairment, seed))
        assert rate >= 0.7, f"VHT MCS 8 high-SNR multipath: {rate:.0%} < 70%"


# ============================================================================
# Short GI trial helpers
# ============================================================================

def trial_ht_sgi(mcs: int, impairment_fn, seed: int) -> bool:
    """Single HT short-GI trial. Returns True if FCS OK and payload matches."""
    iq, meta = generate_ht_frame(mcs, PAYLOAD, short_gi=True)
    iq = impairment_fn(iq, seed)
    result = decode_with_detection(iq)
    if result is None or not result.get("fcs_ok"):
        return False
    return result["psdu"][:len(PAYLOAD)] == PAYLOAD


def trial_vht_sgi(mcs: int, impairment_fn, seed: int) -> bool:
    """Single VHT short-GI trial. Returns True if FCS OK and payload matches."""
    iq, meta = generate_vht_frame(mcs, PAYLOAD, short_gi=True)
    iq = impairment_fn(iq, seed)
    result = decode_with_detection(iq)
    if result is None or not result.get("fcs_ok"):
        return False
    return result["psdu"][:len(PAYLOAD)] == PAYLOAD


# ============================================================================
# Short GI AWGN tests — slightly higher SNR thresholds than long GI
# ============================================================================

class TestShortGIAWGN:
    """Short GI AWGN performance (same modulation, needs ~1-2 dB more SNR)."""

    @pytest.mark.parametrize("mcs,min_snr", [
        (0, 10),    # BPSK 1/2 — same as LGI
        (1, 12),    # QPSK 1/2
        (3, 16),    # 16QAM 1/2
        (5, 24),    # 64QAM 2/3
        (7, 28),    # 64QAM 5/6
    ])
    def test_ht_sgi_awgn(self, mcs, min_snr):
        """HT short-GI must decode at threshold SNR with >=80% success."""
        def impairment(iq, seed):
            return add_awgn(iq, snr_db=min_snr, seed=seed)
        success_rate = run_trials(lambda s: trial_ht_sgi(mcs, impairment, s))
        assert success_rate >= 0.8, (
            f"HT SGI MCS {mcs} at {min_snr} dB: {success_rate:.0%} < 80%"
        )

    @pytest.mark.parametrize("mcs,snr_db", [
        (0, 12), (3, 17), (5, 23), (7, 27), (8, 30),
    ])
    def test_vht_sgi_awgn(self, mcs, snr_db):
        """VHT short-GI decode at conservative SNR."""
        rate = run_trials(
            lambda seed: trial_vht_sgi(mcs, lambda iq, s: add_awgn(iq, snr_db, s), seed))
        assert rate >= 0.8, f"VHT SGI MCS {mcs} at {snr_db} dB: {rate:.0%} < 80%"


# ============================================================================
# Short GI CFO tests
# ============================================================================

class TestShortGICFO:
    """Short GI CFO tolerance (should be similar to long GI)."""

    @pytest.mark.parametrize("cfo_hz", [5000, 10000, -5000, -10000])
    def test_ht_sgi_cfo(self, cfo_hz):
        """HT SGI MCS 0 must tolerate CFO at high SNR."""
        def impairment(iq, seed):
            iq = add_cfo(iq, cfo_hz)
            return add_awgn(iq, snr_db=25, seed=seed)
        success_rate = run_trials(lambda s: trial_ht_sgi(0, impairment, s))
        assert success_rate >= 0.8, (
            f"HT SGI MCS 0 at CFO {cfo_hz} Hz: {success_rate:.0%} < 80%"
        )


# ============================================================================
# Short GI multipath tests — only mild (within 8-sample CP)
# ============================================================================

class TestShortGIMultipath:
    """Short GI multipath: only mild preset is within CP boundary."""

    @pytest.mark.parametrize("mcs,snr_db", [
        (0, 22),    # BPSK 1/2 — generous margin
        (3, 22),    # 16QAM 1/2
        (7, 30),    # 64QAM 5/6 — needs more headroom with multipath
    ])
    def test_ht_sgi_mild_multipath(self, mcs, snr_db):
        """HT SGI with mild multipath (max delay 3 < 8 samples CP)."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            return add_awgn(iq, snr_db=snr_db, seed=seed)
        success_rate = run_trials(lambda s: trial_ht_sgi(mcs, impairment, s))
        assert success_rate >= 0.8, (
            f"HT SGI MCS {mcs} mild multipath: {success_rate:.0%} < 80%"
        )

    @pytest.mark.parametrize("mcs", [0, 3, 5, 8])
    def test_vht_sgi_mild_multipath(self, mcs):
        """VHT SGI with mild multipath and per-trial tap variation."""
        base_taps = MULTIPATH_PRESETS["mild"]

        def impairment(iq, seed):
            rng = np.random.default_rng(seed + 2000)
            varied_taps = []
            for delay, gain in base_taps:
                if delay == 0:
                    varied_taps.append((delay, gain))
                else:
                    mag = abs(gain) * (1.0 + 0.3 * (2 * rng.random() - 1))
                    phase = np.angle(gain) + rng.uniform(-np.pi / 4, np.pi / 4)
                    varied_taps.append((delay, mag * np.exp(1j * phase)))
            return apply_multipath(iq, varied_taps)

        rate = run_trials(lambda seed: trial_vht_sgi(mcs, impairment, seed))
        assert rate >= 0.8, f"VHT SGI MCS {mcs} mild multipath: {rate:.0%} < 80%"


# ============================================================================
# Short GI combined impairments
# ============================================================================

class TestShortGICombined:
    """Short GI under combined realistic impairments."""

    def test_ht_sgi_realistic(self):
        """HT SGI MCS 0 with AWGN + mild multipath + small CFO."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            iq = add_cfo(iq, 3000)
            return add_awgn(iq, snr_db=20, seed=seed)
        success_rate = run_trials(lambda s: trial_ht_sgi(0, impairment, s))
        assert success_rate >= 0.8, (
            f"HT SGI MCS 0 realistic: {success_rate:.0%} < 80%"
        )

    def test_vht_sgi_realistic(self):
        """VHT SGI MCS 3 under mild multipath + AWGN + CFO."""
        def impairment(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            iq = add_cfo(iq, 3000.0)
            iq = add_awgn(iq, 22.0, seed)
            return iq
        rate = run_trials(lambda seed: trial_vht_sgi(3, impairment, seed))
        assert rate >= 0.7, f"VHT SGI realistic: {rate:.0%} < 70%"


# ============================================================================
# VHT deterministic pilot tracking
# ============================================================================

class TestQuantization:
    """Verify ADC quantization impairment model."""

    @pytest.mark.parametrize("mcs,bits", [
        (0, 8), (3, 10), (7, 10), (7, 12),
    ])
    def test_adc_quantization_ht(self, mcs, bits):
        """HT decode succeeds with ADC quantization."""
        from py80211.gen_ofdm_frame import generate_ht_frame
        from py80211.decode_frame import decode_frame
        from py80211.impairments import add_quantization
        psdu = bytes(range(100))
        iq, _ = generate_ht_frame(mcs, psdu)
        iq = add_quantization(iq, bits=bits)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]


class TestVHTDeterministicPilots:
    """Verify deterministic VHT pilot tracking works at low SNR."""

    def test_vht_pilot_deterministic_low_snr(self):
        """VHT decode with deterministic pilots works at lower SNR than blind detection."""
        # MCS 3 (16-QAM 1/2) which needs ~14 dB with margin
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(3, psdu)
        iq = add_awgn(iq, snr_db=14.0, seed=42)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]


# ============================================================================
# SFO (Sampling Frequency Offset) tests
# ============================================================================

# Padding simulates a realistic receiver that always has a continuous sample
# stream — negative SFO shortens the resampled signal, so without padding the
# decoder would truncate the last symbol.
_SFO_PAD = 200


class TestSFO:
    """Verify decode survives ±20 ppm SFO (typical crystal tolerance).

    The existing pilot slope EWMA tracking implicitly compensates for SFO,
    which manifests as a linearly-increasing phase slope across symbols.
    These tests verify that compensation is effective up to ±20 ppm for all
    frame types and modulation orders.
    """

    @pytest.mark.parametrize("rate", [6, 24, 54])
    def test_sfo_20ppm_legacy(self, rate):
        """Legacy decode survives 20 ppm SFO."""
        psdu = bytes(range(100))
        iq, _ = generate_frame(rate, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=20.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("rate", [6, 24, 54])
    def test_sfo_neg20ppm_legacy(self, rate):
        """Legacy decode survives -20 ppm SFO."""
        psdu = bytes(range(100))
        iq, _ = generate_frame(rate, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=-20.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("mcs", [0, 3, 7])
    def test_sfo_20ppm_ht(self, mcs):
        """HT decode survives 20 ppm SFO."""
        psdu = bytes(range(100))
        iq, _ = generate_ht_frame(mcs, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=20.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("mcs", [0, 3, 7])
    def test_sfo_neg20ppm_ht(self, mcs):
        """HT decode survives -20 ppm SFO."""
        psdu = bytes(range(100))
        iq, _ = generate_ht_frame(mcs, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=-20.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("mcs", [0, 4, 8])
    def test_sfo_20ppm_vht(self, mcs):
        """VHT decode survives 20 ppm SFO."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(mcs, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=20.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("mcs", [0, 4, 8])
    def test_sfo_neg20ppm_vht(self, mcs):
        """VHT decode survives -20 ppm SFO."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(mcs, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=-20.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("rate", [6, 54])
    def test_sfo_40ppm_legacy(self, rate):
        """Legacy decode survives 40 ppm SFO (stress test)."""
        psdu = bytes(range(100))
        iq, _ = generate_frame(rate, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=40.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("mcs", [0, 7])
    def test_sfo_40ppm_ht(self, mcs):
        """HT decode survives 40 ppm SFO (stress test)."""
        psdu = bytes(range(100))
        iq, _ = generate_ht_frame(mcs, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=40.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("mcs", [0, 8])
    def test_sfo_40ppm_vht(self, mcs):
        """VHT decode survives 40 ppm SFO (stress test)."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(mcs, psdu)
        iq = np.concatenate([iq, np.zeros(_SFO_PAD, dtype=complex)])
        iq = add_sfo(iq, ppm=40.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]


# ============================================================================
# IQ imbalance tests — verify time-domain correction handles RF front-end
# gain and phase mismatch between I and Q channels
# ============================================================================

class TestIQImbalance:
    """Verify decode survives IQ imbalance (2 dB gain + 10° phase target)."""

    @pytest.mark.parametrize("mcs", [0, 3, 7])
    def test_iq_imbalance_ht(self, mcs):
        """HT decode survives 2 dB gain + 10° phase IQ imbalance."""
        psdu = bytes(range(100))
        iq, _ = generate_ht_frame(mcs, psdu)
        iq = add_iq_imbalance(iq, gain_db=2.0, phase_deg=10.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("mcs", [0, 4, 8])
    def test_iq_imbalance_vht(self, mcs):
        """VHT decode survives 2 dB gain + 10° phase IQ imbalance."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(mcs, psdu)
        iq = add_iq_imbalance(iq, gain_db=2.0, phase_deg=10.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    def test_iq_imbalance_moderate(self):
        """Moderate IQ imbalance (1 dB, 5°) doesn't affect high MCS."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(7, psdu)
        iq = add_iq_imbalance(iq, gain_db=1.0, phase_deg=5.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    def test_iq_imbalance_severe(self):
        """Severe IQ imbalance (3 dB, 15°) still decodes low MCS."""
        psdu = bytes(range(100))
        iq, _ = generate_ht_frame(0, psdu)
        iq = add_iq_imbalance(iq, gain_db=3.0, phase_deg=15.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("rate", [6, 24, 54])
    def test_iq_imbalance_legacy(self, rate):
        """Legacy decode survives 2 dB gain + 10° phase IQ imbalance."""
        psdu = bytes(range(100))
        iq, _ = generate_frame(rate, psdu)
        iq = add_iq_imbalance(iq, gain_db=2.0, phase_deg=10.0)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]


# ============================================================================
# Phase noise tests — verify pilot tracking handles per-symbol CPE
# ============================================================================

class TestPhaseNoise:
    """Verify decode survives phase noise (strength=0.02 target)."""

    @pytest.mark.parametrize("mcs", [0, 3, 7])
    def test_phase_noise_ht(self, mcs):
        """HT decode survives moderate phase noise (strength=0.02)."""
        psdu = bytes(range(100))
        iq, _ = generate_ht_frame(mcs, psdu)
        iq = add_phase_noise(iq, strength=0.02, seed=42)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    @pytest.mark.parametrize("mcs", [0, 4, 7])
    def test_phase_noise_vht(self, mcs):
        """VHT decode survives moderate phase noise (strength=0.02)."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(mcs, psdu)
        iq = add_phase_noise(iq, strength=0.02, seed=42)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]

    def test_phase_noise_severe_low_mcs(self):
        """Low MCS survives severe phase noise (strength=0.05)."""
        psdu = bytes(range(100))
        iq, _ = generate_ht_frame(0, psdu)  # BPSK 1/2 — very robust
        iq = add_phase_noise(iq, strength=0.05, seed=42)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]


# ============================================================================
# DC offset tests — verify decode survives direct-conversion LO leakage
# ============================================================================

class TestDCOffset:
    """Verify decode survives DC offset from PlutoSDR direct-conversion.

    AD9361 DC leakage is typically -40 to -30 dBc after calibration,
    which corresponds to ~1-3% of signal RMS. Subcarrier 0 is nulled
    in 802.11, but DC offset can bleed into adjacent bins through
    timing misalignment.
    """

    @pytest.mark.parametrize("mcs", [0, 5, 8])
    def test_vht_dc_offset(self, mcs):
        """VHT decode survives 3% DC offset on both channels."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(mcs, psdu)
        iq = add_dc_offset(iq, dc_i=0.03, dc_q=0.02)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"VHT MCS {mcs} failed with 3% DC offset"

    @pytest.mark.parametrize("rate", [6, 36, 54])
    def test_legacy_dc_offset(self, rate):
        """Legacy decode survives 5% DC offset."""
        psdu = bytes(range(100))
        iq, _ = generate_frame(rate, psdu)
        iq = add_dc_offset(iq, dc_i=0.05, dc_q=0.03)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"Rate {rate} failed with 5% DC offset"

    def test_severe_dc_offset_low_mcs(self):
        """BPSK 1/2 survives 10% DC offset (stress test)."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(0, psdu)
        iq = add_dc_offset(iq, dc_i=0.10, dc_q=0.10)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]


# ============================================================================
# AGC settling tests — verify STF detection survives gain ramp
# ============================================================================

class TestAGCSettling:
    """Verify decode survives AGC settling transient during STF.

    PlutoSDR AGC settles in ~10-15 us (200-300 samples at 20 MSPS).
    The first STF repetitions arrive at reduced gain. The STF correlator
    must still lock and the decoder must produce correct output.
    """

    @pytest.mark.parametrize("mcs", [0, 5])
    def test_vht_agc_ramp(self, mcs):
        """VHT decode survives AGC ramp (-20 dB initial, 128 sample settle)."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(mcs, psdu)
        iq = add_agc_ramp(iq, settle_samples=128, initial_gain_db=-20.0)
        result = decode_with_detection(iq)
        assert result is not None and result["fcs_ok"], (
            f"VHT MCS {mcs} failed with AGC ramp (-20 dB, 128 samples)"
        )

    @pytest.mark.parametrize("rate", [6, 54])
    def test_legacy_agc_ramp(self, rate):
        """Legacy decode survives AGC ramp (-15 dB initial, 100 sample settle)."""
        psdu = bytes(range(100))
        iq, _ = generate_frame(rate, psdu)
        iq = add_agc_ramp(iq, settle_samples=100, initial_gain_db=-15.0)
        result = decode_with_detection(iq)
        assert result is not None and result["fcs_ok"], (
            f"Rate {rate} failed with AGC ramp (-15 dB, 100 samples)"
        )

    def test_vht_severe_agc(self):
        """VHT MCS 0 survives severe AGC transient (-30 dB, 200 samples)."""
        psdu = bytes(range(100))
        iq, _ = generate_vht_frame(0, psdu)
        iq = add_agc_ramp(iq, settle_samples=200, initial_gain_db=-30.0)
        result = decode_with_detection(iq)
        assert result is not None and result["fcs_ok"]

    def test_agc_ramp_combined(self):
        """VHT MCS 3 with AGC ramp + mild CFO + AWGN (PlutoSDR-realistic)."""
        def impairment(iq, seed):
            iq = add_agc_ramp(iq, settle_samples=150, initial_gain_db=-15.0)
            iq = add_cfo(iq, 5000.0)
            return add_awgn(iq, 20.0, seed)

        rate = run_trials(lambda s: trial_vht(3, impairment, s))
        assert rate >= 0.8, f"VHT MCS 3 AGC+CFO+AWGN: {rate:.0%} < 80%"


# ============================================================================
# LDPC impairment tests
# ============================================================================

def trial_vht_ldpc(mcs: int, impairment_fn, seed: int, psdu_len: int = 80) -> bool:
    """Single VHT LDPC trial."""
    psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
    iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
    iq = impairment_fn(iq, seed)
    result = decode_with_detection(iq)
    if result is None or not result.get("fcs_ok"):
        return False
    return result["psdu"][:psdu_len] == psdu


class TestLDPCImpairments:
    """LDPC decode under realistic impairments (not just AWGN)."""

    @pytest.mark.parametrize("mcs,snr", [
        (0, 8), (3, 14), (5, 20), (7, 24), (8, 28),
    ])
    def test_ldpc_awgn(self, mcs, snr):
        """LDPC AWGN baseline."""
        fn = lambda iq, seed: add_awgn(iq, snr_db=snr, seed=seed)
        rate = run_trials(lambda seed: trial_vht_ldpc(mcs, fn, seed))
        assert rate >= 0.8, f"VHT LDPC MCS {mcs} at {snr} dB: {rate:.0%}"

    @pytest.mark.parametrize("mcs,cfo_khz", [
        (0, 25), (3, 25), (5, 25), (8, 15),
    ])
    def test_ldpc_cfo(self, mcs, cfo_khz):
        """LDPC with CFO."""
        snr_map = {0: 12, 3: 18, 5: 24, 8: 32}
        snr = snr_map[mcs]
        def fn(iq, seed):
            iq = add_cfo(iq, cfo_hz=cfo_khz * 1000)
            return add_awgn(iq, snr_db=snr, seed=seed)
        rate = run_trials(lambda seed: trial_vht_ldpc(mcs, fn, seed))
        assert rate >= 0.8, f"VHT LDPC MCS {mcs} + {cfo_khz}kHz CFO: {rate:.0%}"

    @pytest.mark.parametrize("mcs", [0, 3, 5, 8])
    def test_ldpc_multipath_mild(self, mcs):
        """LDPC with mild multipath."""
        snr_map = {0: 14, 3: 20, 5: 26, 8: 34}
        snr = snr_map[mcs]
        def fn(iq, seed):
            iq = apply_multipath(iq, MULTIPATH_PRESETS["mild"])
            return add_awgn(iq, snr_db=snr, seed=seed)
        rate = run_trials(lambda seed: trial_vht_ldpc(mcs, fn, seed))
        assert rate >= 0.7, f"VHT LDPC MCS {mcs} + mild multipath: {rate:.0%}"
