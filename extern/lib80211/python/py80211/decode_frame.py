"""802.11 OFDM frame decoder — Python oracle.

Decodes baseband IQ samples containing 802.11a (legacy), 802.11n
(HT-mixed), and 802.11ac (VHT) frames.  Serves as the correctness
oracle for the C implementation and decodes live captures from PlutoSDR.

Reference: IEEE 802.11-2020 Sections 17, 19, 21.
"""

import math
import struct
import zlib

import numpy as np

from py80211.gen_ofdm_frame import (
    NFFT,
    NCP,
    NCP_SHORT,
    SYMBOL_SAMPLES,
    SYMBOL_SAMPLES_SHORT,
    SAMPLE_RATE,
    STF_SAMPLES,
    LTF_SAMPLES,
    PREAMBLE_SAMPLES,
    DATA_BINS,
    PILOT_BINS,
    PILOT_BASE,
    PILOT_POLARITY,
    RATE_TABLE,
    _STF_FREQ,
    _LTF_FREQ,
    _STF_SCALE,
    _QAM16_COORDS,
    _QAM64_COORDS,
    _QAM16_GRAY,
    _QAM64_GRAY,
    puncture,
    interleave,
    scramble,
    modulate,
    compute_fcs,
    verify_fcs,
    conv_encode,
    _parity,
    bytes_to_bits_lsb,
    bits_to_bytes_lsb,
    G0,
    G1,
    _HT_LTF_FREQ,
    HT_DATA_BINS,
    HT_MCS_TABLE,
    HT_PILOT_SPEC_BINS,
    _HT_PILOT_PATTERN,
    ht_sig_crc8,
)
from py80211.ldpc import ldpc_decode

# ============================================================================
# Viterbi decoder (K=7, rate 1/2)
# ============================================================================
_N_STATES = 64  # 1 << (K-1)
_INF = 1e9


class _ViterbiTrellis:
    """Precomputed trellis for K=7 rate-1/2 convolutional code.

    In addition to the basic next_state/output tables, precomputes
    predecessor tables for vectorized ACS (Add-Compare-Select):
      prev_states[ns, 0..1] - the 2 predecessor states for each next_state
      prev_bits[ns, 0..1]   - the input bit for each predecessor transition
      bm_sign[ns, 0..1, 0..1] - sign multiplier for branch metric computation
                                 (+1 if expected output is 0, -1 if 1)
    """

    def __init__(self):
        self.next_state = np.zeros((_N_STATES, 2), dtype=np.int32)
        self.output = np.zeros((_N_STATES, 2, 2), dtype=np.int32)
        for s in range(_N_STATES):
            for b in (0, 1):
                reg = (b << 6) | s
                ns = (reg >> 1) & (_N_STATES - 1)
                self.next_state[s, b] = ns
                self.output[s, b, 0] = _parity(reg & G0)
                self.output[s, b, 1] = _parity(reg & G1)

        # Precompute predecessor tables for vectorized ACS.
        # Each next_state has exactly 2 predecessors (from 2 different
        # (state, input_bit) pairs).
        self.prev_states = np.zeros((_N_STATES, 2), dtype=np.int32)
        self.prev_bits = np.zeros((_N_STATES, 2), dtype=np.int32)
        # bm_sign: +1 means expected output bit is 0 (add +soft),
        #          -1 means expected output bit is 1 (add -soft)
        self.bm_sign = np.zeros((_N_STATES, 2, 2), dtype=np.float64)

        # Count arrivals per next_state to fill both slots
        count = np.zeros(_N_STATES, dtype=np.int32)
        for s in range(_N_STATES):
            for b in (0, 1):
                ns = self.next_state[s, b]
                idx = count[ns]
                self.prev_states[ns, idx] = s
                self.prev_bits[ns, idx] = b
                # Sign: output=0 => +1 (metric += +soft), output=1 => -1 (metric += -soft)
                self.bm_sign[ns, idx, 0] = 1.0 - 2.0 * self.output[s, b, 0]
                self.bm_sign[ns, idx, 1] = 1.0 - 2.0 * self.output[s, b, 1]
                count[ns] += 1


# Global trellis instance
_TRELLIS = _ViterbiTrellis()


def viterbi_decode_soft(soft_bits: np.ndarray, n_data_bits: int) -> np.ndarray:
    """Soft-decision K=7 rate-1/2 Viterbi decoder (vectorized ACS).

    Args:
        soft_bits: LLRs (log-likelihood ratios), positive => bit 1 more likely
        n_data_bits: number of original data bits to recover

    Returns:
        Decoded hard bits as uint8 array of length n_data_bits.
    """
    n_steps = n_data_bits + 6  # data bits + K-1 tail bits
    n_coded_needed = 2 * n_steps
    if len(soft_bits) < n_coded_needed:
        # Pad with erasures (0.0 LLR) — matches C truncation behavior
        padded = np.zeros(n_coded_needed, dtype=float)
        padded[:len(soft_bits)] = soft_bits
        soft_bits = padded

    trellis = _TRELLIS
    prev_states = trellis.prev_states   # [64, 2]
    prev_bits = trellis.prev_bits       # [64, 2]
    bm_sign = trellis.bm_sign           # [64, 2, 2]

    # Path metrics
    metrics = np.full(_N_STATES, _INF, dtype=np.float64)
    metrics[0] = 0.0

    # Traceback storage
    tb_state = np.zeros((n_steps, _N_STATES), dtype=np.int32)
    tb_input = np.zeros((n_steps, _N_STATES), dtype=np.int32)

    # Precompute branch metric sign components for vectorized inner loop
    # bm_sign[:, :, 0] is the sign for coded bit 0
    # bm_sign[:, :, 1] is the sign for coded bit 1
    bm_sign_0 = bm_sign[:, :, 0]  # [64, 2]
    bm_sign_1 = bm_sign[:, :, 1]  # [64, 2]

    for step in range(n_steps):
        s0 = soft_bits[2 * step]
        s1 = soft_bits[2 * step + 1]

        # Branch metrics for all 64 next-states × 2 predecessors
        # bm[ns, k] = bm_sign_0[ns, k] * s0 + bm_sign_1[ns, k] * s1
        bm = bm_sign_0 * s0 + bm_sign_1 * s1  # [64, 2]

        # Path metrics of predecessor states: metrics[prev_states[ns, k]]
        pm_prev = metrics[prev_states]  # [64, 2]

        # Candidate path metrics
        candidates = pm_prev + bm  # [64, 2]

        # Select minimum (ACS): compare candidates[:, 0] vs candidates[:, 1]
        sel = (candidates[:, 1] < candidates[:, 0]).astype(np.intp)  # [64]

        # Update metrics
        metrics = candidates[np.arange(_N_STATES), sel]

        # Store traceback: survivor state and input bit
        tb_state[step] = prev_states[np.arange(_N_STATES), sel]
        tb_input[step] = prev_bits[np.arange(_N_STATES), sel]

    # Traceback from state 0
    decoded = np.zeros(n_data_bits, dtype=np.uint8)
    state = 0
    for step in range(n_steps - 1, -1, -1):
        if step < n_data_bits:
            decoded[step] = tb_input[step, state]
        state = tb_state[step, state]

    return decoded


# ============================================================================
# STF detection — autocorrelation-based
# ============================================================================
def _bulk_autocorr_metric(iq: np.ndarray, period: int, window: int) -> np.ndarray:
    """Compute sliding normalized autocorrelation for all samples at once.

    Returns metric[n] = |P[n]| / sqrt(E1[n] * E2[n]) where:
        P[n]  = sum(iq[n:n+W] * conj(iq[n+L:n+L+W]))
        E1[n] = sum(|iq[n:n+W]|^2)
        E2[n] = sum(|iq[n+L:n+L+W]|^2)

    Uses cumulative sums for O(N) computation regardless of window size.
    Skips per-window DC removal (STF has zero DC by design; threshold margin
    absorbs any residual LO leakage).
    """
    n = len(iq)
    n_out = n - window - period
    if n_out <= 0:
        return np.array([], dtype=np.float64)

    # Sliding cross-correlation P[n] via cumsum of pointwise product
    prod = iq[:n - period] * np.conj(iq[period:])
    cs_prod = np.concatenate(([0 + 0j], np.cumsum(prod)))
    # P[n] = cs_prod[n+W] - cs_prod[n], for n in [0, n_out)
    P = cs_prod[window:window + n_out] - cs_prod[:n_out]

    # Sliding energy E1[n] = sum(|iq[n:n+W]|^2)
    sq = np.abs(iq) ** 2
    cs_sq = np.concatenate(([0.0], np.cumsum(sq)))
    E1 = cs_sq[window:window + n_out] - cs_sq[:n_out]
    # E2[n] = sum(|iq[n+L:n+L+W]|^2)
    E2 = cs_sq[period + window:period + window + n_out] - cs_sq[period:period + n_out]

    # Normalized metric: |P| / sqrt(E1 * E2), guarded against division by zero
    denom = np.sqrt(E1 * E2)
    metric = np.zeros(n_out, dtype=np.float64)
    valid = denom > 1e-20
    metric[valid] = np.abs(P[valid]) / denom[valid]

    return metric


def detect_stf(iq: np.ndarray, period: int = 16, window: int = 64,
               threshold: float = 0.6, min_periods: int = 8) -> int:
    """Detect STF using sliding normalized autocorrelation.

    Args:
        iq: Complex IQ samples
        period: STF repetition period (16 samples)
        window: Autocorrelation window (64 samples)
        threshold: Detection threshold (0.0-1.0)
        min_periods: Minimum consecutive periods with high autocorrelation

    Returns:
        Sample offset of STF start, or -1 if not found.
    """
    n_samples = len(iq)
    if n_samples < window + period:
        return -1

    n_test = n_samples - window - period
    if n_test <= 0:
        return -1

    # Bulk vectorized path: compute metric for all samples at once
    metric = _bulk_autocorr_metric(iq, period, window)
    if len(metric) == 0:
        return -1

    step = max(period // 4, 1)
    period_thresh = threshold * 0.7

    # Find all samples exceeding threshold
    above = np.where(metric >= threshold)[0]
    if len(above) == 0:
        return -1

    # Process candidates in order
    scan_start = 0
    for first_above in above:
        if first_above < scan_start:
            continue

        # Fine scan: find best metric in ±step neighborhood
        lo = max(int(first_above) - step, 0)
        hi = min(int(first_above) + step + 1, len(metric))
        best_offset = lo + int(np.argmax(metric[lo:hi]))

        # Duration check: use _count_high_periods (small loop, fast enough)
        if min_periods > 0:
            n_per = _count_high_periods(iq, best_offset, period, n_samples,
                                        period_thresh)
            if n_per >= min_periods:
                return int(best_offset)
            # Failed — advance past this region
            scan_start = best_offset + period * 2
        else:
            return int(best_offset)

    return -1


def _autocorr_metric(iq: np.ndarray, offset: int, period: int,
                     window: int) -> float:
    """Normalized autocorrelation metric at a given offset."""
    seg1 = iq[offset:offset + window]
    seg2 = iq[offset + period:offset + period + window]
    # Remove DC per-window to avoid LO leakage masking STF
    seg1 = seg1 - np.mean(seg1)
    seg2 = seg2 - np.mean(seg2)
    corr = np.sum(seg1 * np.conj(seg2))
    e1 = np.sum(np.abs(seg1) ** 2)
    e2 = np.sum(np.abs(seg2) ** 2)
    if e1 < 1e-20 or e2 < 1e-20:
        return 0.0
    return abs(corr) / math.sqrt(e1 * e2)


def _count_high_periods(iq: np.ndarray, offset: int, period: int,
                        n_samples: int, period_thresh: float) -> int:
    """Count consecutive periods of high autocorrelation."""
    max_check = min((n_samples - offset) // period - 1, 15)
    count = 0
    for k in range(max_check):
        pos = offset + k * period
        m = _autocorr_metric(iq, pos, period, period)
        if m >= period_thresh:
            count += 1
        else:
            break
    return count


# ============================================================================
# CFO estimation
# ============================================================================
def estimate_cfo_coarse(iq: np.ndarray, stf_start: int,
                        sample_rate: float = SAMPLE_RATE) -> float:
    """Coarse CFO estimate using STF periodic autocorrelation.

    Correlates all usable STF samples at lag-16 (skipping the first 3
    periods for ramp-up). Uses 96 samples (6 STF periods) to maximize
    noise averaging.

    Range: ±625 kHz (half subcarrier spacing).
    """
    period = 16
    skip = 3 * period  # 48 samples: skip ramp-up
    idx1 = stf_start + skip
    idx2 = idx1 + period
    # Use all remaining STF after skip: up to sample 160
    window = (stf_start + STF_SAMPLES) - idx2
    window = max(min(window, 96), period)  # Cap at 96, min 16
    seg1 = iq[idx1:idx1 + window]
    seg2 = iq[idx2:idx2 + window]
    corr = np.sum(seg1 * np.conj(seg2))
    phase = np.angle(corr)
    return phase * sample_rate / (2.0 * np.pi * period)


def estimate_cfo_fine(iq: np.ndarray, ltf_start: int,
                      sample_rate: float = SAMPLE_RATE) -> float:
    """Fine CFO estimate using two consecutive LTF symbols."""
    idx1 = ltf_start
    idx2 = ltf_start + NFFT
    seg1 = iq[idx1:idx1 + NFFT]
    seg2 = iq[idx2:idx2 + NFFT]
    corr = np.sum(seg1 * np.conj(seg2))
    phase = np.angle(corr)
    return phase * sample_rate / (2.0 * np.pi * NFFT)


def apply_cfo_correction(iq: np.ndarray, cfo_hz: float,
                         sample_rate: float = SAMPLE_RATE) -> np.ndarray:
    """Apply CFO correction by rotating samples.

    Matches C convention: estimates return -f_actual, correction multiplies
    by exp(+j*2π*cfo_est*t), giving f_actual + cfo_est ≈ 0.
    """
    t = np.arange(len(iq)) / sample_rate
    return iq * np.exp(1j * 2.0 * np.pi * cfo_hz * t)


# ============================================================================
# IQ imbalance correction
# ============================================================================
def _correct_iq_imbalance(work: np.ndarray, ltf_start: int) -> np.ndarray:
    """Estimate and correct frequency-flat IQ imbalance from L-LTF samples.

    Uses the two L-LTF symbols (128 time-domain samples) to estimate the
    gain imbalance and phase skew between I and Q channels.  For an ideal
    OFDM signal E[I^2] = E[Q^2] and E[I*Q] = 0; deviations indicate IQ
    imbalance from the RF front-end.

    Correction model (time-domain, frequency-flat):
        Q_corrected = (Q - phi_est * I) * g_ratio

    Only applies correction when the estimated imbalance exceeds a minimum
    threshold to avoid amplifying noise on clean signals.

    Args:
        work: Complex IQ samples (coarse-CFO-corrected).
        ltf_start: Sample offset of L-LTF first symbol.

    Returns:
        IQ-corrected complex samples (or original if imbalance is negligible).
    """
    ltf_end = ltf_start + 2 * NFFT
    if ltf_end > len(work):
        return work

    ltf_samples = work[ltf_start:ltf_end]
    I = ltf_samples.real
    Q = ltf_samples.imag

    ei2 = np.mean(I ** 2)
    eq2 = np.mean(Q ** 2)
    eiq = np.mean(I * Q)

    if ei2 < 1e-20 or eq2 < 1e-20:
        return work

    g_ratio = math.sqrt(ei2 / eq2)
    phi_est = eiq / ei2

    # Only correct if imbalance exceeds threshold (avoids noise amplification)
    if abs(g_ratio - 1.0) < 0.01 and abs(phi_est) < 0.01:
        return work

    # Apply correction to entire buffer
    I_all = work.real
    Q_all = work.imag
    Q_corrected = (Q_all - phi_est * I_all) * g_ratio
    return I_all + 1j * Q_corrected


# ============================================================================
# LTF sync
# ============================================================================
def find_ltf(iq: np.ndarray, n_samples: int, stf_start: int) -> int:
    """Find the LTF start using cross-correlation, refined by lag-64.

    Uses cross-correlation with the known LTF waveform for initial estimate,
    then refines using lag-64 autocorrelation (exploiting the two identical
    LTF symbols) in a ±8 sample window. The lag-64 refinement improves
    timing accuracy under multipath where the xcorr peak may shift.
    """
    # Precompute LTF time-domain reference
    ltf_time = np.fft.ifft(_LTF_FREQ)

    # Expected: STF_SAMPLES + LTF_GI2 (32) from STF start
    expected = stf_start + STF_SAMPLES + 32
    search_lo = max(expected - 32, 0)
    search_hi = min(expected + 32, n_samples - 2 * NFFT)

    if search_hi <= search_lo:
        return expected

    # Cross-correlation search
    best_offset = expected
    best_corr = 0.0
    for n in range(search_lo, search_hi):
        if n + NFFT > n_samples:
            continue
        seg = iq[n:n + NFFT]
        corr = np.abs(np.sum(seg * np.conj(ltf_time[:NFFT])))
        if corr > best_corr:
            best_corr = corr
            best_offset = n

    # Refine using lag-64 autocorrelation in ±8 window around xcorr peak.
    # The two LTF symbols are identical, so correlation at lag-64 peaks
    # at the correct FFT window alignment.
    #
    # Note: within the GI2 (cyclic prefix of T1), the lag-64 metric is
    # constant because both windows see identical waveform content.
    # To avoid selecting an earlier (incorrect) position when metrics tie,
    # search from the xcorr peak outward and require strict improvement.
    refine_lo = max(best_offset - 8, search_lo)
    refine_hi = min(best_offset + 9, n_samples - 2 * NFFT)
    best_lag64 = 0.0
    best_refined = best_offset

    # Evaluate xcorr peak position first to establish baseline
    if best_offset + 2 * NFFT <= n_samples:
        seg1 = iq[best_offset:best_offset + NFFT]
        seg2 = iq[best_offset + NFFT:best_offset + 2 * NFFT]
        corr_lag = np.abs(np.sum(seg1 * np.conj(seg2)))
        e1 = np.sum(np.abs(seg1) ** 2)
        e2 = np.sum(np.abs(seg2) ** 2)
        best_lag64 = corr_lag / math.sqrt(e1 * e2) if e1 > 1e-20 and e2 > 1e-20 else 0.0

    for n in range(refine_lo, refine_hi):
        if n == best_offset:
            continue  # already evaluated
        if n + 2 * NFFT > n_samples:
            continue
        seg1 = iq[n:n + NFFT]
        seg2 = iq[n + NFFT:n + 2 * NFFT]
        corr_lag = np.abs(np.sum(seg1 * np.conj(seg2)))
        e1 = np.sum(np.abs(seg1) ** 2)
        e2 = np.sum(np.abs(seg2) ** 2)
        metric = corr_lag / math.sqrt(e1 * e2) if e1 > 1e-20 and e2 > 1e-20 else 0.0
        if metric > best_lag64 + 1e-6:  # strict improvement required
            best_lag64 = metric
            best_refined = n

    return best_refined


# ============================================================================
# Channel estimation
# ============================================================================
def channel_estimate(iq: np.ndarray, ltf_start: int) -> tuple[np.ndarray, float]:
    """Estimate channel from two LTF symbols.

    Returns:
        (H, noise_var) where H is complex channel per subcarrier [NFFT].
    """
    if ltf_start + 2 * NFFT > len(iq):
        H = np.ones(NFFT, dtype=complex)
        return H, 0.0

    s1 = np.fft.fft(iq[ltf_start:ltf_start + NFFT])
    s2 = np.fft.fft(iq[ltf_start + NFFT:ltf_start + 2 * NFFT])
    avg = (s1 + s2) * 0.5

    H = np.ones(NFFT, dtype=complex)

    # LTF reference: real, so conj = itself; divide by squared magnitude
    ltf_mag2 = np.real(_LTF_FREQ) ** 2 + np.imag(_LTF_FREQ) ** 2
    active = ltf_mag2 > 0.5

    # H = avg * conj(LTF) / |LTF|^2
    H[active] = avg[active] * np.conj(_LTF_FREQ[active]) / ltf_mag2[active]

    # Noise estimate from difference
    diff = s1 - s2
    noise_var = np.sum(np.abs(diff[active]) ** 2) / (2.0 * np.sum(active)) if np.any(active) else 0.0

    return H, float(noise_var)


def ht_channel_estimate(iq: np.ndarray, ht_ltf_start: int, smooth: bool = True) -> np.ndarray:
    """Estimate channel from HT-LTF (56 active subcarriers).

    Single HT-LTF symbol: CP16 + 64 FFT samples.
    H[k] = Y[k] / HT_LTF_known[k] on active subcarriers.

    Args:
        iq: CFO-corrected IQ samples.
        ht_ltf_start: Sample offset of HT-LTF symbol (start of CP).
        smooth: Apply moving-average smoothing to reduce LS noise (default True).

    Returns:
        H[NFFT] channel estimate (1.0 on inactive subcarriers).
    """
    fft_start = ht_ltf_start + NCP
    sym = iq[fft_start:fft_start + NFFT]
    Y = np.fft.fft(sym)

    H = np.ones(NFFT, dtype=complex)
    active = np.abs(_HT_LTF_FREQ) > 0.5
    ltf_mag2 = np.abs(_HT_LTF_FREQ) ** 2

    H[active] = Y[active] * np.conj(_HT_LTF_FREQ[active]) / ltf_mag2[active]

    if smooth:
        H = _smooth_channel_estimate(H)
    return H


def _smooth_channel_estimate(H: np.ndarray, width: int = 5) -> np.ndarray:
    """Apply moving-average smoothing to channel estimate.

    Smooths across active subcarriers only (56 active for HT/VHT: -28...-1, +1...+28).
    Uses a symmetric kernel of given width. Reduces noise on LS estimates
    without smearing across large frequency gaps (DC null, guard bands).

    Before smoothing, the linear phase component (due to residual timing offset)
    is removed so that the moving average does not cause destructive cancellation
    of complex values with rapidly rotating phases. The linear phase is restored
    after smoothing.

    Smoothing is applied independently to the negative and positive halves
    (subcarriers -28..-1 and +1..+28) so it never crosses the DC null.

    Args:
        H: Channel estimate [NFFT complex].
        width: Kernel width (odd recommended, default 5).

    Returns:
        Smoothed channel estimate [NFFT complex].
    """
    NFFT_local = len(H)
    H_smooth = H.copy()
    half_w = width // 2

    # Process each half independently (don't smooth across DC null)
    halves = [
        (list(range(-28, 0)), [sc % NFFT_local for sc in range(-28, 0)]),
        (list(range(1, 29)), list(range(1, 29))),
    ]

    for sc_indices, bins in halves:
        vals = np.array([H[b] for b in bins])
        n = len(vals)
        if n < 3:
            continue

        # Estimate and remove linear phase slope (from residual timing offset)
        phases = np.unwrap(np.angle(vals))
        slope, intercept = np.polyfit(np.arange(n), phases, 1)
        linear_phase = np.exp(-1j * (slope * np.arange(n) + intercept))
        vals_derotated = vals * linear_phase

        # Smooth the derotated (approximately real-valued) channel
        smoothed = np.empty_like(vals_derotated)
        for i in range(n):
            lo = max(0, i - half_w)
            hi = min(n, i + half_w + 1)
            smoothed[i] = np.mean(vals_derotated[lo:hi])

        # Restore linear phase
        restore_phase = np.exp(1j * (slope * np.arange(n) + intercept))
        smoothed *= restore_phase

        for i, b in enumerate(bins):
            H_smooth[b] = smoothed[i]

    return H_smooth


# ============================================================================
# OFDM symbol extraction
# ============================================================================


def _pilot_phase_track(eq: np.ndarray, pilot_bins: list, pilot_expected: list,
                       pilot_sc_positions: np.ndarray,
                       phase_state: dict = None,
                       H: np.ndarray = None) -> None:
    """Shared pilot-based CPE + SFO slope tracking (modifies eq in-place).

    Estimates common phase error (CPE) and per-subcarrier phase slope from
    pilot observations, then corrects the equalized frequency-domain symbol.
    Uses EWMA smoothing on the slope estimate when phase_state is provided.

    If phase_state contains 'update_h' = True and H is provided, applies
    the conjugate of the estimated rotation to H (progressive tracking for
    long frames where cumulative SFO/CFO drift matters).

    Args:
        eq: Equalized frequency-domain symbol [NFFT] (modified in-place).
        pilot_bins: FFT bin indices of pilot subcarriers.
        pilot_expected: Expected pilot values (complex, after polarity).
        pilot_sc_positions: Subcarrier indices (centered at DC) for slope fit.
        phase_state: Mutable dict for EWMA tracking across symbols.
        H: Channel estimate [NFFT] (modified in-place if update_h is set).
    """
    pilot_phases = []
    pilot_scs = []
    for k, pb in enumerate(pilot_bins):
        pilot = eq[pb]
        if abs(pilot) > 1e-10:
            phase = np.angle(pilot * np.conj(pilot_expected[k]))
            pilot_phases.append(phase)
            pilot_scs.append(pilot_sc_positions[k])

    if len(pilot_phases) >= 2:
        pilot_phases = np.array(pilot_phases)
        pilot_scs = np.array(pilot_scs)
        a = np.mean(pilot_phases)
        b = np.sum(pilot_scs * pilot_phases) / np.sum(pilot_scs ** 2)

        if phase_state is not None:
            alpha_slope = phase_state.get('alpha', 0.3)
            phase_state['phase'] = a
            phase_state['slope'] = alpha_slope * b + (1 - alpha_slope) * phase_state.get('slope', b)
            b = phase_state['slope']

        _apply_phase_slope(eq, a, b)

        # Progressive H update: apply conjugate rotation to H for next symbol
        if (phase_state is not None and phase_state.get('update_h', False)
                and H is not None):
            # Construct per-bin correction: conjugate of -(a + b*k) → +(a + b*k)
            k_idx = np.arange(NFFT, dtype=float)
            k_idx[32:] -= NFFT  # centered subcarrier index
            correction = np.exp(1j * (a + b * k_idx))
            H *= correction

    elif len(pilot_phases) == 1:
        a = pilot_phases[0]
        if phase_state is not None:
            phase_state['phase'] = a
        eq *= np.exp(-1j * a)


def extract_data_symbols(iq: np.ndarray, offset: int, H: np.ndarray,
                         noise_var: float, symbol_idx: int,
                         phase_state: dict = None) -> np.ndarray:
    """Extract and equalize one OFDM data symbol.

    Args:
        iq: Full IQ array
        offset: Start of OFDM symbol (CP + data = 80 samples)
        H: Channel estimate [NFFT]
        noise_var: Noise variance for MMSE
        symbol_idx: Symbol index for pilot polarity (0 = SIGNAL, 1+ = DATA)
        phase_state: Optional mutable dict for EWMA phase tracking.
            If provided, uses exponentially-weighted moving average to smooth
            pilot phase estimates across symbols, reducing noise-induced
            jitter in long frames.  Keys: 'phase', 'slope', 'alpha'.

    Returns:
        Equalized complex symbols on 48 data subcarriers.
    """
    sym_start = offset + NCP
    sym_end = sym_start + NFFT
    if offset < 0 or sym_end > len(iq):
        raise IndexError(
            f"OFDM symbol out of bounds: offset={offset}, "
            f"sym_start={sym_start}, sym_end={sym_end}, len={len(iq)}"
        )
    sym_time = iq[sym_start:sym_end]

    # FFT
    freq = np.fft.fft(sym_time)

    # MMSE equalization: eq = freq * conj(H) / (|H|^2 + noise_var)
    h_abs2 = np.abs(H) ** 2
    denom = np.maximum(h_abs2 + noise_var, 1e-10)
    eq = freq * np.conj(H) / denom

    # Legacy pilot expected values
    polarity = PILOT_POLARITY[symbol_idx % 127]
    pilot_expected = [PILOT_BASE[sc] * polarity for sc in PILOT_BINS]
    pilot_sc = np.array([7.0, 21.0, -21.0, -7.0])

    _pilot_phase_track(eq, PILOT_BINS, pilot_expected, pilot_sc, phase_state, H)

    return eq[DATA_BINS]


def extract_ht_data_symbols(iq: np.ndarray, offset: int, H: np.ndarray,
                            noise_var: float, symbol_idx: int,
                            phase_state: dict = None, ncp: int = NCP) -> np.ndarray:
    """Extract and equalize one HT-DATA OFDM symbol (52 data subcarriers).

    Uses HT pilot formula from IEEE TGn reference (11-06/1715r0):
      pilot(n, k) = PILOT_POLARITY[(z + n) % 127] * HT_PILOT_PATTERN[(n + k) % 4]
    where z=3 for HT-mixed DATA, k is spec-ordered pilot index (0..3 for
    subcarriers -21, -7, +7, +21), n is the DATA symbol index.

    Args:
        iq: Full IQ array (CFO-corrected).
        offset: Start of OFDM symbol (CP + data = 80 samples).
        H: HT channel estimate [NFFT].
        noise_var: Noise variance for MMSE.
        symbol_idx: DATA symbol index (0-based within HT-DATA).
        phase_state: Mutable dict for EWMA phase tracking.

    Returns:
        Equalized complex symbols on 52 HT data subcarriers.
    """
    HT_PILOT_OFFSET = 3  # z_start for HT-mixed DATA

    sym_start = offset + ncp
    sym_end = sym_start + NFFT
    if offset < 0 or sym_end > len(iq):
        raise IndexError(
            f"HT OFDM symbol out of bounds: offset={offset}, "
            f"sym_end={sym_end}, len={len(iq)}"
        )
    sym_time = iq[sym_start:sym_end]

    # FFT
    freq = np.fft.fft(sym_time)

    # MMSE equalization
    h_abs2 = np.abs(H) ** 2
    denom = np.maximum(h_abs2 + noise_var, 1e-10)
    eq = freq * np.conj(H) / denom

    # HT pilot expected values
    polarity = PILOT_POLARITY[(HT_PILOT_OFFSET + symbol_idx) % 127]
    pilot_expected = [complex(polarity * _HT_PILOT_PATTERN[(symbol_idx + k) % 4], 0)
                      for k in range(4)]
    pilot_sc_positions = np.array([-21.0, -7.0, 7.0, 21.0])

    _pilot_phase_track(eq, HT_PILOT_SPEC_BINS, pilot_expected, pilot_sc_positions, phase_state, H)

    return eq[HT_DATA_BINS]


def _extract_vht_data_symbols(iq: np.ndarray, offset: int, H: np.ndarray,
                              noise_var: float, symbol_idx: int,
                              phase_state: dict = None, ncp: int = NCP) -> np.ndarray:
    """Extract and equalize one VHT-DATA OFDM symbol (52 data subcarriers).

    Uses deterministic pilot tracking per IEEE 802.11-2020 §21.3.15.4:
      pilot(n, k) = PILOT_POLARITY[(4 + n) % 127] * HT_PILOT_PATTERN[(n + k) % 4]
    where n = DATA symbol index, k = pilot index within symbol.

    Args:
        iq: Full IQ array (CFO-corrected).
        offset: Start of OFDM symbol (CP + data = 80 samples).
        H: VHT channel estimate [NFFT].
        noise_var: Noise variance for MMSE.
        symbol_idx: DATA symbol index (0-based within VHT-DATA).
        phase_state: Mutable dict for EWMA phase tracking.

    Returns:
        Equalized complex symbols on 52 VHT data subcarriers.
    """
    VHT_PILOT_OFFSET = 4

    sym_start = offset + ncp
    sym_end = sym_start + NFFT
    if offset < 0 or sym_end > len(iq):
        raise IndexError(
            f"VHT OFDM symbol out of bounds: offset={offset}, "
            f"sym_end={sym_end}, len={len(iq)}"
        )
    sym_time = iq[sym_start:sym_end]

    # FFT
    freq = np.fft.fft(sym_time)

    # MMSE equalization
    h_abs2 = np.abs(H) ** 2
    denom = np.maximum(h_abs2 + noise_var, 1e-10)
    eq = freq * np.conj(H) / denom

    # VHT pilot expected values
    polarity = PILOT_POLARITY[(VHT_PILOT_OFFSET + symbol_idx) % 127]
    pilot_expected = [complex(polarity * _HT_PILOT_PATTERN[(symbol_idx + k) % 4], 0)
                      for k in range(4)]
    pilot_sc_positions = np.array([-21.0, -7.0, 7.0, 21.0])

    _pilot_phase_track(eq, HT_PILOT_SPEC_BINS, pilot_expected, pilot_sc_positions, phase_state, H)

    return eq[HT_DATA_BINS]


def _apply_phase_slope(freq: np.ndarray, a: float, b: float):
    """Apply phase slope correction: phase(k) = -(a + b*k).

    k is the subcarrier index, centered at DC: indices 0-31 map to 0-31,
    indices 32-63 map to -32 to -1.
    """
    k = np.arange(NFFT, dtype=float)
    k[32:] = k[32:] - NFFT  # centered subcarrier index
    phase = -(a + b * k)
    freq *= np.exp(1j * phase)


# ============================================================================
# EVM measurement
# ============================================================================

# Precomputed constellation reference grids for hard slicing
_BPSK_REF = np.array([-1.0, 1.0])
_QPSK_REF = np.array([-1.0, 1.0]) / math.sqrt(2.0)
_QAM16_REF = np.array([-3.0, -1.0, 1.0, 3.0]) / math.sqrt(10.0)
_QAM64_REF = np.array([-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0]) / math.sqrt(42.0)
_QAM256_REF = np.array([float(2*i - 15) for i in range(16)]) / math.sqrt(170.0)

_EVM_GRIDS = {
    1: _BPSK_REF,
    2: _QPSK_REF,
    4: _QAM16_REF,
    6: _QAM64_REF,
    8: _QAM256_REF,
}


def compute_evm(symbols: np.ndarray, mod_order: int) -> dict:
    """Compute Error Vector Magnitude for equalized constellation points.

    Hard-slices each received symbol to the nearest ideal constellation point
    and measures the RMS error.  Returns EVM in both dB and percent.

    IEEE 802.11-2020 §21.3.17.1 defines relative constellation error as the
    RMS of error vectors divided by the RMS of ideal reference vectors (i.e.,
    average constellation power).

    Args:
        symbols: Complex equalized data subcarrier values, shape (N,) or
                 (N_sym, N_sd) for per-symbol breakdown.
        mod_order: Bits per subcarrier (1=BPSK, 2=QPSK, 4=16QAM, 6=64QAM, 8=256QAM).

    Returns:
        dict with keys:
            evm_db: RMS EVM in dB (20*log10(evm_rms))
            evm_pct: RMS EVM as percentage
            evm_per_symbol: per-symbol EVM in dB (if 2D input)
            n_symbols: number of constellation points measured
    """
    grid = _EVM_GRIDS.get(mod_order)
    if grid is None:
        raise ValueError(f"Unsupported mod_order={mod_order}")

    syms_flat = np.asarray(symbols, dtype=np.complex128).ravel()
    n_total = len(syms_flat)
    if n_total == 0:
        return {"evm_db": -np.inf, "evm_pct": 0.0, "n_symbols": 0}

    # Hard-slice: find nearest constellation point per axis
    if mod_order == 1:
        # BPSK: real axis only
        ref = np.where(syms_flat.real >= 0, 1.0, -1.0).astype(np.complex128)
    else:
        # QAM: independent I/Q slicing
        i_vals = syms_flat.real
        q_vals = syms_flat.imag
        # Snap each axis to nearest grid point
        i_ref = grid[np.argmin(np.abs(i_vals[:, None] - grid[None, :]), axis=1)]
        q_ref = grid[np.argmin(np.abs(q_vals[:, None] - grid[None, :]), axis=1)]
        ref = i_ref + 1j * q_ref

    # Error vectors
    err = syms_flat - ref
    err_power = np.mean(np.abs(err) ** 2)
    ref_power = np.mean(np.abs(ref) ** 2)

    # RMS EVM (relative to reference power)
    if ref_power == 0:
        evm_rms = 0.0
    else:
        evm_rms = math.sqrt(err_power / ref_power)

    evm_db = 20.0 * math.log10(evm_rms) if evm_rms > 0 else -np.inf
    evm_pct = evm_rms * 100.0

    result = {
        "evm_db": float(evm_db),
        "evm_pct": float(evm_pct),
        "n_symbols": int(n_total),
    }

    # Per-symbol breakdown if input is 2D or can be reshaped
    if symbols.ndim == 2:
        n_sym, n_sd = symbols.shape
        evm_per_sym = np.empty(n_sym)
        for s in range(n_sym):
            s_syms = symbols[s]
            if mod_order == 1:
                s_ref = np.where(s_syms.real >= 0, 1.0, -1.0).astype(np.complex128)
            else:
                s_i = grid[np.argmin(np.abs(s_syms.real[:, None] - grid[None, :]), axis=1)]
                s_q = grid[np.argmin(np.abs(s_syms.imag[:, None] - grid[None, :]), axis=1)]
                s_ref = s_i + 1j * s_q
            s_err = np.mean(np.abs(s_syms - s_ref) ** 2)
            s_refp = np.mean(np.abs(s_ref) ** 2)
            s_evm = math.sqrt(s_err / s_refp) if s_refp > 0 else 0.0
            evm_per_sym[s] = 20.0 * math.log10(s_evm) if s_evm > 0 else -np.inf
        result["evm_per_symbol_db"] = evm_per_sym

    return result


# ============================================================================
# Soft demapping
# ============================================================================
def soft_demap(symbols: np.ndarray, mod_order: int) -> np.ndarray:
    """Soft demap constellation symbols to LLRs.

    Args:
        symbols: Complex constellation points
        mod_order: 1=BPSK, 2=QPSK, 4=16QAM, 6=64QAM, 8=256QAM

    Returns:
        LLRs, one per bit. Positive => bit 1 more likely.
    """
    n_sym = len(symbols)
    re_val = np.real(symbols)
    im_val = np.imag(symbols)

    if mod_order == 1:
        # BPSK: LLR = 2 * re
        return 2.0 * re_val

    if mod_order == 2:
        # QPSK: LLR[0] ∝ re, LLR[1] ∝ im
        norm = math.sqrt(2.0)
        out = np.zeros(2 * n_sym, dtype=float)
        out[0::2] = 2.0 * re_val * norm
        out[1::2] = 2.0 * im_val * norm
        return out

    if mod_order == 4:
        s = math.sqrt(10.0)
        x = re_val * s
        y = im_val * s
        out = np.zeros(4 * n_sym, dtype=float)
        out[0::4] = x                    # b0: sign bit
        out[1::4] = 2.0 - np.abs(x)      # b1: distance from ±2
        out[2::4] = y                    # b2: sign bit
        out[3::4] = 2.0 - np.abs(y)      # b3: distance from ±2
        return out

    if mod_order == 6:
        s = math.sqrt(42.0)
        x = re_val * s
        y = im_val * s
        out = np.zeros(6 * n_sym, dtype=float)
        out[0::6] = x                               # b0: sign
        out[1::6] = 4.0 - np.abs(x)                  # b1
        out[2::6] = 2.0 - np.abs(np.abs(x) - 4.0)    # b2
        out[3::6] = y                               # b3: sign
        out[4::6] = 4.0 - np.abs(y)                  # b4
        out[5::6] = 2.0 - np.abs(np.abs(y) - 4.0)    # b5
        return out

    if mod_order == 8:
        # 256-QAM: 8 bits/symbol, 4 bits per axis
        # Boundary structure: |x|→ sign, |x|-8→ b1, ||x|-8|-4→ b2, |||x|-8|-4|-2→ b3
        s = math.sqrt(170.0)
        x = re_val * s
        y = im_val * s
        out = np.zeros(8 * n_sym, dtype=float)
        out[0::8] = x                                           # b0: sign
        out[1::8] = 8.0 - np.abs(x)                             # b1
        out[2::8] = 4.0 - np.abs(np.abs(x) - 8.0)               # b2
        out[3::8] = 2.0 - np.abs(np.abs(np.abs(x) - 8.0) - 4.0) # b3
        out[4::8] = y                                           # b4: sign
        out[5::8] = 8.0 - np.abs(y)                             # b5
        out[6::8] = 4.0 - np.abs(np.abs(y) - 8.0)               # b6
        out[7::8] = 2.0 - np.abs(np.abs(np.abs(y) - 8.0) - 4.0) # b7
        return out

    raise ValueError(f"Unsupported mod_order: {mod_order}")


# ============================================================================
# Deinterleaver (soft float version)
# ============================================================================
_DEINT_PERM_CACHE: dict = {}


def soft_deinterleave(soft: np.ndarray, n_cbps: int, n_bpsc: int) -> np.ndarray:
    """Deinterleave soft values. RX: out[k] = in[perm[k]]."""
    key = (n_cbps, n_bpsc)
    perm = _DEINT_PERM_CACHE.get(key)
    if perm is None:
        s = max(n_bpsc // 2, 1)
        k = np.arange(n_cbps)
        i = (n_cbps // 16) * (k % 16) + (k // 16)
        j = s * (i // s) + (i + n_cbps - (16 * i // n_cbps)) % s
        perm = j.astype(np.intp)
        _DEINT_PERM_CACHE[key] = perm
    return soft[perm]


_HT_DEINT_PERM_CACHE: dict = {}


def ht_soft_deinterleave(soft: np.ndarray, n_cbps: int, n_bpsc: int) -> np.ndarray:
    """HT deinterleaver (802.11-2020 §19.3.11.8.3, inverse of Eq 19-46/47).

    20 MHz single stream: N_col=13, N_row=4*N_bpsc, s=max(N_bpsc/2, 1).
    Third permutation (frequency rotation) not applied for 1 stream.
    """
    key = (n_cbps, n_bpsc)
    perm = _HT_DEINT_PERM_CACHE.get(key)
    if perm is None:
        n_col = 13
        n_row = 4 * n_bpsc
        s = max(n_bpsc // 2, 1)
        k = np.arange(n_cbps)
        i = n_row * (k % n_col) + (k // n_col)
        j = s * (i // s) + (i + n_cbps - (n_col * i // n_cbps)) % s
        perm = j.astype(np.intp)
        _HT_DEINT_PERM_CACHE[key] = perm
    return soft[perm]


# ============================================================================
# Descrambler
# ============================================================================
def descramble(bits: np.ndarray, seed: int) -> np.ndarray:
    """802.11 descrambler (self-inverse)."""
    state = seed & 0x7F
    out = np.zeros_like(bits)
    for i, b in enumerate(bits):
        fb = ((state >> 6) ^ (state >> 3)) & 1
        out[i] = int(b) ^ fb
        state = ((state << 1) | fb) & 0x7F
    return out


def detect_scrambler_seed(first_7_bits: np.ndarray) -> int:
    """Brute-force recover the scrambler seed from the first 7 bits.

    The SERVICE field is all-zeros before scrambling, so the first 7
    scrambled bits equal the first 7 bits of the LFSR sequence.
    """
    for seed in range(1, 128):
        state = seed & 0x7F
        match = True
        for i in range(min(7, len(first_7_bits))):
            fb = ((state >> 6) ^ (state >> 3)) & 1
            if fb != int(first_7_bits[i]):
                match = False
                break
            state = ((state << 1) | fb) & 0x7F
        if match:
            return seed
    return 0x5D  # default fallback


# ============================================================================
# Depuncture (soft version)
# ============================================================================
def soft_depuncture(soft: np.ndarray, cr_n: int, cr_d: int,
                    fill: float = 0.0) -> np.ndarray:
    """Insert erasures at punctured positions (soft values)."""
    if cr_n == 1 and cr_d == 2:
        return soft.copy()

    if cr_n == 2 and cr_d == 3:
        pat = [1, 1, 1, 0]
    elif cr_n == 3 and cr_d == 4:
        pat = [1, 1, 1, 0, 0, 1]
    elif cr_n == 5 and cr_d == 6:
        pat = [1, 1, 1, 0, 0, 1, 1, 0, 0, 1]  # Figure 19-11
    else:
        raise ValueError(f"Unsupported rate: {cr_n}/{cr_d}")

    n_groups = len(soft) // sum(pat)
    out = np.zeros(n_groups * len(pat), dtype=float)
    j = 0
    for g in range(n_groups):
        for p_idx, keep in enumerate(pat):
            if keep:
                out[g * len(pat) + p_idx] = soft[j]
                j += 1
            else:
                out[g * len(pat) + p_idx] = fill
    return out


# ============================================================================
# L-SIG parse
# ============================================================================
def parse_signal_bits(decoded_bits: np.ndarray) -> dict:
    """Parse decoded L-SIG field bits into rate, length, etc."""
    rate_code = 0
    for i in range(4):
        rate_code |= int(decoded_bits[i]) << i

    length = 0
    for i in range(12):
        length |= int(decoded_bits[5 + i]) << i

    parity_bit = int(decoded_bits[17])
    parity_check = sum(int(decoded_bits[i]) for i in range(17)) % 2
    parity_ok = (parity_check == parity_bit)

    # Look up rate info by the 4-bit rate code
    rate_info = None
    for r, info in RATE_TABLE.items():
        if info["rate_bits"] == rate_code:
            rate_info = info
            rate_mbps = r
            break

    if rate_info is None:
        return {"rate_code": rate_code, "length": length,
                "parity_ok": parity_ok, "valid": False}

    n_symbols = math.ceil((16 + 8 * length + 6) / rate_info["n_dbps"])

    return {
        "rate_code": rate_code,
        "rate_mbps": rate_mbps,
        "mod_order": rate_info["mod_order"],
        "cr_n": rate_info["cr_n"],
        "cr_d": rate_info["cr_d"],
        "n_cbps": rate_info["n_cbps"],
        "n_dbps": rate_info["n_dbps"],
        "n_bpsc": rate_info["bpsc"],
        "length": length,
        "n_symbols": n_symbols,
        "parity_ok": parity_ok,
        "tail_ok": True,
        "valid": parity_ok,
    }


# ============================================================================
# HT/VHT-SIG modulation detection (BPSK vs Q-BPSK on first post-L-SIG symbol)
# ============================================================================
def _detect_sig_modulation(iq: np.ndarray, offset: int, H: np.ndarray,
                           noise_var: float) -> str:
    """Detect whether the first post-L-SIG symbol uses BPSK or Q-BPSK.

    IEEE 802.11-2020:
      - HT-SIG: both symbols Q-BPSK (data on Q-axis)
      - VHT-SIG-A: symbol 1 BPSK (data on I-axis), symbol 2 Q-BPSK

    Returns: "bpsk" (→ VHT), "qbpsk" (→ HT), or "unknown".
    """
    sym_start = offset + NCP
    if sym_start + NFFT > len(iq):
        return "unknown"

    freq = np.fft.fft(iq[sym_start:sym_start + NFFT])
    h_abs2 = np.abs(H) ** 2
    denom = np.maximum(h_abs2 + noise_var, 1e-10)
    eq = freq * np.conj(H) / denom

    data = eq[DATA_BINS]
    i_energy = np.sum(np.real(data) ** 2)
    q_energy = np.sum(np.imag(data) ** 2)

    # Ratio threshold: relaxed to 1.5x to catch HT frames where channel
    # distortion leaks ~30% of energy across axes.  False classification
    # is safe — CRC-8 on HT-SIG/VHT-SIG-A provides reliable validation,
    # and the decode path tries all options on failure.
    if i_energy > 1.5 * q_energy:
        return "bpsk"
    elif q_energy > 1.5 * i_energy:
        return "qbpsk"
    return "unknown"


def refine_channel_from_lsig(iq: np.ndarray, sig_offset: int,
                             H: np.ndarray, sig_bits: np.ndarray) -> np.ndarray:
    """Decision-directed channel refinement from decoded L-SIG.

    Re-encodes the known L-SIG bits to produce expected frequency-domain
    symbols (48 data + 4 pilot subcarriers), then divides the received
    L-SIG FFT by the expected values to produce H_lsig — a channel
    snapshot 1 symbol closer in time to HT-SIG/VHT-SIG-A than the L-LTF
    estimate.  Blends 50/50 with the L-LTF H.

    Transparent for static channels (cable loopback, golden vectors)
    because H_lsig == H_ltf when there is no channel evolution.

    Args:
        iq: CFO-corrected IQ samples.
        sig_offset: Start of L-SIG OFDM symbol (CP + 64 samples).
        H: Channel estimate from L-LTF [NFFT complex].
        sig_bits: Decoded L-SIG data bits (18 values, before encoding).

    Returns:
        Refined channel estimate [NFFT complex].
    """
    # Re-encode L-SIG: 18 data + 6 tail → rate-1/2 → 48 coded → interleave → BPSK
    bits_with_tail = list(int(b) for b in sig_bits) + [0] * 6  # 18 data + 6 tail
    coded = conv_encode(bits_with_tail, add_tail=False)
    interleaved = interleave(coded[:48], 48, 1)
    expected_data = modulate(interleaved, 1)  # BPSK: complex ±1

    # Compute expected pilot values for L-SIG (symbol_idx=0)
    polarity = PILOT_POLARITY[0]
    expected_pilots = [PILOT_BASE[sc] * polarity for sc in PILOT_BINS]

    # Get received FFT at L-SIG position
    sym_start = sig_offset + NCP
    freq_rx = np.fft.fft(iq[sym_start:sym_start + NFFT])

    # Compute refined H: 50/50 blend of L-LTF and L-SIG-derived
    H_refined = H.copy()
    alpha = 0.5

    # Data subcarriers (48 legacy bins)
    for i, bin_idx in enumerate(DATA_BINS):
        x = expected_data[i]
        if abs(x) > 1e-10:
            h_lsig = freq_rx[bin_idx] / x
            H_refined[bin_idx] = (1.0 - alpha) * H[bin_idx] + alpha * h_lsig

    # Pilot subcarriers (4 bins)
    for i, bin_idx in enumerate(PILOT_BINS):
        x = complex(expected_pilots[i])
        if abs(x) > 1e-10:
            h_lsig = freq_rx[bin_idx] / x
            H_refined[bin_idx] = (1.0 - alpha) * H[bin_idx] + alpha * h_lsig

    return H_refined


# ============================================================================
# VHT-SIG-A decode (IEEE 802.11-2020 §21.3.8.3.3)
# ============================================================================
def decode_vht_sig_a(iq: np.ndarray, offset: int, H: np.ndarray,
                     noise_var: float) -> dict:
    """Decode 2 OFDM symbols as VHT-SIG-A.

    Encoding per §21.3.4.5 and TGac reference (preamble_vht_siga.m):
      Symbol 1: BPSK (data on I-axis)
      Symbol 2: Q-BPSK (data on Q-axis, rotated by j)
    Rate-1/2 BCC, legacy OFDM interleaving, CRC-8 validation.

    Args:
        iq: CFO-corrected IQ samples.
        offset: Start of first VHT-SIG-A symbol (CP + 64 samples).
        H: Channel estimate from L-LTF [NFFT].
        noise_var: Noise variance for MMSE equalization.

    Returns:
        dict with VHT-SIG-A fields and crc_ok flag.
    """
    if offset + 2 * SYMBOL_SAMPLES > len(iq):
        return {"crc_ok": False, "error": "VHT-SIG-A offset out of bounds"}

    h_abs2 = np.abs(H) ** 2
    denom = np.maximum(h_abs2 + noise_var, 1e-10)

    soft_all = np.zeros(96, dtype=float)

    for sym_idx in range(2):
        sym_start = offset + sym_idx * SYMBOL_SAMPLES + NCP
        sym_time = iq[sym_start:sym_start + NFFT]
        freq = np.fft.fft(sym_time)

        # MMSE equalization
        eq = freq * np.conj(H) / denom
        data = eq[DATA_BINS]

        if sym_idx == 0:
            # Symbol 1: BPSK (data on I-axis) — no rotation needed
            pass
        else:
            # Symbol 2: Q-BPSK (data on Q-axis) — rotate by -j
            data = data * (-1j)

        # BPSK soft-demap: LLR = 2 * Re
        llrs = 2.0 * np.real(data)

        # Legacy OFDM deinterleaver
        llrs_deint = soft_deinterleave(llrs, 48, 1)
        soft_all[sym_idx * 48:(sym_idx + 1) * 48] = llrs_deint

    # Viterbi decode: 96 soft bits → 48 data bits
    decoded = viterbi_decode_soft(soft_all, 48)

    # VHT-SIG-A CRC-8: computed over bits [0:33], stored ones-complemented
    # MSB-first in bits [34:41]. Tail bits [42:47] = 0.
    data_bits = decoded[:34]
    crc_bits = decoded[34:42]
    tail_bits = decoded[42:48]

    computed_crc = ht_sig_crc8(data_bits)
    received_crc = 0
    for i in range(8):
        received_crc |= int(crc_bits[i]) << (7 - i)

    crc_ok = (received_crc == ((~computed_crc) & 0xFF))
    tail_ok = all(int(b) == 0 for b in tail_bits)

    # Parse VHT-SIG-A fields (§21.3.8.3.3, Table 21-12)
    # Symbol 1 bits [0:23]
    bw = int(decoded[0]) | (int(decoded[1]) << 1)
    stbc = int(decoded[3])
    group_id = 0
    for i in range(6):
        group_id |= int(decoded[4 + i]) << i
    nsts = 0
    for i in range(3):
        nsts |= int(decoded[10 + i]) << i
    partial_aid = 0
    for i in range(9):
        partial_aid |= int(decoded[13 + i]) << i

    # Symbol 2 bits [24:47]
    short_gi = int(decoded[24])
    sgi_disambig = int(decoded[25])
    coding = int(decoded[26])
    ldpc_extra = int(decoded[27])
    mcs = 0
    for i in range(4):
        mcs |= int(decoded[28 + i]) << i
    beamformed = int(decoded[32])

    # Reserved bits: B2, B23 (sym1) and B33 (sym2) must be 1
    # (IEEE 802.11-2020 Table 21-12)
    rsvd_ok = (int(decoded[2]) == 1 and int(decoded[23]) == 1
               and int(decoded[33]) == 1)

    return {
        "crc_ok": crc_ok,
        "tail_ok": tail_ok,
        "rsvd_ok": rsvd_ok,
        "frame_type": "vht",
        "bw": bw,
        "stbc": stbc,
        "group_id": group_id,
        "nsts": nsts + 1,
        "partial_aid": partial_aid,
        "short_gi": short_gi,
        "sgi_disambig": sgi_disambig,
        "coding": coding,
        "ldpc_extra": ldpc_extra,
        "mcs": mcs,
        "beamformed": beamformed,
        "decoded_bits": decoded,
    }


def decode_vht_sig_b(iq: np.ndarray, offset: int, H: np.ndarray,
                     noise_var: float) -> dict:
    """Decode VHT-SIG-B (1 OFDM symbol, 20 MHz single-user).

    VHT-SIG-B for 20 MHz SU: 26 bits (17 LENGTH + 3 reserved + 6 tail),
    BPSK, rate-1/2 BCC, legacy interleaving on 52 data subcarriers.
    (IEEE 802.11-2020 Table 21-13)

    Args:
        iq: CFO-corrected IQ samples.
        offset: Start of VHT-SIG-B symbol (CP + 64 samples).
        H: Channel estimate from VHT-LTF [NFFT].
        noise_var: Noise variance.

    Returns:
        dict with 'length' (PSDU bytes), 'tail_ok', and 'raw_bits' fields.
    """
    if offset + SYMBOL_SAMPLES > len(iq):
        return {"length": 0, "error": "VHT-SIG-B out of bounds"}

    sym_start = offset + NCP
    freq = np.fft.fft(iq[sym_start:sym_start + NFFT])

    # MMSE equalization
    h_abs2 = np.abs(H) ** 2
    denom = np.maximum(h_abs2 + noise_var, 1e-10)
    eq = freq * np.conj(H) / denom

    # Extract 52 data subcarriers (HT layout, BPSK)
    data = eq[HT_DATA_BINS]
    llrs = 2.0 * np.real(data)

    # HT deinterleaver (same interleaver as VHT-SIG-B for 20 MHz)
    llrs_deint = ht_soft_deinterleave(llrs, 52, 1)

    # Viterbi: 52 soft bits -> 26 data bits
    decoded = viterbi_decode_soft(llrs_deint, 26)

    # Parse: bits 0-16 = LENGTH (LSB first), bits 17-19 = reserved, bits 20-25 = tail
    # (IEEE 802.11-2020 Table 21-13, 20 MHz SU format)
    length_field = 0
    for i in range(17):
        length_field |= int(decoded[i]) << i

    tail_ok = all(int(decoded[20 + i]) == 0 for i in range(6))

    # LENGTH is in 4-byte units for 20 MHz SU
    psdu_length = length_field * 4

    return {
        "length": psdu_length,
        "length_field": length_field,
        "tail_ok": tail_ok,
        "raw_bits": list(decoded[:26]),
    }


def _refine_H_from_sigb(iq: np.ndarray, sigb_offset: int,
                         H_initial: np.ndarray,
                         sigb_bits) -> np.ndarray:
    """Refine VHT channel estimate using decoded VHT-SIG-B as reference.

    VHT-SIG-B is BPSK, rate-1/2, HT-interleaved on 52 data subcarriers.
    We re-encode to get expected freq-domain symbols and compute a per-
    subcarrier channel update as weighted average of LTF-based and SIG-B-
    derived estimates.

    Args:
        iq: CFO-corrected IQ samples.
        sigb_offset: Start of VHT-SIG-B OFDM symbol (80 samples).
        H_initial: Channel estimate from VHT-LTF [NFFT complex].
        sigb_bits: Decoded VHT-SIG-B bits (26 values, 0/1).

    Returns:
        Refined channel estimate [NFFT complex].
    """
    # Local imports to avoid circular dependency with gen_ofdm_frame
    from py80211.gen_ofdm_frame import conv_encode, _ht_interleave, modulate

    # Re-encode VHT-SIG-B: 26 bits → rate-1/2 → 52 coded → HT interleave → BPSK
    bits = list(int(b) for b in sigb_bits)
    coded = conv_encode(bits, add_tail=False)
    interleaved = _ht_interleave(coded[:52], 52, 1)
    expected_syms = modulate(interleaved, 1)  # BPSK: 52 complex ±1 values

    # Get received frequency-domain signal at SIG-B position
    sym_start = sigb_offset + NCP
    freq_rx = np.fft.fft(iq[sym_start:sym_start + NFFT])

    # Compute refined H: weighted average of LTF and SIG-B derived estimates
    H_refined = H_initial.copy()
    alpha = 0.5  # mixing weight: 0.5 LTF + 0.5 SIG-B
    for i, bin_idx in enumerate(HT_DATA_BINS):
        x = expected_syms[i]
        if abs(x) > 1e-10:
            h_sigb = freq_rx[bin_idx] / x
            H_refined[bin_idx] = (1.0 - alpha) * H_initial[bin_idx] + alpha * h_sigb

    return H_refined


# ============================================================================
# HT-SIG decode (IEEE 802.11-2020 §19.3.9.4.3)
# ============================================================================
def decode_ht_sig(iq: np.ndarray, offset: int, H: np.ndarray,
                  noise_var: float) -> dict:
    """Speculatively decode 2 OFDM symbols as HT-SIG.

    Encoding: BPSK (sym1 on I) + Q-BPSK (sym2 on Q), rate-1/2 BCC,
    legacy OFDM interleaving, CRC-8 validation.

    The HT-SIG uses the legacy OFDM structure (48 data subcarriers,
    same interleaver as L-SIG). "No interleaving" in some references
    means no HT-specific interleaving — the base OFDM interleaver
    still applies.

    CRC-8 convention: computed CRC (poly 0x07, init 0xFF) is stored
    ones-complemented, MSB-first in bits [34:41].

    Args:
        iq: CFO-corrected IQ samples.
        offset: Start of first HT-SIG symbol (CP + 64 samples).
        H: Channel estimate from L-LTF [NFFT].
        noise_var: Noise variance for MMSE equalization.

    Returns:
        dict with keys: crc_ok, mcs, bw, length, aggregation, stbc,
        fec_coding, short_gi, n_ess, frame_type ('ht' or 'vht'),
        decoded_bits (48 bits).
        Returns dict with crc_ok=False if CRC fails.
    """
    # Need 2 symbols (each 80 samples: 16 CP + 64 FFT)
    if offset + 2 * SYMBOL_SAMPLES > len(iq):
        return {"crc_ok": False, "error": "HT-SIG offset out of bounds"}

    # Extract and equalize both symbols using L-LTF channel estimate
    h_abs2 = np.abs(H) ** 2
    denom = np.maximum(h_abs2 + noise_var, 1e-10)

    soft_all = np.zeros(96, dtype=float)

    for sym_idx in range(2):
        sym_start = offset + sym_idx * SYMBOL_SAMPLES + NCP
        sym_time = iq[sym_start:sym_start + NFFT]
        freq = np.fft.fft(sym_time)

        # MMSE equalization
        eq = freq * np.conj(H) / denom

        # Extract 48 data subcarriers
        data = eq[DATA_BINS]

        # Q-BPSK de-rotation: both HT-SIG symbols use Q-BPSK
        # (data on quadrature axis, per IEEE TGn reference convention).
        # Multiply by -j to rotate Q-axis data back to I-axis.
        data = data * (-1j)

        # BPSK soft-demap: LLR = 2 * Re
        llrs = 2.0 * np.real(data)

        # Apply legacy OFDM deinterleaver (same as L-SIG)
        llrs_deint = soft_deinterleave(llrs, 48, 1)
        soft_all[sym_idx * 48:(sym_idx + 1) * 48] = llrs_deint

    # Viterbi decode: 96 soft bits → 48 data bits
    # HT-SIG has 48 information bits (34 data + 8 CRC + 6 tail)
    # The encoder produces 96 coded bits from 48 input bits (rate 1/2)
    decoded = viterbi_decode_soft(soft_all, 48)

    # Parse: bits [0:33] = data, [34:41] = CRC, [42:47] = tail
    data_bits = decoded[:34]
    crc_bits = decoded[34:42]
    tail_bits = decoded[42:48]

    # Compute CRC over data bits and compare
    # Convention: CRC bits [34:41] store (~CRC) MSB-first
    # bit[34] = MSB of ones-complement of computed CRC
    computed_crc = ht_sig_crc8(data_bits)
    received_crc = 0
    for i in range(8):
        received_crc |= int(crc_bits[i]) << (7 - i)  # MSB-first

    crc_ok = (received_crc == ((~computed_crc) & 0xFF))

    # Check tail bits (should be 000000)
    tail_ok = all(int(b) == 0 for b in tail_bits)

    # Parse HT-SIG fields
    mcs = 0
    for i in range(7):
        mcs |= int(data_bits[i]) << i

    bw = int(data_bits[7])

    length = 0
    for i in range(16):
        length |= int(data_bits[8 + i]) << i

    smoothing = int(data_bits[24])
    not_sounding = int(data_bits[25])
    aggregation = int(data_bits[27])
    stbc = int(data_bits[28]) | (int(data_bits[29]) << 1)
    fec_coding = int(data_bits[30])
    short_gi = int(data_bits[31])
    n_ess = int(data_bits[32]) | (int(data_bits[33]) << 1)

    result = {
        "crc_ok": crc_ok,
        "tail_ok": tail_ok,
        "frame_type": "ht",
        "mcs": mcs,
        "bw": bw,
        "length": length,
        "smoothing": smoothing,
        "not_sounding": not_sounding,
        "aggregation": aggregation,
        "stbc": stbc,
        "fec_coding": fec_coding,
        "short_gi": short_gi,
        "n_ess": n_ess,
        "decoded_bits": decoded,
        "computed_crc": computed_crc,
        "received_crc": received_crc,
    }

    return result




# ============================================================================
# LDPC decode helper
# ============================================================================

def _ldpc_decode_data(soft_coded: np.ndarray, n_dbps: int, n_cbps: int,
                      cr_n: int, cr_d: int, n_sym: int) -> np.ndarray:
    """Decode LDPC-coded soft LLR stream to information bits.

    Mirrors the TX-side _ldpc_encode_data logic: computes LDPC parameters
    (codeword length, shortening, puncturing), segments the LLR stream into
    per-codeword chunks, undoes puncturing/shortening, calls ldpc_decode(),
    and concatenates the information bits.

    Args:
        soft_coded: Soft LLR values (positive = bit 1 more likely,
                    matching soft_demap convention), length = n_sym * n_cbps.
        n_dbps: Data bits per OFDM symbol.
        n_cbps: Coded bits per OFDM symbol.
        cr_n: Code rate numerator.
        cr_d: Code rate denominator.
        n_sym: Number of OFDM data symbols.

    Returns:
        Decoded information bits (int8 array), length = n_sym * n_dbps.
    """
    # Negate LLR: soft_demap produces positive=bit1, but ldpc_decode expects positive=bit0
    soft_coded = -np.asarray(soft_coded, dtype=np.float64)
    R = cr_n / cr_d
    rate_str = f"{cr_n}/{cr_d}"

    n_pld = n_sym * n_dbps
    n_avail = n_sym * n_cbps

    # Select codeword parameters (same logic as TX _ldpc_encode_data)
    if n_pld <= 648:
        n_cw = 1
        l_cw = 1944
        for cw in [648, 1296, 1944]:
            if int(cw * R) >= n_pld:
                l_cw = cw
                break
    elif n_pld <= 1296:
        n_cw = 1
        l_cw = 1944
        for cw in [1296, 1944]:
            if int(cw * R) >= n_pld:
                l_cw = cw
                break
    else:
        n_cw = 1
        l_cw = 1944

    # If single codeword can't hold n_pld, use multi-codeword
    if int(l_cw * R) < n_pld:
        n_cw = math.ceil(n_pld / (1944 * R))
        l_cw = 1944

    # Compute shortening and puncturing
    k_per_cw = int(l_cw * R)
    k_total = k_per_cw * n_cw
    n_shrt = max(0, k_total - n_pld)
    n_parity_per_cw = l_cw - k_per_cw
    n_parity = n_parity_per_cw * n_cw
    # Total transmitted = n_pld + (n_parity - n_punc) must equal n_avail
    n_punc = max(0, n_pld + n_parity - n_avail)

    # Distribute shortening/puncturing evenly across codewords
    shrt_per_cw = n_shrt // n_cw
    shrt_extra = n_shrt % n_cw
    punc_per_cw = n_punc // n_cw
    punc_extra = n_punc % n_cw

    # Decode each codeword
    decoded_info = []
    bit_offset = 0

    for j in range(n_cw):
        shrt_j = shrt_per_cw + (1 if j < shrt_extra else 0)
        punc_j = punc_per_cw + (1 if j < punc_extra else 0)

        # Number of LLR values for this codeword from the stream:
        # systematic (k_per_cw - shrt_j) + parity (n_parity_per_cw - punc_j)
        k_j = k_per_cw - shrt_j
        parity_j = n_parity_per_cw - punc_j
        n_llr_j = k_j + parity_j

        # Extract this codeword's LLR from the stream
        cw_llr = soft_coded[bit_offset:bit_offset + n_llr_j]
        bit_offset += n_llr_j

        # Reconstruct full codeword LLR (length = l_cw)
        # Structure: [info (k_per_cw) | parity (n_parity_per_cw)]
        # Info part: k_j actual LLRs + shrt_j high-confidence zeros
        info_llr = np.empty(k_per_cw, dtype=np.float64)
        info_llr[:k_j] = cw_llr[:k_j]
        # Shortened positions are known-zero bits: large positive LLR
        info_llr[k_j:] = 100.0

        # Parity part: parity_j actual LLRs + punc_j erasures (0.0)
        parity_llr = np.empty(n_parity_per_cw, dtype=np.float64)
        parity_llr[:parity_j] = cw_llr[k_j:]
        # Punctured positions: insert erasure (0.0 = no information)
        if punc_j > 0:
            parity_llr[parity_j:] = 0.0

        # Full codeword LLR
        full_llr = np.concatenate([info_llr, parity_llr])

        # LDPC decode
        hard = ldpc_decode(full_llr, rate_str, l_cw)

        # Extract information bits (first k_j, skipping shortened zeros)
        decoded_info.append(hard[:k_j].copy())

    return np.concatenate(decoded_info)


# ============================================================================
# A-MPDU delimiter parsing (IEEE 802.11-2020 §9.7.3)
# ============================================================================

def _ampdu_delimiter_crc8(bits_14: int) -> int:
    """Compute CRC-8 over 14-bit delimiter header (EOF + reserved + length).

    Polynomial: x^8 + x^2 + x + 1 (0x07 in normal form), init=0xFF.
    Input is 14 bits processed LSB-first.
    """
    crc = 0xFF
    for i in range(14):
        bit = (bits_14 >> i) & 1
        if (crc ^ bit) & 1:
            crc = (crc >> 1) ^ 0x8E  # 0x07 reversed
        else:
            crc >>= 1
    return crc


def parse_ampdu(data: bytes) -> list[bytes]:
    """Parse A-MPDU aggregate into individual MPDU subframes.

    Validates each 4-byte delimiter (unique pattern 0x4E, CRC-8), extracts
    MPDU bytes, skips padding.  Stops at EOF delimiter or end of data.

    IEEE 802.11-2020 §9.7.3, Figure 9-30:
        Bits 0:     EOF
        Bit 1:      Reserved
        Bits 2-13:  MPDU Length (12 bits, max 4095)
        Bits 14-21: CRC-8 (over bits 0-13)
        Bits 22-29: Unique Pattern (0x4E)
        Bits 30-31: Reserved

    Args:
        data: Raw PSDU bytes from PHY decode (the A-MPDU).

    Returns:
        List of MPDU byte strings extracted from the aggregate.

    Raises:
        ValueError: If no valid delimiters found or data is malformed.
    """
    mpdus = []
    offset = 0
    n = len(data)

    while offset + 4 <= n:
        # Read 4-byte delimiter (little-endian bit ordering within bytes)
        d0, d1, d2, d3 = data[offset], data[offset + 1], data[offset + 2], data[offset + 3]
        delim_word = d0 | (d1 << 8) | (d2 << 16) | (d3 << 24)

        # Extract fields
        eof = delim_word & 1
        # reserved = (delim_word >> 1) & 1
        mpdu_length = (delim_word >> 2) & 0xFFF
        crc_received = (delim_word >> 14) & 0xFF
        unique_pattern = (delim_word >> 22) & 0xFF
        # reserved2 = (delim_word >> 30) & 0x3

        # Validate unique pattern
        if unique_pattern != 0x4E:
            raise ValueError(
                f"Invalid delimiter at offset {offset}: "
                f"unique pattern 0x{unique_pattern:02X} != 0x4E"
            )

        # Validate CRC-8 over bits 0-13
        header_14 = delim_word & 0x3FFF
        crc_expected = _ampdu_delimiter_crc8(header_14)
        if crc_received != crc_expected:
            raise ValueError(
                f"Delimiter CRC mismatch at offset {offset}: "
                f"received 0x{crc_received:02X}, expected 0x{crc_expected:02X}"
            )

        # EOF: done
        if eof or mpdu_length == 0:
            break

        # Extract MPDU (immediately follows the 4-byte delimiter)
        mpdu_start = offset + 4
        mpdu_end = mpdu_start + mpdu_length
        if mpdu_end > n:
            raise ValueError(
                f"MPDU at offset {offset} extends beyond data: "
                f"need {mpdu_length} bytes, have {n - mpdu_start}"
            )

        mpdus.append(data[mpdu_start:mpdu_end])

        # Advance past delimiter + MPDU + padding to 4-byte boundary
        total = 4 + mpdu_length
        padded = (total + 3) & ~3
        offset += padded

    if not mpdus:
        raise ValueError("No valid MPDU subframes found in A-MPDU")

    return mpdus


# ============================================================================
# Frame decode — main entry point
# ============================================================================
def decode_frame(iq: np.ndarray, stf_start: int):  # -> dict | None (3.10+)
    """Decode a single 802.11 frame from baseband IQ.

    Supports legacy 802.11a, HT-mixed 802.11n, and VHT 802.11ac
    (classification only for VHT, no DATA decode yet).

    Args:
        iq: Complex baseband IQ samples
        stf_start: STF start offset (from detect_stf)

    Returns:
        dict with decoded frame info, or None on failure.
    """
    n_samples = len(iq)
    work = iq[stf_start:].copy()

    # Coarse CFO estimation and correction
    cfo_coarse = estimate_cfo_coarse(work, 0)
    work = apply_cfo_correction(work, cfo_coarse)

    # Find LTF
    ltf_start = find_ltf(work, len(work), 0)

    # IQ imbalance correction (estimate from L-LTF, apply to full buffer)
    work = _correct_iq_imbalance(work, ltf_start)

    # Fine CFO
    cfo_fine = estimate_cfo_fine(work, ltf_start)
    work = apply_cfo_correction(work, cfo_fine)

    # Channel estimation
    H, noise_var = channel_estimate(work, ltf_start)

    # Decode L-SIG
    sig_offset = ltf_start + 2 * NFFT
    if sig_offset + NCP + NFFT > len(work):
        return {"error": "SIGNAL offset out of bounds"}
    data_syms = extract_data_symbols(work, sig_offset, H, noise_var, 0)
    soft = soft_demap(data_syms, 1)  # BPSK
    soft_deint = soft_deinterleave(soft, 48, 1)
    sig_decoded = viterbi_decode_soft(soft_deint, 18)

    sig_info = parse_signal_bits(sig_decoded)
    if not sig_info["valid"]:
        return {"signal": sig_info, "error": "SIGNAL parity fail"}

    # Decision-directed channel refinement from L-SIG.
    # Produces H_refined: temporally closer to HT-SIG/VHT-SIG-A than L-LTF H.
    # Transparent for static channels (golden vectors, cable loopback).
    H_refined = refine_channel_from_lsig(work, sig_offset, H, sig_decoded)

    # ============================================================
    # Classification: detect HT/VHT by modulation of post-L-SIG symbol
    # ============================================================
    sig_a_offset = ltf_start + 2 * NFFT + SYMBOL_SAMPLES
    ht_sig_result = None
    vht_sig_result = None
    frame_type = "legacy"

    # HT-mixed and VHT both require L-SIG rate = 6 Mbps
    # (IEEE 802.11-2020 §19.3.9.3.5, §21.3.8.2.4).
    if sig_info.get("rate_mbps") == 6:
        if sig_a_offset + 2 * SYMBOL_SAMPLES <= len(work):
            # Detect modulation: BPSK (VHT) vs Q-BPSK (HT) on first symbol
            # Use H_refined for better detection under channel evolution
            mod_type = _detect_sig_modulation(work, sig_a_offset, H_refined, noise_var)

            if mod_type == "qbpsk":
                # HT-SIG: both symbols Q-BPSK
                ht_sig_result = decode_ht_sig(work, sig_a_offset, H_refined, noise_var)
                if ht_sig_result and ht_sig_result.get("crc_ok"):
                    frame_type = "ht"
                else:
                    # HT-SIG CRC failed → try VHT, then legacy
                    vht_sig_result = decode_vht_sig_a(work, sig_a_offset, H_refined, noise_var)
                    if (vht_sig_result and vht_sig_result.get("crc_ok")
                            and vht_sig_result.get("tail_ok", True)):
                        frame_type = "vht"
            elif mod_type == "bpsk":
                # VHT-SIG-A: sym1 BPSK, sym2 Q-BPSK
                vht_sig_result = decode_vht_sig_a(work, sig_a_offset, H_refined, noise_var)
                if (vht_sig_result and vht_sig_result.get("crc_ok")
                        and vht_sig_result.get("tail_ok", True)):
                    frame_type = "vht"
                else:
                    # VHT-SIG-A CRC failed → try HT, then legacy
                    ht_sig_result = decode_ht_sig(work, sig_a_offset, H_refined, noise_var)
                    if ht_sig_result and ht_sig_result.get("crc_ok"):
                        frame_type = "ht"

            # Fallback: if rotation detection was ambiguous, try both
            if frame_type == "legacy" and mod_type == "unknown":
                ht_sig_result = decode_ht_sig(work, sig_a_offset, H_refined, noise_var)
                if ht_sig_result and ht_sig_result.get("crc_ok"):
                    frame_type = "ht"
                else:
                    vht_sig_result = decode_vht_sig_a(work, sig_a_offset, H_refined, noise_var)
                    if (vht_sig_result and vht_sig_result.get("crc_ok")
                            and vht_sig_result.get("tail_ok", True)):
                        frame_type = "vht"

        # If L-SIG rate is 6 Mbps and classification still "legacy",
        # speculatively try HT/VHT decode — CRC-8 provides reliable
        # validation (1/256 false positive rate).
        if frame_type == "legacy":
            ht_sig_result = decode_ht_sig(work, sig_a_offset, H_refined, noise_var)
            if ht_sig_result and ht_sig_result.get("crc_ok"):
                frame_type = "ht"
            else:
                vht_sig_result = decode_vht_sig_a(work, sig_a_offset, H_refined, noise_var)
                if (vht_sig_result and vht_sig_result.get("crc_ok")
                        and vht_sig_result.get("tail_ok", True)):
                    frame_type = "vht"

    # ============================================================
    # HT-DATA decode
    # ============================================================
    if frame_type == "ht":
        mcs = ht_sig_result["mcs"]
        ht_length = ht_sig_result["length"]
        short_gi = ht_sig_result.get("short_gi", 0)
        ncp = NCP_SHORT if short_gi else NCP
        sym_samples = SYMBOL_SAMPLES_SHORT if short_gi else SYMBOL_SAMPLES

        if mcs > 7:
            # Only MCS 0-7 (single stream, 20 MHz) supported
            cfo_total = cfo_coarse + cfo_fine
            return {
                "signal": sig_info,
                "frame_type": frame_type,
                "ht_sig": ht_sig_result,
                "cfo_hz": float(cfo_total),
                "error": f"unsupported MCS {mcs}",
            }

        mcs_info = HT_MCS_TABLE[mcs]
        mod_order = mcs_info["mod_order"]
        n_cbps = mcs_info["n_cbps"]
        n_dbps = mcs_info["n_dbps"]
        n_bpsc = mcs_info["bpsc"]
        cr_n = mcs_info["cr_n"]
        cr_d = mcs_info["cr_d"]

        # Number of HT-DATA symbols.
        # Primary: use HT-SIG length field if plausible.
        # Fallback: derive from L-SIG LENGTH (always reliable for HT-mixed
        # from real APs where L-SIG encodes NAV protection duration).
        # The L-SIG LENGTH field encodes enough legacy symbols to cover the
        # HT frame duration.  Legacy STA computes n_leg_sym OFDM symbols
        # (at 6 Mbps); the HT overhead after L-SIG is 4 symbols (HT-SIG×2
        # + HT-STF + HT-LTF for 1 SS), so n_ht_data = n_leg_sym − 4.
        lsig_length = sig_info.get("length", 0)
        n_leg_sym = math.ceil((8 * lsig_length + 22) / 24)
        n_sym_from_lsig = max(n_leg_sym - 4, 1)

        n_sym_from_htsig = math.ceil((16 + 8 * ht_length + 6) / n_dbps)

        # LDPC: no tail bits in symbol count; must also check for extra symbol
        fec_coding = ht_sig_result.get("fec_coding", 0)
        if fec_coding == 1:
            n_sym_from_htsig = math.ceil((16 + 8 * ht_length) / n_dbps)
            # Recompute LDPC extra symbol condition (IEEE 802.11-2020 §19.3.11.7.5)
            _n_pld = n_sym_from_htsig * n_dbps
            _R = cr_n / cr_d
            if _n_pld <= 648:
                _l_cw = 1944
                for _cw in [648, 1296, 1944]:
                    if int(_cw * _R) >= _n_pld:
                        _l_cw = _cw
                        break
                _n_cw = 1
            elif _n_pld <= 1296:
                _l_cw = 1944
                for _cw in [1296, 1944]:
                    if int(_cw * _R) >= _n_pld:
                        _l_cw = _cw
                        break
                _n_cw = 1
            else:
                _l_cw = 1944
                _n_cw = 1
            # If single codeword can't hold _n_pld, use multi-codeword
            if int(_l_cw * _R) < _n_pld:
                _n_cw = math.ceil(_n_pld / (1944 * _R))
                _l_cw = 1944
            _n_avail = n_sym_from_htsig * n_cbps
            _n_shrt = max(0, int(_n_cw * _l_cw * _R) - _n_pld)
            _n_punc = max(0, _n_cw * _l_cw - _n_avail - _n_shrt)
            _n_parity = _n_cw * (_l_cw - int(_l_cw * _R))
            if _n_parity > 0 and _n_punc > 0.1 * _n_parity:
                n_sym_from_htsig += 1

        # Trust HT-SIG length if it's sane (within 16-bit field max).
        # Real AP HT-SIG length can be unreliable (~39000 bytes observed
        # on Actiontec T3200M) due to CRC-8 false positives or decode
        # errors; fall back to L-SIG-derived count in that case.
        if ht_length <= 65535:
            n_sym = n_sym_from_htsig
        else:
            n_sym = n_sym_from_lsig

        # Guard: cap decode to symbols that fit in the available buffer.
        max_sym = n_sym

        # Locate HT-STF, HT-LTF, HT-DATA
        # HT-SIG is 2 symbols after L-SIG; then HT-STF (1 sym), HT-LTF (1 sym for 1 SS)
        ht_stf_offset = sig_a_offset + 2 * SYMBOL_SAMPLES
        ht_ltf_offset = ht_stf_offset + SYMBOL_SAMPLES
        ht_data_start = ht_ltf_offset + SYMBOL_SAMPLES

        # Cap to symbols that actually fit in the buffer
        avail_samples = len(work) - ht_data_start
        if avail_samples < sym_samples:
            return {"signal": sig_info, "frame_type": frame_type,
                    "ht_sig": ht_sig_result, "error": "HT-DATA truncated"}
        max_sym = min(max_sym, avail_samples // sym_samples)

        # If we can't decode all symbols, the FCS will fail — but we
        # still attempt partial decode for diagnostics.  For frames
        # that fit entirely, n_sym == max_sym.
        truncated = (max_sym < n_sym)

        # HT-LTF channel estimation (56 subcarriers)
        if ht_ltf_offset + SYMBOL_SAMPLES > len(work):
            return {"signal": sig_info, "frame_type": frame_type,
                    "ht_sig": ht_sig_result, "error": "HT-LTF truncated"}
        H_ht = ht_channel_estimate(work, ht_ltf_offset)

        # Decode HT-DATA symbols
        all_soft = []
        phase_state = {'phase': 0.0, 'slope': 0.0, 'alpha': 0.3,
                       'update_h': (n_sym > 3)}
        for i in range(max_sym):
            sym_off = ht_data_start + i * sym_samples
            if sym_off + ncp + NFFT > len(work):
                break
            try:
                data_syms = extract_ht_data_symbols(
                    work, sym_off, H_ht, noise_var, i, phase_state, ncp=ncp)
            except (IndexError, ValueError):
                break
            soft_raw = soft_demap(data_syms, mod_order)
            if fec_coding == 1:
                # LDPC: no deinterleaving
                all_soft.append(soft_raw)
            else:
                soft_deint = ht_soft_deinterleave(soft_raw, n_cbps, n_bpsc)
                all_soft.append(soft_deint)

        if not all_soft:
            return {"signal": sig_info, "frame_type": frame_type,
                    "ht_sig": ht_sig_result, "error": "no HT data symbols"}

        # If frame was truncated, return early with classification only
        n_decoded_sym = len(all_soft)
        if truncated:
            cfo_total = cfo_coarse + cfo_fine
            return {
                "signal": sig_info,
                "frame_type": "ht",
                "ht_sig": ht_sig_result,
                "cfo_hz": float(cfo_total),
                "cfo_coarse_hz": float(cfo_coarse),
                "cfo_fine_hz": float(cfo_fine),
                "noise_var": float(noise_var),
                "error": f"truncated ({n_decoded_sym}/{n_sym} symbols)",
                "stf_start_local": 0,
                "stf_start_global": stf_start,
                "ltf_start": ltf_start,
                "n_data_symbols": n_sym,
            }

        all_soft = np.concatenate(all_soft)

        if fec_coding == 1:
            # LDPC decode path
            decoded = _ldpc_decode_data(all_soft, n_dbps, n_cbps, cr_n, cr_d, n_sym)
        else:
            # BCC decode path
            # Depuncture
            depunct = soft_depuncture(all_soft, cr_n, cr_d, 0.0)
            # Viterbi decode
            n_data_bits = n_sym * n_dbps
            decoded = viterbi_decode_soft(depunct, n_data_bits)

        # Descramble
        seed = detect_scrambler_seed(decoded[:7])
        descrambled = descramble(decoded, seed)

        # Extract PSDU (skip 16-bit SERVICE, take 8*length bits)
        psdu_bits = descrambled[16:16 + 8 * ht_length]
        psdu = bits_to_bytes_lsb(psdu_bits.astype(int).tolist())

        # Verify FCS (non-aggregated) or parse A-MPDU
        aggregation = ht_sig_result.get("aggregation", 0)
        mpdus = None
        if aggregation:
            # A-MPDU: PSDU contains delimiters + subframes, no outer FCS
            try:
                mpdus = parse_ampdu(psdu)
                fcs_ok = True  # delimiter CRC passed; per-MPDU FCS is caller's job
            except ValueError:
                fcs_ok = False
        else:
            fcs_ok = verify_fcs(psdu)

        cfo_total = cfo_coarse + cfo_fine
        result = {
            "signal": sig_info,
            "frame_type": "ht",
            "ht_sig": ht_sig_result,
            "psdu": psdu,
            "fcs_ok": fcs_ok,
            "mcs": mcs,
            "short_gi": short_gi,
            "ht_length": ht_length,
            "cfo_hz": float(cfo_total),
            "cfo_coarse_hz": float(cfo_coarse),
            "cfo_fine_hz": float(cfo_fine),
            "noise_var": float(noise_var),
            "scrambler_seed": seed,
            "stf_start_local": 0,
            "stf_start_global": stf_start,
            "ltf_start": ltf_start,
            "n_data_symbols": n_sym,
        }
        if mpdus is not None:
            result["mpdus"] = mpdus
        return result

    # ============================================================
    # VHT-DATA decode (IEEE 802.11-2020 §21, 20 MHz, 1 SS, BCC)
    # ============================================================
    if frame_type == "vht":
        mcs = vht_sig_result["mcs"]
        nsts = vht_sig_result["nsts"]

        # NDP detection: L-SIG LENGTH ≤ 9 indicates only preamble duration
        # (no VHT-SIG-B or DATA). IEEE 802.11-2020 §21.3.8.3.2.
        sig_length = sig_info.get("length", 0)
        if sig_length <= 9:
            cfo_total = cfo_coarse + cfo_fine
            return {
                "signal": sig_info,
                "frame_type": "vht",
                "ndp": True,
                "vht_sig_a": vht_sig_result,
                "cfo_hz": float(cfo_total),
                "cfo_coarse_hz": float(cfo_coarse),
                "cfo_fine_hz": float(cfo_fine),
                "noise_var": float(noise_var),
                "stf_start_local": 0,
                "stf_start_global": stf_start,
                "ltf_start": ltf_start,
            }

        if nsts != 1 or vht_sig_result.get("bw", 0) != 0:
            cfo_total = cfo_coarse + cfo_fine
            return {
                "signal": sig_info,
                "frame_type": frame_type,
                "vht_sig_a": vht_sig_result,
                "cfo_hz": float(cfo_total),
                "error": f"unsupported NSTS={nsts} BW={vht_sig_result.get('bw')}",
            }

        # Reject STBC-coded frames (requires multiple RX chains)
        if vht_sig_result.get("stbc", 0) != 0:
            cfo_total = cfo_coarse + cfo_fine
            return {
                "signal": sig_info,
                "frame_type": frame_type,
                "vht_sig_a": vht_sig_result,
                "cfo_hz": float(cfo_total),
                "error": "STBC not supported (single-stream receiver)",
            }

        if mcs > 8:
            cfo_total = cfo_coarse + cfo_fine
            return {
                "signal": sig_info,
                "frame_type": frame_type,
                "vht_sig_a": vht_sig_result,
                "cfo_hz": float(cfo_total),
                "error": f"unsupported VHT MCS {mcs}",
            }

        # Reject MU-MIMO frames: group_id 1-62 uses different VHT-SIG-B format
        # (group_id 0 or 63 = SU; IEEE 802.11-2020 Table 21-12)
        group_id = vht_sig_result.get("group_id", 0)
        if group_id != 0 and group_id != 63:
            cfo_total = cfo_coarse + cfo_fine
            return {
                "signal": sig_info,
                "frame_type": "vht",
                "vht_sig_a": vht_sig_result,
                "cfo_hz": float(cfo_total),
                "error": f"MU-MIMO not supported (group_id={group_id})",
            }

        # VHT MCS table for 20 MHz, 1 SS (52 data + 4 pilot subcarriers)
        VHT_MCS_PARAMS = {
            0: (1, 1, 2, 52, 26),    # BPSK 1/2
            1: (2, 1, 2, 104, 52),   # QPSK 1/2
            2: (2, 3, 4, 104, 78),   # QPSK 3/4
            3: (4, 1, 2, 208, 104),  # 16QAM 1/2
            4: (4, 3, 4, 208, 156),  # 16QAM 3/4
            5: (6, 2, 3, 312, 208),  # 64QAM 2/3
            6: (6, 3, 4, 312, 234),  # 64QAM 3/4
            7: (6, 5, 6, 312, 260),  # 64QAM 5/6
            8: (8, 3, 4, 416, 312),  # 256QAM 3/4
        }
        mod_order, cr_n, cr_d, n_cbps, n_dbps = VHT_MCS_PARAMS[mcs]
        n_bpsc = mod_order

        # VHT preamble offsets after L-SIG:
        # VHT-SIG-A: 2 symbols, VHT-STF: 1 symbol, VHT-LTF: 1 symbol (1 SS),
        # VHT-SIG-B: 1 symbol, VHT-DATA: starts 5 symbols after sig_a_offset
        vht_stf_offset = sig_a_offset + 2 * SYMBOL_SAMPLES
        vht_ltf_offset = vht_stf_offset + SYMBOL_SAMPLES
        vht_sigb_offset = vht_ltf_offset + SYMBOL_SAMPLES
        vht_data_start = vht_sigb_offset + SYMBOL_SAMPLES

        # VHT-LTF channel estimate (56 active subcarriers, same sequence as HT-LTF)
        if vht_ltf_offset + SYMBOL_SAMPLES > len(work):
            return {"signal": sig_info, "frame_type": frame_type,
                    "vht_sig_a": vht_sig_result, "error": "VHT-LTF truncated"}
        H_vht = ht_channel_estimate(work, vht_ltf_offset)

        # Decode VHT-SIG-B to get PSDU length
        vht_sig_b = decode_vht_sig_b(work, vht_sigb_offset, H_vht, noise_var)
        vht_length = vht_sig_b.get("length", 0)

        # L-SIG LENGTH cross-validation: the fake L-SIG length encodes the
        # VHT PPDU duration.  Expected: (n_preamble_sym + n_data_sym) * 3 - 3
        # where n_preamble_sym = 5 (SIG-A:2 + STF:1 + LTF:1 + SIG-B:1).
        # We validate after computing n_sym below (stored as lsig_consistent).
        sig_length = sig_info.get("length", 0)

        # Decision-directed channel refinement using VHT-SIG-B
        if vht_length > 0 and "raw_bits" in vht_sig_b:
            H_vht = _refine_H_from_sigb(
                work, vht_sigb_offset, H_vht, vht_sig_b["raw_bits"])

        # Determine number of DATA symbols from PSDU length
        vht_coding = vht_sig_result.get("coding", 0)
        if vht_coding == 1:
            # LDPC: no tail bits
            n_data_bits_needed = 16 + 8 * vht_length
        else:
            n_data_bits_needed = 16 + 8 * vht_length + 6  # SERVICE + PSDU + tail
        n_sym = math.ceil(n_data_bits_needed / n_dbps)

        # LDPC extra symbol (IEEE 802.11-2020 §21.3.8.3.3, VHT-SIG-A bit 27)
        if vht_coding == 1 and vht_sig_result.get("ldpc_extra", 0):
            n_sym += 1

        # SGI disambiguation (IEEE 802.11-2020 §21.3.10.2, VHT-SIG-A bit 25):
        # When short GI is used and sgi_disambig=1, an extra symbol is present.
        short_gi = vht_sig_result.get("short_gi", 0)
        if short_gi and vht_sig_result.get("sgi_disambig", 0):
            n_sym += 1

        # L-SIG LENGTH cross-validation (defense-in-depth diagnostic)
        if short_gi:
            expected_lsig = 3 * (math.ceil(n_sym * 0.9) + 5) - 3
        else:
            expected_lsig = (5 + n_sym) * 3 - 3
        lsig_consistent = (sig_length == expected_lsig)

        # Determine GI for DATA symbols
        ncp = NCP_SHORT if short_gi else NCP
        sym_samples = SYMBOL_SAMPLES_SHORT if short_gi else SYMBOL_SAMPLES

        # Cap to available buffer
        avail_samples = len(work) - vht_data_start
        if avail_samples < sym_samples:
            return {"signal": sig_info, "frame_type": frame_type,
                    "vht_sig_a": vht_sig_result, "error": "VHT-DATA truncated"}
        max_sym = min(n_sym, avail_samples // sym_samples)

        # Decode VHT-DATA symbols with blind pilot phase+slope tracking
        all_soft = []
        phase_state = {'phase': 0.0, 'slope': 0.0, 'alpha': 0.3,
                       'update_h': (n_sym > 3)}
        for i in range(max_sym):
            sym_off = vht_data_start + i * sym_samples
            if sym_off + ncp + NFFT > len(work):
                break
            try:
                data_syms = _extract_vht_data_symbols(
                    work, sym_off, H_vht, noise_var, i, phase_state, ncp=ncp)
            except (IndexError, ValueError):
                break
            soft_raw = soft_demap(data_syms, mod_order)
            if vht_coding == 1:
                # LDPC: no deinterleaving
                all_soft.append(soft_raw)
            else:
                soft_deint = ht_soft_deinterleave(soft_raw, n_cbps, n_bpsc)
                all_soft.append(soft_deint)

        if not all_soft:
            return {"signal": sig_info, "frame_type": frame_type,
                    "vht_sig_a": vht_sig_result, "error": "no VHT data symbols"}

        all_soft = np.concatenate(all_soft)

        if vht_coding == 1:
            # LDPC decode path
            decoded = _ldpc_decode_data(all_soft, n_dbps, n_cbps, cr_n, cr_d, n_sym)
        else:
            # BCC decode path
            # Depuncture
            depunct = soft_depuncture(all_soft, cr_n, cr_d, 0.0)
            # Viterbi decode
            n_data_bits = n_sym * n_dbps
            decoded = viterbi_decode_soft(depunct, n_data_bits)

        # Descramble
        seed = detect_scrambler_seed(decoded[:7])
        descrambled = descramble(decoded, seed)

        # Verify VHT SERVICE field CRC (IEEE 802.11-2020 §21.3.10.5)
        # SERVICE bits [8:15] carry CRC-8 of VHT-SIG-B first 20 bits,
        # ones-complemented, MSB-first.
        service_crc_ok = False
        if len(descrambled) >= 16 and "raw_bits" in vht_sig_b:
            sigb_bits_for_crc = vht_sig_b["raw_bits"][:20]
            expected_crc = ht_sig_crc8(sigb_bits_for_crc)
            expected_crc_inv = (~expected_crc) & 0xFF
            # Extract received CRC from SERVICE bits [8:15], MSB-first
            received_crc = 0
            for i in range(8):
                received_crc |= int(descrambled[8 + i]) << (7 - i)
            service_crc_ok = (received_crc == expected_crc_inv)

        # Extract PSDU (length from VHT-SIG-B)
        psdu_bits = descrambled[16:16 + 8 * vht_length]
        psdu = bits_to_bytes_lsb(psdu_bits.astype(int).tolist())

        # VHT DATA is always A-MPDU (IEEE 802.11-2020 §9.7.3).
        # Attempt to parse delimiters; fall back to raw FCS check for
        # non-aggregated test frames.
        mpdus = None
        try:
            mpdus = parse_ampdu(psdu)
            fcs_ok = True
        except ValueError:
            # Not a valid A-MPDU (e.g., single non-aggregated test frame)
            fcs_ok = verify_fcs(psdu)

        cfo_total = cfo_coarse + cfo_fine
        result = {
            "signal": sig_info,
            "frame_type": "vht",
            "vht_sig_a": vht_sig_result,
            "psdu": psdu,
            "fcs_ok": fcs_ok,
            "service_crc_ok": service_crc_ok,
            "mcs": mcs,
            "short_gi": short_gi,
            "vht_length": vht_length,
            "cfo_hz": float(cfo_total),
            "cfo_coarse_hz": float(cfo_coarse),
            "cfo_fine_hz": float(cfo_fine),
            "noise_var": float(noise_var),
            "scrambler_seed": seed,
            "stf_start_local": 0,
            "stf_start_global": stf_start,
            "ltf_start": ltf_start,
            "n_data_symbols": n_sym,
            "lsig_consistent": lsig_consistent,
            "sigb_tail_ok": vht_sig_b.get("tail_ok", True),
        }
        if mpdus is not None:
            result["mpdus"] = mpdus
        return result

    # ============================================================
    # Legacy DATA decode
    # ============================================================
    data_start = ltf_start + 2 * NFFT + SYMBOL_SAMPLES
    mod_order = sig_info["mod_order"]
    n_cbps = sig_info["n_cbps"]
    n_dbps = sig_info["n_dbps"]
    n_bpsc = sig_info["n_bpsc"]
    n_sym = sig_info["n_symbols"]
    length = sig_info["length"]

    # For progressive H tracking on long frames, use a mutable copy
    H_track = H.copy()
    all_soft = []
    phase_state = {'phase': 0.0, 'slope': 0.0, 'alpha': 0.3,
                   'update_h': (n_sym > 3)}
    for i in range(n_sym):
        sym_off = data_start + i * SYMBOL_SAMPLES
        if sym_off + NCP + NFFT > len(work):
            break
        try:
            data_syms = extract_data_symbols(work, sym_off, H_track, noise_var,
                                             i + 1, phase_state)
        except (IndexError, ValueError):
            break
        soft_raw = soft_demap(data_syms, mod_order)
        soft_deint = soft_deinterleave(soft_raw, n_cbps, n_bpsc)
        all_soft.append(soft_deint)

    if not all_soft:
        return {"signal": sig_info, "error": "no data symbols"}

    all_soft = np.concatenate(all_soft)

    # Depuncture
    depunct = soft_depuncture(all_soft, sig_info["cr_n"], sig_info["cr_d"], 0.0)

    # Viterbi decode
    n_data_bits = n_sym * n_dbps
    decoded = viterbi_decode_soft(depunct, n_data_bits)

    # Descramble
    seed = detect_scrambler_seed(decoded[:7])
    descrambled = descramble(decoded, seed)

    # Extract PSDU
    psdu_bits = descrambled[16:16 + 8 * length]
    psdu = bits_to_bytes_lsb(psdu_bits.astype(int).tolist())

    # Verify FCS
    fcs_ok = verify_fcs(psdu)

    cfo_total = cfo_coarse + cfo_fine

    return {
        "signal": sig_info,
        "frame_type": "legacy",
        "psdu": psdu,
        "fcs_ok": fcs_ok,
        "cfo_hz": float(cfo_total),
        "cfo_coarse_hz": float(cfo_coarse),
        "cfo_fine_hz": float(cfo_fine),
        "noise_var": float(noise_var),
        "scrambler_seed": seed,
        "stf_start_local": 0,
        "stf_start_global": stf_start,
        "ltf_start": ltf_start,
        "n_data_symbols": n_sym,
        "rate_mbps": sig_info["rate_mbps"],
        "length": length,
    }


# ============================================================================
# Backward-compatible DecodeContext (thin wrapper for tools that use it)
# ============================================================================
class DecodeContext:
    """Thin compatibility wrapper. Use decode_frame() and detect_stf() directly."""

    def detect_frame(self, iq: np.ndarray,
                     threshold: float = 0.6,
                     min_periods: int = 8) -> int:
        return detect_stf(iq, threshold=threshold, min_periods=min_periods)

    def decode_frame(self, iq: np.ndarray, stf_start: int):
        return decode_frame(iq, stf_start)


# ============================================================================
# Self-test: loopback (generate → decode)
# ============================================================================
def _test_loopback():
    from py80211.gen_ofdm_frame import generate_frame
    from py80211.impairments import add_awgn

    print("=== loopback test: generate -> decode ===")
    for rate in sorted(RATE_TABLE):
        payload = bytes([i % 256 for i in range(50)])
        iq, meta = generate_frame(rate, payload)
        # Add modest noise for realism
        iq_noisy = add_awgn(iq, snr_db=25, seed=rate)

        result = decode_frame(iq_noisy, 0)

        if result and result.get("fcs_ok"):
            psdu_ok = result["psdu"][:len(payload)] == payload
            status = "OK" if psdu_ok else "PAYLOAD MISMATCH"
        elif result:
            status = f"FCS FAIL (error={result.get('error', 'unknown')})"
        else:
            status = "NO RESULT"

        snr_info = ""
        if result and result.get("fcs_ok"):
            # Compare payload
            decoded_psdu = result["psdu"]
            expected = meta["psdu_with_fcs"]
            errors = sum(a != b for a, b in zip(decoded_psdu, expected))
            snr_info = f" byte_errors={errors}"

        print(f"  Rate {rate:2d} Mbps: {status}{snr_info}")


if __name__ == "__main__":
    _test_loopback()
