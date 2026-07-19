"""RX path golden vector tests — decode verification against IEEE reference waveforms.

Tests the decode pipeline (decode_frame.py) in two levels:

1. Block-level: Feed known frequency-domain symbols through demod/deinterleave/Viterbi
   and verify against the corresponding TX-stage golden vectors.

2. Full-frame: Feed the complete waveform vectors through the full decode pipeline
   and verify the recovered PSDU matches the known input.

Vectors from: IEEE TGn reference waveform generator (11-06/1715r0).
Stored in: vectors/*.json
"""

import math

import numpy as np
import pytest

from conftest import (
    load_vector,
    ANNEX_I1_PSDU_BYTES,
    ANNEX_I1_SCRAMBLER_SEED,
)

from py80211.gen_ofdm_frame import (
    NFFT,
    NCP,
    SYMBOL_SAMPLES,
    RATE_TABLE,
    DATA_BINS,
    PILOT_BINS,
    PILOT_BASE,
    PILOT_POLARITY,
    HT_MCS_TABLE,
    HT_DATA_BINS,
    _ht_interleave,
    conv_encode,
    puncture,
    scramble,
    modulate,
    bytes_to_bits_lsb,
    generate_frame,
    generate_ht_frame,
)

from py80211.decode_frame import (
    decode_frame,
    viterbi_decode_soft,
    soft_demap,
    soft_deinterleave,
    ht_soft_deinterleave,
    soft_depuncture,
    descramble,
    detect_scrambler_seed,
    parse_signal_bits,
    decode_ht_sig,
    channel_estimate,
    ht_channel_estimate,
    extract_data_symbols,
    extract_ht_data_symbols,
    bits_to_bytes_lsb,
    verify_fcs,
)


# ============================================================================
# Helper: reconstruct waveform from golden vectors as complex array
# ============================================================================

def waveform_from_vector(name: str) -> np.ndarray:
    """Load a waveform golden vector as complex numpy array."""
    vec = load_vector(name)
    return np.array(vec["real"], dtype=float) + 1j * np.array(vec["imag"], dtype=float)


# ============================================================================
# Block-level RX tests: Soft demap → known TX constellation
# ============================================================================

class TestSoftDemap:
    """Verify soft demapper produces correct hard decisions on clean symbols."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_demap_ht_clean(self, mcs):
        """Demapping clean QAM symbols must recover the interleaved bits."""
        vec = load_vector(f"ht_mcs{mcs}_qam_symbols")
        info = HT_MCS_TABLE[mcs]
        mod_order = info["mod_order"]

        # Clean constellation points from TX golden vector
        symbols = np.array(vec["real"], dtype=float) + 1j * np.array(vec["imag"], dtype=float)
        # First OFDM symbol worth (52 subcarriers)
        symbols = symbols[:52]

        # Soft demap
        soft = soft_demap(symbols, mod_order)

        # Hard decision: positive LLR → bit 1, negative → bit 0
        hard = (soft > 0).astype(int).tolist()

        # Expected: the interleaved bits for first symbol
        vec_int = load_vector(f"ht_mcs{mcs}_interleaved")
        n_cbps = info["n_cbps"]
        expected = vec_int["data"][:n_cbps]

        assert hard == expected, (
            f"MCS {mcs}: hard decision mismatch, "
            f"first diff at {next(i for i in range(len(hard)) if hard[i] != expected[i])}"
        )

    def test_demap_256qam_roundtrip(self):
        """256-QAM modulate→demap round-trip must produce zero-error hard decisions."""
        rng = np.random.default_rng(0)
        bits = rng.integers(0, 2, size=52 * 8).tolist()  # 52 subcarriers × 8 bits
        symbols = modulate(bits, mod_order=8)
        soft = soft_demap(symbols, mod_order=8)
        hard = (soft > 0).astype(int).tolist()
        assert hard == bits


# ============================================================================
# Block-level RX tests: Deinterleaver
# ============================================================================

class TestHTDeinterleave:
    """Verify HT deinterleaver inverts the HT interleaver."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_deinterleave_inverts_interleave(self, mcs):
        """Deinterleaving interleaved bits must recover original encoded bits."""
        vec_int = load_vector(f"ht_mcs{mcs}_interleaved")
        vec_enc = load_vector(f"ht_mcs{mcs}_encoded")
        info = HT_MCS_TABLE[mcs]
        n_cbps = info["n_cbps"]
        n_bpsc = info["bpsc"]

        # Interleaved bits as soft values (+1.0 for bit 1, -1.0 for bit 0)
        interleaved = vec_int["data"][:n_cbps]
        soft_interleaved = np.array([1.0 if b else -1.0 for b in interleaved])

        # Deinterleave
        soft_deint = ht_soft_deinterleave(soft_interleaved, n_cbps, n_bpsc)

        # Hard decision
        hard = [1 if x > 0 else 0 for x in soft_deint]

        # Expected: first n_cbps encoded bits
        expected = vec_enc["data"][:n_cbps]
        assert hard == expected, (
            f"MCS {mcs}: deinterleave mismatch, "
            f"first diff at {next(i for i in range(len(hard)) if hard[i] != expected[i])}"
        )


# ============================================================================
# Block-level RX tests: Viterbi decoder
# ============================================================================

class TestViterbiDecode:
    """Verify Viterbi decoder recovers scrambled bits from encoded bits."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_viterbi_ht(self, mcs):
        """Viterbi decode of encoded bits must recover the scrambled stream."""
        vec_enc = load_vector(f"ht_mcs{mcs}_encoded")
        vec_scr = load_vector(f"ht_mcs{mcs}_scrambled")
        info = HT_MCS_TABLE[mcs]
        cr_n = info["cr_n"]
        cr_d = info["cr_d"]
        n_dbps = info["n_dbps"]

        encoded = vec_enc["data"]
        expected_scrambled = vec_scr["data"]
        n_data_bits = len(expected_scrambled)

        # Convert encoded hard bits to soft LLRs (+1 for bit 1, -1 for bit 0)
        soft_encoded = np.array([1.0 if b else -1.0 for b in encoded])

        # Depuncture to rate-1/2
        soft_depunct = soft_depuncture(soft_encoded, cr_n, cr_d, fill=0.0)

        # Viterbi decode
        decoded = viterbi_decode_soft(soft_depunct, n_data_bits)

        # Compare (skip tail bits at position 816..821 which are zeroed post-scramble)
        tail_start = 16 + 8 * len(ANNEX_I1_PSDU_BYTES)  # 816
        mismatches_pre = [i for i in range(tail_start)
                          if int(decoded[i]) != expected_scrambled[i]]
        assert not mismatches_pre, (
            f"MCS {mcs}: {len(mismatches_pre)} Viterbi decode errors before tail, "
            f"first at bit {mismatches_pre[0]}"
        )

        # Check pad bits (after tail)
        pad_start = tail_start + 6
        mismatches_pad = [i for i in range(pad_start, min(len(decoded), n_data_bits))
                          if int(decoded[i]) != expected_scrambled[i]]
        assert not mismatches_pad, (
            f"MCS {mcs}: {len(mismatches_pad)} Viterbi errors in pad bits, "
            f"first at bit {mismatches_pad[0]}"
        )


# ============================================================================
# Block-level RX tests: Descrambler + PSDU extraction
# ============================================================================

class TestDescramble:
    """Verify descrambler recovers PSDU from scrambled bits."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_descramble_ht(self, mcs):
        """Descrambling with known seed must recover PSDU bits."""
        vec_scr = load_vector(f"ht_mcs{mcs}_scrambled")
        scrambled = np.array(vec_scr["data"], dtype=np.uint8)

        # Zero tail bits (as the TX path does)
        tail_start = 16 + 8 * len(ANNEX_I1_PSDU_BYTES)
        for i in range(6):
            if tail_start + i < len(scrambled):
                scrambled[tail_start + i] = 0

        # Descramble
        descrambled = descramble(scrambled, ANNEX_I1_SCRAMBLER_SEED)

        # SERVICE field should be all zeros
        service = descrambled[:16]
        assert all(int(b) == 0 for b in service), "SERVICE bits not all zero"

        # PSDU bits should match
        psdu_bits = descrambled[16:16 + 8 * len(ANNEX_I1_PSDU_BYTES)]
        expected_bits = bytes_to_bits_lsb(ANNEX_I1_PSDU_BYTES)
        mismatches = [i for i in range(len(expected_bits))
                      if int(psdu_bits[i]) != expected_bits[i]]
        assert not mismatches, (
            f"MCS {mcs}: {len(mismatches)} PSDU bit errors, "
            f"first at bit {mismatches[0]}"
        )


# ============================================================================
# Block-level RX tests: Scrambler seed detection
# ============================================================================

class TestSeedDetection:
    """Verify scrambler seed detection from SERVICE field."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_detect_seed_ht(self, mcs):
        """Seed detection from first 7 scrambled bits must recover 0x5D."""
        vec_scr = load_vector(f"ht_mcs{mcs}_scrambled")
        first_7 = np.array(vec_scr["data"][:7], dtype=np.uint8)
        seed = detect_scrambler_seed(first_7)
        assert seed == ANNEX_I1_SCRAMBLER_SEED, (
            f"MCS {mcs}: detected seed 0x{seed:02X}, expected 0x{ANNEX_I1_SCRAMBLER_SEED:02X}"
        )


# ============================================================================
# Full-frame RX tests: Decode complete waveform → verify PSDU
# ============================================================================

class TestFullFrameDecodeHT:
    """Decode complete HT waveform vectors and verify PSDU recovery."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_decode_ht_waveform(self, mcs):
        """Full decode of IEEE reference waveform must recover correct PSDU."""
        iq = waveform_from_vector(f"ht_mcs{mcs}_waveform")

        # The waveform is clean (no noise, no CFO) — decode should succeed
        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert "error" not in result, (
            f"MCS {mcs}: decode error: {result.get('error')}"
        )
        assert result.get("frame_type") == "ht", (
            f"MCS {mcs}: expected frame_type='ht', got '{result.get('frame_type')}'"
        )
        assert result.get("fcs_ok"), (
            f"MCS {mcs}: FCS verification failed"
        )

        # Verify decoded PSDU matches known input
        psdu = result.get("psdu")
        assert psdu is not None, f"MCS {mcs}: no PSDU in result"
        assert psdu == ANNEX_I1_PSDU_BYTES, (
            f"MCS {mcs}: PSDU mismatch, "
            f"got {len(psdu)} bytes, expected {len(ANNEX_I1_PSDU_BYTES)}"
        )

    @pytest.mark.parametrize("mcs", range(8))
    def test_decode_ht_sig_fields(self, mcs):
        """Decoded HT-SIG fields must match expected values."""
        iq = waveform_from_vector(f"ht_mcs{mcs}_waveform")
        result = decode_frame(iq, 0)

        assert result is not None
        ht_sig = result.get("ht_sig")
        assert ht_sig is not None, f"MCS {mcs}: no ht_sig in result"
        assert ht_sig.get("crc_ok"), f"MCS {mcs}: HT-SIG CRC failed"
        assert ht_sig.get("mcs") == mcs, (
            f"MCS {mcs}: decoded MCS={ht_sig.get('mcs')}"
        )


# ============================================================================
# Full-frame RX tests: Legacy decode (generate → decode, clean)
# ============================================================================

class TestFullFrameDecodeLegacy:
    """Decode our own TX output (clean, no noise) for all legacy rates.

    This is a self-consistency check: if test_tx.py passes (TX matches
    IEEE vectors) and this passes (RX decodes TX output), then the RX
    path is functionally correct for the clean case.
    """

    @pytest.mark.parametrize("rate", sorted(RATE_TABLE.keys()))
    def test_decode_legacy_clean(self, rate):
        """TX→RX loopback at each legacy rate must recover PSDU."""
        # Use a shorter payload to keep things fast
        payload = bytes([i % 256 for i in range(50)])
        iq, meta = generate_frame(rate, payload)

        result = decode_frame(iq, 0)

        assert result is not None, f"Rate {rate}: decode returned None"
        assert "error" not in result, (
            f"Rate {rate}: decode error: {result.get('error')}"
        )
        assert result.get("fcs_ok"), f"Rate {rate}: FCS failed"

        # Verify payload portion of PSDU
        psdu = result.get("psdu")
        assert psdu is not None
        assert psdu[:len(payload)] == payload, (
            f"Rate {rate}: payload mismatch"
        )


# ============================================================================
# Full-frame RX tests: VHT classification (detect VHT-SIG-A, parse fields)
# ============================================================================

class TestVHTClassification:
    """Decode VHT waveform vectors and verify classification + DATA decode.

    Tests verify:
    - Frame is classified as VHT (not HT or legacy)
    - VHT-SIG-A CRC passes
    - MCS, NSTS, BW fields are correct
    - VHT-DATA decode (MCS 0-7): FCS pass and PSDU matches
    """

    @pytest.mark.parametrize("mcs", range(9))
    def test_classify_vht_waveform(self, mcs):
        """VHT waveform must be classified as VHT with correct SIG-A fields."""
        iq = waveform_from_vector(f"vht_mcs{mcs}_waveform")

        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "vht", (
            f"MCS {mcs}: expected frame_type='vht', "
            f"got '{result.get('frame_type')}'"
        )

        vht_sig = result.get("vht_sig_a")
        assert vht_sig is not None, f"MCS {mcs}: no vht_sig_a in result"
        assert vht_sig.get("crc_ok"), f"MCS {mcs}: VHT-SIG-A CRC failed"
        assert vht_sig.get("mcs") == mcs, (
            f"MCS {mcs}: decoded MCS={vht_sig.get('mcs')}"
        )
        assert vht_sig.get("nsts") == 1, (
            f"MCS {mcs}: decoded NSTS={vht_sig.get('nsts')}, expected 1"
        )
        assert vht_sig.get("bw") == 0, (
            f"MCS {mcs}: decoded BW={vht_sig.get('bw')}, expected 0 (20 MHz)"
        )

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_sig_a_bits_match_vector(self, mcs):
        """Decoded VHT-SIG-A bits must match golden vector."""
        iq = waveform_from_vector(f"vht_mcs{mcs}_waveform")
        result = decode_frame(iq, 0)

        assert result is not None
        vht_sig = result.get("vht_sig_a")
        assert vht_sig is not None and vht_sig.get("crc_ok")

        # Compare decoded bits against golden vector
        vec = load_vector(f"vht_mcs{mcs}_vhtsiga_bits")
        expected_bits = vec["data"]
        decoded_bits = [int(b) for b in vht_sig["decoded_bits"][:48]]

        assert decoded_bits == expected_bits, (
            f"MCS {mcs}: VHT-SIG-A bits mismatch, "
            f"first diff at {next(i for i in range(48) if decoded_bits[i] != expected_bits[i])}"
        )

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_data_decode(self, mcs):
        """VHT-DATA decode (MCS 0-8) must produce FCS-valid PSDU."""
        iq = waveform_from_vector(f"vht_mcs{mcs}_waveform")
        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "vht"
        assert "error" not in result, (
            f"MCS {mcs}: decode error: {result.get('error')}"
        )
        assert result.get("fcs_ok"), f"MCS {mcs}: FCS failed"
        assert result.get("mcs") == mcs

        # All VHT golden vectors use the same 100-byte PSDU (Annex I.1)
        psdu = result.get("psdu")
        assert psdu is not None
        assert psdu[:len(ANNEX_I1_PSDU_BYTES)] == ANNEX_I1_PSDU_BYTES, (
            f"MCS {mcs}: PSDU payload mismatch"
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])


class TestVHTLoopback:
    """VHT TX→RX loopback (no impairments)."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_loopback_clean(self, mcs):
        """VHT loopback with no impairments must decode FCS-valid."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        payload = bytes([(i * 7 + 13) % 256 for i in range(80)])
        iq, meta = generate_vht_frame(mcs, payload)
        result = decode_frame(iq, 0)
        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "vht", f"MCS {mcs}: not classified as VHT"
        assert "error" not in result, f"MCS {mcs}: {result.get('error')}"
        assert result.get("fcs_ok"), f"MCS {mcs}: FCS failed"
        assert result["psdu"][:len(payload)] == payload


# ============================================================================
# HT Short GI loopback tests
# ============================================================================

class TestHTShortGILoopback:
    """HT short-GI TX->RX loopback (no impairments)."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_sgi_loopback_clean(self, mcs):
        """Generate HT short-GI frame, decode it, verify FCS and PSDU."""
        from py80211.gen_ofdm_frame import generate_ht_frame

        payload = ANNEX_I1_PSDU_BYTES
        iq, meta = generate_ht_frame(
            mcs=mcs,
            psdu_bytes=payload,
            scrambler_seed=ANNEX_I1_SCRAMBLER_SEED,
            short_gi=True,
        )

        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "ht", (
            f"MCS {mcs}: expected 'ht', got '{result.get('frame_type')}'"
        )
        assert "error" not in result, f"MCS {mcs}: {result.get('error')}"
        assert result.get("fcs_ok"), f"MCS {mcs}: FCS failed"
        assert result["psdu"][:len(payload)] == payload, (
            f"MCS {mcs}: PSDU payload mismatch"
        )

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_sgi_golden_waveform_decode(self, mcs):
        """Decode HT short-GI golden waveform vector."""
        iq = waveform_from_vector(f"ht_mcs{mcs}_sgi_waveform")
        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "ht"
        assert "error" not in result, f"MCS {mcs}: {result.get('error')}"
        assert result.get("fcs_ok"), f"MCS {mcs}: FCS failed"
        # Golden vectors use ANNEX_I1_PSDU_BYTES as input to generate_ht_frame
        psdu = result.get("psdu")
        assert psdu[:len(ANNEX_I1_PSDU_BYTES)] == ANNEX_I1_PSDU_BYTES


# ============================================================================
# VHT Short GI loopback tests
# ============================================================================

class TestVHTShortGILoopback:
    """VHT short-GI TX->RX loopback (no impairments)."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_sgi_loopback_clean(self, mcs):
        """Generate VHT short-GI frame, decode it, verify FCS and PSDU."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        payload = bytes([(i * 7 + 13) % 256 for i in range(80)])
        iq, meta = generate_vht_frame(
            mcs=mcs,
            psdu_bytes=payload,
            scrambler_seed=ANNEX_I1_SCRAMBLER_SEED,
            short_gi=True,
        )

        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "vht", (
            f"MCS {mcs}: expected 'vht', got '{result.get('frame_type')}'"
        )
        assert "error" not in result, f"MCS {mcs}: {result.get('error')}"
        assert result.get("fcs_ok"), f"MCS {mcs}: FCS failed"
        assert result["psdu"][:len(payload)] == payload, (
            f"MCS {mcs}: PSDU payload mismatch"
        )

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_sgi_golden_waveform_decode(self, mcs):
        """Decode VHT short-GI golden waveform vector."""
        iq = waveform_from_vector(f"vht_mcs{mcs}_sgi_waveform")
        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "vht"
        assert "error" not in result, f"MCS {mcs}: {result.get('error')}"
        assert result.get("fcs_ok"), f"MCS {mcs}: FCS failed"


# ============================================================================
# HT LDPC loopback tests
# ============================================================================

class TestHTLDPCLoopback:
    """HT LDPC frame: TX -> RX recovers PSDU with FCS OK."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_ldpc_loopback(self, mcs):
        """Generate HT LDPC frame, decode it, verify FCS and PSDU."""
        from py80211.gen_ofdm_frame import generate_ht_frame

        psdu = bytes(range(100))
        iq, meta = generate_ht_frame(mcs, psdu, coding="ldpc")
        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "ht", (
            f"MCS {mcs}: expected 'ht', got '{result.get('frame_type')}'"
        )
        assert "error" not in result, f"MCS {mcs}: {result.get('error')}"
        assert result.get("fcs_ok"), f"MCS {mcs}: FCS failed"
        # Verify the LDPC coding bit was indicated
        ht_sig = result.get("ht_sig", {})
        assert ht_sig.get("fec_coding") == 1, "fec_coding not 1 (LDPC)"
        # Verify PSDU content (payload before FCS)
        assert result["psdu"][:len(psdu)] == psdu, f"MCS {mcs}: PSDU mismatch"


# ============================================================================
# VHT LDPC loopback tests
# ============================================================================

class TestVHTLDPCLoopback:
    """VHT LDPC frame: TX -> RX recovers PSDU with FCS OK."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_ldpc_loopback(self, mcs):
        """Generate VHT LDPC frame, decode it, verify FCS and PSDU."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        psdu = bytes(range(100))
        iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
        result = decode_frame(iq, 0)

        assert result is not None, f"MCS {mcs}: decode returned None"
        assert result.get("frame_type") == "vht", (
            f"MCS {mcs}: expected 'vht', got '{result.get('frame_type')}'"
        )
        assert "error" not in result, f"MCS {mcs}: {result.get('error')}"
        assert result.get("fcs_ok"), f"MCS {mcs}: FCS failed"
        # Verify the LDPC coding bit was indicated
        vht_sig = result.get("vht_sig_a", {})
        assert vht_sig.get("coding") == 1, "VHT coding not 1 (LDPC)"
        # Verify PSDU content (payload before FCS)
        assert result["psdu"][:len(psdu)] == psdu, f"MCS {mcs}: PSDU mismatch"


# ============================================================================
# Large frame decode tests (no artificial size caps)
# ============================================================================

class TestLargeFrames:
    def test_large_frame_ht(self):
        """HT frame with >4000 bytes decodes correctly."""
        from py80211.gen_ofdm_frame import generate_ht_frame
        psdu = bytes(range(256)) * 20  # 5120 bytes
        iq, _ = generate_ht_frame(7, psdu)  # MCS 7 for speed
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]
        assert result["psdu"][:len(psdu)] == psdu

    def test_large_frame_vht(self):
        """VHT frame with >4000 bytes decodes correctly."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        psdu = bytes(range(256)) * 20  # 5120 bytes
        iq, _ = generate_vht_frame(8, psdu)  # MCS 8 for speed
        result = decode_frame(iq, 0)
        assert result["fcs_ok"]
        assert result["psdu"][:len(psdu)] == psdu


# ============================================================================
# A-MPDU delimiter parsing tests
# ============================================================================

class TestAMPDUParsing:
    """A-MPDU delimiter parsing: bytes in, MPDUs out or ValueError."""

    def _make_delimiter(self, mpdu_length: int, eof: bool = False) -> bytes:
        """Construct a valid 4-byte A-MPDU delimiter."""
        from py80211.decode_frame import _ampdu_delimiter_crc8
        header_14 = (int(eof) & 1) | ((mpdu_length & 0xFFF) << 2)
        crc = _ampdu_delimiter_crc8(header_14)
        delim_word = header_14 | (crc << 14) | (0x4E << 22)
        return delim_word.to_bytes(4, 'little')

    def _pad_to_4(self, data: bytes) -> bytes:
        """Pad data to next 4-byte boundary."""
        rem = len(data) % 4
        return data + b'\x00' * ((4 - rem) % 4)

    def test_single_mpdu(self):
        """Single MPDU subframe extracted correctly."""
        from py80211.decode_frame import parse_ampdu
        mpdu = b'Hello, 802.11ac!'
        ampdu = self._make_delimiter(len(mpdu)) + mpdu + self._make_delimiter(0, eof=True)
        result = parse_ampdu(ampdu)
        assert result == [mpdu]

    def test_multiple_mpdus(self):
        """Multiple MPDUs with padding between them."""
        from py80211.decode_frame import parse_ampdu
        mpdu1 = bytes(range(30))   # needs 2 bytes padding (4+30=34 -> 36)
        mpdu2 = bytes(range(45))   # needs 3 bytes padding (4+45=49 -> 52)
        mpdu3 = bytes(range(64))   # aligned (4+64=68, already 4-aligned)

        parts = []
        for mpdu in [mpdu1, mpdu2, mpdu3]:
            chunk = self._make_delimiter(len(mpdu)) + mpdu
            parts.append(self._pad_to_4(chunk))
        parts.append(self._make_delimiter(0, eof=True))
        ampdu = b''.join(parts)

        result = parse_ampdu(ampdu)
        assert len(result) == 3
        assert result[0] == mpdu1
        assert result[1] == mpdu2
        assert result[2] == mpdu3

    def test_max_length_mpdu(self):
        """MPDU at maximum delimiter length (4095 bytes)."""
        from py80211.decode_frame import parse_ampdu
        mpdu = bytes(range(256)) * 16  # 4096 bytes, truncate to 4095
        mpdu = mpdu[:4095]
        chunk = self._make_delimiter(len(mpdu)) + mpdu
        ampdu = self._pad_to_4(chunk) + self._make_delimiter(0, eof=True)
        result = parse_ampdu(ampdu)
        assert len(result) == 1
        assert result[0] == mpdu

    def test_invalid_unique_pattern(self):
        """Raises ValueError on bad unique pattern."""
        from py80211.decode_frame import parse_ampdu
        # Construct delimiter with wrong pattern
        bad = b'\x00\x00\x00\x00'  # pattern=0x00, not 0x4E
        with pytest.raises(ValueError, match="unique pattern"):
            parse_ampdu(bad + b'\x00' * 20)

    def test_invalid_crc(self):
        """Raises ValueError on CRC mismatch."""
        from py80211.decode_frame import parse_ampdu
        good = self._make_delimiter(16)
        bad = bytearray(good)
        bad[1] ^= 0x40  # corrupt CRC
        with pytest.raises(ValueError, match="CRC mismatch"):
            parse_ampdu(bytes(bad) + b'\x00' * 20)

    def test_truncated_mpdu(self):
        """Raises ValueError when MPDU extends beyond data."""
        from py80211.decode_frame import parse_ampdu
        delim = self._make_delimiter(1000)
        with pytest.raises(ValueError, match="extends beyond"):
            parse_ampdu(delim + b'\x00' * 10)

    def test_no_valid_mpdus(self):
        """Raises ValueError when only EOF delimiter present."""
        from py80211.decode_frame import parse_ampdu
        with pytest.raises(ValueError, match="No valid MPDU"):
            parse_ampdu(self._make_delimiter(0, eof=True))

    def test_stops_at_eof(self):
        """Ignores data after EOF delimiter."""
        from py80211.decode_frame import parse_ampdu
        mpdu = b'first frame'
        chunk = self._pad_to_4(self._make_delimiter(len(mpdu)) + mpdu)
        # EOF followed by garbage
        ampdu = chunk + self._make_delimiter(0, eof=True) + b'\xff' * 100
        result = parse_ampdu(ampdu)
        assert result == [mpdu]

    def test_realistic_wifi_mpdus(self):
        """Simulates realistic A-MPDU with FCS-bearing MPDUs."""
        from py80211.decode_frame import parse_ampdu
        from py80211.gen_ofdm_frame import compute_fcs

        # Build 3 MPDUs each with their own FCS (as real APs would)
        payloads = [b'frame_one_data', b'frame_two_longer_data', b'f3']
        mpdus = [p + compute_fcs(p) for p in payloads]

        parts = []
        for mpdu in mpdus:
            chunk = self._make_delimiter(len(mpdu)) + mpdu
            parts.append(self._pad_to_4(chunk))
        parts.append(self._make_delimiter(0, eof=True))
        ampdu = b''.join(parts)

        result = parse_ampdu(ampdu)
        assert len(result) == 3
        # Verify each MPDU has valid FCS
        from py80211.gen_ofdm_frame import verify_fcs
        for i, mpdu in enumerate(result):
            assert verify_fcs(mpdu), f"MPDU {i} FCS failed"


class TestFCSFailure:
    """Verify decoder behavior when FCS is invalid."""

    def test_corrupted_payload_returns_fcs_false(self):
        """Corrupted frame returns fcs_ok=False but still returns PSDU."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes(80)
        iq, meta = generate_vht_frame(3, psdu)
        # Phase-flip a chunk in the middle of DATA symbols
        mid = len(iq) // 2
        iq_bad = iq.copy()
        iq_bad[mid:mid+80] *= -1
        result = decode_frame(iq_bad, 0)
        assert result.get("fcs_ok") is False
        assert "psdu" in result  # PSDU still returned for diagnostics


class TestFrameLengthEdges:
    """Verify TX/RX handles extreme PSDU sizes."""

    @pytest.mark.parametrize("psdu_len", [1, 2, 3, 4, 5])
    def test_tiny_psdu(self, psdu_len):
        """Very small PSDUs encode and decode correctly."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
        iq, meta = generate_vht_frame(3, psdu)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"{psdu_len}B PSDU: FCS failed"
        assert result["psdu"][:psdu_len] == psdu

    def test_large_frame_8000_bytes(self):
        """8000-byte frame (multi-codeword LDPC)."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(8000)])
        iq, meta = generate_vht_frame(8, psdu, coding="ldpc")
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], "8000B LDPC MCS 8: FCS failed"
        assert result["psdu"][:8000] == psdu


# ============================================================================
# NDP (Null Data Packet) generation and detection
# ============================================================================

class TestNDP:
    """VHT NDP (Null Data Packet) generation and detection."""

    def test_ndp_generation_structure(self):
        """NDP has preamble fields but no SIG-B or DATA."""
        from py80211.gen_ofdm_frame import generate_vht_ndp

        iq, meta = generate_vht_ndp()
        assert meta["ndp"] is True
        # Structure: L-STF(160) + L-LTF(160) + L-SIG(80) + VHT-SIG-A(160) + VHT-STF(80) + VHT-LTF(80) = 720
        assert len(iq) == 720
        assert meta["lsig_length"] == 9

    def test_ndp_decode_clean(self):
        """Decoder returns ndp=True for clean NDP frames."""
        from py80211.gen_ofdm_frame import generate_vht_ndp
        from py80211.decode_frame import decode_frame

        iq, meta = generate_vht_ndp()
        padded = np.concatenate([np.zeros(100, dtype=np.complex64), iq,
                                 np.zeros(100, dtype=np.complex64)])
        result = decode_frame(padded, 100)
        assert result is not None
        assert result.get("ndp") is True
        assert result["frame_type"] == "vht"
        assert "error" not in result

    def test_ndp_decode_with_noise(self):
        """NDP detected correctly under AWGN."""
        from py80211.gen_ofdm_frame import generate_vht_ndp
        from py80211.decode_frame import decode_frame
        from py80211.impairments import add_awgn

        iq, _ = generate_vht_ndp()
        padded = np.concatenate([np.zeros(100, dtype=np.complex64), iq,
                                 np.zeros(100, dtype=np.complex64)])
        noisy = add_awgn(padded, snr_db=20, seed=42)
        result = decode_frame(noisy, 100)
        assert result is not None
        assert result.get("ndp") is True


# ============================================================================
# MU-MIMO rejection
# ============================================================================

class TestMUMIMORejection:
    """Verify MU-MIMO frames are cleanly rejected."""

    def test_mu_mimo_group_id_rejected(self):
        """Frame with group_id=1 returns explicit MU-MIMO error."""
        from py80211.gen_ofdm_frame import (
            generate_vht_frame, make_vht_sig_a_bits, conv_encode,
            interleave, modulate,
            NFFT, NCP, DATA_BINS, PILOT_BINS, PILOT_BASE, PILOT_POLARITY,
            PREAMBLE_SAMPLES, SYMBOL_SAMPLES,
        )
        from py80211.decode_frame import decode_frame

        # Generate a normal VHT frame
        iq, meta = generate_vht_frame(0, b'\x00' * 20)
        iq = iq.copy()  # make writable

        # Patch VHT-SIG-A to have group_id=1
        vht_sig_a_bits = make_vht_sig_a_bits(
            mcs=0, length=meta["psdu_length"], group_id=1)
        coded = conv_encode(vht_sig_a_bits, add_tail=False)
        coded_sym1 = interleave(coded[:48], 48, 1)
        coded_sym2 = interleave(coded[48:], 48, 1)
        bpsk_sym1 = modulate(coded_sym1, 1)
        bpsk_sym2 = modulate(coded_sym2, 1) * 1j

        sig_a_start = PREAMBLE_SAMPLES + SYMBOL_SAMPLES
        for sym_idx, data_syms in enumerate([bpsk_sym1, bpsk_sym2]):
            freq = np.zeros(NFFT, dtype=complex)
            for i, bin_idx in enumerate(DATA_BINS):
                freq[bin_idx] = data_syms[i]
            pilot_idx = sym_idx + 1
            polarity = PILOT_POLARITY[pilot_idx % 127]
            for pb in PILOT_BINS:
                freq[pb] = polarity * PILOT_BASE[pb]
            time_64 = np.fft.ifft(freq)
            cp = time_64[-NCP:]
            sym = np.concatenate([cp, time_64])
            offset = sig_a_start + sym_idx * SYMBOL_SAMPLES
            iq[offset:offset + SYMBOL_SAMPLES] = sym.astype(np.complex64)

        result = decode_frame(iq, 0)
        assert result is not None
        assert "error" in result
        assert "MU-MIMO" in result["error"]
        assert result["vht_sig_a"]["group_id"] == 1


# ============================================================================
# SGI disambiguation
# ============================================================================

class TestSGIDisambig:
    """VHT SGI disambiguation bit is correctly set and used."""

    def test_disambig_not_set_normal_case(self):
        """Normal SGI frames don't set disambiguation bit."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        # 50 bytes with MCS 0: n_data_bits = 16 + 8*56 + 6 = 470
        # n_sym = ceil(470/26) = 19. 19 % 10 = 9 → TRIGGERS!
        # Use a size that doesn't trigger: 30 bytes → 16+8*36+6=310, ceil(310/26)=12
        iq, meta = generate_vht_frame(0, bytes(30), short_gi=True)
        sig_a_bits = meta["vht_sig_a_bits"]
        assert sig_a_bits[25] == 0, "Disambig should be 0 when n_sym % 10 != 9"

    def test_disambig_set_when_triggered(self):
        """SGI disambiguation bit set when n_sym_init % 10 == 9."""
        from py80211.gen_ofdm_frame import generate_vht_frame, VHT_MCS_TABLE
        import math

        # Find a PSDU size where n_sym_init % 10 == 9 for MCS 0 (n_dbps=26)
        # n_sym_init = ceil((16 + 8*L + 6) / 26) where L = psdu_length with FCS
        # We need ceil((22 + 8*L) / 26) == 9 → 208 < 22 + 8*L ≤ 234
        # → 186 < 8*L ≤ 212 → L = 24..26
        # psdu_length includes FCS (4 bytes), and 4-byte aligned.
        # payload of 20 bytes → padded_payload = 20, +FCS = 24 bytes, 4-aligned → L=24
        # ceil((22 + 192)/26) = ceil(214/26) = ceil(8.23) = 9 ✓
        iq, meta = generate_vht_frame(0, bytes(20), short_gi=True)
        sig_a_bits = meta["vht_sig_a_bits"]
        assert sig_a_bits[25] == 1, "Disambig should be 1 when n_sym_init % 10 == 9"

    def test_disambig_loopback(self):
        """Frame with SGI disambig bit decodes correctly."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        # Use the triggering case
        payload = bytes([i % 256 for i in range(20)])
        iq, meta = generate_vht_frame(0, payload, short_gi=True)
        result = decode_frame(iq, 0)
        assert result is not None
        assert result.get("fcs_ok"), f"FCS failed: {result.get('error')}"
        assert result["psdu"][:20] == payload
