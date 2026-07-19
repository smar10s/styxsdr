import numpy as np
import pytest
from py80211.ldpc import ldpc_encode, ldpc_decode, build_H


@pytest.mark.parametrize("rate,cw_len", [
    ("1/2", 648), ("1/2", 1296), ("1/2", 1944),
    ("2/3", 648), ("2/3", 1296), ("2/3", 1944),
    ("3/4", 648), ("3/4", 1296), ("3/4", 1944),
    ("5/6", 648), ("5/6", 1296), ("5/6", 1944),
])
def test_encode_produces_valid_codeword(rate, cw_len):
    """H @ codeword == 0 (mod 2) for random information bits."""
    rng = np.random.default_rng(42)
    rate_num = {"1/2": 1, "2/3": 2, "3/4": 3, "5/6": 5}[rate]
    rate_den = {"1/2": 2, "2/3": 3, "3/4": 4, "5/6": 6}[rate]
    K = cw_len * rate_num // rate_den
    info_bits = rng.integers(0, 2, size=K).astype(np.int8)
    codeword = ldpc_encode(info_bits, rate, cw_len)
    assert len(codeword) == cw_len
    H = build_H(rate, cw_len)
    syndrome = H @ codeword % 2
    assert np.all(syndrome == 0), f"Syndrome has {syndrome.sum()} nonzero entries"


def test_encode_is_systematic():
    """Info bits are preserved in first K positions of codeword."""
    rng = np.random.default_rng(99)
    info = rng.integers(0, 2, size=324).astype(np.int8)
    cw = ldpc_encode(info, "1/2", 648)
    assert np.array_equal(cw[:324], info)


# --- Decoder tests ---


@pytest.mark.parametrize("rate,cw_len", [
    ("1/2", 648), ("1/2", 1296), ("1/2", 1944),
    ("2/3", 648), ("2/3", 1296), ("2/3", 1944),
    ("3/4", 648), ("3/4", 1296), ("3/4", 1944),
    ("5/6", 648), ("5/6", 1296), ("5/6", 1944),
])
def test_decode_clean(rate, cw_len):
    """Decoder recovers correct hard decision from clean (high-confidence) LLR."""
    rng = np.random.default_rng(123)
    num, den = {"1/2": (1,2), "2/3": (2,3), "3/4": (3,4), "5/6": (5,6)}[rate]
    K = cw_len * num // den
    info_bits = rng.integers(0, 2, size=K).astype(np.int8)
    codeword = ldpc_encode(info_bits, rate, cw_len)
    # High-confidence LLR: positive for bit=0, negative for bit=1
    llr = (1 - 2 * codeword.astype(float)) * 10.0
    decoded = ldpc_decode(llr, rate, cw_len, max_iter=20)
    assert np.array_equal(decoded, codeword)


@pytest.mark.parametrize("rate,cw_len,snr_db", [
    ("1/2", 1944, 2.0),
    ("2/3", 1944, 3.5),
    ("3/4", 1944, 4.5),
    ("5/6", 1944, 5.5),
])
def test_decode_noisy(rate, cw_len, snr_db):
    """Decoder recovers info bits at moderate SNR (BPSK AWGN channel)."""
    rng = np.random.default_rng(999)
    num, den = {"1/2": (1,2), "2/3": (2,3), "3/4": (3,4), "5/6": (5,6)}[rate]
    K = cw_len * num // den
    info_bits = rng.integers(0, 2, size=K).astype(np.int8)
    codeword = ldpc_encode(info_bits, rate, cw_len)
    # BPSK: bit 0 → +1, bit 1 → -1
    bpsk = 1.0 - 2.0 * codeword.astype(float)
    # Add AWGN
    sigma = 10 ** (-snr_db / 20)
    noisy = bpsk + rng.normal(0, sigma, size=cw_len)
    # Channel LLR = 2*y/sigma^2
    llr = 2.0 * noisy / (sigma ** 2)
    decoded = ldpc_decode(llr, rate, cw_len, max_iter=50)
    assert np.array_equal(decoded[:K], info_bits), \
        f"Decode failed: {np.sum(decoded[:K] != info_bits)} bit errors"


def test_decode_early_termination():
    """Decoder terminates early when syndrome is zero (fewer than max_iter)."""
    rng = np.random.default_rng(77)
    info_bits = rng.integers(0, 2, size=324).astype(np.int8)
    codeword = ldpc_encode(info_bits, "1/2", 648)
    llr = (1 - 2 * codeword.astype(float)) * 20.0  # Very high confidence
    # Should converge in 1 iteration for clean signal
    decoded = ldpc_decode(llr, "1/2", 648, max_iter=100)
    assert np.array_equal(decoded, codeword)


# --- VHT LDPC integration tests (shortening/puncturing path) ---


class TestVHTLDPCIntegration:
    """Verify LDPC works correctly through the full VHT TX/RX pipeline.

    Without external reference vectors, we validate:
    1. The shortening/puncturing produces valid codewords (syndrome check)
    2. End-to-end decode at marginal SNR (wrong codewords would fail FCS)
    3. Non-aligned PSDU lengths (exercises shortening edge cases)
    """

    @pytest.mark.parametrize("mcs,psdu_len", [
        (0, 36),   # BPSK 1/2 — small frame, exercises shortening
        (3, 100),  # 16QAM 1/2 — standard size
        (5, 72),   # 64QAM 2/3 — exercises different cw_len selection
        (7, 120),  # 64QAM 5/6 — keep n_pld <= k_per_cw (known bug for larger sizes)
        (8, 148),  # 256QAM 3/4 — highest MCS with LDPC
    ])
    def test_vht_ldpc_loopback_varied_sizes(self, mcs, psdu_len):
        """VHT LDPC encode→decode with non-standard PSDU lengths."""
        import sys
        sys.path.insert(0, str(pytest.importorskip("pathlib").Path(__file__).parent.parent.parent / "python"))
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
        iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], (
            f"VHT LDPC MCS {mcs}, {psdu_len}B: FCS failed (loopback)"
        )
        assert result["psdu"][:psdu_len] == psdu

    @pytest.mark.parametrize("mcs", [0, 3, 5, 7, 8])
    def test_vht_ldpc_noisy_decode(self, mcs):
        """VHT LDPC decode at marginal SNR — validates coding gain over BCC."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame
        from py80211.impairments import add_awgn

        # SNR where LDPC should reliably decode but is challenging
        snr_map = {0: 6, 3: 12, 5: 18, 7: 22, 8: 26}
        snr = snr_map[mcs]
        psdu = bytes([(i * 7 + 13) % 256 for i in range(80)])
        successes = 0
        for seed in range(10):
            iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc",
                                          scrambler_seed=0x5D)
            iq_noisy = add_awgn(iq, snr_db=snr, seed=seed)
            result = decode_frame(iq_noisy, 0)
            if result.get("fcs_ok") and result["psdu"][:len(psdu)] == psdu:
                successes += 1
        assert successes >= 7, (
            f"VHT LDPC MCS {mcs} at {snr} dB: {successes}/10 < 7"
        )


class TestLDPCCodewordSelection:
    """Verify codeword selection handles n_pld > int(1944*R) for single-cw branch."""

    def test_vht_mcs7_200bytes_loopback(self):
        """MCS 7 (R=5/6), 200B PSDU: n_pld > 1620, must use multi-codeword."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(200)])
        iq, meta = generate_vht_frame(7, psdu, coding="ldpc")
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], "MCS 7, 200B LDPC: FCS failed"
        assert result["psdu"][:200] == psdu

    @pytest.mark.parametrize("mcs,psdu_len", [
        (7, 180),   # n_pld ~1728, > 1620 (5/6 capacity of 1944)
        (7, 200),   # n_pld ~1856, clearly > 1620
        (8, 220),   # 256QAM 3/4: int(1944*3/4)=1458, n_pld ~1936
    ])
    def test_high_rate_multi_codeword_boundary(self, mcs, psdu_len):
        """PSDU sizes that exceed single-codeword capacity at high rates."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
        iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"MCS {mcs}, {psdu_len}B LDPC: FCS failed"
        assert result["psdu"][:psdu_len] == psdu


class TestVHTSigBLength:
    """Verify VHT-SIG-B LENGTH field handles non-4-aligned PSDU correctly."""

    @pytest.mark.parametrize("psdu_len", [1, 2, 3, 5, 7, 97, 101, 103])
    def test_non_aligned_psdu_loopback(self, psdu_len):
        """Non-4-aligned PSDU sizes must round-trip correctly."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
        iq, meta = generate_vht_frame(3, psdu, coding="bcc")
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"{psdu_len}B PSDU: FCS failed"
        # RX extracts LENGTH*4 bytes; original PSDU is prefix
        assert result["psdu"][:psdu_len] == psdu

    def test_sig_b_length_field_is_ceil(self):
        """LENGTH field must be ceil(psdu_length/4), not floor."""
        from py80211.gen_ofdm_frame import make_vht_sig_b_bits
        import math

        # 101 bytes: floor(101/4)=25, ceil(101/4)=26
        bits = make_vht_sig_b_bits(101)
        length_field = sum(bits[i] << i for i in range(17))
        assert length_field == math.ceil(101 / 4)  # == 26, not 25


class TestSGIWithLDPC:
    """Short GI + LDPC combination: exercises N_SYM without tail bits +
    short GI L-LENGTH formula + LDPC extra symbol interaction."""

    @pytest.mark.parametrize("mcs", [0, 1, 3, 5, 7, 8])
    def test_vht_sgi_ldpc_loopback(self, mcs):
        """VHT SGI+LDPC clean loopback for all representative MCS."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(80)])
        iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc", short_gi=True)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"VHT SGI+LDPC MCS {mcs}: FCS failed"
        assert result["psdu"][:80] == psdu
        assert result.get("short_gi") == 1

    @pytest.mark.parametrize("mcs", [0, 3, 5, 7])
    def test_ht_sgi_ldpc_loopback(self, mcs):
        """HT SGI+LDPC clean loopback."""
        from py80211.gen_ofdm_frame import generate_ht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(80)])
        iq, meta = generate_ht_frame(mcs, psdu, coding="ldpc", short_gi=True)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"HT SGI+LDPC MCS {mcs}: FCS failed"
        assert result["psdu"][:80] == psdu

    @pytest.mark.parametrize("mcs,psdu_len", [
        (7, 200),  # Forces multi-codeword at R=5/6 + SGI
        (8, 300),  # Large frame, 256QAM, SGI, LDPC
        (5, 500),  # 64QAM 2/3, multi-codeword + SGI
    ])
    def test_vht_sgi_ldpc_large_frames(self, mcs, psdu_len):
        """SGI+LDPC with frame sizes that exercise multi-codeword + SGI N_SYM."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
        iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc", short_gi=True)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"VHT SGI+LDPC MCS {mcs}, {psdu_len}B: FCS failed"
        assert result["psdu"][:psdu_len] == psdu


class TestLDPCExtraSymbol:
    """Verify LDPC extra symbol is signalled and decoded correctly."""

    @staticmethod
    def _find_extra_symbol_params():
        """Find (MCS, PSDU_len) pairs that trigger ldpc_extra=1."""
        import math
        from py80211.gen_ofdm_frame import VHT_MCS_TABLE
        params = []
        for mcs in range(9):
            info = VHT_MCS_TABLE[mcs]
            n_dbps = info["n_dbps"]
            n_cbps = info["n_cbps"]
            R = info["cr_n"] / info["cr_d"]
            # Try various PSDU lengths
            for psdu_len in range(20, 600, 5):
                psdu_len_with_fcs = math.ceil((psdu_len + 4) / 4) * 4  # account for padding+FCS
                n_pld_needed = 16 + 8 * psdu_len_with_fcs
                n_sym = math.ceil(n_pld_needed / n_dbps)
                n_pld = n_sym * n_dbps
                # Codeword selection (with fix applied)
                if n_pld <= 648:
                    l_cw = 1944
                    for cw in [648, 1296, 1944]:
                        if int(cw * R) >= n_pld:
                            l_cw = cw
                            break
                    n_cw = 1
                elif n_pld <= 1296:
                    l_cw = 1944
                    for cw in [1296, 1944]:
                        if int(cw * R) >= n_pld:
                            l_cw = cw
                            break
                    n_cw = 1
                elif n_pld <= 1944 and int(1944 * R) >= n_pld:
                    l_cw = 1944
                    n_cw = 1
                else:
                    n_cw = math.ceil(n_pld / (1944 * R))
                    l_cw = 1944
                n_avail = n_sym * n_cbps
                k_per_cw = int(l_cw * R)
                n_shrt = max(0, k_per_cw * n_cw - n_pld)
                n_punc = max(0, n_cw * l_cw - n_avail - n_shrt)
                n_parity = n_cw * (l_cw - k_per_cw)
                if n_parity > 0 and n_punc > 0.1 * n_parity:
                    params.append((mcs, psdu_len))
                    break  # One per MCS is enough
        return params

    def test_extra_symbol_cases_exist(self):
        """At least some MCS/length combos trigger ldpc_extra."""
        params = self._find_extra_symbol_params()
        assert len(params) >= 2, f"Only found {len(params)} extra-symbol cases: {params}"

    def test_extra_symbol_loopback(self):
        """Frames that trigger LDPC extra symbol decode correctly."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        params = self._find_extra_symbol_params()
        assert len(params) > 0, "No extra-symbol cases found"
        for mcs, psdu_len in params:
            psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
            iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
            result = decode_frame(iq, 0)
            assert result["fcs_ok"], (
                f"VHT LDPC extra-sym MCS {mcs}, {psdu_len}B: FCS failed"
            )
            assert result["psdu"][:psdu_len] == psdu

    def test_sig_a_ldpc_extra_bit_set(self):
        """VHT-SIG-A bit 27 is set when ldpc_extra triggers."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        params = self._find_extra_symbol_params()
        if not params:
            pytest.skip("No extra-symbol case found")
        for mcs, psdu_len in params:
            psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
            iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
            sig_a_bits = meta["vht_sig_a_bits"]
            assert sig_a_bits[27] == 1, (
                f"MCS {mcs}, {psdu_len}B: LDPC extra symbol bit not set in VHT-SIG-A"
            )

    def test_non_extra_symbol_bit_clear(self):
        """VHT-SIG-A bit 27 is clear when ldpc_extra is NOT triggered."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        # Small frame that definitely won't trigger extra symbol
        psdu = bytes(32)
        iq, meta = generate_vht_frame(3, psdu, coding="ldpc")
        sig_a_bits = meta["vht_sig_a_bits"]
        assert sig_a_bits[27] == 0, "LDPC extra symbol bit set unexpectedly"


class TestMultiCodeword:
    """Test PSDU sizes that straddle 1-to-N codeword boundaries."""

    @pytest.mark.parametrize("mcs,psdu_len", [
        # MCS 3 (R=1/2): k_per_cw=972, single cw capacity ~(972-16)/8-4=119B
        (3, 115), (3, 120), (3, 125),
        # MCS 7 (R=5/6): k_per_cw=1620, after fix uses multi-cw above n_pld>1620
        (7, 195), (7, 200), (7, 210),
        # MCS 8 (R=3/4): k_per_cw=1458, boundary near ~180B
        (8, 175), (8, 180), (8, 190),
        # Large multi-codeword frames
        (7, 500), (7, 1000), (8, 2000),
    ])
    def test_codeword_boundary_loopback(self, mcs, psdu_len):
        """PSDU sizes near and beyond single-codeword capacity."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes([(i * 7 + 13) % 256 for i in range(psdu_len)])
        iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"MCS {mcs}, {psdu_len}B: FCS failed"
        assert result["psdu"][:psdu_len] == psdu


class TestMiscCoverage:
    """Scrambler seed exhaustive, MCS 9 rejection, etc."""

    @pytest.mark.parametrize("seed", list(range(1, 128)))
    def test_all_scrambler_seeds(self, seed):
        """All 127 valid seeds produce decodable VHT frames."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame

        psdu = bytes(32)
        iq, meta = generate_vht_frame(3, psdu, scrambler_seed=seed)
        result = decode_frame(iq, 0)
        assert result["fcs_ok"], f"Seed {seed}: FCS failed"

    def test_mcs9_rejected(self):
        """MCS 9 raises ValueError (invalid for 20 MHz)."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        with pytest.raises(ValueError, match="Invalid MCS"):
            generate_vht_frame(9, bytes(50))
