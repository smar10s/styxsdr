"""TX path golden vector tests — IEEE 802.11-2020 Annex I.1 + TGn/TGac reference.

Each test verifies one TX processing block in isolation against the spec's
worked example or the IEEE reference generator output.

Vector provenance:
  Legacy: IEEE 802.11-2020, Annex I, Tables I-1 through I-30.
  HT:    IEEE TGn reference waveform generator (doc 11-06/1715r0).
  VHT:   IEEE TGac reference waveform generator (doc 11-14/0571r10, CSR).
Stored in: vectors/*.json
"""

import math

import numpy as np
import pytest

from conftest import (
    load_vector,
    json_index_to_fft_bin,
    ANNEX_I1_PSDU_BYTES,
    ANNEX_I1_SCRAMBLER_SEED,
)

from py80211.gen_ofdm_frame import (
    RATE_TABLE,
    NFFT,
    NCP,
    DATA_BINS,
    PILOT_BINS,
    PILOT_BASE,
    PILOT_POLARITY,
    _STF_FREQ,
    _LTF_FREQ,
    generate_stf,
    generate_ltf,
    make_signal_bits,
    conv_encode,
    puncture,
    interleave,
    modulate,
    scramble,
    bytes_to_bits_lsb,
    HT_MCS_TABLE,
    HT_DATA_BINS,
    _HT_LTF_FREQ,
    _ht_interleave,
    make_ht_sig_bits,
    VHT_MCS_TABLE,
    VHT_PILOT_OFFSET,
    HT_PILOT_SPEC_BINS,
    _HT_PILOT_PATTERN,
    make_vht_sig_a_bits,
    make_vht_sig_b_bits,
    ht_sig_crc8,
)


# ============================================================================
# Legacy TX tests — Annex I.1 (BCC, 36 Mbps, 100-byte PSDU)
# ============================================================================

class TestRateTable:
    """Verify RATE_TABLE matches IEEE 802.11-2020, Table 17-6."""

    # Table 17-6: R1R2R3R4 notation (R1 is MSB of the 4-char string)
    # We store R1 at bit 0 (LSB of integer), so '1011' → 0b1101
    SPEC_RATES = {
        6:  "1101",
        9:  "1111",
        12: "0101",
        18: "0111",
        24: "1001",
        36: "1011",
        48: "0001",
        54: "0011",
    }

    @pytest.mark.parametrize("rate_mbps,r1r2r3r4", SPEC_RATES.items())
    def test_rate_bits(self, rate_mbps, r1r2r3r4):
        """Each rate's stored bits must produce [R1,R2,R3,R4] matching Table 17-6."""
        stored = RATE_TABLE[rate_mbps]["rate_bits"]
        extracted = "".join(str((stored >> i) & 1) for i in range(4))
        assert extracted == r1r2r3r4, (
            f"Rate {rate_mbps}: expected R1R2R3R4={r1r2r3r4}, "
            f"got {extracted} (stored=0b{stored:04b})"
        )


# ============================================================================
# SIGNAL field generation (Tables I-7, I-8, I-9, I-10, I-11, I-12)
# ============================================================================

class TestSignalField:
    """Verify SIGNAL field processing against Annex I.1."""

    def test_signal_bits(self):
        """make_signal_bits must produce Table I-7 bits for rate=36, length=100."""
        vec = load_vector("annex_i1_signal_bits")
        rate_bits = RATE_TABLE[36]["rate_bits"]
        bits = make_signal_bits(rate_bits, 100)
        assert bits == vec["bits"]

    def test_signal_encoded(self):
        """Convolutional encoding of SIGNAL bits must produce Table I-8."""
        input_bits = load_vector("annex_i1_signal_bits")["bits"]
        coded = conv_encode(input_bits, add_tail=False)
        expected = load_vector("annex_i1_signal_encoded")["bits"]
        assert coded == expected

    def test_signal_interleaved(self):
        """Interleaving encoded SIGNAL bits must produce Table I-9."""
        encoded = load_vector("annex_i1_signal_encoded")["bits"]
        interleaved = interleave(encoded, n_cbps=48, n_bpsc=1)
        expected = load_vector("annex_i1_signal_interleaved")["bits"]
        assert interleaved == expected

    def test_signal_freq_domain(self):
        """BPSK modulation + subcarrier mapping must produce Table I-10."""
        interleaved = load_vector("annex_i1_signal_interleaved")["bits"]
        symbols = modulate(interleaved, mod_order=1)

        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(DATA_BINS):
            freq[bin_idx] = symbols[i]

        expected_data = load_vector("annex_i1_signal_freq")
        expected_data = expected_data.get("subcarriers", expected_data.get("data"))
        for json_idx in range(NFFT):
            entry = expected_data[json_idx]
            if entry is None:
                continue  # pilot position
            exp_re, exp_im = entry
            fft_bin = json_index_to_fft_bin(json_idx)
            got = freq[fft_bin]
            assert abs(got.real - exp_re) < 0.001 and abs(got.imag - exp_im) < 0.001, (
                f"SIGNAL freq subcarrier {json_idx-32}: "
                f"got ({got.real:.4f}, {got.imag:.4f}), expected ({exp_re}, {exp_im})"
            )

    def test_signal_freq_with_pilots(self):
        """BPSK + pilots (polarity p0=1) must produce Table I-11."""
        interleaved = load_vector("annex_i1_signal_interleaved")["bits"]
        symbols = modulate(interleaved, mod_order=1)

        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(DATA_BINS):
            freq[bin_idx] = symbols[i]
        polarity = PILOT_POLARITY[0]
        for pb in PILOT_BINS:
            freq[pb] = polarity * PILOT_BASE[pb]

        vec = load_vector("annex_i1_signal_freq_with_pilots")
        expected_data = vec.get("subcarriers", vec.get("data"))
        for json_idx in range(NFFT):
            exp_re, exp_im = expected_data[json_idx]
            fft_bin = json_index_to_fft_bin(json_idx)
            got = freq[fft_bin]
            assert abs(got.real - exp_re) < 0.001 and abs(got.imag - exp_im) < 0.001, (
                f"SIGNAL freq+pilots subcarrier {json_idx-32}: "
                f"got ({got.real:.4f}, {got.imag:.4f}), expected ({exp_re}, {exp_im})"
            )

    def test_signal_time_domain(self):
        """IFFT of SIGNAL freq domain must produce Table I-12 (windowed)."""
        vec_freq = load_vector("annex_i1_signal_freq_with_pilots")
        expected_freq_data = vec_freq.get("subcarriers", vec_freq.get("data"))
        freq = np.zeros(NFFT, dtype=complex)
        for json_idx in range(NFFT):
            entry = expected_freq_data[json_idx]
            if entry is None:
                continue
            r, i = entry
            freq[json_index_to_fft_bin(json_idx)] = complex(r, i)

        time_64 = np.fft.ifft(freq)
        cp = time_64[-NCP:]
        full_sym = np.concatenate([cp, time_64])

        vec_time = load_vector("annex_i1_signal_time")
        expected_time_data = vec_time.get("samples", vec_time.get("data"))

        # Windowed sample 0: W[0] = 0.5
        exp_re, exp_im = expected_time_data[0]
        got = full_sym[0] * 0.5
        assert abs(got.real - exp_re) < 0.002 and abs(got.imag - exp_im) < 0.002, (
            f"SIGNAL time[0] (windowed): got ({got.real:.4f}, {got.imag:.4f}), "
            f"expected ({exp_re}, {exp_im})"
        )

        # Unwindowed samples 1..79
        for k in range(1, 80):
            exp_re, exp_im = expected_time_data[k]
            got = full_sym[k]
            assert abs(got.real - exp_re) < 0.002 and abs(got.imag - exp_im) < 0.002, (
                f"SIGNAL time[{k}]: got ({got.real:.4f}, {got.imag:.4f}), "
                f"expected ({exp_re}, {exp_im})"
            )


# ============================================================================
# STF generation (Tables I-2, I-3)
# ============================================================================

class TestSTF:
    """Verify STF against Annex I.1."""

    def test_stf_freq_domain(self):
        """Internal _STF_FREQ must match Table I-2."""
        vec = load_vector("annex_i1_stf_freq")
        expected_data = vec.get("subcarriers", vec.get("data"))
        for json_idx in range(NFFT):
            exp_re, exp_im = expected_data[json_idx]
            fft_bin = json_index_to_fft_bin(json_idx)
            got = _STF_FREQ[fft_bin]
            assert abs(got.real - exp_re) < 0.002 and abs(got.imag - exp_im) < 0.002, (
                f"STF freq subcarrier {json_idx-32}: "
                f"got ({got.real:.4f}, {got.imag:.4f}), expected ({exp_re}, {exp_im})"
            )

    def test_stf_time_one_period(self):
        """IFFT of STF freq truncated to 16 samples must match Table I-3."""
        vec = load_vector("annex_i1_stf_time_one_period")
        time_64 = np.fft.ifft(_STF_FREQ)
        one_period = time_64[:16]

        expected_data = vec.get("samples_one_period", vec.get("samples"))
        for k in range(16):
            exp_re, exp_im = expected_data[k]
            got = one_period[k]
            assert abs(got.real - exp_re) < 0.002 and abs(got.imag - exp_im) < 0.002, (
                f"STF period[{k}]: got ({got.real:.4f}, {got.imag:.4f}), "
                f"expected ({exp_re}, {exp_im})"
            )

    def test_stf_full_length(self):
        """generate_stf() must produce 160 samples (10 repetitions of 16)."""
        stf = generate_stf()
        assert len(stf) == 160
        period = stf[:16]
        for i in range(1, 10):
            chunk = stf[16*i:16*(i+1)]
            assert np.allclose(chunk, period, atol=1e-10), f"STF period {i} differs"


# ============================================================================
# LTF generation (Tables I-5, I-6)
# ============================================================================

class TestLTF:
    """Verify LTF against Annex I.1."""

    def test_ltf_freq_domain(self):
        """Internal _LTF_FREQ must match Table I-5."""
        vec = load_vector("annex_i1_ltf_freq")
        expected_data = vec.get("subcarriers", vec.get("data"))
        for json_idx in range(NFFT):
            exp_re, exp_im = expected_data[json_idx]
            fft_bin = json_index_to_fft_bin(json_idx)
            got = _LTF_FREQ[fft_bin]
            assert abs(got.real - exp_re) < 0.001 and abs(got.imag - exp_im) < 0.001, (
                f"LTF freq subcarrier {json_idx-32}: "
                f"got ({got.real:.4f}, {got.imag:.4f}), expected ({exp_re}, {exp_im})"
            )

    def test_ltf_structure(self):
        """generate_ltf() must be 160 samples: 32 GI2 + 64 T1 + 64 T2."""
        ltf = generate_ltf()
        assert len(ltf) == 160
        t1 = ltf[32:96]
        t2 = ltf[96:160]
        assert np.allclose(t1, t2, atol=1e-10), "LTF T1 != T2"
        gi2 = ltf[:32]
        assert np.allclose(gi2, t1[32:], atol=1e-10), "LTF GI2 != last 32 of T1"


# ============================================================================
# DATA field scrambling (Table I-15)
# ============================================================================

class TestDataScramble:
    """Verify scrambler against Annex I.1."""

    def test_scramble(self):
        """Scrambling the padded PSDU+SERVICE must produce Table I-15."""
        psdu_bits = bytes_to_bits_lsb(ANNEX_I1_PSDU_BYTES)

        service_bits = [0] * 16
        tail_bits = [0] * 6
        n_dbps = RATE_TABLE[36]["n_dbps"]  # 144
        all_bits = service_bits + psdu_bits + tail_bits
        n_symbols = math.ceil(len(all_bits) / n_dbps)
        n_pad = n_symbols * n_dbps - len(all_bits)
        all_bits_padded = all_bits + [0] * n_pad

        scrambled = scramble(all_bits_padded, ANNEX_I1_SCRAMBLER_SEED)

        # Zero tail bits after scrambling
        tail_start = 16 + 8 * len(ANNEX_I1_PSDU_BYTES)
        for i in range(6):
            scrambled[tail_start + i] = 0

        expected = load_vector("annex_i1_data_scrambled")["data"]
        assert scrambled == expected


# ============================================================================
# DATA field convolutional encoding (Table I-16)
# ============================================================================

class TestDataEncode:
    """Verify convolutional encoder + puncturer against Annex I.1."""

    def test_encode_and_puncture(self):
        """Encoding + 3/4 puncturing of scrambled DATA must produce Table I-16."""
        scrambled = load_vector("annex_i1_data_scrambled")["data"]
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, cr_n=3, cr_d=4)
        expected = load_vector("annex_i1_data_encoded")["data"]
        assert punctured == expected


# ============================================================================
# DATA field interleaving (Table I-19 — first symbol only)
# ============================================================================

class TestDataInterleave:
    """Verify interleaver against Annex I.1 (first DATA symbol)."""

    def test_interleave_first_symbol(self):
        """Interleaving first 192 encoded bits must produce Table I-19."""
        encoded = load_vector("annex_i1_data_encoded")["data"]
        n_cbps = RATE_TABLE[36]["n_cbps"]  # 192
        n_bpsc = RATE_TABLE[36]["bpsc"]    # 4
        interleaved = interleave(encoded[:n_cbps], n_cbps, n_bpsc)
        expected = load_vector("annex_i1_data_interleaved")["data"]
        assert interleaved == expected


# ============================================================================
# DATA field QAM mapping (Table I-20)
# ============================================================================

class TestDataModulate:
    """Verify 16-QAM modulation against Annex I.1 (first DATA symbol)."""

    def test_qam_mapping(self):
        """16-QAM mapping of interleaved bits must produce Table I-20 data subcarriers."""
        interleaved = load_vector("annex_i1_data_interleaved")["data"]
        symbols = modulate(interleaved, mod_order=4)

        vec = load_vector("annex_i1_data_qam")
        expected_data = vec.get("data", vec.get("subcarriers"))
        for i in range(min(len(symbols), len(expected_data))):
            exp_re, exp_im = expected_data[i]
            got = symbols[i]
            assert abs(got.real - exp_re) < 0.002 and abs(got.imag - exp_im) < 0.002, (
                f"QAM symbol {i}: got ({got.real:.4f}, {got.imag:.4f}), "
                f"expected ({exp_re}, {exp_im})"
            )


# ============================================================================
# DATA field frequency domain with pilots (Table I-20 full)
# ============================================================================

class TestDataFreqDomain:
    """Verify full frequency domain (data + pilots) for first DATA symbol."""

    def test_freq_with_pilots(self):
        """Data subcarriers + pilot insertion must produce Table I-20 full."""
        interleaved = load_vector("annex_i1_data_interleaved")["data"]
        symbols = modulate(interleaved, mod_order=4)

        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(DATA_BINS):
            freq[bin_idx] = symbols[i]

        # First DATA symbol uses pilot polarity index 1 (SIGNAL was index 0)
        polarity = PILOT_POLARITY[1]
        for pb in PILOT_BINS:
            freq[pb] = polarity * PILOT_BASE[pb]

        vec = load_vector("annex_i1_data_freq_with_pilots")
        expected_data = vec.get("subcarriers", vec.get("data"))
        for json_idx in range(NFFT):
            exp_re, exp_im = expected_data[json_idx]
            fft_bin = json_index_to_fft_bin(json_idx)
            got = freq[fft_bin]
            assert abs(got.real - exp_re) < 0.002 and abs(got.imag - exp_im) < 0.002, (
                f"DATA freq subcarrier {json_idx-32}: "
                f"got ({got.real:.4f}, {got.imag:.4f}), expected ({exp_re}, {exp_im})"
            )


# ============================================================================
# HT Golden Vector Tests — IEEE TGn Reference Waveform Generator
# ============================================================================

def ht_build_scrambled_bits(mcs: int) -> list[int]:
    """Reproduce the scrambled bit stream for a given MCS."""
    info = HT_MCS_TABLE[mcs]
    n_dbps = info["n_dbps"]

    service_bits = [0] * 16
    psdu_bits = bytes_to_bits_lsb(ANNEX_I1_PSDU_BYTES)
    tail_bits = [0] * 6
    all_bits = service_bits + psdu_bits + tail_bits

    n_symbols = math.ceil(len(all_bits) / n_dbps)
    n_pad = n_symbols * n_dbps - len(all_bits)
    all_bits_padded = all_bits + [0] * n_pad

    scrambled = scramble(all_bits_padded, ANNEX_I1_SCRAMBLER_SEED)
    # Zero tail bits after scrambling
    tail_start = 16 + 8 * len(ANNEX_I1_PSDU_BYTES)
    for i in range(6):
        if tail_start + i < len(scrambled):
            scrambled[tail_start + i] = 0

    return scrambled


class TestHTParameters:
    """Verify HT_MCS_TABLE matches IEEE reference generator parameters."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_params(self, mcs):
        """N_CBPS, N_DBPS, N_BPSC must match reference generator output."""
        vec = load_vector(f"ht_mcs{mcs}_params")
        info = HT_MCS_TABLE[mcs]
        assert info["n_cbps"] == vec["n_cbps"], f"MCS {mcs}: n_cbps mismatch"
        assert info["n_dbps"] == vec["n_dbps"], f"MCS {mcs}: n_dbps mismatch"
        assert info["bpsc"] == vec["n_bpsc"], f"MCS {mcs}: n_bpsc mismatch"


class TestHTSigBits:
    """Verify HT-SIG field bit generation against reference."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_sig_bits(self, mcs):
        """HT-SIG bits must match reference generator output."""
        vec = load_vector(f"ht_mcs{mcs}_htsig_bits")
        expected = vec["data"]
        # The reference uses PSDU length = 100 (same Annex I.1 PSDU)
        got = make_ht_sig_bits(mcs, length=100)
        assert got == expected, (
            f"MCS {mcs}: HT-SIG bits mismatch at "
            f"indices {[i for i in range(len(got)) if got[i] != expected[i]]}"
        )


class TestHTLTF:
    """Verify HT-LTF frequency-domain values against reference."""

    def test_htltf_freq(self):
        """_HT_LTF_FREQ must match reference generator's HTLTF20."""
        vec = load_vector("ht_mcs0_htltf_freq")
        ref_real = vec["real"]
        ref_imag = vec["imag"]

        for i in range(NFFT):
            got = _HT_LTF_FREQ[i]
            exp = complex(ref_real[i], ref_imag[i])
            assert abs(got.real - exp.real) < 1e-10 and abs(got.imag - exp.imag) < 1e-10, (
                f"HT-LTF bin {i}: got ({got.real}, {got.imag}), "
                f"expected ({exp.real}, {exp.imag})"
            )


class TestHTScramble:
    """Verify scrambler output against reference for all MCS."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_scramble(self, mcs):
        """Scrambled bits must match reference generator output."""
        vec = load_vector(f"ht_mcs{mcs}_scrambled")
        expected = vec["data"]
        got = ht_build_scrambled_bits(mcs)
        assert len(got) == len(expected), (
            f"MCS {mcs}: length mismatch: got {len(got)}, expected {len(expected)}"
        )
        # Compare up to tail start (16 SERVICE + 800 PSDU = 816 bits)
        tail_start = 16 + 8 * len(ANNEX_I1_PSDU_BYTES)
        mismatches = [i for i in range(tail_start) if got[i] != expected[i]]
        assert not mismatches, (
            f"MCS {mcs}: {len(mismatches)} bit mismatches before tail, "
            f"first at index {mismatches[0]}"
        )
        # Pad bits should also match
        pad_start = tail_start + 6
        pad_mismatches = [i for i in range(pad_start, len(got))
                          if got[i] != expected[i]]
        assert not pad_mismatches, (
            f"MCS {mcs}: {len(pad_mismatches)} pad bit mismatches, "
            f"first at index {pad_mismatches[0]}"
        )


class TestHTEncode:
    """Verify BCC encoding + puncturing against reference."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_encode(self, mcs):
        """Encoded+punctured bits must match reference generator output."""
        vec = load_vector(f"ht_mcs{mcs}_encoded")
        expected = vec["data"]
        info = HT_MCS_TABLE[mcs]

        scrambled = ht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])

        compare_len = min(len(punctured), len(expected))
        mismatches = [i for i in range(compare_len) if punctured[i] != expected[i]]
        assert not mismatches, (
            f"MCS {mcs}: {len(mismatches)} bit mismatches in first {compare_len} bits, "
            f"first at index {mismatches[0]}"
        )


class TestHTInterleave:
    """Verify HT interleaver against reference."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_interleave(self, mcs):
        """Interleaved bits (first symbol) must match reference generator output."""
        vec = load_vector(f"ht_mcs{mcs}_interleaved")
        expected_all = vec["data"]
        info = HT_MCS_TABLE[mcs]
        n_cbps = info["n_cbps"]
        n_bpsc = info["bpsc"]

        scrambled = ht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])

        first_sym = punctured[:n_cbps]
        interleaved = _ht_interleave(first_sym, n_cbps, n_bpsc)

        expected_first = expected_all[:n_cbps]
        mismatches = [i for i in range(n_cbps)
                      if interleaved[i] != expected_first[i]]
        assert not mismatches, (
            f"MCS {mcs}: {len(mismatches)} bit mismatches in first symbol, "
            f"first at index {mismatches[0]}"
        )


class TestHTQAM:
    """Verify QAM constellation mapping against reference."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_qam_first_symbol(self, mcs):
        """QAM symbols for first OFDM symbol must match reference."""
        vec = load_vector(f"ht_mcs{mcs}_qam_symbols")
        ref_real = vec["real"]
        ref_imag = vec["imag"]
        info = HT_MCS_TABLE[mcs]
        n_cbps = info["n_cbps"]
        n_bpsc = info["bpsc"]
        mod_order = info["mod_order"]

        scrambled = ht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])
        first_sym = punctured[:n_cbps]
        interleaved = _ht_interleave(first_sym, n_cbps, n_bpsc)
        symbols = modulate(interleaved, mod_order)

        for i in range(52):
            got = symbols[i]
            exp = complex(ref_real[i], ref_imag[i])
            assert abs(got.real - exp.real) < 0.001 and abs(got.imag - exp.imag) < 0.001, (
                f"MCS {mcs}, symbol {i}: got ({got.real:.4f}, {got.imag:.4f}), "
                f"expected ({exp.real:.4f}, {exp.imag:.4f})"
            )


class TestHTFreqDomain:
    """Verify full OFDM frequency-domain symbols (data + pilots) against reference."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_freq_first_symbol(self, mcs):
        """Frequency-domain OFDM symbol (64-pt, with pilots) must match reference."""
        vec = load_vector(f"ht_mcs{mcs}_freq_symbols")
        ref_real = vec["real"]
        ref_imag = vec["imag"]
        info = HT_MCS_TABLE[mcs]
        n_cbps = info["n_cbps"]
        n_bpsc = info["bpsc"]
        mod_order = info["mod_order"]

        scrambled = ht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])
        first_sym = punctured[:n_cbps]
        interleaved = _ht_interleave(first_sym, n_cbps, n_bpsc)
        symbols = modulate(interleaved, mod_order)

        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(HT_DATA_BINS):
            freq[bin_idx] = symbols[i]

        # HT-DATA symbol 0 uses pilot polarity index 3
        pilot_offset = 3
        polarity = PILOT_POLARITY[pilot_offset % 127]
        for pb in PILOT_BINS:
            freq[pb] = polarity * PILOT_BASE[pb]

        for i in range(NFFT):
            got = freq[i]
            exp = complex(ref_real[i], ref_imag[i])
            assert abs(got.real - exp.real) < 0.001 and abs(got.imag - exp.imag) < 0.001, (
                f"MCS {mcs}, bin {i}: got ({got.real:.4f}, {got.imag:.4f}), "
                f"expected ({exp.real:.4f}, {exp.imag:.4f})"
            )


# ============================================================================
# VHT Golden Vector Tests — IEEE TGac Reference Waveform Generator
# Source: IEEE 802.11 TGac doc 11-14/0571r10 (CSR reference implementation)
# ============================================================================

def vht_build_scrambled_bits(mcs: int) -> list[int]:
    """Reproduce the scrambled bit stream for VHT DATA at given MCS.

    VHT SERVICE field carries CRC-8 of VHT-SIG-B in bits 8-15
    (MSB-first, ones-complemented, same polynomial as HT-SIG CRC).

    VHT scrambling convention (TGac bcc_encoder.m): scramble SERVICE +
    PSDU + PAD (no tail in scrambler input), then insert 6 zero tail bits.
    """
    info = VHT_MCS_TABLE[mcs]
    n_dbps = info["n_dbps"]

    # VHT SERVICE field: bits 0-7 zeros, bits 8-15 = CRC-8 of VHT-SIG-B
    psdu_length = len(ANNEX_I1_PSDU_BYTES)  # 100 bytes (includes FCS)
    service_bits = [0] * 16
    sigb_bits_for_crc = make_vht_sig_b_bits(psdu_length)[:20]
    sigb_crc = ht_sig_crc8(sigb_bits_for_crc)
    sigb_crc_inv = (~sigb_crc) & 0xFF
    for i in range(8):
        service_bits[8 + i] = (sigb_crc_inv >> (7 - i)) & 1

    psdu_bits = bytes_to_bits_lsb(ANNEX_I1_PSDU_BYTES)

    # TGac convention: scramble SERVICE + PSDU + PAD (no tail), then append 6 zeros
    n_data_bits = 16 + 8 * psdu_length
    n_symbols = math.ceil((n_data_bits + 6) / n_dbps)
    n_total = n_symbols * n_dbps
    n_pad = n_total - n_data_bits - 6

    scrambler_input = service_bits + psdu_bits + [0] * n_pad
    scrambled = scramble(scrambler_input, ANNEX_I1_SCRAMBLER_SEED) + [0] * 6

    return scrambled


class TestVHTParameters:
    """Verify VHT_MCS_TABLE against TGac golden parameter vectors."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_params(self, mcs):
        """N_SD=52, N_SP=4, and N_SYM must match reference generator output."""
        vec = load_vector(f"vht_mcs{mcs}_params")
        info = VHT_MCS_TABLE[mcs]
        n_dbps = info["n_dbps"]

        # n_sd = number of data subcarriers = 52
        assert vec["n_sd"] == 52, f"MCS {mcs}: expected n_sd=52, got {vec['n_sd']}"
        # n_sp = number of pilot subcarriers = 4
        assert vec["n_sp"] == 4, f"MCS {mcs}: expected n_sp=4, got {vec['n_sp']}"
        # n_sym = ceil((16 + 8*length_bytes + 6) / n_dbps)
        length_bytes = vec["length_bytes"]
        expected_n_sym = math.ceil((16 + 8 * length_bytes + 6) / n_dbps)
        assert vec["n_sym"] == expected_n_sym, (
            f"MCS {mcs}: n_sym mismatch: vector={vec['n_sym']}, "
            f"computed={expected_n_sym} (n_dbps={n_dbps})"
        )


class TestVHTSigABits:
    """Verify make_vht_sig_a_bits against TGac golden vectors."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_sig_a_bits(self, mcs):
        """VHT-SIG-A bits must match reference generator output."""
        vec = load_vector(f"vht_mcs{mcs}_vhtsiga_bits")
        expected = vec["data"]
        got = make_vht_sig_a_bits(mcs, 100)
        mismatches = [i for i in range(min(len(got), len(expected)))
                      if got[i] != expected[i]]
        assert not mismatches, (
            f"MCS {mcs}: VHT-SIG-A bits mismatch at indices {mismatches}"
        )


class TestVHTSigBBits:
    """Verify make_vht_sig_b_bits against TGac golden vectors."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_sig_b_bits(self, mcs):
        """VHT-SIG-B bits must match reference generator output."""
        vec = load_vector(f"vht_mcs{mcs}_vhtsigb_bits")
        expected = vec["data"]
        got = make_vht_sig_b_bits(100)
        mismatches = [i for i in range(min(len(got), len(expected)))
                      if got[i] != expected[i]]
        assert not mismatches, (
            f"MCS {mcs}: VHT-SIG-B bits mismatch at indices {mismatches}"
        )


class TestVHTScramble:
    """Verify scrambler output for VHT DATA against reference."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_scramble(self, mcs):
        """Scrambled bits must match reference generator output."""
        vec = load_vector(f"vht_mcs{mcs}_scrambled")
        expected = vec["data"]
        got = vht_build_scrambled_bits(mcs)
        assert len(got) == len(expected), (
            f"MCS {mcs}: length mismatch: got {len(got)}, expected {len(expected)}"
        )
        # Compare up to tail start (16 SERVICE + 800 PSDU = 816 bits)
        tail_start = 16 + 8 * len(ANNEX_I1_PSDU_BYTES)
        mismatches = [i for i in range(tail_start) if got[i] != expected[i]]
        assert not mismatches, (
            f"MCS {mcs}: {len(mismatches)} bit mismatches before tail, "
            f"first at index {mismatches[0]}"
        )
        # Pad bits should also match
        pad_start = tail_start + 6
        pad_mismatches = [i for i in range(pad_start, len(got))
                          if got[i] != expected[i]]
        assert not pad_mismatches, (
            f"MCS {mcs}: {len(pad_mismatches)} pad bit mismatches, "
            f"first at index {pad_mismatches[0]}"
        )


class TestVHTEncode:
    """Verify BCC encoding + puncturing for VHT DATA against reference."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_encode(self, mcs):
        """Encoded+punctured bits must match reference generator output."""
        vec = load_vector(f"vht_mcs{mcs}_encoded")
        expected = vec["data"]
        info = VHT_MCS_TABLE[mcs]

        scrambled = vht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])

        compare_len = min(len(punctured), len(expected))
        mismatches = [i for i in range(compare_len) if punctured[i] != expected[i]]
        assert not mismatches, (
            f"MCS {mcs}: {len(mismatches)} bit mismatches in first {compare_len} bits, "
            f"first at index {mismatches[0]}"
        )


class TestVHTInterleave:
    """Verify HT interleaver for VHT DATA against reference."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_interleave(self, mcs):
        """Interleaved bits (first symbol) must match reference generator output."""
        vec = load_vector(f"vht_mcs{mcs}_interleaved")
        expected_all = vec["data"]
        info = VHT_MCS_TABLE[mcs]
        n_cbps = info["n_cbps"]
        n_bpsc = info["bpsc"]

        scrambled = vht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])

        first_sym = punctured[:n_cbps]
        interleaved = _ht_interleave(first_sym, n_cbps, n_bpsc)

        expected_first = expected_all[:n_cbps]
        mismatches = [i for i in range(n_cbps)
                      if interleaved[i] != expected_first[i]]
        assert not mismatches, (
            f"MCS {mcs}: {len(mismatches)} bit mismatches in first symbol, "
            f"first at index {mismatches[0]}"
        )


class TestVHTQAM:
    """Verify QAM constellation mapping for VHT DATA against reference."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_qam_first_symbol(self, mcs):
        """QAM symbols for first OFDM symbol must match reference."""
        vec = load_vector(f"vht_mcs{mcs}_qam_symbols")
        ref_real = vec["real"]
        ref_imag = vec["imag"]
        info = VHT_MCS_TABLE[mcs]
        n_cbps = info["n_cbps"]
        n_bpsc = info["bpsc"]
        mod_order = info["mod_order"]

        scrambled = vht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])
        first_sym = punctured[:n_cbps]
        interleaved = _ht_interleave(first_sym, n_cbps, n_bpsc)
        symbols = modulate(interleaved, mod_order)

        for i in range(52):
            got = symbols[i]
            exp = complex(ref_real[i], ref_imag[i])
            assert abs(got.real - exp.real) < 0.001 and abs(got.imag - exp.imag) < 0.001, (
                f"MCS {mcs}, symbol {i}: got ({got.real:.4f}, {got.imag:.4f}), "
                f"expected ({exp.real:.4f}, {exp.imag:.4f})"
            )


class TestVHTFreqDomain:
    """Verify full OFDM frequency-domain symbols (data + pilots) for VHT DATA.

    VHT freq_symbols vectors use spec subcarrier ordering (index 0 = sc -32,
    index 32 = DC, index 63 = sc +31) and are normalized by 1/sqrt(N_active)
    where N_active = 56 (52 data + 4 pilot subcarriers).
    """

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_freq_first_symbol(self, mcs):
        """Frequency-domain OFDM symbol (with pilots) must match reference."""
        vec = load_vector(f"vht_mcs{mcs}_freq_symbols")
        ref_real = vec["real"]
        ref_imag = vec["imag"]
        info = VHT_MCS_TABLE[mcs]
        n_cbps = info["n_cbps"]
        n_bpsc = info["bpsc"]
        mod_order = info["mod_order"]

        scrambled = vht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])
        first_sym = punctured[:n_cbps]
        interleaved = _ht_interleave(first_sym, n_cbps, n_bpsc)
        symbols = modulate(interleaved, mod_order)

        # Build freq in FFT bin order
        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(HT_DATA_BINS):
            freq[bin_idx] = symbols[i]

        # VHT-DATA symbol 0 pilots
        polarity = PILOT_POLARITY[(VHT_PILOT_OFFSET + 0) % 127]
        for k, pb in enumerate(HT_PILOT_SPEC_BINS):
            freq[pb] = polarity * _HT_PILOT_PATTERN[(0 + k) % 4]

        # Convert from FFT bin order to spec subcarrier order for comparison
        # Spec order: index i = subcarrier (i - 32), so sc k -> index (k + 32)
        # FFT bin order: bin = sc % 64
        # Mapping: spec_index = (fft_bin + 32) % 64 ... actually:
        # sc = fft_bin if fft_bin < 32, else fft_bin - 64
        # spec_index = sc + 32
        # So spec_index = fft_bin + 32 if fft_bin < 32, else fft_bin - 32
        norm = 1.0 / math.sqrt(56)  # TGac normalization factor

        for fft_bin in range(NFFT):
            sc = fft_bin if fft_bin < 32 else fft_bin - 64
            spec_idx = sc + 32
            got = freq[fft_bin] * norm
            exp = complex(ref_real[spec_idx], ref_imag[spec_idx])
            assert abs(got.real - exp.real) < 0.002 and abs(got.imag - exp.imag) < 0.002, (
                f"MCS {mcs}, sc {sc:+d} (fft_bin={fft_bin}, spec_idx={spec_idx}): "
                f"got ({got.real:.4f}, {got.imag:.4f}), "
                f"expected ({exp.real:.4f}, {exp.imag:.4f})"
            )

    @pytest.mark.parametrize("mcs", [0, 4, 7, 8])
    def test_vht_freq_multi_symbol(self, mcs):
        """Verify pilot cyclic rotation across symbols 0-3 matches reference.

        Symbols 0-3 cover one full cycle of _HT_PILOT_PATTERN = [1,1,1,-1],
        confirming the (k+n)%4 rotation and polarity sequence advance.
        """
        vec = load_vector(f"vht_mcs{mcs}_freq_symbols")
        ref_real = vec["real"]
        ref_imag = vec["imag"]
        info = VHT_MCS_TABLE[mcs]
        n_cbps = info["n_cbps"]
        n_bpsc = info["bpsc"]
        mod_order = info["mod_order"]

        scrambled = vht_build_scrambled_bits(mcs)
        coded = conv_encode(scrambled, add_tail=False)
        punctured = puncture(coded, info["cr_n"], info["cr_d"])
        norm = 1.0 / math.sqrt(56)

        # Number of available symbols (MCS 8 has only 3)
        n_sym = len(ref_real) // NFFT
        n_check = min(4, n_sym)

        for sym_idx in range(n_check):
            sym_start = sym_idx * n_cbps
            sym_coded = punctured[sym_start:sym_start + n_cbps]
            interleaved = _ht_interleave(sym_coded, n_cbps, n_bpsc)
            symbols = modulate(interleaved, mod_order)

            freq = np.zeros(NFFT, dtype=complex)
            for i, bin_idx in enumerate(HT_DATA_BINS):
                freq[bin_idx] = symbols[i]

            # VHT pilot formula for symbol sym_idx
            polarity = PILOT_POLARITY[(VHT_PILOT_OFFSET + sym_idx) % 127]
            for k, pb in enumerate(HT_PILOT_SPEC_BINS):
                freq[pb] = polarity * _HT_PILOT_PATTERN[(sym_idx + k) % 4]

            # Compare against reference (spec subcarrier order, normalized)
            ref_offset = sym_idx * NFFT
            for fft_bin in range(NFFT):
                sc = fft_bin if fft_bin < 32 else fft_bin - 64
                spec_idx = sc + 32
                got = freq[fft_bin] * norm
                exp = complex(ref_real[ref_offset + spec_idx],
                              ref_imag[ref_offset + spec_idx])
                assert abs(got.real - exp.real) < 0.002 and \
                       abs(got.imag - exp.imag) < 0.002, (
                    f"MCS {mcs} sym {sym_idx}, sc {sc:+d}: "
                    f"got ({got.real:.4f}, {got.imag:.4f}), "
                    f"expected ({exp.real:.4f}, {exp.imag:.4f})"
                )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])


# ============================================================================
# HT Short GI TX tests
# ============================================================================

# The reference generator treats the 100-byte ANNEX_I1_PSDU as the complete PSDU
# (including FCS). Our functions add FCS internally, so we pass the first 96 bytes
# and let the code append the matching 4-byte FCS to produce the same 100-byte PSDU.
_SGI_PSDU_PAYLOAD = ANNEX_I1_PSDU_BYTES[:96]


class TestHTShortGI:
    """Verify HT short-GI DATA symbols match IEEE TGn reference vectors."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_sgi_symbol_length(self, mcs):
        """HT short-GI DATA symbols must be 72 samples (NFFT + NCP_SHORT)."""
        from py80211.gen_ofdm_frame import make_ht_data_symbols, NCP_SHORT, NFFT

        data_symbols, _, _ = make_ht_data_symbols(
            _SGI_PSDU_PAYLOAD, mcs, ANNEX_I1_SCRAMBLER_SEED, short_gi=True
        )

        for i, sym in enumerate(data_symbols):
            assert len(sym) == NFFT + NCP_SHORT, (
                f"MCS {mcs}, symbol {i}: expected {NFFT + NCP_SHORT} samples, got {len(sym)}"
            )

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_sgi_waveform(self, mcs):
        """Complete HT short-GI frame waveform matches IEEE reference."""
        from py80211.gen_ofdm_frame import generate_ht_frame, NFFT

        vec = load_vector(f"ht_mcs{mcs}_sgi_waveform")
        expected = np.array(vec["real"]) + 1j * np.array(vec["imag"])

        iq, meta = generate_ht_frame(
            mcs=mcs,
            psdu_bytes=_SGI_PSDU_PAYLOAD,
            scrambler_seed=ANNEX_I1_SCRAMBLER_SEED,
            short_gi=True,
        )

        got = iq.astype(np.complex128)

        assert len(got) == len(expected), (
            f"MCS {mcs}: waveform length mismatch: got {len(got)}, expected {len(expected)}"
        )

        # The reference waveform applies per-field normalization:
        #   Legacy fields (STF, LTF, L-SIG, HT-SIG, HT-STF): NFFT/sqrt(52)
        #   HT fields (HT-LTF, DATA): NFFT/sqrt(56)
        norm_legacy = NFFT / math.sqrt(52)
        norm_ht = NFFT / math.sqrt(56)
        scaled = np.empty_like(got)
        scaled[:640] = got[:640] * norm_legacy   # Legacy preamble + HT-STF
        scaled[640:] = got[640:] * norm_ht       # HT-LTF + DATA

        np.testing.assert_allclose(scaled, expected, atol=1e-6,
            err_msg=f"MCS {mcs} HT SGI waveform mismatch")

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_sgi_htsig_bits(self, mcs):
        """HT-SIG bits have short_gi flag set correctly and match golden vector."""
        from py80211.gen_ofdm_frame import generate_ht_frame

        vec = load_vector(f"ht_mcs{mcs}_sgi_htsig_bits")
        ref_bits = vec["data"]

        _, meta = generate_ht_frame(
            mcs=mcs,
            psdu_bytes=_SGI_PSDU_PAYLOAD,
            scrambler_seed=ANNEX_I1_SCRAMBLER_SEED,
            short_gi=True,
        )

        # HT-SIG bit 31 is the short GI flag
        assert meta["ht_sig_bits"][31] == 1, "HT-SIG short_gi bit should be 1"

        # Full 48-bit comparison with golden vector
        assert meta["ht_sig_bits"] == ref_bits, (
            f"MCS {mcs}: HT-SIG bits mismatch with golden vector"
        )


# ============================================================================
# VHT Short GI TX tests
# ============================================================================

class TestVHTShortGI:
    """Verify VHT short-GI signalling against IEEE TGac reference vectors."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_sgi_symbol_length(self, mcs):
        """VHT short-GI DATA symbols must be 72 samples (NFFT + NCP_SHORT)."""
        from py80211.gen_ofdm_frame import make_vht_data_symbols, NCP_SHORT, NFFT

        data_symbols, _, _, _, _ = make_vht_data_symbols(
            _SGI_PSDU_PAYLOAD, mcs, ANNEX_I1_SCRAMBLER_SEED, short_gi=True
        )

        for i, sym in enumerate(data_symbols):
            assert len(sym) == NFFT + NCP_SHORT, (
                f"MCS {mcs}, symbol {i}: expected {NFFT + NCP_SHORT} samples, got {len(sym)}"
            )

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_sgi_vhtsiga_bits(self, mcs):
        """VHT-SIG-A bits match reference with short_gi=1."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        vec = load_vector(f"vht_mcs{mcs}_sgi_vhtsiga_bits")
        ref_bits = vec["data"]

        _, meta = generate_vht_frame(
            mcs=mcs,
            psdu_bytes=_SGI_PSDU_PAYLOAD,
            scrambler_seed=ANNEX_I1_SCRAMBLER_SEED,
            short_gi=True,
        )

        # VHT-SIG-A bit 24 is the short GI flag
        assert meta["vht_sig_a_bits"][24] == 1, "VHT-SIG-A short_gi bit should be 1"

        # Full 48-bit comparison with golden vector
        assert meta["vht_sig_a_bits"] == ref_bits, (
            f"MCS {mcs}: VHT-SIG-A bits mismatch with golden vector"
        )

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_sgi_metadata(self, mcs):
        """VHT short-GI frame metadata is populated correctly."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        _, meta = generate_vht_frame(
            mcs=mcs,
            psdu_bytes=_SGI_PSDU_PAYLOAD,
            scrambler_seed=ANNEX_I1_SCRAMBLER_SEED,
            short_gi=True,
        )

        assert meta["short_gi"] is True
        assert meta["psdu_length"] == 100  # 96 payload + 4 FCS


# ============================================================================
# HT LDPC TX tests
# ============================================================================

class TestHTLDPCGeneration:
    """HT frame generation with LDPC coding."""

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_ldpc_generates_frame(self, mcs):
        """HT LDPC frame generates without error."""
        from py80211.gen_ofdm_frame import generate_ht_frame

        psdu = bytes(range(100))
        iq, meta = generate_ht_frame(mcs, psdu, coding="ldpc")
        assert len(iq) > 0
        assert meta["coding"] == "ldpc"

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_ldpc_sig_bit_set(self, mcs):
        """HT-SIG FEC coding bit is set for LDPC."""
        from py80211.gen_ofdm_frame import generate_ht_frame

        psdu = bytes(range(100))
        iq, meta = generate_ht_frame(mcs, psdu, coding="ldpc")
        # HT-SIG bit 30 should be 1 for LDPC
        assert meta["ht_sig_bits"][30] == 1

    @pytest.mark.parametrize("mcs", range(8))
    def test_ht_bcc_sig_bit_clear(self, mcs):
        """HT-SIG FEC coding bit is clear for BCC (backward compat)."""
        from py80211.gen_ofdm_frame import generate_ht_frame

        psdu = bytes(range(100))
        iq, meta = generate_ht_frame(mcs, psdu, coding="bcc")
        # HT-SIG bit 30 should be 0 for BCC
        assert meta["ht_sig_bits"][30] == 0


# ============================================================================
# VHT LDPC TX tests
# ============================================================================

class TestVHTLDPCGeneration:
    """VHT frame generation with LDPC coding."""

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_ldpc_generates_frame(self, mcs):
        """VHT LDPC frame generates without error."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        psdu = bytes(range(100))
        iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
        assert len(iq) > 0
        assert meta["coding"] == "ldpc"

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_ldpc_sig_a_bit_set(self, mcs):
        """VHT-SIG-A coding bit is set for LDPC."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        psdu = bytes(range(100))
        iq, meta = generate_vht_frame(mcs, psdu, coding="ldpc")
        # VHT-SIG-A bit 26 should be 1 for LDPC
        assert meta["vht_sig_a_bits"][26] == 1

    @pytest.mark.parametrize("mcs", range(9))
    def test_vht_bcc_sig_a_bit_clear(self, mcs):
        """VHT-SIG-A coding bit is clear for BCC (backward compat)."""
        from py80211.gen_ofdm_frame import generate_vht_frame

        psdu = bytes(range(100))
        iq, meta = generate_vht_frame(mcs, psdu, coding="bcc")
        # VHT-SIG-A bit 26 should be 0 for BCC
        assert meta["vht_sig_a_bits"][26] == 0


# ============================================================================
# VHT-STF/LTF golden vector tests — extracted from TGac waveform
# ============================================================================

class TestVHTTrainingFields:
    """Verify VHT-STF and VHT-LTF against TGac reference waveform (11-14/0571r10).

    The TGac waveform has 1 leading sample trimmed. The standalone generators
    produce isolated symbols; comparing from sample [1:] matches the reference
    exactly (to within floating point).

    Normalization:
      VHT-STF: NFFT/sqrt(52) — uses 12 active tones (same as L-STF)
      VHT-LTF: NFFT/sqrt(56) — uses 56 active tones
    """

    def test_vht_stf_matches_reference(self):
        """VHT-STF time-domain samples match TGac waveform to machine precision."""
        from py80211.gen_ofdm_frame import generate_vht_stf, NFFT

        vec = load_vector("vht_mcs0_waveform")
        ref = np.array(vec["real"]) + 1j * np.array(vec["imag"])

        # VHT-STF occupies samples 560-639 in the full PPDU.
        # Reference has 1 leading sample trimmed, so ref[560:639] corresponds
        # to our samples [1:80] (skip the first sample which is CP from prior symbol).
        norm_legacy = NFFT / math.sqrt(52)
        our_stf = generate_vht_stf().astype(np.complex128) * norm_legacy

        # Compare samples [1:] of our generator vs ref[560:639]
        np.testing.assert_allclose(
            our_stf[1:], ref[560:639], atol=1e-10,
            err_msg="VHT-STF mismatch vs TGac reference waveform"
        )

    def test_vht_ltf_matches_reference(self):
        """VHT-LTF time-domain samples match TGac waveform to machine precision."""
        from py80211.gen_ofdm_frame import generate_vht_ltf, NFFT

        vec = load_vector("vht_mcs0_waveform")
        ref = np.array(vec["real"]) + 1j * np.array(vec["imag"])

        # VHT-LTF occupies samples 640-719 in the full PPDU.
        norm_vht = NFFT / math.sqrt(56)
        our_ltf = generate_vht_ltf().astype(np.complex128) * norm_vht

        # Compare samples [1:] vs ref[640:719]
        np.testing.assert_allclose(
            our_ltf[1:], ref[640:719], atol=1e-10,
            err_msg="VHT-LTF mismatch vs TGac reference waveform"
        )

    def test_vht_stf_periodicity(self):
        """VHT-STF must have 16-sample periodicity (same structure as L-STF)."""
        from py80211.gen_ofdm_frame import generate_vht_stf, NCP

        stf = generate_vht_stf()
        # The 64-sample FFT core (after CP) should repeat every 16 samples
        core = stf[NCP:]  # 64 samples
        for offset in [16, 32, 48]:
            np.testing.assert_allclose(
                core[:16], core[offset:offset + 16], atol=1e-14,
                err_msg=f"VHT-STF 16-sample periodicity broken at offset {offset}"
            )

    def test_vht_ltf_56_active_subcarriers(self):
        """VHT-LTF must have 56 active subcarriers (±28, excluding DC)."""
        from py80211.gen_ofdm_frame import generate_vht_ltf, NCP, NFFT

        ltf = generate_vht_ltf()
        core = ltf[NCP:]  # 64-sample FFT window
        freq = np.fft.fft(core)

        # Active subcarriers: bins 1-28 and 36-63 (subcarriers +1..+28 and -28..-1)
        active_bins = list(range(1, 29)) + list(range(36, 64))
        null_bins = [0, 29, 30, 31, 32, 33, 34, 35]  # DC + outer guard

        for b in active_bins:
            assert abs(freq[b]) > 0.01, f"VHT-LTF bin {b} should be active"
        for b in null_bins:
            assert abs(freq[b]) < 1e-10, f"VHT-LTF bin {b} should be null"


class TestTXWindowing:
    """TX spectral windowing reduces out-of-band leakage."""

    def test_windowed_output_decodes(self):
        """Windowed VHT frame still decodes correctly via loopback."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        from py80211.decode_frame import decode_frame
        import numpy as np

        payload = bytes([i % 256 for i in range(50)])
        iq, meta = generate_vht_frame(4, payload, windowing=2)
        padded = np.concatenate([np.zeros(100, dtype=np.complex64), iq,
                                 np.zeros(100, dtype=np.complex64)])
        result = decode_frame(padded, 100)
        assert result is not None
        assert result.get("fcs_ok"), f"FCS failed: {result.get('error')}"

    def test_windowing_reduces_sidelobes(self):
        """Windowed waveform modifies only CP boundary regions."""
        from py80211.gen_ofdm_frame import generate_vht_frame, apply_tx_windowing, make_vht_data_symbols
        import numpy as np

        payload = bytes(200)

        # Verify windowing modifies the signal at symbol boundaries
        data_symbols, _, _, _, _ = make_vht_data_symbols(payload, 4, 0x5D)
        plain = np.concatenate(data_symbols)
        windowed = apply_tx_windowing(data_symbols, W=2)

        # Output should be same length
        assert len(windowed) == len(plain)

        # Windowed should differ from plain (at CP boundaries)
        assert not np.array_equal(windowed, plain)

        # Modifications should be localized to 2*W samples at each boundary
        diff = np.abs(windowed - plain)
        n_modified = np.sum(diff > 1e-10)
        n_sym = len(data_symbols)
        sym_len = len(data_symbols[0])
        # At most 2*W samples per inter-symbol boundary (n_sym - 1 boundaries)
        max_expected = 2 * 2 * (n_sym - 1)
        assert n_modified <= max_expected + 4, (
            f"Too many samples modified: {n_modified}, expected <= {max_expected}")

        # Modifications should NOT touch the FFT period (last 64 samples of each sym)
        # They should only affect the CP region (first NCP samples)
        ncp = sym_len - 64
        for i in range(n_sym):
            fft_start = i * sym_len + ncp
            fft_end = fft_start + 64
            fft_diff = diff[fft_start:fft_end]
            assert np.max(fft_diff) < 1e-10, (
                f"FFT period of symbol {i} was modified by windowing")

    def test_windowing_zero_is_noop(self):
        """windowing=0 produces identical output to default."""
        from py80211.gen_ofdm_frame import generate_vht_frame
        import numpy as np

        payload = bytes(50)
        iq_default, _ = generate_vht_frame(4, payload)
        iq_w0, _ = generate_vht_frame(4, payload, windowing=0)
        np.testing.assert_array_equal(iq_default, iq_w0)
