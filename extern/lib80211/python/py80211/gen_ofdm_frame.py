"""802.11 OFDM frame generator — Python oracle.

Generates baseband IQ samples for legacy 802.11a (all 8 rates) and
HT-mixed 802.11n (MCS 0-7, 20 MHz, 1 SS) frames.  Serves as the
correctness oracle for the C implementation.

Reference: IEEE 802.11-2020 Sections 17, 19.
"""

import math
import struct
import zlib

import numpy as np

from .ldpc import ldpc_encode

# ============================================================================
# 802.11a Rate Table (Table 17-3)
#   code  mod_order  cr_n  cr_d  n_cbps  n_dbps  rate_mbps
# ============================================================================
RATE_TABLE = {
    # rate_bits: 4-bit field as transmitted over the air (R1 at bit 0 = LSB).
    # IEEE 802.11-2020 Table 17-4 lists R1–R4 left-to-right (MSB notation).
    # We store R1 at LSB so that make_signal_bits can extract with (>> i) & 1
    # and signal_bits[0] = R1 (first transmitted bit).
    #
    # Table 17-4 mapping (R1R2R3R4 → stored value):
    #   6 Mbps: 1101 → 0b1011,  9 Mbps: 1111 → 0b1111
    #  12 Mbps: 0101 → 0b1010, 18 Mbps: 0111 → 0b1110
    #  24 Mbps: 1001 → 0b1001, 36 Mbps: 1011 → 0b1101
    #  48 Mbps: 0001 → 0b1000, 54 Mbps: 0011 → 0b1100
    6:  {"rate_bits": 0b1011, "mod_order": 1, "cr_n": 1, "cr_d": 2,
         "n_cbps": 48,  "n_dbps": 24,   "bpsc": 1},
    9:  {"rate_bits": 0b1111, "mod_order": 1, "cr_n": 3, "cr_d": 4,
         "n_cbps": 48,  "n_dbps": 36,   "bpsc": 1},
    12: {"rate_bits": 0b1010, "mod_order": 2, "cr_n": 1, "cr_d": 2,
         "n_cbps": 96,  "n_dbps": 48,   "bpsc": 2},
    18: {"rate_bits": 0b1110, "mod_order": 2, "cr_n": 3, "cr_d": 4,
         "n_cbps": 96,  "n_dbps": 72,   "bpsc": 2},
    24: {"rate_bits": 0b1001, "mod_order": 4, "cr_n": 1, "cr_d": 2,
         "n_cbps": 192, "n_dbps": 96,   "bpsc": 4},
    36: {"rate_bits": 0b1101, "mod_order": 4, "cr_n": 3, "cr_d": 4,
         "n_cbps": 192, "n_dbps": 144,  "bpsc": 4},
    48: {"rate_bits": 0b1000, "mod_order": 6, "cr_n": 2, "cr_d": 3,
         "n_cbps": 288, "n_dbps": 192,  "bpsc": 6},
    54: {"rate_bits": 0b1100, "mod_order": 6, "cr_n": 3, "cr_d": 4,
         "n_cbps": 288, "n_dbps": 216,  "bpsc": 6},
}

# ============================================================================
# PHY constants
# ============================================================================
NFFT = 64
NCP = 16
NCP_SHORT = 8               # Short GI: 400 ns at 20 MSPS
SYMBOL_SAMPLES = 80  # NFFT + NCP
SYMBOL_SAMPLES_SHORT = NFFT + NCP_SHORT  # 72
SAMPLE_RATE = 20_000_000

STF_SAMPLES = 160
LTF_SAMPLES = 160
PREAMBLE_SAMPLES = STF_SAMPLES + LTF_SAMPLES  # 320

# ============================================================================
# Subcarrier maps
# ============================================================================

# 12 null bins (DC + guard subcarriers), FFT bin indices
NULL_BINS = [0, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37]

# 4 pilot bins: subcarriers -21, -7, +7, +21 → FFT bins 43, 57, 7, 21
PILOT_BINS = [7, 21, 43, 57]
PILOT_BASE = {7: complex(1, 0), 21: complex(-1, 0), 43: complex(1, 0), 57: complex(1, 0)}

# 48 data subcarriers mapped to FFT bins (negative indices first)
DATA_BINS = (
    [38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50,
     51, 52, 53, 54, 55, 56, 58, 59, 60, 61, 62, 63]  # subcarriers -26 to -1
    + [1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13,
       14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26]  # subcarriers +1 to +26
)
assert len(DATA_BINS) == 48

# Pilot polarity sequence (Table 17-6 / 17-8, 127 elements)
PILOT_POLARITY = [
    1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1,
    -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1,
    1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1,
    -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1,
    -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1,
    -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1,
    -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, -1,
    -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1,
]

# ============================================================================
# Preamble sequences
# ============================================================================

# STF frequency-domain (Table 17-4, 64 complex values)
# Only every 4th subcarrier is non-zero: {-24, -20, -16, -12, -8, -4,
#   4, 8, 12, 16, 20, 24}
_STF_SCALE = np.sqrt(13.0 / 6.0)
_STF_FREQ = np.zeros(NFFT, dtype=complex)
_stf_nonzero = {
    -24: +1 + 1j, -20: -1 - 1j, -16: +1 + 1j, -12: -1 - 1j,
    -8:  -1 - 1j, -4:  +1 + 1j, 4:   -1 - 1j, 8:   -1 - 1j,
    12:  +1 + 1j, 16:  +1 + 1j, 20:  +1 + 1j, 24:  +1 + 1j,
}
for _k, _v in _stf_nonzero.items():
    _STF_FREQ[_k % NFFT] = _STF_SCALE * _v

# LTF frequency-domain (Table 17-5, 64 complex values, ±1 real)
_LTF_FREQ = np.zeros(NFFT, dtype=complex)
_ltf_vals = [
    # L_{-26..-1}
    1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1,
    -1, 1, 1, -1, 1, -1, 1, 1, 1, 1,
    # L_{0}=0, L_{1..26}
    0,
    1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1,
    -1, -1, 1, -1, 1, -1, 1, 1, 1, 1,
]
for _i in range(26):
    _LTF_FREQ[(_i - 26) % NFFT] = _ltf_vals[_i]
for _i in range(26):
    _LTF_FREQ[_i + 1] = _ltf_vals[27 + _i]


def generate_stf() -> np.ndarray:
    """Generate Short Training Field (160 samples).

    10 repetitions of a 16-sample IFFT of the STF frequency-domain signal.
    """
    time_64 = np.fft.ifft(_STF_FREQ)
    return np.tile(time_64[0:16], 10)


def generate_ltf() -> np.ndarray:
    """Generate Long Training Field (160 samples).

    GI2 (last 32 samples of IFFT) + 2 repetitions of the 64-sample IFFT.
    """
    time_64 = np.fft.ifft(_LTF_FREQ)
    gi2 = time_64[32:]
    return np.concatenate([gi2, time_64, time_64])


# ============================================================================
# Convolutional encoder (K=7, rate 1/2)
# ============================================================================
G0 = 0o133  # 0b1011011 — output A (IEEE 802.11-2020 §17.3.5.5)
G1 = 0o171  # 0b1111001 — output B


def conv_encode(bits: list[int], add_tail: bool = True) -> list[int]:
    """Rate-1/2 K=7 convolutional encoder. Matches src/fec.c: fec_encode().

    Args:
        bits: Input data bits (0 or 1)
        add_tail: If True, append 6 zero tail bits and flush the encoder
                  (produces 2*(n_bits+6) output bits). If False, encode
                  only the input bits without tail flushing.

    Returns coded bits: [g0_0, g1_0, g0_1, g1_1, ...].
    """
    n_total = len(bits) + (6 if add_tail else 0)
    state = 0
    coded = []
    for i in range(n_total):
        bit = bits[i] if i < len(bits) else 0
        reg = (bit << 6) | state
        coded.append(_parity(reg & G0))
        coded.append(_parity(reg & G1))
        state = (reg >> 1) & 0x3F
    return coded


def _parity(x: int) -> int:
    """Bit parity: 1 if odd number of bits set, 0 if even."""
    x ^= x >> 8
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1


# ============================================================================
# Puncturing
# ============================================================================
_PUNCTURE_PATTERNS = {
    (1, 2): None,           # no puncturing
    (2, 3): [1, 1, 1, 0],
    (3, 4): [1, 1, 1, 0, 0, 1],
    (5, 6): [1, 1, 1, 0, 0, 1, 1, 0, 0, 1],  # Figure 19-11
}


def puncture(coded: list[int], cr_n: int, cr_d: int) -> list[int]:
    """Remove punctured bits from rate-1/2 coded stream."""
    if cr_n == 1 and cr_d == 2:
        return list(coded)
    pat = _PUNCTURE_PATTERNS.get((cr_n, cr_d))
    if pat is None:
        raise ValueError(f"Unsupported puncture rate: {cr_n}/{cr_d}")
    n_groups = len(coded) // len(pat)
    out = []
    for g in range(n_groups):
        for p_idx, keep in enumerate(pat):
            if keep:
                out.append(coded[g * len(pat) + p_idx])
    return out


# ============================================================================
# Interleaver (Equation 17-17)
# ============================================================================
def interleave(bits: list[int], n_cbps: int, n_bpsc: int) -> list[int]:
    """802.11a interleaver (Equation 17-17).

    TX: out[perm[k]] = in[k]
    """
    s = max(n_bpsc // 2, 1)
    out = [0] * n_cbps
    for k in range(n_cbps):
        # First permutation
        i = (n_cbps // 16) * (k % 16) + (k // 16)
        # Second permutation
        j = s * (i // s) + (i + n_cbps - (16 * i // n_cbps)) % s
        out[j] = bits[k]
    return out


# ============================================================================
# Modulation
# ============================================================================
def _gray_to_bin(g: int) -> int:
    """Gray-to-binary conversion."""
    b = g
    mask = g >> 1
    while mask:
        b ^= mask
        mask >>= 1
    return b


# Precomputed QAM coordinate tables
_QAM16_COORDS = np.array([-3.0, -1.0, 1.0, 3.0], dtype=float)
_QAM64_COORDS = np.array([-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0], dtype=float)
_QAM256_COORDS = np.array(
    [-15.0, -13.0, -11.0, -9.0, -7.0, -5.0, -3.0, -1.0,
     1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0], dtype=float)

# Precomputed gray-to-bin lookup
_QAM16_GRAY = np.array([_gray_to_bin(g) for g in range(4)], dtype=int)
_QAM64_GRAY = np.array([_gray_to_bin(g) for g in range(8)], dtype=int)
_QAM256_GRAY = np.array([_gray_to_bin(g) for g in range(16)], dtype=int)


def modulate(bits: list[int], mod_order: int) -> np.ndarray:
    """Map bits to constellation symbols (normalized).

    BPSK:   bit=1 → +1.0, bit=0 → -1.0
    QPSK:   (b0,b1) → (I,Q) with I=(b0?+1:-1)/√2, Q=(b1?+1:-1)/√2
    16QAM:  Gray-coded, normalized by 1/√10
    64QAM:  Gray-coded, normalized by 1/√42
    256QAM: Gray-coded, normalized by 1/√170
    """
    n_sym = len(bits) // mod_order
    out = np.zeros(n_sym, dtype=complex)

    if mod_order == 1:
        # BPSK: bit 1 → +1, bit 0 → -1
        for i in range(n_sym):
            out[i] = 1.0 if bits[i] else -1.0
        return out

    if mod_order == 2:
        norm = 1.0 / math.sqrt(2.0)
        for i in range(n_sym):
            b0, b1 = bits[2 * i], bits[2 * i + 1]
            out[i] = complex(
                (1.0 if b0 else -1.0) * norm,
                (1.0 if b1 else -1.0) * norm,
            )
        return out

    if mod_order == 4:
        norm = 1.0 / math.sqrt(10.0)
        for i in range(n_sym):
            off = 4 * i
            b0, b1, b2, b3 = bits[off], bits[off + 1], bits[off + 2], bits[off + 3]
            i_gray = (b0 << 1) | b1
            q_gray = (b2 << 1) | b3
            out[i] = complex(
                _QAM16_COORDS[_QAM16_GRAY[i_gray]] * norm,
                _QAM16_COORDS[_QAM16_GRAY[q_gray]] * norm,
            )
        return out

    if mod_order == 6:
        norm = 1.0 / math.sqrt(42.0)
        for i in range(n_sym):
            off = 6 * i
            b0, b1, b2 = bits[off], bits[off + 1], bits[off + 2]
            b3, b4, b5 = bits[off + 3], bits[off + 4], bits[off + 5]
            i_gray = (b0 << 2) | (b1 << 1) | b2
            q_gray = (b3 << 2) | (b4 << 1) | b5
            out[i] = complex(
                _QAM64_COORDS[_QAM64_GRAY[i_gray]] * norm,
                _QAM64_COORDS[_QAM64_GRAY[q_gray]] * norm,
            )
        return out

    if mod_order == 8:
        # 256-QAM: 8 bits/symbol, 4 bits per axis, normalized by 1/√170
        norm = 1.0 / math.sqrt(170.0)
        for i in range(n_sym):
            off = 8 * i
            b0, b1, b2, b3 = bits[off], bits[off + 1], bits[off + 2], bits[off + 3]
            b4, b5, b6, b7 = bits[off + 4], bits[off + 5], bits[off + 6], bits[off + 7]
            i_gray = (b0 << 3) | (b1 << 2) | (b2 << 1) | b3
            q_gray = (b4 << 3) | (b5 << 2) | (b6 << 1) | b7
            out[i] = complex(
                _QAM256_COORDS[_QAM256_GRAY[i_gray]] * norm,
                _QAM256_COORDS[_QAM256_GRAY[q_gray]] * norm,
            )
        return out

    raise ValueError(f"Unsupported mod_order: {mod_order}")


# ============================================================================
# Scrambler (x^7 + x^4 + 1)
# ============================================================================
def scramble(data_bits: list[int], seed: int) -> list[int]:
    """802.11 scrambler: x^7 + x^4 + 1. Self-inverse."""
    state = seed & 0x7F
    out = []
    for b in data_bits:
        fb = ((state >> 6) ^ (state >> 3)) & 1
        out.append(b ^ fb)
        state = ((state << 1) | fb) & 0x7F
    return out


# ============================================================================
# FCS (CRC-32, IEEE 802.11-2020 Section 9.2.4.8)
# ============================================================================
def compute_fcs(data_bytes: bytes) -> bytes:
    """IEEE 802.3 CRC-32, little-endian 4 bytes.

    Equivalent to: zlib.crc32(data_bytes) packed as little-endian uint32.
    """
    crc = zlib.crc32(data_bytes) & 0xFFFFFFFF
    return struct.pack('<I', crc)


def verify_fcs(data_bytes: bytes) -> bool:
    """Verify the FCS of a MAC frame.

    Computes CRC-32 over the entire frame including FCS.
    A correct frame produces the "magic" residue 0x2144DF1C
    (or 0x00000000 if invert is skipped, depending on convention).
    """
    crc = zlib.crc32(data_bytes) & 0xFFFFFFFF
    return crc == 0x2144DF1C


# ============================================================================
# Byte/bit conversion
# ============================================================================
def bytes_to_bits_lsb(data_bytes: bytes) -> list[int]:
    """Bytes to bit list, LSB first per byte."""
    bits = []
    for b in data_bytes:
        for i in range(8):
            bits.append((b >> i) & 1)
    return bits


def bits_to_bytes_lsb(bits: list[int]) -> bytes:
    """Bit list to bytes, LSB first per byte."""
    nbytes = len(bits) // 8
    out = bytearray(nbytes)
    for j in range(nbytes):
        b = 0
        for i in range(8):
            b |= (bits[j * 8 + i] & 1) << i
        out[j] = b
    return bytes(out)


# ============================================================================
# SIGNAL field
# ============================================================================
def make_signal_bits(rate_bits: int, length: int) -> list[int]:
    """Build 24-bit SIGNAL field as bit list [b0..b23]."""
    sig = rate_bits & 0xF
    sig |= 0 << 4          # reserved
    sig |= (length & 0xFFF) << 5
    parity = bin(sig & 0x1FFFF).count('1') % 2
    sig |= parity << 17
    return [(sig >> i) & 1 for i in range(24)]


def make_signal_int(rate_bits: int, length: int) -> int:
    """Build 24-bit SIGNAL field as integer."""
    sig = rate_bits & 0xF
    sig |= 0 << 4
    sig |= (length & 0xFFF) << 5
    parity = bin(sig & 0x1FFFF).count('1') % 2
    sig |= parity << 17
    return sig


def generate_signal_symbol(rate_bits: int, length: int) -> tuple[np.ndarray, list[int], int]:
    """Generate the SIGNAL OFDM symbol (80 samples: 16 CP + 64 data).

    The SIGNAL field is always BPSK rate-1/2 regardless of data rate.

    Returns: (iq_samples, signal_bits_list, signal_integer)
    """
    signal_bits = make_signal_bits(rate_bits, length)
    signal_int = make_signal_int(rate_bits, length)

    coded = conv_encode(signal_bits, add_tail=False)
    # 24 signal bits (with embedded tail zeros) encode to exactly 48 coded bits
    interleaved = interleave(coded, 48, 1)
    symbols = modulate(interleaved, 1)  # BPSK

    # Build frequency-domain OFDM symbol
    freq = np.zeros(NFFT, dtype=complex)
    for i, bin_idx in enumerate(DATA_BINS):
        freq[bin_idx] = symbols[i]
    for idx in PILOT_BINS:
        freq[idx] = PILOT_BASE[idx]

    time_64 = np.fft.ifft(freq)
    cp = time_64[-NCP:]
    return np.concatenate([cp, time_64]), signal_bits, signal_int


# ============================================================================
# DATA symbol generation (all legacy rates)
# ============================================================================
def make_data_symbols(psdu_bytes: bytes, rate_mbps: int,
                      scrambler_seed: int = 0x5D) -> tuple[list[np.ndarray], int, bytes]:
    """Build DATA OFDM symbols for a PSDU at the given rate.

    Args:
        psdu_bytes: PSDU payload bytes
        rate_mbps: Data rate in Mbps (6, 9, 12, 18, 24, 36, 48, 54)
        scrambler_seed: 7-bit scrambler seed

    Returns:
        (symbols_list, psdu_length_with_fcs, psdu_with_fcs_bytes)
    """
    rate_info = RATE_TABLE[rate_mbps]
    mod_order = rate_info["mod_order"]
    cr_n = rate_info["cr_n"]
    cr_d = rate_info["cr_d"]
    n_cbps = rate_info["n_cbps"]
    n_dbps = rate_info["n_dbps"]
    n_bpsc = rate_info["bpsc"]

    fcs = compute_fcs(psdu_bytes)
    psdu_with_fcs = bytes(psdu_bytes) + fcs
    psdu_length = len(psdu_with_fcs)

    service_bits = [0] * 16
    psdu_bits = bytes_to_bits_lsb(psdu_with_fcs)
    tail_bits = [0] * 6
    all_bits = service_bits + psdu_bits + tail_bits

    # Pad to fill whole OFDM symbols
    n_symbols = math.ceil(len(all_bits) / n_dbps)
    n_pad = n_symbols * n_dbps - len(all_bits)
    all_bits_padded = all_bits + [0] * n_pad

    # Scramble
    scrambled = scramble(all_bits_padded, scrambler_seed)
    # Zero tail bits after scrambling (re-scramble with zeros)
    tail_start = 16 + 8 * psdu_length
    for i in range(6):
        if tail_start + i < len(scrambled):
            scrambled[tail_start + i] = 0

    # Convolutional encode (rate 1/2 base code)
    coded = conv_encode(scrambled, add_tail=False)

    # Compute expected number of coded bits after puncturing
    n_coded_per_sym = n_cbps  # after puncturing
    total_needed = n_symbols * n_coded_per_sym
    # The number of rate-1/2 coded bits needed before puncturing
    if cr_n == 1 and cr_d == 2:
        needed_pre_puncture = total_needed
    elif cr_n == 2 and cr_d == 3:
        needed_pre_puncture = (total_needed * 4) // 3
    elif cr_n == 3 and cr_d == 4:
        needed_pre_puncture = (total_needed * 6) // 4
    else:
        raise ValueError(f"Unsupported rate {cr_n}/{cr_d}")

    # Truncate or pad coded bits
    coded = coded[:needed_pre_puncture]

    # Puncture
    punctured = puncture(coded, cr_n, cr_d)
    if len(punctured) < total_needed:
        punctured += [0] * (total_needed - len(punctured))

    # Build OFDM symbols
    ofdm_symbols = []
    for s in range(n_symbols):
        sym_coded = punctured[s * n_cbps:(s + 1) * n_cbps]
        if len(sym_coded) < n_cbps:
            sym_coded += [0] * (n_cbps - len(sym_coded))

        sym_interleaved = interleave(sym_coded, n_cbps, n_bpsc)
        symbols = modulate(sym_interleaved, mod_order)

        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(DATA_BINS):
            freq[bin_idx] = symbols[i]

        pilot_idx = s + 1  # first DATA symbol is index 1
        polarity = PILOT_POLARITY[pilot_idx % 127]
        for pb in PILOT_BINS:
            freq[pb] = polarity * PILOT_BASE[pb]

        time_64 = np.fft.ifft(freq)
        cp = time_64[-NCP:]
        ofdm_symbols.append(np.concatenate([cp, time_64]))

    return ofdm_symbols, psdu_length, psdu_with_fcs


# ============================================================================
# Full frame generation
# ============================================================================
def generate_frame(rate_mbps: int, psdu_bytes: bytes,
                   scrambler_seed: int = 0x5D) -> tuple[np.ndarray, dict]:
    """Generate a complete 802.11a frame at baseband.

    Args:
        rate_mbps: Data rate (6, 9, 12, 18, 24, 36, 48, 54)
        psdu_bytes: PSDU payload bytes
        scrambler_seed: 7-bit scrambler seed (default 0x5D)

    Returns:
        (iq_samples, metadata_dict)
    """
    if rate_mbps not in RATE_TABLE:
        raise ValueError(f"Invalid rate: {rate_mbps}. Valid: {sorted(RATE_TABLE)}")

    rate_info = RATE_TABLE[rate_mbps]
    rate_bits = rate_info["rate_bits"]

    psdu_with_fcs = bytes(psdu_bytes) + compute_fcs(psdu_bytes)
    psdu_length = len(psdu_with_fcs)

    stf = generate_stf()
    ltf = generate_ltf()
    signal_iq, signal_bits, signal_int = generate_signal_symbol(rate_bits, psdu_length)
    data_symbols, _, _ = make_data_symbols(psdu_bytes, rate_mbps, scrambler_seed)

    parts = [stf, ltf, signal_iq] + data_symbols
    iq = np.concatenate(parts)

    meta = {
        "stf_start": 0,
        "ltf_start": STF_SAMPLES,
        "signal_start": PREAMBLE_SAMPLES,
        "data_start": PREAMBLE_SAMPLES + SYMBOL_SAMPLES,
        "signal_bits": signal_bits,
        "signal_int": signal_int,
        "rate_bits": rate_bits,
        "rate_mbps": rate_mbps,
        "psdu_length": psdu_length,
        "psdu_with_fcs": psdu_with_fcs,
        "psdu_payload": psdu_bytes,
        "scrambler_seed": scrambler_seed,
        "n_data_symbols": len(data_symbols),
        "n_samples": len(iq),
        "sample_rate": SAMPLE_RATE,
    }
    return iq.astype(np.complex64), meta


def generate_preamble(rate_bits: int, length: int) -> tuple[np.ndarray, dict]:
    """Generate just the preamble (STF + LTF + SIGNAL). No DATA symbols.

    Useful for STF detection and SIGNAL parsing tests.
    """
    stf = generate_stf()
    ltf = generate_ltf()
    signal_iq, signal_bits, signal_int = generate_signal_symbol(rate_bits, length)

    iq = np.concatenate([stf, ltf, signal_iq])

    meta = {
        "stf_start": 0,
        "ltf_start": STF_SAMPLES,
        "signal_start": PREAMBLE_SAMPLES,
        "signal_bits": signal_bits,
        "signal_int": signal_int,
        "rate_bits": rate_bits,
        "length_bytes": length,
        "n_samples": len(iq),
        "sample_rate": SAMPLE_RATE,
    }
    return iq.astype(np.complex64), meta


# ============================================================================
# HT-mixed generation (802.11n, MCS 0–7, 20 MHz, 1 spatial stream, BCC)
# ============================================================================

# HT MCS parameters (20 MHz, 1 SS, BCC) — IEEE 802.11-2020 Table 19-27
HT_MCS_TABLE = {
    0: {"mod_order": 1, "cr_n": 1, "cr_d": 2, "n_cbps": 52,  "n_dbps": 26,  "bpsc": 1},
    1: {"mod_order": 2, "cr_n": 1, "cr_d": 2, "n_cbps": 104, "n_dbps": 52,  "bpsc": 2},
    2: {"mod_order": 2, "cr_n": 3, "cr_d": 4, "n_cbps": 104, "n_dbps": 78,  "bpsc": 2},
    3: {"mod_order": 4, "cr_n": 1, "cr_d": 2, "n_cbps": 208, "n_dbps": 104, "bpsc": 4},
    4: {"mod_order": 4, "cr_n": 3, "cr_d": 4, "n_cbps": 208, "n_dbps": 156, "bpsc": 4},
    5: {"mod_order": 6, "cr_n": 2, "cr_d": 3, "n_cbps": 312, "n_dbps": 208, "bpsc": 6},
    6: {"mod_order": 6, "cr_n": 3, "cr_d": 4, "n_cbps": 312, "n_dbps": 234, "bpsc": 6},
    7: {"mod_order": 6, "cr_n": 5, "cr_d": 6, "n_cbps": 312, "n_dbps": 260, "bpsc": 6},
}

# HT data subcarriers: -28..-1, +1..+28 excluding pilots at ±7, ±21
# That's 52 data subcarriers mapped to FFT bins
HT_DATA_BINS = []
for _sc in range(-28, 0):
    if _sc in (-21, -7):
        continue  # pilot
    HT_DATA_BINS.append(_sc % NFFT)
for _sc in range(1, 29):
    if _sc in (7, 21):
        continue  # pilot
    HT_DATA_BINS.append(_sc)
assert len(HT_DATA_BINS) == 52

# HT pilot bins in IEEE spec order: subcarriers -21, -7, +7, +21 → FFT bins 43, 57, 7, 21
HT_PILOT_SPEC_BINS = [43, 57, 7, 21]

# HT per-pilot cyclic pattern (Table 19-10, 1 spatial stream, 4 pilots)
# Applied as: pilot(n, k) = PILOT_POLARITY[(z+n) % 127] * pattern[(n+k) % 4]
_HT_PILOT_PATTERN = [1, 1, 1, -1]

# HT-LTF subcarrier values (Eq 19-23): ±1 on subcarriers -28..+28
# Per NOTE 1: "extension of the L-LTF where the four extra subcarriers are
# filled with +1 for negative frequencies and -1 for positive frequencies."
_HT_LTF_FREQ = np.zeros(NFFT, dtype=complex)
# Copy legacy LTF values for -26..+26
for _i in range(26):
    _HT_LTF_FREQ[(_i - 26) % NFFT] = _ltf_vals[_i]
for _i in range(26):
    _HT_LTF_FREQ[_i + 1] = _ltf_vals[27 + _i]
# Extend to ±27, ±28 per Eq 19-23 NOTE 1:
#   +1 for negative frequencies (sc -28, -27)
#   -1 for positive frequencies (sc +27, +28)
_HT_LTF_FREQ[(-27) % NFFT] = 1.0
_HT_LTF_FREQ[(-28) % NFFT] = 1.0
_HT_LTF_FREQ[27] = -1.0
_HT_LTF_FREQ[28] = -1.0


def ht_sig_crc8(bits) -> int:
    """CRC-8 for HT-SIG (poly 0x07, init 0xFF).

    Polynomial: x^8+x^2+x+1 (0x07 normal form).
    Initial state: 0xFF.
    Processes bits MSB-first through a bit-serial shift register.

    Args:
        bits: sequence of 0/1 values (typically 34 bits for HT-SIG).

    Returns:
        8-bit CRC value (0-255).
    """
    state = 0xFF
    for b in bits:
        feedback = ((state >> 7) ^ int(b)) & 1
        state = (state << 1) & 0xFF
        if feedback:
            state ^= 0x07
    return state


def make_ht_sig_bits(mcs: int, length: int, bw: int = 0,
                     short_gi: int = 0, aggregation: int = 0,
                     coding: str = "bcc") -> list[int]:
    """Build 48-bit HT-SIG field: 34 data + 8 CRC + 6 tail.

    Returns list of 48 ints (0 or 1).
    """
    data_bits = [0] * 34
    # MCS: bits [0:6]
    for i in range(7):
        data_bits[i] = (mcs >> i) & 1
    # BW: bit 7
    data_bits[7] = bw & 1
    # Length: bits [8:23] (16 bits LSB first)
    for i in range(16):
        data_bits[8 + i] = (length >> i) & 1
    # Smoothing: bit 24
    data_bits[24] = 1
    # Not sounding: bit 25
    data_bits[25] = 1
    # Reserved: bit 26 (set to 1 per IEEE TGn reference implementation)
    data_bits[26] = 1
    # Aggregation: bit 27
    data_bits[27] = aggregation & 1
    # STBC: bits [28:29]
    data_bits[28] = 0
    data_bits[29] = 0
    # FEC coding: bit 30 (0=BCC, 1=LDPC)
    data_bits[30] = 1 if coding == "ldpc" else 0
    # Short GI: bit 31
    data_bits[31] = short_gi & 1
    # N_ESS: bits [32:33]
    data_bits[32] = 0
    data_bits[33] = 0

    # CRC-8: ones-complement, MSB-first in bits [34:41]
    crc = ht_sig_crc8(data_bits)
    crc_inv = (~crc) & 0xFF
    crc_bits = [((crc_inv >> (7 - i)) & 1) for i in range(8)]

    # Tail: 6 zeros
    tail_bits = [0] * 6

    return data_bits + crc_bits + tail_bits


def generate_ht_sig_symbols(mcs: int, length: int, bw: int = 0,
                             short_gi: int = 0,
                             coding: str = "bcc") -> tuple[np.ndarray, list[int]]:
    """Generate 2 HT-SIG OFDM symbols (each 80 samples).

    Encoding: BCC rate-1/2, legacy OFDM interleaving, Q-BPSK both symbols.

    Per IEEE TGn reference implementation (11-06/1715r0): both HT-SIG
    OFDM symbols use Q-BPSK modulation (data on quadrature axis).
    Pilots remain on the in-phase axis with standard polarity.

    Returns: (iq_samples [160], ht_sig_bits [48])
    """
    ht_sig_bits = make_ht_sig_bits(mcs, length, bw, short_gi, coding=coding)

    # BCC encode: 48 bits → 96 coded bits
    coded = conv_encode(ht_sig_bits, add_tail=False)

    # Split into 2 symbols, interleave each (legacy interleaver)
    coded_sym1 = interleave(coded[:48], 48, 1)
    coded_sym2 = interleave(coded[48:], 48, 1)

    # Q-BPSK modulate both symbols (data on Q-axis, per TGn reference)
    bpsk_sym1 = modulate(coded_sym1, 1) * 1j  # Q-BPSK
    bpsk_sym2 = modulate(coded_sym2, 1) * 1j  # Q-BPSK

    symbols_out = []
    for sym_idx, data_syms in enumerate([bpsk_sym1, bpsk_sym2]):
        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(DATA_BINS):
            freq[bin_idx] = data_syms[i]
        # Pilots (polarity: L-SIG=0, HT-SIG1=1, HT-SIG2=2)
        pilot_idx = sym_idx + 1
        polarity = PILOT_POLARITY[pilot_idx % 127]
        for pb in PILOT_BINS:
            freq[pb] = polarity * PILOT_BASE[pb]

        time_64 = np.fft.ifft(freq)
        cp = time_64[-NCP:]
        symbols_out.append(np.concatenate([cp, time_64]))

    iq = np.concatenate(symbols_out)
    return iq, ht_sig_bits


def generate_ht_stf() -> np.ndarray:
    """Generate HT-STF (80 samples).

    HT-STF is a scaled version of legacy STF, one symbol duration.
    IEEE 802.11-2020 §19.3.9.4.5.
    """
    time_64 = np.fft.ifft(_STF_FREQ)
    # HT-STF is 1 symbol: CP (16) + 64 samples
    return np.concatenate([time_64[-NCP:], time_64])


def generate_ht_ltf() -> np.ndarray:
    """Generate HT-LTF (80 samples, 1 spatial stream).

    For 1 spatial stream: 1 HT-LTF symbol = CP (16) + 64 FFT.
    Uses HT-LTF subcarrier values (extended to ±28).
    IEEE 802.11-2020 §19.3.9.4.6.
    """
    time_64 = np.fft.ifft(_HT_LTF_FREQ)
    return np.concatenate([time_64[-NCP:], time_64])


def _ht_interleave(bits: list[int], n_cbps: int, n_bpsc: int) -> list[int]:
    """HT interleaver (802.11-2020 §19.3.11.8.3, Eq 19-46/47).

    For 20 MHz, single spatial stream:
      N_col=13, N_row=4*N_bpsc, s=max(N_bpsc/2, 1)
    Third permutation (frequency rotation) not applied for 1 stream.
    """
    n_col = 13
    n_row = 4 * n_bpsc
    s = max(n_bpsc // 2, 1)

    out = [0] * n_cbps
    for k in range(n_cbps):
        # First permutation (Eq 19-46)
        i = n_row * (k % n_col) + (k // n_col)
        # Second permutation (Eq 19-47)
        j = s * (i // s) + (i + n_cbps - (n_col * i // n_cbps)) % s
        out[j] = bits[k]
    return out


def _ldpc_encode_data(payload_bits: list[int], n_dbps: int, n_cbps: int,
                      cr_n: int, cr_d: int) -> tuple[list[int], int, int]:
    """LDPC encoding per IEEE 802.11-2020 §19.3.11.7 / §21.3.12.5.

    Args:
        payload_bits: Scrambled SERVICE + PSDU bits (no tail bits for LDPC).
        n_dbps: Data bits per OFDM symbol.
        n_cbps: Coded bits per OFDM symbol.
        cr_n: Code rate numerator.
        cr_d: Code rate denominator.

    Returns:
        (coded_bits, n_symbols, ldpc_extra_flag)
    """
    R = cr_n / cr_d
    rate_str = f"{cr_n}/{cr_d}"

    # Step 1: Compute N_SYM (no tail bits for LDPC)
    n_pld_needed = len(payload_bits)  # SERVICE (16) + PSDU bits
    n_symbols = math.ceil(n_pld_needed / n_dbps)
    n_pld = n_symbols * n_dbps

    # Check for LDPC extra symbol
    ldpc_extra = 0
    # Select codeword parameters to check if extra symbol needed
    if n_pld <= 648:
        l_cw_check = 1944
        for cw in [648, 1296, 1944]:
            if int(cw * R) >= n_pld:
                l_cw_check = cw
                break
        n_cw_check = 1
    elif n_pld <= 1296:
        l_cw_check = 1944
        for cw in [1296, 1944]:
            if int(cw * R) >= n_pld:
                l_cw_check = cw
                break
        n_cw_check = 1
    elif n_pld <= 1944:
        l_cw_check = 1944
        n_cw_check = 1
    else:
        l_cw_check = 1944
        n_cw_check = 1

    # If single codeword can't hold n_pld, use multi-codeword
    if int(l_cw_check * R) < n_pld:
        n_cw_check = math.ceil(n_pld / (1944 * R))
        l_cw_check = 1944

    # LDPC extra symbol check (IEEE 802.11-2020 §19.3.11.7.5, Eq 19-60/19-66)
    # N_punc = max(0, N_CW * L_CW - N_avail - N_shrt)
    # Extra symbol needed when N_punc > 0.1 * N_CW * (L_CW - k_per_cw)
    n_avail = n_symbols * n_cbps
    n_shrt_test = max(0, int(n_cw_check * l_cw_check * R) - n_pld)
    n_punc_test = max(0, n_cw_check * l_cw_check - n_avail - n_shrt_test)
    n_parity_total = n_cw_check * (l_cw_check - int(l_cw_check * R))
    if n_parity_total > 0 and n_punc_test > 0.1 * n_parity_total:
        ldpc_extra = 1
        n_symbols += 1
        n_pld = n_symbols * n_dbps

    # Step 2: Total available coded bits
    n_avail = n_symbols * n_cbps

    # Step 3: Select codeword parameters (Table 19-14)
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

    # Step 4: Compute shortening and puncturing (IEEE 802.11-2020 Eq 19-65/19-66)
    k_total = int(l_cw * R) * n_cw  # Total info capacity
    n_shrt = max(0, k_total - n_pld)
    n_punc = max(0, n_cw * l_cw - n_avail - n_shrt)

    # Distribute shortening evenly across codewords
    shrt_per_cw = n_shrt // n_cw
    shrt_extra = n_shrt % n_cw
    # Distribute puncturing evenly across codewords
    punc_per_cw = n_punc // n_cw
    punc_extra = n_punc % n_cw

    # Step 5: Encode each codeword
    # Pad payload to N_pld
    padded_payload = list(payload_bits) + [0] * (n_pld - len(payload_bits))

    k_per_cw = int(l_cw * R)  # Info bits per full codeword
    coded_bits = []
    bit_offset = 0

    for j in range(n_cw):
        # Shortening for this codeword
        shrt_j = shrt_per_cw + (1 if j < shrt_extra else 0)
        # Puncturing for this codeword
        punc_j = punc_per_cw + (1 if j < punc_extra else 0)

        # Info bits for this codeword (K_j = k_per_cw - shrt_j)
        k_j = k_per_cw - shrt_j
        info_j = padded_payload[bit_offset:bit_offset + k_j]
        bit_offset += k_j

        # Pad with shortening zeros to make full info block
        info_padded = np.array(info_j + [0] * shrt_j, dtype=np.int8)

        # Encode
        codeword = ldpc_encode(info_padded, rate_str, l_cw)

        # Remove shortening zeros from systematic part
        # Systematic part is first k_per_cw bits; remove the last shrt_j
        systematic = codeword[:k_per_cw - shrt_j]
        parity = codeword[k_per_cw:]  # Parity bits

        # Remove punctured parity bits from the end
        if punc_j > 0:
            parity = parity[:-punc_j]

        coded_bits.extend(systematic.tolist())
        coded_bits.extend(parity.tolist())

    # Pad to N_avail if needed (handles rounding in multi-codeword case)
    if len(coded_bits) < n_avail:
        coded_bits.extend([0] * (n_avail - len(coded_bits)))
    elif len(coded_bits) > n_avail:
        coded_bits = coded_bits[:n_avail]

    return coded_bits, n_symbols, ldpc_extra


def make_ht_data_symbols(psdu_bytes: bytes, mcs: int,
                         scrambler_seed: int = 0x5D,
                         short_gi: bool = False,
                         coding: str = "bcc") -> tuple[list[np.ndarray], int, bytes]:
    """Build HT-DATA OFDM symbols (52 data subcarriers per symbol).

    Args:
        psdu_bytes: PSDU payload bytes (FCS will be appended)
        mcs: HT MCS index (0–7)
        scrambler_seed: 7-bit scrambler seed
        short_gi: If True, use 8-sample cyclic prefix (400 ns) instead of 16
        coding: "bcc" or "ldpc"

    Returns: (symbols_list, psdu_length_with_fcs, psdu_with_fcs_bytes)
    """
    mcs_info = HT_MCS_TABLE[mcs]
    mod_order = mcs_info["mod_order"]
    cr_n = mcs_info["cr_n"]
    cr_d = mcs_info["cr_d"]
    n_cbps = mcs_info["n_cbps"]
    n_dbps = mcs_info["n_dbps"]
    n_bpsc = mcs_info["bpsc"]

    fcs = compute_fcs(psdu_bytes)
    psdu_with_fcs = bytes(psdu_bytes) + fcs
    psdu_length = len(psdu_with_fcs)

    if coding == "ldpc":
        # LDPC path: no tail bits, no interleaving
        service_bits = [0] * 16
        psdu_bits = bytes_to_bits_lsb(psdu_with_fcs)
        all_bits = service_bits + psdu_bits  # No tail bits for LDPC

        # Scramble (pad to N_SYM * N_DBPS will be done inside _ldpc_encode_data)
        # First compute N_SYM for scrambling extent
        n_pld_needed = len(all_bits)
        n_symbols_init = math.ceil(n_pld_needed / n_dbps)
        n_pad_init = n_symbols_init * n_dbps - n_pld_needed
        all_bits_padded = all_bits + [0] * n_pad_init

        scrambled = scramble(all_bits_padded, scrambler_seed)

        # LDPC encode
        coded_bits, n_symbols, _ = _ldpc_encode_data(
            scrambled, n_dbps, n_cbps, cr_n, cr_d
        )

        # Build OFDM symbols — NO interleaving for LDPC
        pilot_offset = 3
        ofdm_symbols = []
        for s in range(n_symbols):
            sym_coded = coded_bits[s * n_cbps:(s + 1) * n_cbps]
            if len(sym_coded) < n_cbps:
                sym_coded += [0] * (n_cbps - len(sym_coded))

            # No interleaving for LDPC — map directly to constellation
            symbols = modulate(sym_coded, mod_order)

            freq = np.zeros(NFFT, dtype=complex)
            for i, bin_idx in enumerate(HT_DATA_BINS):
                freq[bin_idx] = symbols[i]

            # HT pilots with per-subcarrier cyclic pattern
            polarity = PILOT_POLARITY[(pilot_offset + s) % 127]
            for k, pb in enumerate(HT_PILOT_SPEC_BINS):
                freq[pb] = polarity * _HT_PILOT_PATTERN[(s + k) % 4]

            time_64 = np.fft.ifft(freq)
            ncp = NCP_SHORT if short_gi else NCP
            cp = time_64[-ncp:]
            ofdm_symbols.append(np.concatenate([cp, time_64]))

        return ofdm_symbols, psdu_length, psdu_with_fcs

    # BCC path (original behavior)
    service_bits = [0] * 16
    psdu_bits = bytes_to_bits_lsb(psdu_with_fcs)
    tail_bits = [0] * 6
    all_bits = service_bits + psdu_bits + tail_bits

    # Pad to fill whole OFDM symbols
    n_symbols = math.ceil(len(all_bits) / n_dbps)
    n_pad = n_symbols * n_dbps - len(all_bits)
    all_bits_padded = all_bits + [0] * n_pad

    # Scramble
    scrambled = scramble(all_bits_padded, scrambler_seed)
    # Zero tail bits after scrambling
    tail_start = 16 + 8 * psdu_length
    for i in range(6):
        if tail_start + i < len(scrambled):
            scrambled[tail_start + i] = 0

    # Convolutional encode
    coded = conv_encode(scrambled, add_tail=False)

    # Compute expected coded bits
    n_coded_per_sym = n_cbps
    total_needed = n_symbols * n_coded_per_sym
    if cr_n == 1 and cr_d == 2:
        needed_pre_puncture = total_needed
    elif cr_n == 2 and cr_d == 3:
        needed_pre_puncture = (total_needed * 4) // 3
    elif cr_n == 3 and cr_d == 4:
        needed_pre_puncture = (total_needed * 6) // 4
    elif cr_n == 5 and cr_d == 6:
        needed_pre_puncture = (total_needed * 10) // 6
    else:
        raise ValueError(f"Unsupported rate {cr_n}/{cr_d}")

    coded = coded[:needed_pre_puncture]

    # Puncture
    punctured = puncture(coded, cr_n, cr_d)
    if len(punctured) < total_needed:
        punctured += [0] * (total_needed - len(punctured))

    # Build OFDM symbols (52 data subcarriers, HT interleaver)
    # HT pilot formula per IEEE TGn reference (11-06/1715r0):
    #   pilot(n, k) = PILOT_POLARITY[(z_start + n) % 127] * HT_PILOT_PATTERN[(n + k) % 4]
    # where z_start=3 for HT-mixed DATA, k is the spec-ordered pilot index
    # (k=0: sc -21, k=1: sc -7, k=2: sc +7, k=3: sc +21),
    # and HT_PILOT_PATTERN = [1, 1, 1, -1].
    pilot_offset = 3

    ofdm_symbols = []
    for s in range(n_symbols):
        sym_coded = punctured[s * n_cbps:(s + 1) * n_cbps]
        if len(sym_coded) < n_cbps:
            sym_coded += [0] * (n_cbps - len(sym_coded))

        sym_interleaved = _ht_interleave(sym_coded, n_cbps, n_bpsc)
        symbols = modulate(sym_interleaved, mod_order)

        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(HT_DATA_BINS):
            freq[bin_idx] = symbols[i]

        # HT pilots with per-subcarrier cyclic pattern
        polarity = PILOT_POLARITY[(pilot_offset + s) % 127]
        for k, pb in enumerate(HT_PILOT_SPEC_BINS):
            freq[pb] = polarity * _HT_PILOT_PATTERN[(s + k) % 4]

        time_64 = np.fft.ifft(freq)
        ncp = NCP_SHORT if short_gi else NCP
        cp = time_64[-ncp:]
        ofdm_symbols.append(np.concatenate([cp, time_64]))

    return ofdm_symbols, psdu_length, psdu_with_fcs


def generate_ht_frame(mcs: int, psdu_bytes: bytes,
                      scrambler_seed: int = 0x5D,
                      short_gi: bool = False,
                      coding: str = "bcc") -> tuple[np.ndarray, dict]:
    """Generate a complete HT-mixed frame at baseband.

    Structure: L-STF | L-LTF | L-SIG | HT-SIG1 | HT-SIG2 |
               HT-STF | HT-LTF | HT-DATA

    Args:
        mcs: HT MCS index (0–7)
        psdu_bytes: PSDU payload bytes
        scrambler_seed: 7-bit scrambler seed
        short_gi: If True, use short guard interval (400 ns)
        coding: "bcc" or "ldpc"

    Returns:
        (iq_samples, metadata_dict)
    """
    if mcs not in HT_MCS_TABLE:
        raise ValueError(f"Invalid MCS: {mcs}. Valid: 0–7")

    psdu_with_fcs = bytes(psdu_bytes) + compute_fcs(psdu_bytes)
    psdu_length = len(psdu_with_fcs)

    # L-SIG: rate=6 Mbps, LENGTH encodes NAV protection duration.
    # L-SIG LENGTH: IEEE 802.11-2020 Equation (10-17), §10.27.4.
    # L_LENGTH = ceil((TXTIME - SignalExtension - 20) / 4) * 3 - 3
    #
    # For HT-mixed, 1 SS (Eq 19-91):
    #   TXTIME = T_LEG_PREAMBLE + T_L-SIG + T_HT-SIG + T_HT_TRAINING + T_SYM * N_SYM
    # where T_HT_TRAINING = T_HT-STF + T_HT-LTF1 = 4 + 4 = 8 µs (1 SS, N_HT-LTF=1)
    #
    # Long GI (T_SYM = 4 µs):
    #   TXTIME = 16 + 4 + 8 + 8 + 4*N_SYM = 36 + 4*N_SYM (µs)
    #   L_LENGTH = (4 + N_SYM) * 3 - 3
    #
    # Short GI (T_SYM = 3.6 µs, symbolTime = 9/10):
    #   L_LENGTH = 3 * (ceil(N_SYM * 9/10) + 4) - 3
    #   (IEEE TGn reference tx_n_highlevel.m, line 1522-1528)
    rate_bits = RATE_TABLE[6]["rate_bits"]
    mcs_info = HT_MCS_TABLE[mcs]
    n_dbps = mcs_info["n_dbps"]

    # Generate data symbols first to get actual symbol count (includes ldpc_extra)
    data_symbols, _, _ = make_ht_data_symbols(psdu_bytes, mcs, scrambler_seed,
                                              short_gi=short_gi, coding=coding)
    n_data_sym = len(data_symbols)

    if short_gi:
        lsig_length = 3 * (math.ceil(n_data_sym * 0.9) + 4) - 3
    else:
        lsig_length = (4 + n_data_sym) * 3 - 3

    # Generate components
    stf = generate_stf()
    ltf = generate_ltf()
    signal_iq, signal_bits, signal_int = generate_signal_symbol(rate_bits, lsig_length)
    ht_sig_iq, ht_sig_bits = generate_ht_sig_symbols(mcs, psdu_length,
                                                      short_gi=int(short_gi),
                                                      coding=coding)
    ht_stf = generate_ht_stf()
    ht_ltf = generate_ht_ltf()

    parts = [stf, ltf, signal_iq, ht_sig_iq, ht_stf, ht_ltf] + data_symbols
    iq = np.concatenate(parts)

    meta = {
        "stf_start": 0,
        "ltf_start": STF_SAMPLES,
        "signal_start": PREAMBLE_SAMPLES,
        "ht_sig_start": PREAMBLE_SAMPLES + SYMBOL_SAMPLES,
        "ht_stf_start": PREAMBLE_SAMPLES + 3 * SYMBOL_SAMPLES,
        "ht_ltf_start": PREAMBLE_SAMPLES + 4 * SYMBOL_SAMPLES,
        "ht_data_start": PREAMBLE_SAMPLES + 5 * SYMBOL_SAMPLES,
        "signal_bits": signal_bits,
        "ht_sig_bits": ht_sig_bits,
        "mcs": mcs,
        "short_gi": short_gi,
        "coding": coding,
        "lsig_length": lsig_length,
        "psdu_length": psdu_length,
        "psdu_with_fcs": psdu_with_fcs,
        "psdu_payload": psdu_bytes,
        "scrambler_seed": scrambler_seed,
        "n_data_symbols": len(data_symbols),
        "n_samples": len(iq),
        "sample_rate": SAMPLE_RATE,
    }
    return iq.astype(np.complex64), meta


# ============================================================================
# VHT generation (802.11ac, MCS 0–8, 20 MHz, 1 spatial stream, BCC)
# ============================================================================

# VHT MCS parameters (20 MHz, 1 SS, BCC) — IEEE 802.11-2020 Table 21-30
# Note: MCS 9 (256-QAM 5/6) is not valid for 20 MHz, 1 SS (N_DBPS non-integer).
VHT_MCS_TABLE = {
    0: {"mod_order": 1, "cr_n": 1, "cr_d": 2, "n_cbps": 52,  "n_dbps": 26,  "bpsc": 1},
    1: {"mod_order": 2, "cr_n": 1, "cr_d": 2, "n_cbps": 104, "n_dbps": 52,  "bpsc": 2},
    2: {"mod_order": 2, "cr_n": 3, "cr_d": 4, "n_cbps": 104, "n_dbps": 78,  "bpsc": 2},
    3: {"mod_order": 4, "cr_n": 1, "cr_d": 2, "n_cbps": 208, "n_dbps": 104, "bpsc": 4},
    4: {"mod_order": 4, "cr_n": 3, "cr_d": 4, "n_cbps": 208, "n_dbps": 156, "bpsc": 4},
    5: {"mod_order": 6, "cr_n": 2, "cr_d": 3, "n_cbps": 312, "n_dbps": 208, "bpsc": 6},
    6: {"mod_order": 6, "cr_n": 3, "cr_d": 4, "n_cbps": 312, "n_dbps": 234, "bpsc": 6},
    7: {"mod_order": 6, "cr_n": 5, "cr_d": 6, "n_cbps": 312, "n_dbps": 260, "bpsc": 6},
    8: {"mod_order": 8, "cr_n": 3, "cr_d": 4, "n_cbps": 416, "n_dbps": 312, "bpsc": 8},
}

# VHT pilot polarity offset (z_start for DATA symbols)
# From ofdmsym_11ac.m:150 — E_VHT uses sym_ofset=4
VHT_PILOT_OFFSET = 4


def make_vht_sig_a_bits(mcs: int, length: int, bw: int = 0,
                        short_gi: int = 0, stbc: int = 0,
                        group_id: int = 0, nsts: int = 1,
                        partial_aid: int = 0x15D,
                        txop_ps_not_allowed: int = 0,
                        coding: str = "bcc",
                        ldpc_extra: int = 0,
                        sgi_disambig: int = 0) -> list[int]:
    """Build 48-bit VHT-SIG-A field: 34 data + 8 CRC + 6 tail.

    VHT-SIG-A spans 2 OFDM symbols (24 bits each).
    IEEE 802.11-2020 §21.3.8.3.3.

    Default partial_aid=0x15D and txop_ps_not_allowed=0 match the TGac
    reference generator for golden vector compatibility.

    Returns list of 48 ints (0 or 1).
    """
    data_bits = [0] * 34

    # Symbol 1 bits [0:23]
    # BW: bits [0:1] (2 bits)
    data_bits[0] = bw & 1
    data_bits[1] = (bw >> 1) & 1
    # Reserved: bit [2]
    data_bits[2] = 1
    # STBC: bit [3]
    data_bits[3] = stbc & 1
    # Group ID: bits [4:9] (6 bits)
    for i in range(6):
        data_bits[4 + i] = (group_id >> i) & 1
    # NSTS-1: bits [10:12] (3 bits, 0-based)
    nsts_minus1 = (nsts - 1) & 0x7
    for i in range(3):
        data_bits[10 + i] = (nsts_minus1 >> i) & 1
    # Partial AID: bits [13:21] (9 bits, 0 for SU)
    for i in range(9):
        data_bits[13 + i] = (partial_aid >> i) & 1
    # TXOP_PS_NOT_ALLOWED: bit [22]
    data_bits[22] = txop_ps_not_allowed & 1
    # Reserved: bit [23]
    data_bits[23] = 1

    # Symbol 2 bits [24:33]
    # Short GI: bit [24]
    data_bits[24] = short_gi & 1
    # SGI disambiguation: bit [25] (IEEE 802.11-2020 §21.3.8.3.3)
    # Resolves N_SYM ambiguity when short GI is used and N_SYM % 10 == 9.
    data_bits[25] = sgi_disambig & 1
    # Coding: bit [26] (0=BCC, 1=LDPC)
    data_bits[26] = 1 if coding == "ldpc" else 0
    # LDPC extra: bit [27]
    data_bits[27] = ldpc_extra & 1
    # MCS: bits [28:31] (4 bits)
    for i in range(4):
        data_bits[28 + i] = (mcs >> i) & 1
    # Beamformed: bit [32]
    data_bits[32] = 0
    # Reserved: bit [33]
    data_bits[33] = 1

    # CRC-8: ones-complement, MSB-first in bits [34:41]
    crc = ht_sig_crc8(data_bits)
    crc_inv = (~crc) & 0xFF
    crc_bits = [((crc_inv >> (7 - i)) & 1) for i in range(8)]

    # Tail: 6 zeros
    tail_bits = [0] * 6

    return data_bits + crc_bits + tail_bits


def generate_vht_sig_a_symbols(mcs: int, length: int, bw: int = 0,
                               short_gi: int = 0,
                               coding: str = "bcc",
                               ldpc_extra: int = 0,
                               sgi_disambig: int = 0) -> tuple[np.ndarray, list[int]]:
    """Generate 2 VHT-SIG-A OFDM symbols (each 80 samples, 160 total).

    Encoding: BCC rate-1/2, legacy OFDM interleaving.
    Symbol 1: BPSK (data on I-axis).
    Symbol 2: Q-BPSK (data on Q-axis).

    Returns: (iq_samples [160], vht_sig_a_bits [48])
    """
    vht_sig_a_bits = make_vht_sig_a_bits(mcs, length, bw, short_gi,
                                         coding=coding, ldpc_extra=ldpc_extra,
                                         sgi_disambig=sgi_disambig)

    # BCC encode: 48 bits → 96 coded bits
    coded = conv_encode(vht_sig_a_bits, add_tail=False)

    # Split into 2 symbols, interleave each (legacy interleaver)
    coded_sym1 = interleave(coded[:48], 48, 1)
    coded_sym2 = interleave(coded[48:], 48, 1)

    # Symbol 1: BPSK (data on I-axis)
    bpsk_sym1 = modulate(coded_sym1, 1)
    # Symbol 2: Q-BPSK (data on Q-axis)
    bpsk_sym2 = modulate(coded_sym2, 1) * 1j

    symbols_out = []
    for sym_idx, data_syms in enumerate([bpsk_sym1, bpsk_sym2]):
        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(DATA_BINS):
            freq[bin_idx] = data_syms[i]
        # Pilots (polarity: L-SIG=0, VHT-SIG-A1=1, VHT-SIG-A2=2)
        pilot_idx = sym_idx + 1
        polarity = PILOT_POLARITY[pilot_idx % 127]
        for pb in PILOT_BINS:
            freq[pb] = polarity * PILOT_BASE[pb]

        time_64 = np.fft.ifft(freq)
        cp = time_64[-NCP:]
        symbols_out.append(np.concatenate([cp, time_64]))

    iq = np.concatenate(symbols_out)
    return iq, vht_sig_a_bits


def make_vht_sig_b_bits(psdu_length: int) -> list[int]:
    """Build 26-bit VHT-SIG-B field for 20 MHz SU.

    Layout: 17-bit LENGTH + 3 reserved (111) + 6 tail (000000).

    LENGTH = ceil(psdu_length / 4) per IEEE 802.11-2020 §21.3.8.3.6.
    The TX must pad the PSDU to LENGTH*4 bytes so the RX can recover
    the correct byte count.

    Returns list of 26 ints (0 or 1).
    """
    length_field = math.ceil(psdu_length / 4)
    bits = [0] * 26

    # LENGTH: bits [0:16] (17 bits, LSB first)
    for i in range(17):
        bits[i] = (length_field >> i) & 1
    # Reserved: bits [17:19] (set to 1,1,1 per TGac convention)
    bits[17] = 1
    bits[18] = 1
    bits[19] = 1
    # Tail: bits [20:25] (6 zeros)
    # Already zero from initialization

    return bits


def generate_vht_sig_b_symbol(psdu_length: int) -> tuple[np.ndarray, list[int]]:
    """Generate 1 VHT-SIG-B OFDM symbol (80 samples).

    Encoding: BPSK, rate-1/2 BCC, HT interleaver (52 data subcarriers).
    IEEE 802.11-2020 §21.3.8.3.6.

    Returns: (iq_samples [80], vht_sig_b_bits [26])
    """
    vht_sig_b_bits = make_vht_sig_b_bits(psdu_length)

    # BCC encode: 26 bits → 52 coded bits (rate-1/2, no puncturing)
    coded = conv_encode(vht_sig_b_bits, add_tail=False)

    # HT interleaver (52 subcarriers, BPSK)
    interleaved = _ht_interleave(coded, 52, 1)

    # BPSK modulate
    symbols = modulate(interleaved, 1)

    freq = np.zeros(NFFT, dtype=complex)
    for i, bin_idx in enumerate(HT_DATA_BINS):
        freq[bin_idx] = symbols[i]

    # Pilots: PILOT_POLARITY[3] * _HT_PILOT_PATTERN[k]
    polarity = PILOT_POLARITY[3]
    for k, pb in enumerate(HT_PILOT_SPEC_BINS):
        freq[pb] = polarity * _HT_PILOT_PATTERN[k]

    time_64 = np.fft.ifft(freq)
    cp = time_64[-NCP:]
    iq = np.concatenate([cp, time_64])
    return iq, vht_sig_b_bits


def generate_vht_stf() -> np.ndarray:
    """Generate VHT-STF (80 samples). Same as HT-STF for 1 SS."""
    return generate_ht_stf()


def generate_vht_ltf() -> np.ndarray:
    """Generate VHT-LTF (80 samples, 1 spatial stream). Same as HT-LTF for 1 SS."""
    return generate_ht_ltf()


def make_vht_data_symbols(psdu_bytes: bytes, mcs: int,
                          scrambler_seed: int = 0x5D,
                          short_gi: bool = False,
                          coding: str = "bcc") -> tuple[list[np.ndarray], int, bytes, int]:
    """Build VHT-DATA OFDM symbols (52 data subcarriers per symbol).

    Nearly identical to make_ht_data_symbols but uses VHT_MCS_TABLE,
    supports MCS 8 (256-QAM 3/4), and uses VHT_PILOT_OFFSET=4.

    VHT SERVICE field (§21.3.10.5): bits 8-15 carry a CRC-8 of VHT-SIG-B
    (first 20 bits, same polynomial as HT-SIG CRC, ones-complemented, MSB-first).

    Args:
        psdu_bytes: PSDU payload bytes (FCS will be appended)
        mcs: VHT MCS index (0–8)
        scrambler_seed: 7-bit scrambler seed
        short_gi: If True, use 8-sample cyclic prefix (400 ns) instead of 16
        coding: "bcc" or "ldpc"

    Returns: (symbols_list, psdu_length_with_fcs, psdu_with_fcs_bytes, ldpc_extra, sgi_disambig)
    """
    mcs_info = VHT_MCS_TABLE[mcs]
    mod_order = mcs_info["mod_order"]
    cr_n = mcs_info["cr_n"]
    cr_d = mcs_info["cr_d"]
    n_cbps = mcs_info["n_cbps"]
    n_dbps = mcs_info["n_dbps"]
    n_bpsc = mcs_info["bpsc"]

    # Pad payload so that psdu_with_fcs is 4-byte aligned
    # (VHT-SIG-B LENGTH = ceil(psdu_length/4), RX extracts LENGTH*4 bytes)
    target_len = math.ceil((len(psdu_bytes) + 4) / 4) * 4  # +4 for FCS
    payload_pad = target_len - 4 - len(psdu_bytes)
    padded_payload = bytes(psdu_bytes) + b'\x00' * payload_pad if payload_pad > 0 else bytes(psdu_bytes)
    fcs = compute_fcs(padded_payload)
    psdu_with_fcs = padded_payload + fcs
    psdu_length = len(psdu_with_fcs)

    # VHT SERVICE field (IEEE 802.11-2020 §21.3.10.5):
    # Bits 0-7: zeros (scrambler init detected from descrambled zeros)
    # Bits 8-15: CRC-8 of VHT-SIG-B (first 20 bits), MSB-first, ones-complemented
    service_bits = [0] * 16
    sigb_bits_for_crc = make_vht_sig_b_bits(psdu_length)[:20]
    sigb_crc = ht_sig_crc8(sigb_bits_for_crc)
    sigb_crc_inv = (~sigb_crc) & 0xFF
    for i in range(8):
        service_bits[8 + i] = (sigb_crc_inv >> (7 - i)) & 1

    psdu_bits = bytes_to_bits_lsb(psdu_with_fcs)

    if coding == "ldpc":
        # LDPC path: no tail bits, no interleaving
        all_bits = service_bits + psdu_bits  # No tail bits for LDPC

        # Scramble (pad to N_SYM * N_DBPS)
        n_pld_needed = len(all_bits)
        n_symbols_init = math.ceil(n_pld_needed / n_dbps)

        # SGI disambiguation for LDPC (same rule as BCC)
        sgi_disambig = 0
        if short_gi and n_symbols_init % 10 == 9:
            sgi_disambig = 1

        n_pad_init = n_symbols_init * n_dbps - n_pld_needed
        all_bits_padded = all_bits + [0] * n_pad_init

        scrambled = scramble(all_bits_padded, scrambler_seed)

        # LDPC encode
        coded_bits, n_symbols, ldpc_extra = _ldpc_encode_data(
            scrambled, n_dbps, n_cbps, cr_n, cr_d
        )

        # If SGI disambig triggered, add extra symbol's worth of padding
        if sgi_disambig:
            n_symbols += 1
            extra_coded = [0] * n_cbps
            coded_bits = coded_bits + extra_coded

        # Build OFDM symbols — NO interleaving for LDPC
        ofdm_symbols = []
        for s in range(n_symbols):
            sym_coded = coded_bits[s * n_cbps:(s + 1) * n_cbps]
            if len(sym_coded) < n_cbps:
                sym_coded += [0] * (n_cbps - len(sym_coded))

            # No interleaving for LDPC — map directly to constellation
            symbols = modulate(sym_coded, mod_order)

            freq = np.zeros(NFFT, dtype=complex)
            for i, bin_idx in enumerate(HT_DATA_BINS):
                freq[bin_idx] = symbols[i]

            # VHT pilots with per-subcarrier cyclic pattern
            polarity = PILOT_POLARITY[(VHT_PILOT_OFFSET + s) % 127]
            for k, pb in enumerate(HT_PILOT_SPEC_BINS):
                freq[pb] = polarity * _HT_PILOT_PATTERN[(s + k) % 4]

            time_64 = np.fft.ifft(freq)
            ncp = NCP_SHORT if short_gi else NCP
            cp = time_64[-ncp:]
            ofdm_symbols.append(np.concatenate([cp, time_64]))

        return ofdm_symbols, psdu_length, psdu_with_fcs, ldpc_extra, sgi_disambig

    # BCC path (original behavior)
    # VHT scrambling convention (per TGac bcc_encoder.m):
    # Scramble SERVICE + PSDU + PAD (no tail in scrambler input).
    # Then append 6 zero tail bits (encoder reset) to the scrambled output.
    n_data_bits = 16 + 8 * psdu_length  # SERVICE + PSDU
    n_symbols = math.ceil((n_data_bits + 6) / n_dbps)

    # SGI disambiguation (IEEE 802.11-2020 §21.3.10.2, Table 21-24):
    # When short GI is used and N_SYM_init % 10 == 9, an extra symbol is needed
    # because the L-SIG LENGTH mapping is ambiguous at these boundary points.
    sgi_disambig = 0
    if short_gi and n_symbols % 10 == 9:
        sgi_disambig = 1
        n_symbols += 1

    n_total = n_symbols * n_dbps
    n_pad = n_total - n_data_bits - 6  # 6 tail bits appended post-scramble

    scrambler_input = service_bits + psdu_bits + [0] * n_pad
    scrambled = scramble(scrambler_input, scrambler_seed) + [0] * 6

    # Convolutional encode
    coded = conv_encode(scrambled, add_tail=False)

    # Compute expected coded bits
    n_coded_per_sym = n_cbps
    total_needed = n_symbols * n_coded_per_sym
    if cr_n == 1 and cr_d == 2:
        needed_pre_puncture = total_needed
    elif cr_n == 2 and cr_d == 3:
        needed_pre_puncture = (total_needed * 4) // 3
    elif cr_n == 3 and cr_d == 4:
        needed_pre_puncture = (total_needed * 6) // 4
    elif cr_n == 5 and cr_d == 6:
        needed_pre_puncture = (total_needed * 10) // 6
    else:
        raise ValueError(f"Unsupported rate {cr_n}/{cr_d}")

    coded = coded[:needed_pre_puncture]

    # Puncture
    punctured = puncture(coded, cr_n, cr_d)
    if len(punctured) < total_needed:
        punctured += [0] * (total_needed - len(punctured))

    # Build OFDM symbols (52 data subcarriers, HT interleaver)
    # VHT pilot formula per IEEE 802.11-2020:
    #   pilot(n, k) = PILOT_POLARITY[(VHT_PILOT_OFFSET + n) % 127] * HT_PILOT_PATTERN[(n + k) % 4]
    # where VHT_PILOT_OFFSET=4 for VHT DATA symbols.
    ofdm_symbols = []
    for s in range(n_symbols):
        sym_coded = punctured[s * n_cbps:(s + 1) * n_cbps]
        if len(sym_coded) < n_cbps:
            sym_coded += [0] * (n_cbps - len(sym_coded))

        sym_interleaved = _ht_interleave(sym_coded, n_cbps, n_bpsc)
        symbols = modulate(sym_interleaved, mod_order)

        freq = np.zeros(NFFT, dtype=complex)
        for i, bin_idx in enumerate(HT_DATA_BINS):
            freq[bin_idx] = symbols[i]

        # VHT pilots with per-subcarrier cyclic pattern
        polarity = PILOT_POLARITY[(VHT_PILOT_OFFSET + s) % 127]
        for k, pb in enumerate(HT_PILOT_SPEC_BINS):
            freq[pb] = polarity * _HT_PILOT_PATTERN[(s + k) % 4]

        time_64 = np.fft.ifft(freq)
        ncp = NCP_SHORT if short_gi else NCP
        cp = time_64[-ncp:]
        ofdm_symbols.append(np.concatenate([cp, time_64]))

    return ofdm_symbols, psdu_length, psdu_with_fcs, 0, sgi_disambig


def generate_vht_frame(mcs: int, psdu_bytes: bytes,
                       scrambler_seed: int = 0x5D,
                       short_gi: bool = False,
                       coding: str = "bcc",
                       windowing: int = 0) -> tuple[np.ndarray, dict]:
    """Generate a complete VHT (802.11ac) frame at baseband.

    Structure: L-STF | L-LTF | L-SIG | VHT-SIG-A (2 sym) | VHT-STF |
               VHT-LTF | VHT-SIG-B | VHT-DATA

    Args:
        mcs: VHT MCS index (0–8)
        psdu_bytes: PSDU payload bytes
        scrambler_seed: 7-bit scrambler seed
        short_gi: If True, use short guard interval (400 ns)
        coding: "bcc" or "ldpc"
        windowing: TX spectral window transition width in samples (0=off).

    Returns:
        (iq_samples, metadata_dict)
    """
    if mcs not in VHT_MCS_TABLE:
        raise ValueError(f"Invalid MCS: {mcs}. Valid: 0–8 (MCS 9 N/A for 20 MHz)")

    # Pad payload so that psdu_with_fcs is 4-byte aligned
    # (VHT-SIG-B LENGTH = ceil(psdu_length/4), RX extracts LENGTH*4 bytes)
    target_len = math.ceil((len(psdu_bytes) + 4) / 4) * 4  # +4 for FCS
    payload_pad = target_len - 4 - len(psdu_bytes)
    padded_payload = bytes(psdu_bytes) + b'\x00' * payload_pad if payload_pad > 0 else bytes(psdu_bytes)
    psdu_with_fcs = padded_payload + compute_fcs(padded_payload)
    psdu_length = len(psdu_with_fcs)

    rate_bits = RATE_TABLE[6]["rate_bits"]
    mcs_info = VHT_MCS_TABLE[mcs]
    n_dbps = mcs_info["n_dbps"]

    # Generate data symbols first to get actual symbol count (includes ldpc_extra)
    data_symbols, _, _, ldpc_extra, sgi_disambig = make_vht_data_symbols(
        psdu_bytes, mcs, scrambler_seed, short_gi=short_gi, coding=coding)
    n_data_sym = len(data_symbols)

    # L-SIG LENGTH encodes NAV protection duration using actual symbol count.
    # For VHT, TXTIME (5 GHz, no signal extension):
    #   TXTIME = T_LEG_PREAMBLE + T_L-SIG + T_VHT-SIG-A + T_VHT_TRAINING + T_SYM * N_SYM
    # where T_VHT_TRAINING = T_VHT-STF + T_VHT-LTF1 = 4 + 4 = 8 µs (1 SS)
    # Plus VHT-SIG-B (4 µs):
    #   TXTIME = 16 + 4 + 8 + 4 + 4 + T_SYM*N_SYM = 40 + T_SYM*N_SYM (µs)
    # Long GI (T_SYM = 4 µs):
    #   L_LENGTH = ceil((TXTIME - 20) / 4) * 3 - 3 = (5 + N_SYM) * 3 - 3
    # Short GI (T_SYM = 3.6 µs, symbolTime = 9/10):
    #   L_LENGTH = 3 * (ceil(N_SYM * 9/10) + 5) - 3
    if short_gi:
        lsig_length = 3 * (math.ceil(n_data_sym * 0.9) + 5) - 3
    else:
        lsig_length = (5 + n_data_sym) * 3 - 3

    # Generate components
    stf = generate_stf()
    ltf = generate_ltf()
    signal_iq, signal_bits, signal_int = generate_signal_symbol(rate_bits, lsig_length)
    vht_sig_a_iq, vht_sig_a_bits = generate_vht_sig_a_symbols(mcs, psdu_length,
                                                                short_gi=int(short_gi),
                                                                coding=coding,
                                                                ldpc_extra=ldpc_extra,
                                                                sgi_disambig=sgi_disambig)
    vht_stf = generate_vht_stf()
    vht_ltf = generate_vht_ltf()
    vht_sig_b_iq, vht_sig_b_bits = generate_vht_sig_b_symbol(psdu_length)

    preamble_parts = [stf, ltf, signal_iq, vht_sig_a_iq, vht_stf, vht_ltf, vht_sig_b_iq]
    preamble = np.concatenate(preamble_parts)
    if windowing > 0 and data_symbols:
        data_iq = apply_tx_windowing(data_symbols, W=windowing)
    else:
        data_iq = np.concatenate(data_symbols) if data_symbols else np.array([], dtype=complex)
    iq = np.concatenate([preamble, data_iq])

    meta = {
        "stf_start": 0,
        "ltf_start": STF_SAMPLES,
        "signal_start": PREAMBLE_SAMPLES,
        "vht_sig_a_start": PREAMBLE_SAMPLES + SYMBOL_SAMPLES,
        "vht_stf_start": PREAMBLE_SAMPLES + 3 * SYMBOL_SAMPLES,
        "vht_ltf_start": PREAMBLE_SAMPLES + 4 * SYMBOL_SAMPLES,
        "vht_sig_b_start": PREAMBLE_SAMPLES + 5 * SYMBOL_SAMPLES,
        "vht_data_start": PREAMBLE_SAMPLES + 6 * SYMBOL_SAMPLES,
        "signal_bits": signal_bits,
        "vht_sig_a_bits": vht_sig_a_bits,
        "vht_sig_b_bits": vht_sig_b_bits,
        "mcs": mcs,
        "short_gi": short_gi,
        "coding": coding,
        "lsig_length": lsig_length,
        "psdu_length": psdu_length,
        "psdu_with_fcs": psdu_with_fcs,
        "psdu_payload": psdu_bytes,
        "scrambler_seed": scrambler_seed,
        "n_data_symbols": len(data_symbols),
        "n_samples": len(iq),
        "sample_rate": SAMPLE_RATE,
    }
    return iq.astype(np.complex64), meta


def apply_tx_windowing(symbols: list[np.ndarray], W: int = 2) -> np.ndarray:
    """Apply raised-cosine overlap-add windowing between OFDM symbols.

    Standard 802.11 TX windowing per IEEE 802.11-2020 §17.3.2.4 and the
    TGac reference generator (fd2td_g.m, App_Win=2, T_TR=100ns → W=2).

    Each symbol is extended by a W-sample cyclic suffix (from the start of
    its FFT period).  This suffix is tapered with a raised-cosine ramp-down.
    The first W samples of the NEXT symbol's cyclic prefix are tapered with
    the complementary ramp-up.  In the overlap region these two tapered
    segments add to produce a smooth crossfade at the symbol boundary.

    Only the first W samples of each symbol's CP are affected.  The receiver's
    FFT window (which starts NCP samples into each symbol period, where
    NCP >= 8 > W) is never touched.

    Output length = sum(symbol lengths).  Same as unwindowed concatenation.

    Args:
        symbols: List of time-domain OFDM symbols (each CP + FFT period).
        W: Window transition width in samples (default 2 = 100ns @ 20 MSPS).

    Returns:
        Windowed baseband waveform (same length as concatenated input).
    """
    if W == 0 or len(symbols) == 0:
        return np.concatenate(symbols) if symbols else np.array([], dtype=complex)

    # Raised-cosine ramps satisfying ramp_up[k] + ramp_down[k] = 1
    ramp_up = 0.5 * (1 - np.cos(np.pi * (np.arange(W) + 0.5) / W))
    ramp_down = ramp_up[::-1]

    n_sym = len(symbols)
    sym_lens = [len(s) for s in symbols]
    total_len = sum(sym_lens)
    out = np.concatenate(symbols).astype(complex)

    # At each inter-symbol boundary, replace the hard transition with a
    # smooth crossfade.  The boundary is between the last sample of symbol i
    # (end of its FFT period) and the first sample of symbol i+1 (start of CP).
    #
    # We do this by:
    # 1. Tapering the last W samples of the FFT period of sym i with ramp_down
    #    (but wait, we DON'T want to modify the FFT period!)
    #
    # Actually, the correct approach that preserves the FFT period:
    # The first W samples of each symbol's CP are replaced with a crossfade
    # between the cyclic suffix of the previous symbol and the original CP.
    # cyclic_suffix_i = fft_i[:W] (cyclic continuation of sym i)
    # cp_start_{i+1} = original first W samples of sym i+1's CP
    # Result: ramp_down * cyclic_suffix_i + ramp_up * cp_start_{i+1}
    #
    # This smooths the transition WITHOUT modifying any FFT period content.

    pos = 0
    for i in range(n_sym - 1):
        pos += sym_lens[i]
        # `pos` is the start of symbol i+1 in the output

        # Cyclic suffix of symbol i: first W samples of its FFT period
        # (what would come after the symbol if the periodic signal continued)
        sym_i = symbols[i]
        ncp_i = sym_lens[i] - NFFT
        fft_i = sym_i[ncp_i:]
        cyclic_suffix = fft_i[:W]

        # First W samples of symbol i+1's CP
        cp_start = out[pos:pos + W].copy()

        # Crossfade: smooth transition from cyclic suffix to actual CP
        out[pos:pos + W] = ramp_down * cyclic_suffix + ramp_up * cp_start

    return out


def generate_vht_ndp() -> tuple[np.ndarray, dict]:
    """Generate a VHT NDP (Null Data Packet) at baseband.

    Structure: L-STF | L-LTF | L-SIG | VHT-SIG-A (2 sym) | VHT-STF | VHT-LTF

    NDP has no VHT-SIG-B or DATA field. Used for beamforming sounding.
    VHT-SIG-A signals MCS=0 with LENGTH=0.
    IEEE 802.11-2020 §21.3.8.3.2.

    Returns:
        (iq_samples, metadata_dict)
    """
    # NDP L-SIG LENGTH: covers VHT preamble duration only.
    # After L-SIG: VHT-SIG-A(2) + VHT-STF(1) + VHT-LTF(1) = 4 symbols = 16 µs
    # L-SIG LENGTH = ceil((TXTIME - 20) / 4) * 3 - 3
    # TXTIME = 20 + 16 = 36 µs → LENGTH = ceil(16/4)*3 - 3 = 9
    lsig_length = 9

    stf = generate_stf()
    ltf = generate_ltf()
    rate_bits = RATE_TABLE[6]["rate_bits"]
    signal_iq, signal_bits, signal_int = generate_signal_symbol(rate_bits, lsig_length)
    vht_sig_a_iq, vht_sig_a_bits = generate_vht_sig_a_symbols(
        mcs=0, length=0, bw=0, short_gi=0, coding="bcc", ldpc_extra=0)
    vht_stf = generate_vht_stf()
    vht_ltf = generate_vht_ltf()

    parts = [stf, ltf, signal_iq, vht_sig_a_iq, vht_stf, vht_ltf]
    iq = np.concatenate(parts)

    meta = {
        "ndp": True,
        "stf_start": 0,
        "ltf_start": STF_SAMPLES,
        "signal_start": PREAMBLE_SAMPLES,
        "vht_sig_a_start": PREAMBLE_SAMPLES + SYMBOL_SAMPLES,
        "vht_stf_start": PREAMBLE_SAMPLES + 3 * SYMBOL_SAMPLES,
        "vht_ltf_start": PREAMBLE_SAMPLES + 4 * SYMBOL_SAMPLES,
        "signal_bits": signal_bits,
        "vht_sig_a_bits": vht_sig_a_bits,
        "lsig_length": lsig_length,
        "n_samples": len(iq),
        "sample_rate": SAMPLE_RATE,
    }
    return iq.astype(np.complex64), meta


# ============================================================================
# Self-test
# ============================================================================
if __name__ == "__main__":
    # Preamble-only test
    for rate in sorted(RATE_TABLE):
        iq, meta = generate_preamble(RATE_TABLE[rate]["rate_bits"], 100)
        print(f"Rate {rate:2d} Mbps preamble: {len(iq)} samples, "
              f"signal_int=0x{meta['signal_int']:06x}")

    # Full frame test (legacy)
    for rate in sorted(RATE_TABLE):
        payload = bytes(32)  # 32 bytes of zeros
        iq, meta = generate_frame(rate, payload)
        fcs_ok = verify_fcs(meta["psdu_with_fcs"])
        print(f"Rate {rate:2d} Mbps: {len(iq)} samples, "
              f"{meta['n_data_symbols']} DATA symbols, "
              f"PSDU={meta['psdu_length']}B, FCS={'OK' if fcs_ok else 'FAIL'}")

    # HT-mixed frame test
    print("\n=== HT-mixed generation ===")
    for mcs in range(8):
        payload = bytes([i % 256 for i in range(50)])
        iq, meta = generate_ht_frame(mcs, payload)
        fcs_ok = verify_fcs(meta["psdu_with_fcs"])
        # Verify HT-SIG CRC
        sig_bits = meta["ht_sig_bits"]
        crc_computed = ht_sig_crc8(sig_bits[:34])
        crc_inv = (~crc_computed) & 0xFF
        crc_in_field = sum(sig_bits[34 + i] << (7 - i) for i in range(8))
        crc_ok = (crc_inv == crc_in_field)
        print(f"  MCS {mcs}: {len(iq)} samples, "
              f"{meta['n_data_symbols']} DATA symbols, "
              f"PSDU={meta['psdu_length']}B, "
              f"FCS={'OK' if fcs_ok else 'FAIL'}, "
              f"HT-SIG CRC={'OK' if crc_ok else 'FAIL'}")

    # VHT frame test
    print("\n=== VHT generation ===")
    for mcs in range(9):
        payload = bytes([i % 256 for i in range(50)])
        iq, meta = generate_vht_frame(mcs, payload)
        fcs_ok = verify_fcs(meta["psdu_with_fcs"])
        print(f"  MCS {mcs}: {len(iq)} samples, "
              f"{meta['n_data_symbols']} DATA symbols, "
              f"PSDU={meta['psdu_length']}B, "
              f"FCS={'OK' if fcs_ok else 'FAIL'}")
