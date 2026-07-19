# ARCHITECTURE.md — lib80211

Portable 802.11 PHY decode library. Decodes 20 MHz single-stream frames
from raw IQ: legacy (802.11a), HT-mixed (802.11n), and VHT (802.11ac).
Target use case: passive capture of beacons, EAPOLs, and management
traffic from commodity APs using PlutoSDR.

## Scope

802.11a/g/n/ac OFDM PHY (20 MHz, 1 spatial stream, BCC + LDPC)

### In scope

- RX decode: L-STF → sync → L-SIG → classify → DATA → FCS
- Legacy 802.11a: rates 6–54 Mbps (all 8)
- HT-mixed 802.11n: MCS 0–7, 20 MHz, single stream, BCC + LDPC
- VHT 802.11ac: MCS 0–8, 20 MHz, single stream, BCC + LDPC
- Frame types: beacon, probe response, EAPOL, any frame with valid FCS
- TX generation: legacy (all rates) + HT-mixed (MCS 0–7) + VHT (MCS 0–8)
- Decision-directed channel refinement (VHT-SIG-B data-aided H update)
- RX corrections: IQ imbalance, phase noise (CPE), SFO via pilot tracking
- Interop: real STA accepts TX frames (legacy + HT)
- Python oracle + C library (Pluto ARM target)

### Out of scope

- MIMO (multiple spatial streams)
- 40/80/160 MHz decode (Pluto maxes at 56 MSPS; 40 MHz capture is
  possible but VHT-DATA at 40 MHz uses 108 subcarriers requiring a
  128-pt FFT — different pipeline. Deferred.)
- DSSS/CCK (802.11b)
- HT-greenfield (deprecated, rare)
- HE/EHT (802.11ax/be)
- Decryption (CCMP/GCMP) — out of scope for PHY library
- MAC-layer state machines (association, retransmission)

### Why this covers beacons and EAPOLs

802.11 mandates that beacons and probe responses are sent at a BSS
basic rate. Modern 5 GHz APs set their basic rate set to include HT
rates — beacons are typically HT MCS 0 (BPSK r=1/2, 6.5 Mbps) in
HT-mixed format.

EAPOL frames are data frames sent during association at a low MCS for
reliability (typically MCS 0–2). They use HT or VHT encoding like any
other data frame.

Both are 20 MHz single-stream on the primary channel. Both decode via
the same HT/VHT-DATA path.

## Frame Structure

```
Legacy 802.11a:
  L-STF (8 µs) | L-LTF (8 µs) | L-SIG (4 µs) | DATA (n × 4 µs)
  160 samples    160 samples     80 samples      n × 80 samples

HT-mixed 802.11n:
  L-STF | L-LTF | L-SIG | HT-SIG1 | HT-SIG2 | HT-STF | HT-LTF | HT-DATA
  160     160     80       80         80         80       80        n × 80

VHT 802.11ac:
  L-STF | L-LTF | L-SIG | VHT-SIG-A1 | VHT-SIG-A2 | VHT-STF | VHT-LTF | VHT-SIG-B | VHT-DATA
  160     160     80       80            80            80         80         80           n × 80
```

All three formats share L-STF + L-LTF + L-SIG. The decode pipeline
shares the entire sync chain (STF detection, CFO estimation, LTF
timing, channel estimation, L-SIG decode). The fork point is after
L-SIG: classify, then dispatch to the appropriate DATA path.

## Classification (After L-SIG)

```
L-SIG decoded (rate, length, parity OK)
  │
  ├─ L-SIG rate must be 6 Mbps for HT/VHT classification (§19.3.9.3.5)
  │
  ├─ Speculatively decode next 2 symbols as HT-SIG / VHT-SIG-A
  │   Both symbols: equalize with L-LTF H, de-rotate by ×(-j), BPSK soft-demap
  │   Concatenate 48+48 = 96 soft bits → Viterbi (rate 1/2) → 48 bits
  │   Check CRC-8: computed over bits [0:33], expected in bits [34:41]
  │   Check tail: bits [42:47] must be 000000
  │
  ├─ CRC OK, parsed as HT-SIG → HT-mixed frame
  │   Parse: MCS, BW, length, aggregation, coding, GI
  │   Proceed to HT-STF → HT-LTF → HT-DATA
  │
  ├─ CRC OK, parsed as VHT-SIG-A → VHT frame
  │   (Same encoding as HT-SIG; different bit field layout)
  │   Heuristic: if HT-SIG MCS > 31, re-parse as VHT-SIG-A
  │   Parse: BW, NSTS, MCS, coding, GI
  │   Proceed to VHT-STF → VHT-LTF → VHT-SIG-B → VHT-DATA
  │
  └─ CRC fails → Legacy 802.11a frame
      Proceed to legacy DATA decode
```

The classification is speculative and non-destructive. If CRC-8 fails,
the frame is legacy — L-SIG rate and length are the actual parameters.
If CRC-8 passes, L-SIG rate and length are NAV protection values (not
the actual data rate).

### HT vs VHT Disambiguation

**Spec-correct approach (§21.3.4.5, §19.3.9.4.3):** HT-SIG and VHT-SIG-A
occupy the same position in the frame (immediately after L-SIG, 2 symbols)
but use different modulation on the first symbol:

- **HT-SIG**: both symbols Q-BPSK (data on Q-axis)
- **VHT-SIG-A**: symbol 1 BPSK (data on I-axis), symbol 2 Q-BPSK

Detection algorithm:
1. Equalize the first post-L-SIG symbol using L-LTF channel estimate
2. Measure I-axis vs Q-axis energy on the 48 data subcarriers
3. I-dominant (>3:1 ratio) → BPSK → VHT-SIG-A (try BPSK/Q-BPSK decode)
4. Q-dominant (>3:1 ratio) → Q-BPSK → HT-SIG (try Q-BPSK/Q-BPSK decode)
5. Ambiguous → try both; accept whichever passes CRC-8
6. Neither passes CRC → legacy frame

Note: there is no "RL-SIG" (repeated L-SIG) in 802.11ac VHT. The VHT
frame structure per Figure 21-4 is:
  L-STF | L-LTF | L-SIG | VHT-SIG-A1 | VHT-SIG-A2 | VHT-STF | VHT-LTF | VHT-SIG-B | DATA

The "RL-SIG" concept appears in 802.11ax (HE) for additional format
disambiguation but is not part of the 802.11ac VHT frame format.

This implementation matches the IEEE TGac reference waveform generator
(11-14/0571r10, CSR) where VHT-SIG-A symbol 1 uses `1i^0 * s_map` (BPSK)
and symbol 2 uses `1i^1 * s_map` (Q-BPSK).

## Decode Pipeline

```
         Shared                    Legacy              HT/VHT
         ──────                    ──────              ──────
  STF detect
  Coarse CFO (STF)
  Fine CFO + LTF timing
  Channel estimate (L-LTF)
  L-SIG decode
         │
         ├── Classify (speculative HT/VHT-SIG decode)
         │
         ├── CRC fail → Legacy DATA decode
         │              48 data subcarriers
         │              Legacy deinterleaver
         │              soft-demap → deinterleave →
         │                depuncture → Viterbi → descramble → FCS
         │
         └── CRC pass → HT/VHT preamble continuation
                         │
                         ├── HT: skip HT-STF (80 samp), est from HT-LTF (80 samp)
                         └── VHT: skip VHT-STF (80 samp), est from VHT-LTF (80 samp)
                              │
                              └── HT/VHT-DATA decode
                                   52 data subcarriers
                                   4 pilots (same positions as legacy)
                                   HT deinterleaver
                                   soft-demap → deinterleave →
                                     depuncture → Viterbi → descramble → FCS
```

## Subcarrier Maps

| Format | FFT size | Data SC | Pilot SC | Null/guard |
|--------|----------|---------|----------|------------|
| Legacy | 64       | 48      | 4        | 12         |
| HT 20  | 64       | 52      | 4        | 8          |
| VHT 20 | 64       | 52      | 4        | 8          |

HT/VHT use subcarriers -28 to -1 and +1 to +28 (excluding pilots at
±7, ±21). Legacy uses -26 to -1 and +1 to +26. The 4 extra data
subcarriers in HT/VHT are at ±27 and ±28.

Pilot positions are the same: ±7, ±21.

## HT-SIG / VHT-SIG-A Encoding

Both use identical PHY encoding:
- 2 OFDM symbols, 48 data subcarriers each (legacy OFDM structure)
- Both symbols: Q-BPSK (data on quadrature axis, pilots on in-phase)
- Rate-1/2 BCC (same convolutional encoder as legacy)
- Legacy OFDM interleaving applied (same as L-SIG: n_cbps=48, n_bpsc=1)
- CRC-8 for validation

Note: Some references state "no interleaving" for HT-SIG. This means no
HT-specific interleaving — the base OFDM interleaver (Equation 17-17)
still applies, same as for L-SIG. Confirmed via real AP captures.

The Q-BPSK convention (both symbols on the Q axis) is per the IEEE TGn
reference waveform generator (11-06/1715r0). The spec text (§19.3.9.4.3)
describes sym1 as "BPSK" and sym2 as "QBPSK", but the reference
implementation and the golden test vectors use Q-BPSK for both.

Decoding:
1. Equalize both symbols using L-LTF channel estimate
2. De-rotate both symbols: multiply by -j (Q→I axis)
3. BPSK soft-demap: LLR = 2 × re (same as L-SIG)
4. Deinterleave each symbol (legacy deinterleaver, n_cbps=48, n_bpsc=1)
5. Concatenate: 96 soft bits
6. Viterbi decode: 48 data bits
7. CRC-8: polynomial x^8+x^2+x+1 (init 0xFF), computed over bits
   [0:33]. Expected value stored ones-complemented, MSB-first in
   bits [34:41]. Tail bits [42:47] = 0.

### HT-SIG Bit Fields (48 bits total)

| Bits  | Field      | Notes |
|-------|------------|-------|
| 0–6   | MCS        | 0–76; 0–7 = single stream 20 MHz |
| 7     | BW         | 0=20 MHz, 1=40 MHz |
| 8–23  | Length     | HT-DATA PSDU length in bytes |
| 24    | Smoothing  | |
| 25    | Not sounding | |
| 26    | Reserved   | Set to 1 per TGn reference |
| 27    | Aggregation | AMPDU if set |
| 28–29 | STBC       | |
| 30    | FEC coding | 0=BCC, 1=LDPC |
| 31    | Short GI   | |
| 32–33 | N_ESS      | Extension spatial streams |
| 34–41 | CRC-8      | |
| 42–47 | Tail       | Must be 000000 |

### VHT-SIG-A Bit Fields (48 bits, 2×24 across symbols)

| Bits (sym1) | Field | Notes |
|-------------|-------|-------|
| 0–1   | BW          | 0=20, 1=40, 2=80, 3=160 |
| 2      | Reserved    | |
| 3      | STBC        | |
| 4–9   | Group ID    | |
| 10–12 | NSTS (SU)   | Spatial streams - 1 |
| 13–21 | Partial AID | |
| 22    | TXOP_PS     | |
| 23    | Reserved    | |

| Bits (sym2) | Field | Notes |
|-------------|-------|-------|
| 0     | Short GI    | |
| 1     | Short GI disambig | |
| 2     | Coding      | 0=BCC, 1=LDPC |
| 3     | LDPC extra  | |
| 4–7   | MCS (SU)    | 0–9 |
| 8     | Beamformed  | |
| 9     | Reserved    | |
| 10–17 | CRC-8       | |
| 18–23 | Tail        | 000000 |

## HT/VHT-DATA Decode Differences from Legacy

| Aspect | Legacy | HT/VHT 20 MHz |
|--------|--------|----------------|
| Data subcarriers | 48 | 52 |
| n_cbps (BPSK r=1/2) | 48 | 52 |
| Deinterleaver | 802.11a formula | 802.11n formula (different N_col) |
| SERVICE field | 16 bits (zeros) | 16 bits (zeros) |
| Tail bits | 6 bits | 6 bits |
| Pilot values | Same 4 pilots, same polarity seq | Per-pilot cyclic pattern (see below) |
| FCS | Same (CRC-32) | Same |
| Scrambler | Same (x^7 + x^4 + 1) | Same |
| Viterbi | Same (K=7, rate 1/2) | Same |
| Puncture patterns | Same | Same |

## HT MCS Parameters (20 MHz, 1 spatial stream, BCC)

| MCS | Modulation | Coding | N_bpsc | N_cbps | N_dbps | Rate (Mbps) |
|-----|-----------|--------|--------|--------|--------|-------------|
| 0   | BPSK      | 1/2    | 1      | 52     | 26     | 6.5         |
| 1   | QPSK      | 1/2    | 2      | 104    | 52     | 13          |
| 2   | QPSK      | 3/4    | 2      | 104    | 78     | 19.5        |
| 3   | 16QAM     | 1/2    | 4      | 208    | 104    | 26          |
| 4   | 16QAM     | 3/4    | 4      | 208    | 156    | 39          |
| 5   | 64QAM     | 2/3    | 6      | 312    | 208    | 52          |
| 6   | 64QAM     | 3/4    | 6      | 312    | 234    | 58.5        |
| 7   | 64QAM     | 5/6    | 6      | 312    | 260    | 65          |

VHT MCS 0–7 at 20 MHz use the same parameters. VHT MCS 8 = 256QAM
r=3/4 (N_bpsc=8, N_cbps=416, N_dbps=312, 78 Mbps). VHT MCS 9 =
256QAM r=5/6 (N_dbps=346.7 — invalid at 20 MHz, not used).

## VHT-SIG-B (20 MHz, single user)

VHT-SIG-B follows VHT-LTF and carries the PSDU length for VHT frames.
For 20 MHz SU: 1 OFDM symbol, BPSK, rate-1/2, 26 data bits:

| Bits  | Field          | Notes |
|-------|----------------|-------|
| 0–16  | VHT-MCS length | PSDU length in units of 4 bytes |
| 17–19 | Reserved       | |
| 20–25 | Tail           | 000000 |

Without VHT-SIG-B, the VHT-DATA PSDU length is unknown. Must be
decoded before VHT-DATA can be descrambled/FCS-checked.

The HT deinterleaver formula (802.11-2020, Section 19.3.11.8.3):
```
N_col = 13 (for 20 MHz HT, Table 19-17)
N_row = 4 × N_bpsc (for 20 MHz HT, Table 19-17)
s = max(N_bpsc / 2, 1) (Equation 19-48)

First permutation (Eq 19-46):
  i = N_row × (k mod N_col) + floor(k / N_col)

Second permutation (Eq 19-47):
  j = s × floor(i / s) + (i + N_cbps - floor(N_col × i / N_cbps)) mod s
```

For single spatial stream, no third permutation (frequency rotation) is
applied. N_rot=11 is only used for multi-stream frequency rotation
(Eq 19-49), which is out of scope for this library.

Note: the second permutation is structurally identical to the legacy
interleaver (§17.3.5.7) but with N_col=13 instead of 16 in the floor
term. For N_bpsc=1 (BPSK), s=1 and the second permutation is identity,
so only the first permutation matters — same behavior as legacy BPSK.

## Key Constants (IEEE 802.11-2020 references)

Primary source: IEEE Std 802.11-2020 (local copy in `docs/802.11-2020.pdf`).
Golden test vectors generated by the IEEE TGn reference waveform generator
(document 11-06/1715r0, in `scripts/octave/tx_n_highlevel.m`).

### Specific Constants

- HT-SIG CRC-8: polynomial x^8+x^2+x+1 (0x07 normal form), init 0xFF,
  no final XOR. IEEE 802.11-2020 §19.3.9.4.4.
- HT subcarrier map (20 MHz): data at -28..-22,-20..-8,-6..-1,+1..+6,
  +8..+20,+22..+28. Pilots at ±7, ±21. Table 19-6.
- HT pilot values: per-pilot cyclic pattern from TGn reference (gen_pilots
  function in 11-06/1715r0). For 20 MHz, 1 SS, 4 pilots at subcarriers
  -21, -7, +7, +21 (spec order k=0..3):
    pilot(n, k) = PILOT_POLARITY[(3 + n) % 127] × HT_PILOT_PATTERN[(n + k) % 4]
  where HT_PILOT_PATTERN = [1, 1, 1, -1] and z_start=3 for HT-MM DATA.
  This differs from legacy where all 4 pilots share a single scalar polarity.
- HT-STF: §19.3.9.4.5 (scaled legacy STF, different amplitude).
- HT-LTF: §19.3.9.4.6, Table 19-25 (52 non-zero subcarriers).
- HT deinterleaver: §19.3.11.8.2 (N_col=13, N_row=4×N_bpsc, N_rot=11
  for 20 MHz stream 0).
- VHT-SIG-A: §21.3.8.3.3.
- VHT-SIG-B: §21.3.8.3.6.
- VHT subcarrier map (20 MHz): same as HT. Table 21-5.
- VHT pilot values (20 MHz, 1 SS): same formula structure as HT but with
  z_start=4 (polarity offset) per ofdmsym_11ac.m:150:
    pilot(n, k) = PILOT_POLARITY[(4 + n) % 127] × HT_PILOT_PATTERN[(n + k) % 4]
  Confirmed by golden vector match on all 32 DATA symbols (MCS 0-7).
- VHT SERVICE field (§21.3.10.5): bits 0-7 zeros (scrambler init), bits 8-15
  carry CRC-8 of VHT-SIG-B (first 20 bits, same polynomial as HT-SIG CRC,
  ones-complemented, MSB-first). Required for spec-exact TX encoding.
- VHT scrambler convention (TGac bcc_encoder.m): scramble SERVICE + PSDU +
  PAD (tail bits NOT included in scrambler input), then append 6 zero tail
  bits post-scramble. Differs from legacy/HT where tail is included then zeroed.
- VHT-SIG-B CRC in SERVICE field used for decision-directed channel refinement:
  re-encode SIG-B → known symbols on 52 subcarriers → refine H for DATA decode.
  This corrects per-subcarrier amplitude drift between VHT-LTF and DATA.

## VHT Golden Vectors

Source: IEEE TGac reference waveform generator (11-14/0571r10, CSR model).
Located in `scripts/octave/ieee_tx11ac/`. Vectors in `vectors/vht_mcs*_*.json`.

**Important:** The 9 VHT waveform JSONs (`vht_mcs*_waveform.json`) were trimmed
of their leading Octave 1-indexing sample. They're gitignored (`*.json`), so
this is a local-only change. If vectors are regenerated from Octave, the
trimming step needs to be reapplied (or fixed in the Octave generation script).

VHT freq_symbols vectors use spec subcarrier ordering (index 0 = sc -32,
index 32 = DC) and are normalized by 1/sqrt(56) where 56 = 52 data + 4 pilot.
This differs from HT vectors which use FFT bin ordering with no normalization.

## LDPC

Encoder and decoder for the 802.11n/ac LDPC codes (IEEE 802.11-2020 §19.3.11.17).

### Encoder

Systematic encoding via pre-computed parity generator matrices. For each
(codeword length, rate) pair, the parity bits are P = M × info_bits where
M is derived from the spec's H matrix. No iterative back-substitution needed
at encode time.

### Decoder

Min-sum belief propagation with early termination. Iteration stops when
syndrome is zero or max iterations reached. Layered scheduling for
convergence speed.

### Parameters

| Codeword length | Rates |
|-----------------|-------|
| 648             | 1/2, 2/3, 3/4, 5/6 |
| 1296            | 1/2, 2/3, 3/4, 5/6 |
| 1944            | 1/2, 2/3, 3/4, 5/6 |

### Integration

- HT-SIG bit 30 selects FEC coding: 0=BCC, 1=LDPC
- VHT-SIG-A symbol 2 bit 2 selects coding: 0=BCC, 1=LDPC
- LDPC path skips the interleaver (LDPC coded bits go directly to
  constellation mapper without interleaving, per §19.3.11.17.3)
- Tone mapping (§20.3.11.17.1) replaces interleaving for LDPC

## RX Corrections

Impairment corrections applied during receive decode.

### IQ Imbalance

Frequency-flat gain/phase imbalance estimated from L-LTF. The known
L-LTF sequence provides a reference to measure differential gain (α)
and phase offset (ε) between I and Q paths. Correction applied as a
2×2 real-valued matrix on all subsequent symbols.

### Phase Noise (CPE)

Instantaneous common phase error estimated from pilots on each OFDM
symbol. No EWMA smoothing on the phase estimate itself — each symbol's
CPE is computed independently from its 4 pilots. EWMA applied only to
the phase slope (SFO tracking), not the intercept.

### SFO (Sampling Frequency Offset)

Handled implicitly by pilot slope tracking. The phase slope across
pilots accumulates linearly with symbol index due to timing drift.
Tracking the slope and applying per-subcarrier phase correction
compensates SFO without explicit resampling.

### VHT Pilot Tracking

Deterministic pilot polarity formula (see Key Constants section).
No blind sign detection — the expected pilot value at each symbol/
subcarrier is computed from the known polarity sequence and cyclic
pattern.
