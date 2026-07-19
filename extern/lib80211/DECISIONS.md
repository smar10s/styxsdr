# DECISIONS — lib80211

Design decisions that constrain implementation choices. Each entry records
a non-obvious constraint that future work must respect.

## D1: OFDM TX scaling — SCALE = 100,000

802.11a OFDM time-domain samples have low peak amplitude (~0.29 per-component
worst case) due to the 64-point IFFT's 1/N normalization. The Pluto's 12-bit
DAC receives 16-bit values with the lower 4 bits discarded (int16 >> 4).

SCALE = 100,000 maps OFDM peaks to ~89% of the 12-bit DAC range. Output
power is then controlled via `tx_hardwaregain_chan0` (attenuation), per ADI's
guidance: drive the DAC near full scale, attenuate in the analog domain.

## D2: 20 MSPS sample rate required for 802.11a

802.11a OFDM occupies 16.25 MHz (52 subcarriers × 312.5 kHz). Sample rates
below 20 MSPS violate Nyquist for the outer subcarriers. Locked at 20 MSPS.

## D3: 64QAM (rates 48/54) under moderate multipath — accepted limitation

Rates 48 and 54 achieve ~80% and ~40% FCS success under moderate multipath
(SNR=20 dB). This is not a decoder bug. 64QAM requires ~20 dB SNR per
subcarrier; when the frequency-selective channel creates nulls (|H[k]| drops
to 0.18), the MMSE equalizer amplifies noise on those subcarriers beyond what
64QAM can tolerate. With only 4 pilot tones, the phase/slope tracking cannot
capture per-subcarrier distortion from a multi-tap channel.

Improving this would require either:
- Time-domain channel smoothing (interpolate H[k] from LTF)
- Decision-directed equalization (update H per symbol)
- Increased MMSE noise regularization (trades bias for noise)

These are optimization opportunities, not correctness issues. The decoder
handles rates 6–36 at 100% under moderate multipath, which covers the
rates used in typical low-SNR/multipath scenarios.

## D4: HT-SIG uses Q-BPSK for both symbols

The IEEE 802.11-2020 spec text (§19.3.9.4.3) describes HT-SIG symbol 1
as "BPSK" and symbol 2 as "QBPSK". However, the IEEE TGn reference
waveform generator (11-06/1715r0) — the de facto standard — uses Q-BPSK
(data on quadrature axis) for both symbols. Pilots remain on the in-phase
axis.

This library matches the reference implementation. Both TX and RX treat
both HT-SIG symbols as Q-BPSK. Validated against all 8 HT-SIG golden
vectors (MCS 0–7).

## D5: HT pilot formula uses per-subcarrier cyclic pattern

HT-DATA pilots differ from legacy. Legacy applies a single scalar
polarity (from the 127-element sequence) to all 4 pilots each symbol.
HT applies a per-pilot cyclic pattern:

    pilot(n, k) = PILOT_POLARITY[(3 + n) % 127] × HT_PILOT_PATTERN[(n + k) % 4]

where HT_PILOT_PATTERN = [1, 1, 1, -1], n is the DATA symbol index,
and k is the pilot index in spec order (k=0: sc -21, k=1: sc -7,
k=2: sc +7, k=3: sc +21). z_start=3 for HT-mixed DATA.

This was reverse-engineered from the TGn reference gen_pilots() function
and validated against golden vectors for all 8 MCS (32 symbols for MCS 0,
16 for MCS 1, etc.).

## D6: HT-SIG reserved bit 26 set to 1

IEEE 802.11-2020 Table 19-11 marks HT-SIG bit 26 as "Reserved". The TGn
reference implementation sets it to 1. Our implementation matches the
reference. This affects the CRC-8 computation (which covers bits 0–33).

## D7: IEEE TGn reference is the source of truth

When the spec text and the reference waveform generator disagree, we
match the reference. The reference output is what real silicon validates
against. Specific deviations documented in D4, D5, D6.

Test validation hierarchy:
1. Golden vectors from IEEE TGn reference (test_tx.py)
2. Full-frame decode of TGn reference waveforms (test_rx.py)
3. TX→RX loopback at all rates/MCS (test_rx.py)
4. Hardware validation with real APs via PlutoSDR (manual)

## D8: No RL-SIG in 802.11ac — VHT uses BPSK/Q-BPSK rotation

"RL-SIG" (repeated L-SIG) does not exist in 802.11ac (VHT). It was
introduced in 802.11ax (HE, Clause 27) and inherited by 802.11be (EHT).

HT vs VHT disambiguation is modulation-based: HT-SIG uses Q-BPSK on
both symbols; VHT-SIG-A uses BPSK on symbol 1 and Q-BPSK on symbol 2.
Both occupy the same position after L-SIG. Detection is by measuring
I-axis vs Q-axis energy on the first post-L-SIG symbol.

Verified against: IEEE 802.11-2020 §21.3.4.5 step (d), §21.3.8.3.3,
Figure 21-4, TGac reference (11-14/0571r10, `preamble_vht_siga.m`
line 128), and all 9 VHT golden waveform vectors (MCS 0–8).

## D9: VHT-SIG-B data-aided channel refinement

Problem: TGac reference generator applies time-domain windowing (fd2td_g.m,
`App_Win=2`) that introduces per-subcarrier amplitude mismatch between the
VHT-LTF training symbol and DATA symbols. At 256-QAM 3/4 (MCS 8), this
causes decode failure because the tight constellation margin cannot absorb
the residual EVM from the stale channel estimate.

Solution: After decoding VHT-SIG-B (which occupies 52 data subcarriers
with known BPSK content), re-encode its bits to produce expected frequency-
domain symbols. Compute H_refined = 0.5 × H_LTF + 0.5 × (Y_sigb / X_sigb)
on each data subcarrier. Use H_refined for all subsequent DATA equalization.

This adds zero cost in clean channels (BPSK reference is identical to LTF-
derived H when there's no drift) and provides ~3 dB effective SNR improvement
at MCS 8 under the windowing artifact.

Result: MCS 8 golden vector now passes (previously failed). Multipath
robustness also improved — all VHT MCS pass under mild multipath at ≥80%.

## D10: VHT scrambler convention differs from legacy/HT

The TGac reference generator (bcc_encoder.m) scrambles SERVICE + PSDU + PAD
without including tail bits in the scrambler input. The 6 zero tail bits are
appended post-scramble as the encoder reset sequence.

Legacy and HT include tail bits in the scrambler input then zero them after
scrambling. The result is functionally equivalent for decode (the tail bits
end up zero either way, and pad bits don't contribute to decoded output), but
produces a different bit pattern in the padding region.

For spec-exact TX generation matching the TGac golden vectors, the VHT
`make_vht_data_symbols` uses the post-append convention.

## D11: VHT pilot formula confirmed

VHT DATA pilots use the same cyclic rotation as HT but with polarity offset 4:
```
pilot(n, k) = PILOT_POLARITY[(4 + n) % 127] × [1, 1, 1, -1][(k + n) % 4]
```
where n = DATA symbol index, k = pilot index in spec order (-21, -7, +7, +21).

Confirmed by exact match against TGac golden vectors across all 32 DATA symbols
for MCS 0, 4, 7 and 3 symbols for MCS 8 (all available symbols). The earlier
report of "69/128 alignment" was incorrect — likely tested with wrong polarity
offset or subcarrier ordering.

