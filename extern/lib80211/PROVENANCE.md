# Vector Provenance

This document establishes the origin, generation method, and legal basis for
the golden test vectors distributed with lib80211.

## Why golden vectors matter

IEEE 802.11 defines precise algorithms for PHY-layer encoding and decoding,
but the standard alone is not sufficient to validate an implementation. The
encoding pipeline has many stages (scrambling, FEC, interleaving, modulation,
OFDM symbol assembly) where a subtle bug produces output that *looks*
plausible but decodes incorrectly. Golden vectors -- known-correct
intermediate and final outputs for specific inputs -- are the only practical
way to isolate which stage diverges.

Despite this, golden vectors for 802.11n and 802.11ac are surprisingly hard
to find. The IEEE working groups produced reference implementations, but these
are buried in the document system and rarely packaged for reuse. This project
publishes them as a resource for anyone implementing these PHY layers.

## Vector categories

The `vectors/` directory contains 459 JSON files across four provenance
categories. Every file carries a `"source"` field identifying its origin.

### 1. IEEE 802.11-2020 Annex I.1 (16 files)

Hand-transcribed values from the normative encoding example in IEEE
802.11-2020, Annex I, Section I.1. These tables walk through a complete
Legacy (11a) OFDM transmission at 36 Mbps step by step.

| Source field | Example |
|---|---|
| `"IEEE 802.11-2020, Annex I.1, Table I-{N}"` | `annex_i1_signal_freq.json` |

**Generation method:** Manual transcription from the published standard. Values
are literal copies of spec tables (bit sequences, frequency-domain symbols).

**Legal basis:** These are factual data points defined by a public standard.
The specific numerical values are the *only* correct output for the given
input under the standard's algorithm -- they are not creative expression.

### 2. HT (802.11n) waveforms (160 files)

Complete transmit waveforms and intermediate values for HT MCS 0-7, 20 MHz,
single spatial stream, BCC encoding, with both normal and short guard
interval.

| Source field | Example |
|---|---|
| `"IEEE 802.11 TGn reference waveform generator (11-06/1715r0)"` | `ht_mcs3_scrambled.json` |

**Generation method:** Produced by running the IEEE 802.11 Task Group N (TGn)
reference waveform generator in GNU Octave. The generator was invoked via a
wrapper script that extracted intermediate values at each processing stage and
wrote them as JSON.

**Reference implementation:** IEEE document 11-06/1715r0, available from the
IEEE 802.11 mentor document system:
<https://mentor.ieee.org/802.11/dcn/06/11-06-1715-00-000n-waveform-generator-code.doc>

This is the official TGn reference transmitter submitted to the IEEE 802.11
working group for validating 802.11n implementations.

### 3. VHT (802.11ac) waveforms (270 files)

Complete transmit waveforms and intermediate values for VHT MCS 0-8, 20 MHz,
single spatial stream, BCC encoding, with both normal and short guard
interval.

| Source field | Example |
|---|---|
| `"IEEE 802.11 TGac reference waveform generator (11-14/0571r10, CSR)"` | `vht_mcs5_encoded.json` |

**Generation method:** Produced by running the IEEE 802.11 Task Group AC
(TGac) reference waveform generator in GNU Octave. The generator was invoked
via a wrapper script that extracted intermediate values and wrote them as JSON.

**Reference implementation:** IEEE document 11-14/0571r10, contributed by
Cambridge Silicon Radio (CSR, now Qualcomm), available from the IEEE 802.11
mentor document system:
<https://mentor.ieee.org/802.11/dcn/14/11-14-0571-10-00ac-11ac-waveform-generator-code.doc>

This is the official TGac reference transmitter submitted to the IEEE 802.11
working group for validating 802.11ac implementations.

### 4. Legacy (802.11a) waveforms (12 files)

Complete transmit waveforms for all 8 legacy OFDM rates (6-54 Mbps), plus
extended-length variants for buffer and PLL stress testing.

| Source field | Example |
|---|---|
| `"py80211 gen_ofdm_frame.py (IEEE 802.11-2020 Section 17)"` | `legacy_36mbps_waveform.json` |

**Generation method:** Produced by the project's own `py80211` Python package,
which implements IEEE 802.11-2020 Section 17 (OFDM PHY) independently.
Correctness is validated against the Annex I.1 intermediate values (category 1
above).

**Generator script:** `scripts/gen_legacy_vectors.py` (included in this
repository, MIT licensed).

## How lib80211 was developed

The C library in this repository is an **independent implementation** of the
IEEE 802.11 PHY layer, written from the published standard (IEEE 802.11-2020).
It was not derived from, translated from, or based on any of the reference
implementations listed above.

The golden vectors served as a **validation oracle**: after implementing each
processing stage from the spec, the output was compared against the
corresponding vector to confirm correctness. This is the standard engineering
practice of testing against known-good reference data.

## Legal basis for redistribution

The vectors are **numerical data** representing the mathematically determined
output of algorithms defined by a public standard. They are redistributable
because:

1. **Facts are not copyrightable.** The numerical output of a deterministic
   algorithm applied to a specific input is a fact, not creative expression.
   (Feist v. Rural Telephone, 499 U.S. 340.)

2. **Merger doctrine.** When there is only one way to express an idea (here:
   the correct output of a standards-defined algorithm for a given input), the
   expression merges with the idea and is not protectable.

3. **Independent generation.** The vectors were generated by *running* the
   reference code on specific inputs. The outputs are data produced by the
   code, not copies of the code itself.

The reference implementations used to produce these vectors are **not**
redistributed with this project. They remain available from the IEEE document
system at the URLs listed above.

## Obtaining the reference implementations

If you wish to regenerate the vectors or produce additional test cases:

### TGn reference (802.11n)

1. Download IEEE doc 11-06/1715r0 from:
   <https://mentor.ieee.org/802.11/dcn/06/11-06-1715-00-000n-waveform-generator-code.doc>
2. Extract the MATLAB/Octave source (embedded in the .doc file)
3. Run in GNU Octave (tested with Octave 8.x)

### TGac reference (802.11ac)

1. Download IEEE doc 11-14/0571r10 from:
   <https://mentor.ieee.org/802.11/dcn/14/11-14-0571-10-00ac-11ac-waveform-generator-code.doc>
2. Extract the MATLAB/Octave source (embedded in the .doc file)
3. The CSR disclaimer permits use "as a reference model for IEEE" -- review
   the included `disclaimer_csr.txt` for terms
4. Run in GNU Octave (tested with Octave 8.x)

Note: The TGac reference code is Copyright Cambridge Silicon Radio Ltd and is
provided under terms that do not permit redistribution. It must be obtained
directly from the IEEE document system.

### Legacy waveforms (802.11a)

```
pip install -e python/
python scripts/gen_legacy_vectors.py
```

## Files excluded from this repository

The following files were used during development but are not included in the
public release due to third-party copyright:

| Path (development) | Reason excluded |
|---|---|
| `scripts/octave/tx_n_highlevel.m` | IEEE TGn reference code (11-06/1715r0); unclear redistribution terms |
| `scripts/octave/ieee_tx11ac/` (entire directory) | Copyright Cambridge Silicon Radio Ltd; "non-transferrable" license |
| `scripts/octave/gen_ht_vectors.m` | Derivative work (directly invokes TGn reference) |
| `scripts/octave/gen_ht_sgi_vectors.m` | Derivative work (directly invokes TGn reference) |
| `scripts/octave/gen_vht_vectors.m` | Derivative work (directly invokes TGac reference) |
| `scripts/octave/gen_vht_sgi_vectors.m` | Derivative work (directly invokes TGac reference) |

## Third-party code included

| Path | License | Copyright |
|---|---|---|
| `test/vendor/cJSON.c`, `test/vendor/cJSON.h` | MIT | (c) 2009-2017 Dave Gamble and cJSON contributors |

## Input data

All vectors use one of the following input PSDUs:

- **Annex I.1 payload:** The 100-byte "Ode to Joy" fragment defined in IEEE
  802.11-2020 Annex I.1.2 (used for all HT, VHT, and Annex I.1 vectors)
- **Arbitrary payloads:** 300-byte and 500-byte sequences for the extended
  legacy vectors (generated by the project for buffer stress testing)

Scrambler seeds are documented in each vector's JSON metadata where applicable.
