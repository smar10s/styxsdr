"""py80211 — 802.11a/g/n/ac OFDM PHY reference implementation.

Pure Python implementation of the 802.11 OFDM PHY (20 MHz, 1 spatial stream):
  - Legacy 802.11a/g: rates 6–54 Mbps (BCC)
  - HT-mixed 802.11n: MCS 0–7 (BCC + LDPC, short/long GI)
  - VHT 802.11ac: MCS 0–8 including 256-QAM (BCC + LDPC, short/long GI)

Provides frame generation (TX), frame decoding (RX), channel impairment
simulation, and composable channel model for multi-frame stream generation.
Validated against IEEE TGn/TGac reference implementations.

Modules:
    gen_ofdm_frame  — Frame generation (TX path)
    decode_frame    — Frame decoding (RX path) from baseband IQ
    impairments     — Channel impairment models (AWGN, CFO, SFO, multipath, etc.)
    channel         — Composable multi-frame channel model with traffic patterns
    ldpc            — LDPC encoder/decoder (all 802.11 code rates and lengths)
"""

__version__ = "0.2.0"
__all__ = ["gen_ofdm_frame", "decode_frame", "impairments", "channel", "ldpc"]
