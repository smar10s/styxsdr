#ifndef LIB80211_SIGNAL_H
#define LIB80211_SIGNAL_H

#include "lib80211/fft.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * Construct the 24-bit L-SIG field (IEEE 802.11-2020 Section 17.3.4).
 *
 * Bit layout: rate_bits[3:0], reserved(0), length[11:0], parity, tail[5:0]
 *
 * @param rate_mbps     Data rate in Mbps (6, 9, 12, 18, 24, 36, 48, 54)
 * @param length_bytes  PSDU length in bytes
 * @param out_bits      Output: 24 bits (one bit per byte)
 * @return 0 on success, -1 if rate is invalid
 */
int lib80211_make_signal_bits(int rate_mbps, int length_bytes, uint8_t *out_bits);

/**
 * Encode and modulate L-SIG field to one OFDM symbol (80 time-domain samples).
 *
 * Pipeline: 24 bits -> rate-1/2 encode (48 coded) -> interleave -> BPSK -> OFDM
 *
 * @param plan          FFT plan
 * @param rate_mbps     Data rate in Mbps
 * @param length_bytes  PSDU length
 * @param out_real      Output: 80 time-domain I samples
 * @param out_imag      Output: 80 time-domain Q samples
 */
void lib80211_make_lsig_symbol(lib80211_fft_plan *plan,
                               int rate_mbps, int length_bytes,
                               float *out_real, float *out_imag);

/* ========================================================================
 * HT-SIG (802.11n, Section 19.3.9.4.3)
 * ======================================================================== */

/**
 * Construct the 48-bit HT-SIG field: 34 data + 8 CRC + 6 tail.
 *
 * @param mcs           HT MCS index (0-7)
 * @param length_bytes  PSDU length in bytes (including FCS)
 * @param short_gi      Short GI flag
 * @param coding_ldpc   true for LDPC, false for BCC
 * @param out_bits      Output: 48 bits (one bit per byte)
 */
void lib80211_make_htsig_bits(int mcs, int length_bytes,
                              bool short_gi, bool coding_ldpc,
                              uint8_t *out_bits);

/**
 * CRC-8 for HT-SIG (poly x^8+x^2+x+1, init 0xFF).
 *
 * @param bits      Input bit array (one bit per byte)
 * @param n_bits    Number of bits (typically 34)
 * @return          8-bit CRC value
 */
uint8_t lib80211_htsig_crc8(const uint8_t *bits, size_t n_bits);

/**
 * Generate 2 HT-SIG OFDM symbols (160 time-domain samples total).
 *
 * Encoding: BCC rate-1/2, legacy interleaving, Q-BPSK modulation.
 * Pilots use standard legacy polarity at symbol indices 1 and 2.
 *
 * @param plan          FFT plan
 * @param mcs           HT MCS index (0-7)
 * @param length_bytes  PSDU length in bytes (including FCS)
 * @param short_gi      Short GI flag
 * @param coding_ldpc   true for LDPC, false for BCC
 * @param out_real      Output: 160 time-domain I samples
 * @param out_imag      Output: 160 time-domain Q samples
 */
void lib80211_make_htsig_symbols(lib80211_fft_plan *plan,
                                 int mcs, int length_bytes,
                                 bool short_gi, bool coding_ldpc,
                                 float *out_real, float *out_imag);

/* ========================================================================
 * VHT-SIG-A (802.11ac, Section 21.3.8.3.3)
 * ======================================================================== */

/**
 * Construct the 48-bit VHT-SIG-A field: 34 data + 8 CRC + 6 tail.
 *
 * @param mcs           VHT MCS index (0-8)
 * @param length_bytes  PSDU length in bytes (including FCS)
 * @param short_gi      Short GI flag
 * @param coding_ldpc   true for LDPC, false for BCC
 * @param ldpc_extra    LDPC extra symbol flag
 * @param out_bits      Output: 48 bits (one bit per byte)
 */
void lib80211_make_vhtsiga_bits(int mcs, int length_bytes, bool short_gi,
                                bool coding_ldpc, int ldpc_extra,
                                uint8_t *out_bits);

/**
 * Generate 2 VHT-SIG-A OFDM symbols (160 time-domain samples total).
 *
 * Symbol 1: BPSK (I-axis). Symbol 2: Q-BPSK (Q-axis).
 * Uses legacy interleaving and legacy subcarrier mapping.
 *
 * @param plan          FFT plan
 * @param mcs           VHT MCS index (0-8)
 * @param length_bytes  PSDU length in bytes (including FCS)
 * @param short_gi      Short GI flag
 * @param coding_ldpc   true for LDPC, false for BCC
 * @param ldpc_extra    LDPC extra symbol flag
 * @param out_real      Output: 160 time-domain I samples
 * @param out_imag      Output: 160 time-domain Q samples
 */
void lib80211_make_vhtsiga_symbols(lib80211_fft_plan *plan,
                                   int mcs, int length_bytes, bool short_gi,
                                   bool coding_ldpc, int ldpc_extra,
                                   float *out_real, float *out_imag);

/* ========================================================================
 * VHT-SIG-B (802.11ac, Section 21.3.8.3.6)
 * ======================================================================== */

/**
 * Construct the 26-bit VHT-SIG-B field (20 MHz):
 * 17-bit LENGTH + 3 reserved + 6 tail.
 *
 * @param psdu_length   PSDU length in bytes
 * @param out_bits      Output: 26 bits (one bit per byte)
 */
void lib80211_make_vhtsigb_bits(int psdu_length, uint8_t *out_bits);

/**
 * Generate VHT-SIG-B OFDM symbol (80 time-domain samples).
 *
 * Encoded: BCC rate-1/2, HT interleaving, BPSK, HT subcarrier map.
 *
 * @param plan          FFT plan
 * @param psdu_length   PSDU length in bytes
 * @param out_real      Output: 80 time-domain I samples
 * @param out_imag      Output: 80 time-domain Q samples
 */
void lib80211_make_vhtsigb_symbol(lib80211_fft_plan *plan,
                                  int psdu_length,
                                  float *out_real, float *out_imag);

#endif /* LIB80211_SIGNAL_H */
