#ifndef LIB80211_FEC_H
#define LIB80211_FEC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * Rate-1/2 K=7 convolutional encoder (G0=133o, G1=171o).
 *
 * Output is interleaved: [g0_0, g1_0, g0_1, g1_1, ...].
 * Output length = 2 * (n_input_bits + 6) if add_tail, else 2 * n_input_bits.
 *
 * @param in_bits       Input data bits (one bit per byte)
 * @param out_bits      Output coded bits (must have room for output length)
 * @param n_input_bits  Number of input bits
 * @param add_tail      If true, append 6 zero tail bits to flush encoder
 */
void lib80211_conv_encode(const uint8_t *in_bits, uint8_t *out_bits,
                          size_t n_input_bits, bool add_tail);

/**
 * Puncture coded bits according to 802.11 puncturing patterns.
 *
 * Supported rates: 1/2 (identity), 2/3, 3/4, 5/6.
 *
 * @param in_bits      Rate-1/2 coded bits
 * @param out_bits     Punctured output
 * @param n_coded_bits Number of input coded bits (must be multiple of pattern length)
 * @param cr_n         Code rate numerator
 * @param cr_d         Code rate denominator
 * @return Number of output bits, or 0 on error
 */
size_t lib80211_puncture(const uint8_t *in_bits, uint8_t *out_bits,
                         size_t n_coded_bits, int cr_n, int cr_d);

/**
 * Depuncture soft bits by inserting erasures (0.0f) at punctured positions.
 *
 * Reverses the puncturing operation for soft-decision decoding.
 * Where the puncture pattern has 1 (kept), copy input soft bit.
 * Where the pattern has 0 (deleted), insert 0.0f (erasure).
 *
 * @param in_soft   Input soft bits (punctured stream)
 * @param out_soft  Output soft bits (rate-1/2 length, with erasures)
 * @param n_input   Number of input soft bits
 * @param cr_n      Code rate numerator
 * @param cr_d      Code rate denominator
 * @return Number of output soft bits, or 0 on error
 */
size_t lib80211_depuncture(const float *in_soft, float *out_soft,
                           size_t n_input, int cr_n, int cr_d);

/**
 * Soft-decision Viterbi decoder (K=7, rate-1/2).
 *
 * Decodes rate-1/2 convolutional code. Input is soft LLRs where
 * positive values indicate bit 1 is more likely.
 * Traceback from state 0 (assumes encoder tail bits flush state to 0).
 *
 * @param soft_bits    Input soft bits (LLRs), length = 2 * (n_data_bits + 6)
 * @param out_bits     Decoded output bits (one bit per byte), length = n_data_bits
 * @param n_coded_bits Number of soft input bits
 * @param n_data_bits  Number of data bits to recover (excluding tail)
 * @return 0 on success, -1 on allocation failure
 */
int lib80211_viterbi_decode(const float *soft_bits, uint8_t *out_bits,
                            size_t n_coded_bits, size_t n_data_bits);

#endif /* LIB80211_FEC_H */
