#ifndef LIB80211_LDPC_H
#define LIB80211_LDPC_H

#include <stdint.h>
#include <stddef.h>

/**
 * LDPC code parameters.
 *
 * IEEE 802.11-2020 defines 12 LDPC codes:
 *   Codeword lengths: 648, 1296, 1944
 *   Rates: 1/2, 2/3, 3/4, 5/6
 */

/** Maximum codeword length */
#define LIB80211_LDPC_MAX_CW  1944

/** Expansion factors (Z) for each codeword length */
#define LIB80211_LDPC_Z_648   27
#define LIB80211_LDPC_Z_1296  54
#define LIB80211_LDPC_Z_1944  81

/**
 * LDPC parity generator in CSR format (binary, data array omitted).
 *
 * For encoding: parity = P_GEN @ info_bits (mod 2)
 * Codeword = [info_bits | parity_bits]
 *
 * The matrix is (N-K) rows x K columns, stored in CSR:
 *   row_ptr[n_rows+1]: start index of each row in col_idx
 *   col_idx[nnz]:      column indices of 1-entries
 */
typedef struct {
    int n_rows;               /* Number of parity bits (N-K) */
    int n_cols;               /* Number of information bits (K) */
    int nnz;                  /* Number of nonzero entries */
    const uint32_t *row_ptr;  /* Row pointer array [n_rows + 1] */
    const uint16_t *col_idx;  /* Column index array [nnz] */
} lib80211_ldpc_pgen;

/**
 * Get the parity generator matrix for a given code.
 *
 * @param cw_len    Codeword length: 648, 1296, or 1944
 * @param rate_n    Rate numerator: 1, 2, 3, or 5
 * @param rate_d    Rate denominator: 2, 3, 4, or 6
 * @return Pointer to static pgen struct, or NULL if invalid params.
 */
const lib80211_ldpc_pgen *lib80211_ldpc_get_pgen(int cw_len, int rate_n, int rate_d);

/**
 * LDPC systematic encoder.
 *
 * Produces codeword = [info_bits | parity_bits], length = cw_len.
 * LDPC path skips interleaving (bits go directly to constellation mapper).
 *
 * @param info_bits   Input information bits (one bit per byte), length K
 * @param codeword    Output codeword bits (one bit per byte), length cw_len
 * @param cw_len      Codeword length: 648, 1296, or 1944
 * @param rate_n      Rate numerator: 1, 2, 3, or 5
 * @param rate_d      Rate denominator: 2, 3, 4, or 6
 * @return 0 on success, -1 on invalid parameters
 */
int lib80211_ldpc_encode(const uint8_t *info_bits, uint8_t *codeword,
                         int cw_len, int rate_n, int rate_d);

/**
 * LDPC DATA encoding for HT/VHT frames.
 *
 * Implements IEEE 802.11-2020 §19.3.11.7 / §21.3.12.5:
 *   - Codeword length selection
 *   - Shortening and puncturing
 *   - LDPC extra symbol detection
 *   - Multi-codeword encoding
 *
 * The LDPC path has NO tail bits and NO interleaving.
 *
 * @param payload_bits   Scrambled SERVICE + PSDU bits (one bit per byte)
 * @param n_payload      Length of payload_bits
 * @param n_dbps         Data bits per symbol (from MCS table)
 * @param n_cbps         Coded bits per symbol (from MCS table)
 * @param cr_n           Code rate numerator
 * @param cr_d           Code rate denominator
 * @param coded_out      Output coded bits (must hold n_sym * n_cbps bytes)
 * @param n_sym_out      Output: number of OFDM symbols
 * @param ldpc_extra_out Output: 1 if LDPC extra symbol was added, 0 otherwise
 * @return 0 on success, -1 on error
 */
int lib80211_ldpc_encode_data(const uint8_t *payload_bits, int n_payload,
                              int n_dbps, int n_cbps, int cr_n, int cr_d,
                              uint8_t *coded_out, int *n_sym_out,
                              int *ldpc_extra_out);

/**
 * LDPC min-sum belief propagation decoder.
 *
 * Decodes a single codeword from channel LLRs.
 * Convention: positive LLR = bit 0 more likely.
 *
 * WARNING: This is the opposite convention from lib80211_soft_demap() and
 * lib80211_viterbi_decode(), which use positive = bit 1 more likely.
 * Negate LLRs at the boundary when connecting soft_demap -> ldpc_decode.
 *
 * @param llr_in          Input LLRs (length = cw_len)
 * @param decoded_bits    Output hard-decision bits (length = cw_len)
 * @param cw_len          Codeword length: 648, 1296, or 1944
 * @param rate_n          Rate numerator: 1, 2, 3, or 5
 * @param rate_d          Rate denominator: 2, 3, 4, or 6
 * @param max_iterations  Maximum BP iterations (0 = use default 30)
 * @return Number of iterations used (>0 = converged), 0 = failed to converge
 */
int lib80211_ldpc_decode(const float *llr_in, uint8_t *decoded_bits,
                         int cw_len, int rate_n, int rate_d,
                         int max_iterations);

/**
 * LDPC DATA decoding for HT/VHT frames.
 *
 * Reverses lib80211_ldpc_encode_data: de-shortening, de-puncturing,
 * multi-codeword decode. Expects LLRs already negated (positive = bit 0).
 *
 * @param llr_in       Input LLRs (length = total_soft, already negated)
 * @param total_soft   Length of llr_in (= n_symbols * n_cbps)
 * @param decoded_out  Output decoded payload bits (caller allocs n_symbols * n_dbps)
 * @param n_symbols    Number of OFDM data symbols
 * @param n_dbps       Data bits per symbol (from MCS table)
 * @param n_cbps       Coded bits per symbol (from MCS table)
 * @param cr_n         Code rate numerator
 * @param cr_d         Code rate denominator
 * @return 0 on success, -1 on failure
 */
int lib80211_ldpc_decode_data(const float *llr_in, size_t total_soft,
                              uint8_t *decoded_out,
                              int n_symbols, int n_dbps, int n_cbps,
                              int cr_n, int cr_d);

#endif /* LIB80211_LDPC_H */
