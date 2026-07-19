#ifndef LIB80211_CONSTANTS_H
#define LIB80211_CONSTANTS_H

#include <stdint.h>
#include <stddef.h>

/* ========================================================================
 * PHY constants
 * ======================================================================== */

#define LIB80211_NFFT           64
#define LIB80211_NCP            16
#define LIB80211_NCP_SHORT      8       /* Short GI: 400 ns at 20 MSPS */
#define LIB80211_SYMBOL_LEN     80      /* NFFT + NCP */
#define LIB80211_SYMBOL_LEN_SGI 72      /* NFFT + NCP_SHORT */
#define LIB80211_SAMPLE_RATE    20000000

#define LIB80211_STF_SAMPLES    160
#define LIB80211_LTF_SAMPLES    160
#define LIB80211_PREAMBLE_SAMPLES 320   /* STF + LTF */

#define LIB80211_N_DATA_SC      48      /* Legacy data subcarriers */
#define LIB80211_N_HT_DATA_SC   52      /* HT/VHT data subcarriers */
#define LIB80211_N_PILOT_SC     4

/* ========================================================================
 * Rate table (802.11a, Table 17-3)
 * ======================================================================== */

typedef struct {
    int rate_mbps;
    uint8_t rate_bits;  /* 4-bit rate field (R1 at LSB) */
    int mod_order;      /* bits per symbol (1=BPSK,2=QPSK,4=16QAM,6=64QAM) */
    int cr_n;           /* code rate numerator */
    int cr_d;           /* code rate denominator */
    int n_cbps;         /* coded bits per symbol */
    int n_dbps;         /* data bits per symbol */
    int n_bpsc;         /* bits per subcarrier (same as mod_order) */
} lib80211_rate_info;

/* 8 legacy rates, indexed 0-7. Use lib80211_rate_lookup() to find by rate_mbps. */
extern const lib80211_rate_info LIB80211_RATE_TABLE[8];

/* Find rate table entry by Mbps. Returns NULL if not found. */
const lib80211_rate_info *lib80211_rate_lookup(int rate_mbps);

/* ========================================================================
 * Subcarrier maps — Legacy (48 data, 4 pilot)
 * ======================================================================== */

/* 48 data subcarrier FFT bin indices (negative freq first, then positive) */
extern const uint8_t LIB80211_DATA_BINS[48];

/* 4 pilot subcarrier FFT bin indices: bins 7, 21, 43, 57 */
extern const uint8_t LIB80211_PILOT_BINS[4];

/* Pilot base values (before polarity): {+1, -1, +1, +1} at bins {7,21,43,57} */
extern const float LIB80211_PILOT_BASE[4];

/* 127-element pilot polarity sequence (Table 17-6) */
extern const int8_t LIB80211_PILOT_POLARITY[127];

/* ========================================================================
 * HT MCS table (802.11n, 20 MHz, 1 SS — Table 19-27)
 * ======================================================================== */

typedef struct {
    int mcs;
    int mod_order;      /* bits per symbol (1=BPSK,2=QPSK,4=16QAM,6=64QAM) */
    int cr_n;           /* code rate numerator */
    int cr_d;           /* code rate denominator */
    int n_cbps;         /* coded bits per symbol (52 * bpsc) */
    int n_dbps;         /* data bits per symbol */
    int n_bpsc;         /* bits per subcarrier */
} lib80211_ht_mcs_info;

/* MCS 0-7, indexed directly. */
extern const lib80211_ht_mcs_info LIB80211_HT_MCS_TABLE[8];

/* ========================================================================
 * HT subcarrier maps (52 data, 4 pilot)
 * ======================================================================== */

/* 52 HT data subcarrier FFT bin indices (-28..-1, +1..+28 minus pilots) */
extern const uint8_t LIB80211_HT_DATA_BINS[52];

/* 4 HT pilot bins in spec order: sc -21, -7, +7, +21 → FFT bins 43, 57, 7, 21 */
extern const uint8_t LIB80211_HT_PILOT_BINS[4];

/* HT per-pilot cyclic pattern (Table 19-10): [1, 1, 1, -1] */
extern const float LIB80211_HT_PILOT_PATTERN[4];

/* HT pilot polarity z_start offset (DATA symbols start at index 3) */
#define LIB80211_HT_PILOT_Z_START 3

/* ========================================================================
 * VHT MCS table (802.11ac, 20 MHz, 1 SS — Table 21-30)
 * ======================================================================== */

/* MCS 0-8, indexed directly. Reuses lib80211_ht_mcs_info struct. */
extern const lib80211_ht_mcs_info LIB80211_VHT_MCS_TABLE[9];

/* VHT pilot polarity z_start offset (DATA symbols start at index 4) */
#define LIB80211_VHT_PILOT_Z_START 4

/* ========================================================================
 * Preamble frequency-domain sequences
 * ======================================================================== */

/* L-STF: 64 complex values (split real/imag) */
extern const float LIB80211_STF_FREQ_REAL[64];
extern const float LIB80211_STF_FREQ_IMAG[64];

/* L-LTF: 64 real values (imag is all zero) */
extern const float LIB80211_LTF_FREQ_REAL[64];

/* HT-LTF: 64 real values (subcarriers -28..+28, imag all zero) */
extern const float LIB80211_HT_LTF_FREQ_REAL[64];

#endif /* LIB80211_CONSTANTS_H */
