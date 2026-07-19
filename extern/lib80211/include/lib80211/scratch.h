#ifndef LIB80211_SCRATCH_H
#define LIB80211_SCRATCH_H

#include <stddef.h>
#include <stdint.h>

/**
 * Bump allocator for working memory.
 *
 * lib80211 data-path functions (RX decode, TX encode) require temporary
 * buffers whose lifetimes don't overlap within a single call.  A bump
 * allocator lets the caller control where that memory lives (static,
 * stack, or heap) while the library uses it with zero malloc/free.
 *
 * Usage (embedded, static allocation):
 *
 *     static uint8_t mem[LIB80211_SCRATCH_SIZE(20000)];
 *     lib80211_scratch scratch;
 *     lib80211_scratch_init(&scratch, mem, sizeof(mem));
 *
 *     lib80211_rx_decode_s(plan, re, im, n, &scratch, &result);
 *
 * Usage (application, heap once at init):
 *
 *     void *mem = malloc(LIB80211_SCRATCH_MAX);
 *     lib80211_scratch scratch;
 *     lib80211_scratch_init(&scratch, mem, LIB80211_SCRATCH_MAX);
 *     // ... use for lifetime of application ...
 *     free(mem);
 */
typedef struct {
    uint8_t *base;
    size_t   cap;
    size_t   pos;   /* current bump offset */
} lib80211_scratch;

/**
 * Initialize a scratch allocator on caller-owned memory.
 */
void lib80211_scratch_init(lib80211_scratch *s, void *mem, size_t len);

/**
 * Reset the scratch allocator (called internally at top of each API call).
 * Also safe to call externally between calls.
 */
void lib80211_scratch_reset(lib80211_scratch *s);

/**
 * Allocate `size` bytes aligned to `align` from the scratch buffer.
 * Returns NULL if insufficient space remains.
 * `align` must be a power of two; 0 means default (16-byte alignment).
 */
void *lib80211_scratch_alloc(lib80211_scratch *s, size_t size, size_t align);

/* ========================================================================
 * Sizing macros
 *
 * These compute the worst-case scratch requirement for a given max
 * sample count.  Use LIB80211_SCRATCH_MAX for "any legal frame."
 * Use LIB80211_SCRATCH_SIZE(n) when you know the max frame length
 * (e.g. 20000 samples covers all frames up to ~1500-byte PSDU at any rate).
 * ======================================================================== */

/* Max soft bits: VHT MCS 8 (256-QAM 3/4), 4095-byte PSDU:
 * ceil((16+32760+6)/(234)) = 141 symbols, 416 cbps = 58656 soft floats.
 * But MCS 0 (BPSK 1/2) has more symbols: ceil((16+32760+6)/26) = 1261,
 * 52 cbps = 65572.  Use 416*1261 = 524576 as safe upper bound. */
#define LIB80211_SCRATCH_SOFT_MAX   (416 * 1261)

/* Max coded bits in TX pipeline (VHT): 2 * (16+32760+6+416) = 66396 */
#define LIB80211_SCRATCH_CODED_MAX  66396

/* Max data bits in TX pipeline */
#define LIB80211_SCRATCH_DATA_MAX   33198

/* LDPC iteration state: R (edges*4) + L (1944*4) + var_map (edges*2)
 * Worst case: 1944-bit CW, rate 1/2, nnz=144*Z=11664 edges
 * R: 46656 bytes, L: 7776 bytes, var_map: 23328 bytes = ~78 KB */
#define LIB80211_SCRATCH_LDPC_MAX   (48 * 1024)

#define LIB80211_SCRATCH_RX_SIZE(max_samples) \
    (2 * (max_samples) * sizeof(float)                  /* IQ work buffers */       \
   + (size_t)LIB80211_SCRATCH_SOFT_MAX * sizeof(float)  /* soft bits */             \
   + (size_t)LIB80211_SCRATCH_DATA_MAX                  /* decoded bits */          \
   + (size_t)LIB80211_SCRATCH_LDPC_MAX                  /* LDPC iteration state */  \
   + 4096)                                              /* alignment headroom */

#define LIB80211_SCRATCH_TX_SIZE \
    (2 * (size_t)LIB80211_SCRATCH_CODED_MAX * sizeof(float)  /* coded + punctured */  \
   + 2 * (size_t)LIB80211_SCRATCH_DATA_MAX                   /* data + scrambled */   \
   + 4096)                                                   /* alignment headroom */

#define LIB80211_SCRATCH_SIZE(max_samples) \
    ((LIB80211_SCRATCH_RX_SIZE(max_samples) > LIB80211_SCRATCH_TX_SIZE) \
     ? LIB80211_SCRATCH_RX_SIZE(max_samples)                            \
     : LIB80211_SCRATCH_TX_SIZE)

/* Any legal 802.11a/n/ac frame (worst case: VHT MCS 0, 4095-byte PSDU,
 * ~102000 samples including preamble) */
#define LIB80211_SCRATCH_MAX  LIB80211_SCRATCH_SIZE(102000)

#endif /* LIB80211_SCRATCH_H */
