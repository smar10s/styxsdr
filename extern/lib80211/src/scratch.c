/**
 * scratch.c -- Bump allocator for lib80211 working memory.
 *
 * Simple, deterministic, zero-fragmentation allocator.  Each top-level
 * API call resets the bump pointer; sub-allocations within a call have
 * non-overlapping lifetimes, so no free is needed.
 */

#include "lib80211/scratch.h"
#include <stdint.h>
#include <string.h>

void lib80211_scratch_init(lib80211_scratch *s, void *mem, size_t len)
{
    if (!s) return;
    s->base = (uint8_t *)mem;
    s->cap  = mem ? len : 0;
    s->pos  = 0;
}

void lib80211_scratch_reset(lib80211_scratch *s)
{
    if (s) s->pos = 0;
}

void *lib80211_scratch_alloc(lib80211_scratch *s, size_t size, size_t align)
{
    if (!s || !s->base || size == 0) return NULL;

    /* Default alignment: 16 bytes (SIMD-friendly) */
    if (align == 0) align = 16;

    /* Align based on absolute address (base may not be aligned) */
    uintptr_t addr = (uintptr_t)(s->base + s->pos);
    uintptr_t mask = (uintptr_t)(align - 1);
    uintptr_t aligned_addr = (addr + mask) & ~mask;
    size_t aligned_pos = s->pos + (size_t)(aligned_addr - addr);

    if (aligned_pos + size > s->cap) return NULL;

    void *ptr = s->base + aligned_pos;
    s->pos = aligned_pos + size;
    return ptr;
}
