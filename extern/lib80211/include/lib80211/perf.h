#ifndef LIB80211_PERF_H
#define LIB80211_PERF_H

/**
 * perf.h -- Lightweight timing for microbenchmarks.
 *
 * Provides perf_now_ns() returning monotonic nanoseconds.
 * macOS: mach_absolute_time.  Linux: clock_gettime(CLOCK_MONOTONIC).
 */

#include <stdint.h>

#ifdef __APPLE__
#include <mach/mach_time.h>
#else
#include <time.h>
#endif

static inline uint64_t perf_now_ns(void)
{
#ifdef __APPLE__
    static mach_timebase_info_data_t tb;
    if (__builtin_expect(tb.denom == 0, 0))
        mach_timebase_info(&tb);
    return mach_absolute_time() * tb.numer / tb.denom;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
#endif
}

#endif /* LIB80211_PERF_H */
