/*
 * compat_glibc225.c — glibc 2.25 compatibility shims for PlutoSDR.
 *
 * The cross-toolchain (Ubuntu 20.04, glibc 2.31) generates versioned
 * symbol references for log/logf that require GLIBC_2.29. The Pluto
 * has glibc 2.25 which doesn't export those versions.
 *
 * Solution: compile this file with -fno-builtin and link with
 * -Wl,--wrap=log -Wl,--wrap=logf. The linker redirects all log/logf
 * calls to __wrap_log/__wrap_logf defined here.
 *
 * FFTW3's planner is the main consumer of log() — it computes
 * log2(N) for the FFT size. Precision is irrelevant for N=64.
 */

/* Don't include math.h to avoid versioned declarations */
double __wrap_log(double x) {
    if (x <= 0.0) return -1.0e30;
    if (x == 1.0) return 0.0;

    /* Reduce x to [1, 2) */
    int exp = 0;
    double m = x;
    while (m >= 2.0) { m *= 0.5; exp++; }
    while (m < 1.0)  { m *= 2.0; exp--; }

    /* log(m) via atanh series: log(m) = 2*atanh((m-1)/(m+1)) */
    double t = (m - 1.0) / (m + 1.0);
    double t2 = t * t;
    double r = t * (1.0 + t2 * (1.0/3.0 + t2 * (1.0/5.0 +
               t2 * (1.0/7.0 + t2 * (1.0/9.0)))));
    return 2.0 * r + (double)exp * 0.6931471805599453;
}

float __wrap_logf(float x) {
    return (float)__wrap_log((double)x);
}
