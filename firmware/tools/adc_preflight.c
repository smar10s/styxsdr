// SPDX-License-Identifier: MIT
/*
 * adc_preflight — establish and verify the ADC sample rate.
 *
 * HIL playback follows the ADC strobe. This tool is the one place that
 * may set the rate; the HIL tools only read it and check the strobe.
 *
 * Usage: adc_preflight [-r rate_hz] [-t timeout_ms] [-q]
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <getopt.h>

#include "hal.h"
#include "styx_tool.h"

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "  Verify that the ADC is streaming before a HIL run.\n"
        "\n"
        "Options:\n"
        "  -r <hz>    Set the ADC sample rate first (0 = leave it alone)\n"
        "  -t <ms>    Strobe wait deadline (default 100)\n"
        "  -q         Quiet: suppress the success message\n"
        "  -h         Show this help\n",
        prog);
}

int main(int argc, char *argv[])
{
    long long set_rate = 0;
    uint32_t timeout_ms = 100;
    bool quiet = false;
    int opt;

    while ((opt = getopt(argc, argv, "r:t:qh")) != -1) {
        switch (opt) {
        case 'r': set_rate = atoll(optarg); break;
        case 't': timeout_ms = (uint32_t)atoi(optarg); break;
        case 'q': quiet = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    if (set_rate > 0) {
        if (hal_ad9361_set_sample_rate((uint64_t)set_rate) != 0) {
            fprintf(stderr, "ERROR: set_sample_rate(%lld) failed\n", set_rate);
            hal_cleanup();
            return 1;
        }
    }

    long long rate = styx_wait_adc_stream(timeout_ms);
    if (rate <= 0) {
        hal_cleanup();
        return 1;
    }

    if (!quiet)
        fprintf(stderr, "ADC streaming at %lld Hz\n", rate);

    hal_cleanup();
    return 0;
}
