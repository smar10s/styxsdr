/**
 * ota_rx.c — On-device passive RX capture and decode via libiio.
 *
 * Captures IQ from the AD9361 via libiio, converts int16 → float32 in
 * split I/Q arrays, then runs the lib80211 sync+decode pipeline. Reports
 * frame statistics as JSON (compatible with test_passive_rx_c.py output).
 *
 * Designed for ADALM-Pluto (Cortex-A9, 512 MB RAM, no storage).
 * All buffering is in memory.
 *
 * Memory budget (512 MB total, ~400 MB available):
 *   2s @ 20 MSPS = 40M samples
 *   Float I + Q = 320 MB → too much
 *   Strategy: capture as int16 (160 MB), convert in-place to float (320 MB)
 *             with sequential free. Or just limit to 1.5s default.
 *
 * Usage: ota_rx [options]
 *   -f <freq_hz>    Center frequency (default: 5180000000 = ch36)
 *   -g <gain_db>    RX gain 0-73 (default: 30)
 *   -d <seconds>    Capture duration (default: 2.0)
 *   -b <buf_size>   IIO buffer size in samples (default: 1048576)
 *   -q              Quiet: JSON only
 *   -v              Verbose: print each frame
 *   -h              Help
 */

#include "lib80211/lib80211.h"
#include "lib80211/sync.h"
#include "lib80211/rx.h"
#include "lib80211/fft.h"
#include "lib80211/constants.h"

#include <iio.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>
#include <math.h>

/* ========================================================================
 * Defaults
 * ======================================================================== */

#define DEFAULT_FREQ      5180000000LL   /* ch36 */
#define DEFAULT_GAIN      30
#define DEFAULT_DURATION  2.0
#define DEFAULT_BUF_SIZE  1048576         /* 1M samples per IIO refill */
#define SAMPLE_RATE       20000000LL
#define MAX_FRAMES        65536
#define MAX_SSIDS         256
#define MAX_TYPES         64

/* ========================================================================
 * Frame info (same structure as rx_file.c for JSON compat)
 * ======================================================================== */

typedef struct {
    size_t offset;
    lib80211_frame_type phy_type;
    int rate_mbps;
    int mcs;
    size_t psdu_len;
    bool fcs_ok;
    char frame_name[32];
    char ssid[33];
    bool has_ssid;
    char bssid[18];
    bool has_bssid;
    char rate_label[32];
    uint8_t fc_type;
    uint8_t fc_subtype;
} frame_info;

typedef struct {
    const char *name;
    int count;
} name_count;

/* ========================================================================
 * Signal handling
 * ======================================================================== */

static volatile sig_atomic_t g_stop = 0;

static void sig_handler(int sig)
{
    (void)sig;
    g_stop = 1;
}

/* ========================================================================
 * Frame type names (duplicated from rx_file.c for standalone binary)
 * ======================================================================== */

static const char *frame_type_name(uint8_t type, uint8_t subtype)
{
    if (type == 0) {
        switch (subtype) {
            case 0:  return "AssocReq";
            case 1:  return "AssocResp";
            case 4:  return "ProbeReq";
            case 5:  return "ProbeResp";
            case 8:  return "Beacon";
            case 10: return "Disassoc";
            case 11: return "Auth";
            case 12: return "Deauth";
            case 13: return "Action";
        }
    } else if (type == 1) {
        switch (subtype) {
            case 8:  return "BAR";
            case 9:  return "BA";
            case 11: return "RTS";
            case 12: return "CTS";
            case 13: return "ACK";
        }
    } else if (type == 2) {
        switch (subtype) {
            case 0:  return "Data";
            case 4:  return "NullData";
            case 8:  return "QoSData";
            case 12: return "QoSNull";
        }
    }
    return NULL;
}

/* ========================================================================
 * Rate label
 * ======================================================================== */

static void format_rate_label(char *buf, size_t buf_len,
                              lib80211_frame_type ftype,
                              int rate_mbps, int mcs)
{
    switch (ftype) {
        case LIB80211_FRAME_LEGACY:
            snprintf(buf, buf_len, "%d Mbps", rate_mbps);
            break;
        case LIB80211_FRAME_HT:
            snprintf(buf, buf_len, "HT MCS%d", mcs);
            break;
        case LIB80211_FRAME_VHT:
            snprintf(buf, buf_len, "VHT MCS%d", mcs);
            break;
    }
}

/* ========================================================================
 * JSON string escaping
 * ======================================================================== */

static void json_escape(FILE *fp, const char *str, size_t len)
{
    fputc('"', fp);
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)str[i];
        if (c == '"' || c == '\\' || c < 0x20 || c > 0x7E)
            fprintf(fp, "\\u%04X", c);
        else
            fputc(c, fp);
    }
    fputc('"', fp);
}

/* ========================================================================
 * SSID / BSSID extraction
 * ======================================================================== */

static bool extract_ssid(const uint8_t *psdu, size_t len,
                         char *out, size_t out_len)
{
    if (len < 40) return false;
    size_t ie_off = 36, ie_end = len - 4;
    while (ie_off + 2 <= ie_end) {
        uint8_t id = psdu[ie_off], tlen = psdu[ie_off + 1];
        if (ie_off + 2 + tlen > ie_end) break;
        if (id == 0) {
            size_t n = tlen < out_len - 1 ? tlen : out_len - 1;
            memcpy(out, psdu + ie_off + 2, n);
            out[n] = '\0';
            return true;
        }
        ie_off += 2 + tlen;
    }
    return false;
}

static bool extract_bssid(const uint8_t *psdu, size_t len, char *out)
{
    if (len < 22) return false;
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             psdu[16], psdu[17], psdu[18], psdu[19], psdu[20], psdu[21]);
    return true;
}

/* ========================================================================
 * Usage
 * ======================================================================== */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -f <freq_hz>    Center frequency (default: 5180000000)\n"
        "  -g <gain_db>    RX gain 0-73 (default: 30)\n"
        "  -d <seconds>    Capture duration (default: 2.0)\n"
        "  -b <buf_size>   IIO buffer size in samples (default: 1048576)\n"
        "  -q              Quiet: JSON output only\n"
        "  -v              Verbose: print each frame\n"
        "  -h              Help\n",
        prog);
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(int argc, char *argv[])
{
    long long freq_hz = DEFAULT_FREQ;
    int gain_db = DEFAULT_GAIN;
    double duration_sec = DEFAULT_DURATION;
    int iio_buf_size = DEFAULT_BUF_SIZE;
    bool quiet = false;
    bool verbose = false;

    /* Parse options */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            freq_hz = atoll(argv[++i]);
        } else if (strcmp(argv[i], "-g") == 0 && i + 1 < argc) {
            gain_db = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            duration_sec = atof(argv[++i]);
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            iio_buf_size = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-q") == 0) {
            quiet = true;
        } else if (strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    /* Validate */
    if (gain_db < 0) gain_db = 0;
    if (gain_db > 73) gain_db = 73;
    if (duration_sec <= 0 || duration_sec > 10) {
        fprintf(stderr, "Error: duration must be 0 < d <= 10 seconds\n");
        return 1;
    }

    size_t n_samples = (size_t)(duration_sec * SAMPLE_RATE);

    /* Memory check: float I + Q = n_samples * 8 bytes */
    size_t float_mem = n_samples * 2 * sizeof(float);
    if (float_mem > 350 * 1024 * 1024UL) {
        fprintf(stderr, "Error: %.1fs requires %zu MB, exceeds 350 MB budget\n",
                duration_sec, float_mem / (1024 * 1024));
        return 1;
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* ---- IIO setup ---- */
    if (!quiet)
        fprintf(stderr, "ota_rx: connecting to local IIO context...\n");

    struct iio_context *ctx = iio_create_local_context();
    if (!ctx) {
        fprintf(stderr, "Error: cannot create local IIO context\n");
        return 1;
    }

    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    if (!phy) {
        fprintf(stderr, "Error: ad9361-phy not found\n");
        iio_context_destroy(ctx);
        return 1;
    }

    /* Configure RX */
    struct iio_channel *rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    struct iio_channel *rx_bb = iio_device_find_channel(phy, "voltage0", false);
    if (!rx_lo || !rx_bb) {
        fprintf(stderr, "Error: RX LO or baseband channel not found\n");
        iio_context_destroy(ctx);
        return 1;
    }

    iio_channel_attr_write_longlong(rx_lo, "frequency", freq_hz);
    iio_channel_attr_write_longlong(rx_bb, "sampling_frequency", SAMPLE_RATE);
    iio_channel_attr_write_longlong(rx_bb, "rf_bandwidth", SAMPLE_RATE);
    iio_channel_attr_write(rx_bb, "gain_control_mode", "manual");
    iio_channel_attr_write_longlong(rx_bb, "hardwaregain", (long long)gain_db);

    /* Open RX streaming device */
    struct iio_device *rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    if (!rxdev) {
        fprintf(stderr, "Error: cf-ad9361-lpc (RX DMA) not found\n");
        iio_context_destroy(ctx);
        return 1;
    }

    struct iio_channel *rx_i = iio_device_find_channel(rxdev, "voltage0", false);
    struct iio_channel *rx_q = iio_device_find_channel(rxdev, "voltage1", false);
    if (!rx_i || !rx_q) {
        fprintf(stderr, "Error: RX I/Q channels not found\n");
        iio_context_destroy(ctx);
        return 1;
    }
    iio_channel_enable(rx_i);
    iio_channel_enable(rx_q);

    /* Create RX buffer */
    struct iio_buffer *rxbuf = iio_device_create_buffer(rxdev, iio_buf_size, false);
    if (!rxbuf) {
        fprintf(stderr, "Error: cannot create RX buffer (%d samples)\n", iio_buf_size);
        iio_context_destroy(ctx);
        return 1;
    }

    if (!quiet) {
        fprintf(stderr, "ota_rx: configured %.0f MHz, gain %d dB, %.1fs capture\n",
                freq_hz / 1e6, gain_db, duration_sec);
        fprintf(stderr, "ota_rx: %zu samples (%.1f MB as float)\n",
                n_samples, float_mem / (1024.0 * 1024.0));
    }

    /* ---- Allocate float buffers ---- */
    float *real = (float *)malloc(n_samples * sizeof(float));
    float *imag = (float *)malloc(n_samples * sizeof(float));
    if (!real || !imag) {
        fprintf(stderr, "Error: cannot allocate %.1f MB for IQ buffers\n",
                float_mem / (1024.0 * 1024.0));
        iio_buffer_destroy(rxbuf);
        iio_context_destroy(ctx);
        free(real);
        free(imag);
        return 1;
    }

    /* ---- Flush initial buffers (PLL settling) ---- */
    for (int flush = 0; flush < 3 && !g_stop; flush++) {
        iio_buffer_refill(rxbuf);
    }

    /* ---- Capture ---- */
    if (!quiet)
        fprintf(stderr, "ota_rx: capturing...\n");

    struct timespec t_start, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    size_t captured = 0;
    while (captured < n_samples && !g_stop) {
        ssize_t nbytes = iio_buffer_refill(rxbuf);
        if (nbytes < 0) {
            fprintf(stderr, "Error: iio_buffer_refill: %zd\n", nbytes);
            break;
        }

        /* IIO buffer contains interleaved int16 I/Q (native endian).
         * The AD9361 is 12-bit, sign-extended to 16-bit. We bulk-convert
         * directly without per-sample iio_channel_convert overhead. */
        int16_t *buf_i16 = (int16_t *)iio_buffer_start(rxbuf);
        size_t buf_samples = (size_t)nbytes / 4;  /* 4 bytes per IQ pair */
        size_t to_copy = buf_samples;
        if (captured + to_copy > n_samples)
            to_copy = n_samples - captured;

        for (size_t s = 0; s < to_copy; s++) {
            real[captured + s] = (float)buf_i16[2 * s]     / 2048.0f;
            imag[captured + s] = (float)buf_i16[2 * s + 1] / 2048.0f;
        }
        captured += to_copy;
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end);
    double capture_elapsed = (t_end.tv_sec - t_start.tv_sec) +
                             (t_end.tv_nsec - t_start.tv_nsec) / 1e9;

    iio_buffer_destroy(rxbuf);
    iio_context_destroy(ctx);

    if (!quiet) {
        fprintf(stderr, "ota_rx: captured %zu samples in %.2fs (%.1f MSPS effective)\n",
                captured, capture_elapsed,
                captured / capture_elapsed / 1e6);
    }

    if (captured < 1000) {
        fprintf(stderr, "Error: insufficient samples captured (%zu)\n", captured);
        free(real);
        free(imag);
        return 1;
    }

    /* Trim to actual captured count */
    n_samples = captured;

    /* ---- Decode ---- */
    if (!quiet)
        fprintf(stderr, "ota_rx: decoding...\n");

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "Error: cannot create FFT plan\n");
        free(real);
        free(imag);
        return 1;
    }

    clock_gettime(CLOCK_MONOTONIC, &t_start);

    frame_info *frames = (frame_info *)calloc(MAX_FRAMES, sizeof(frame_info));
    if (!frames) {
        fprintf(stderr, "Error: cannot allocate frame storage\n");
        lib80211_fft_plan_destroy(plan);
        free(real);
        free(imag);
        return 1;
    }

    int n_frames = 0;
    int stf_detections = 0;
    int fcs_ok_count = 0;
    int fcs_fail_count = 0;

    size_t offset = 0;
    while (offset < n_samples && n_frames < MAX_FRAMES) {
        size_t remaining = n_samples - offset;
        if (remaining < 400) break;

        lib80211_sync_result sync_res;
        int sync_rc = lib80211_sync_detect(plan,
                                           real + offset, imag + offset,
                                           remaining, &sync_res);
        if (sync_rc != 0) break;

        stf_detections++;
        size_t frame_pos = offset + sync_res.frame_start;

        size_t decode_remaining = n_samples - frame_pos;
        lib80211_rx_result rx_result;
        int rx_rc = lib80211_rx_decode(plan,
                                       real + frame_pos, imag + frame_pos,
                                       decode_remaining, &rx_result);

        if (rx_rc == 0) {
            frame_info *fi = &frames[n_frames];
            fi->offset = frame_pos;
            fi->phy_type = rx_result.type;
            fi->rate_mbps = rx_result.rate_mbps;
            fi->mcs = rx_result.mcs;
            fi->psdu_len = rx_result.psdu_len;
            fi->fcs_ok = rx_result.fcs_valid;

            if (rx_result.fcs_valid) fcs_ok_count++;
            else fcs_fail_count++;

            format_rate_label(fi->rate_label, sizeof(fi->rate_label),
                              rx_result.type, rx_result.rate_mbps, rx_result.mcs);

            if (rx_result.psdu_len >= 2) {
                fi->fc_type = (rx_result.psdu[0] >> 2) & 0x03;
                fi->fc_subtype = (rx_result.psdu[0] >> 4) & 0x0F;

                const char *name = frame_type_name(fi->fc_type, fi->fc_subtype);
                if (name)
                    strncpy(fi->frame_name, name, sizeof(fi->frame_name) - 1);
                else
                    snprintf(fi->frame_name, sizeof(fi->frame_name),
                             "Type%d_Sub%d", fi->fc_type, fi->fc_subtype);

                if (fi->fc_type == 0)
                    fi->has_bssid = extract_bssid(rx_result.psdu,
                                                  rx_result.psdu_len, fi->bssid);

                if (fi->fc_type == 0 && (fi->fc_subtype == 8 || fi->fc_subtype == 5))
                    fi->has_ssid = extract_ssid(rx_result.psdu,
                                               rx_result.psdu_len,
                                               fi->ssid, sizeof(fi->ssid));
            } else {
                strncpy(fi->frame_name, "Unknown", sizeof(fi->frame_name) - 1);
            }

            if (verbose) {
                fprintf(stderr, "  [%d] @%zu %s %s len=%zu fcs=%s",
                        n_frames, frame_pos, fi->frame_name,
                        fi->rate_label, fi->psdu_len,
                        fi->fcs_ok ? "OK" : "FAIL");
                if (fi->has_ssid) fprintf(stderr, " ssid=\"%s\"", fi->ssid);
                if (fi->has_bssid) fprintf(stderr, " bssid=%s", fi->bssid);
                fprintf(stderr, "\n");
            }

            n_frames++;

            size_t frame_len = 320 + (size_t)rx_result.n_symbols * 80;
            if (frame_len < 320) frame_len = 320;
            offset = frame_pos + frame_len;
        } else {
            offset = frame_pos + 160;
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end);
    double decode_elapsed = (t_end.tv_sec - t_start.tv_sec) +
                            (t_end.tv_nsec - t_start.tv_nsec) / 1e9;

    if (!quiet) {
        fprintf(stderr, "ota_rx: decoded in %.3fs (%.1fx realtime)\n",
                decode_elapsed,
                ((double)n_samples / SAMPLE_RATE) / decode_elapsed);
    }

    /* ---- Build summary ---- */
    char ssids[MAX_SSIDS][33];
    int n_ssids = 0;
    name_count type_counts[MAX_TYPES];
    int n_type_counts = 0;
    name_count rate_counts[MAX_TYPES];
    int n_rate_counts = 0;
    int beacon_count = 0;

    for (int f = 0; f < n_frames; f++) {
        frame_info *fi = &frames[f];

        if (fi->fc_type == 0 && fi->fc_subtype == 8) beacon_count++;

        /* Unique SSIDs */
        if (fi->has_ssid && fi->ssid[0] != '\0') {
            bool found = false;
            for (int s = 0; s < n_ssids; s++) {
                if (strcmp(ssids[s], fi->ssid) == 0) { found = true; break; }
            }
            if (!found && n_ssids < MAX_SSIDS) {
                strncpy(ssids[n_ssids], fi->ssid, 32);
                ssids[n_ssids][32] = '\0';
                n_ssids++;
            }
        }

        /* Frame type counts */
        {
            bool found = false;
            for (int t = 0; t < n_type_counts; t++) {
                if (strcmp(type_counts[t].name, fi->frame_name) == 0) {
                    type_counts[t].count++;
                    found = true;
                    break;
                }
            }
            if (!found && n_type_counts < MAX_TYPES) {
                type_counts[n_type_counts].name = fi->frame_name;
                type_counts[n_type_counts].count = 1;
                n_type_counts++;
            }
        }

        /* Rate counts */
        {
            bool found = false;
            for (int r = 0; r < n_rate_counts; r++) {
                if (strcmp(rate_counts[r].name, fi->rate_label) == 0) {
                    rate_counts[r].count++;
                    found = true;
                    break;
                }
            }
            if (!found && n_rate_counts < MAX_TYPES) {
                rate_counts[n_rate_counts].name = fi->rate_label;
                rate_counts[n_rate_counts].count = 1;
                n_rate_counts++;
            }
        }
    }

    /* ---- JSON output ---- */
    double actual_duration = (double)n_samples / SAMPLE_RATE;
    double fps = n_frames > 0 ? n_frames / actual_duration : 0;

    printf("{\n");
    printf("  \"n_samples\": %zu,\n", n_samples);
    printf("  \"duration_sec\": %.6f,\n", actual_duration);
    printf("  \"capture_time_sec\": %.3f,\n", capture_elapsed);
    printf("  \"decode_time_sec\": %.3f,\n", decode_elapsed);
    printf("  \"realtime_factor\": %.2f,\n",
           actual_duration / (decode_elapsed > 0 ? decode_elapsed : 1));

    /* Frames array */
    printf("  \"frames\": [\n");
    for (int f = 0; f < n_frames; f++) {
        frame_info *fi = &frames[f];
        const char *phy_name = "legacy";
        if (fi->phy_type == LIB80211_FRAME_HT) phy_name = "ht";
        else if (fi->phy_type == LIB80211_FRAME_VHT) phy_name = "vht";

        printf("    {\"offset\": %zu, \"type\": \"%s\", \"rate_mbps\": %d, "
               "\"mcs\": %d, \"psdu_len\": %zu, \"fcs_ok\": %s, "
               "\"frame_name\": \"%s\"",
               fi->offset, phy_name, fi->rate_mbps, fi->mcs,
               fi->psdu_len, fi->fcs_ok ? "true" : "false", fi->frame_name);
        if (fi->has_ssid) {
            printf(", \"ssid\": ");
            json_escape(stdout, fi->ssid, strlen(fi->ssid));
        }
        if (fi->has_bssid)
            printf(", \"bssid\": \"%s\"", fi->bssid);
        printf("}%s\n", f < n_frames - 1 ? "," : "");
    }
    printf("  ],\n");

    /* Summary */
    printf("  \"summary\": {\n");
    printf("    \"stf_detections\": %d,\n", stf_detections);
    printf("    \"frames_decoded\": %d,\n", n_frames);
    printf("    \"fcs_ok\": %d,\n", fcs_ok_count);
    printf("    \"fcs_fail\": %d,\n", fcs_fail_count);
    printf("    \"beacons\": %d,\n", beacon_count);
    printf("    \"fps\": %.1f,\n", fps);

    /* SSIDs */
    printf("    \"ssids\": [");
    for (int s = 0; s < n_ssids; s++) {
        if (s > 0) printf(", ");
        json_escape(stdout, ssids[s], strlen(ssids[s]));
    }
    printf("],\n");

    /* Frame types */
    printf("    \"frame_types\": {");
    for (int t = 0; t < n_type_counts; t++) {
        if (t > 0) printf(", ");
        printf("\"%s\": %d", type_counts[t].name, type_counts[t].count);
    }
    printf("},\n");

    /* Rate distribution */
    printf("    \"rate_dist\": {");
    for (int r = 0; r < n_rate_counts; r++) {
        if (r > 0) printf(", ");
        printf("\"%s\": %d", rate_counts[r].name, rate_counts[r].count);
    }
    printf("}\n");

    printf("  }\n");
    printf("}\n");

    /* ---- Cleanup ---- */
    free(frames);
    lib80211_fft_plan_destroy(plan);
    free(real);
    free(imag);

    /* ---- Human-readable summary to stderr ---- */
    if (!quiet) {
        fprintf(stderr, "\n");
        fprintf(stderr, "========================================\n");
        fprintf(stderr, "OTA RX Results (%.1fs capture)\n", actual_duration);
        fprintf(stderr, "========================================\n");
        fprintf(stderr, "STF detections:  %d\n", stf_detections);
        fprintf(stderr, "Frames decoded:  %d (%.1f fps)\n", n_frames, fps);
        fprintf(stderr, "FCS OK:          %d\n", fcs_ok_count);
        fprintf(stderr, "FCS fail:        %d\n", fcs_fail_count);
        fprintf(stderr, "Beacons:         %d\n", beacon_count);
        fprintf(stderr, "SSIDs:           ");
        for (int s = 0; s < n_ssids; s++)
            fprintf(stderr, "%s%s", s > 0 ? ", " : "", ssids[s]);
        if (n_ssids == 0) fprintf(stderr, "(none)");
        fprintf(stderr, "\n");
        fprintf(stderr, "Decode perf:     %.2fx realtime\n",
                actual_duration / (decode_elapsed > 0 ? decode_elapsed : 1));
        fprintf(stderr, "========================================\n");

        /* Pass criteria */
        bool pass = fcs_ok_count >= 1 && beacon_count >= 1 && n_ssids >= 1;
        fprintf(stderr, "%s\n", pass ? "PASS" : "FAIL");
        fprintf(stderr, "========================================\n");
    }

    return (fcs_ok_count >= 1 && beacon_count >= 1) ? 0 : 1;
}
