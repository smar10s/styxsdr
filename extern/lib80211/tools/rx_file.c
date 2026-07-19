/**
 * rx_file.c — Multi-frame IQ decoder CLI tool.
 *
 * Reads raw cf32 IQ from a file, iterates through it finding and decoding
 * all 802.11 frames, outputs JSON with per-frame details and a summary.
 *
 * Usage: rx_file [options] <input.cf32>
 *   -o <file>   Output JSON to file (default: stdout)
 *   -v          Verbose: print each frame as decoded
 *   -q          Quiet: only JSON output, no progress
 *   -h          Help
 */

#include "lib80211/lib80211.h"
#include "lib80211/sync.h"
#include "lib80211/rx.h"
#include "lib80211/fft.h"
#include "lib80211/constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

/* ========================================================================
 * Frame type name lookup
 * ======================================================================== */

static const char *frame_type_name(uint8_t type, uint8_t subtype)
{
    if (type == 0) { /* Management */
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
    } else if (type == 1) { /* Control */
        switch (subtype) {
            case 8:  return "BAR";
            case 9:  return "BA";
            case 11: return "RTS";
            case 12: return "CTS";
            case 13: return "ACK";
        }
    } else if (type == 2) { /* Data */
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
 * Rate label formatting
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
 * JSON string escaping (non-printable, quotes, backslash → \uXXXX)
 * ======================================================================== */

static void json_escape_string(FILE *fp, const char *str, size_t len)
{
    fputc('"', fp);
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)str[i];
        if (c == '"') {
            fprintf(fp, "\\u%04X", c);
        } else if (c == '\\') {
            fprintf(fp, "\\u%04X", c);
        } else if (c < 0x20 || c > 0x7E) {
            fprintf(fp, "\\u%04X", c);
        } else {
            fputc(c, fp);
        }
    }
    fputc('"', fp);
}

/* ========================================================================
 * Frame info storage
 * ======================================================================== */

#define MAX_FRAMES 65536
#define MAX_SSIDS  256
#define MAX_FRAME_TYPES 64

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
 * SSID extraction from beacon/probe response
 * ======================================================================== */

static bool extract_ssid(const uint8_t *psdu, size_t psdu_len,
                         char *ssid_out, size_t ssid_buf_len)
{
    /* Tagged parameters start at byte 36 */
    if (psdu_len < 40) return false; /* need at least header + some IEs */

    size_t ie_offset = 36;
    size_t ie_end = psdu_len - 4; /* exclude FCS */

    while (ie_offset + 2 <= ie_end) {
        uint8_t tag_id = psdu[ie_offset];
        uint8_t tag_len = psdu[ie_offset + 1];

        if (ie_offset + 2 + tag_len > ie_end) break;

        if (tag_id == 0) { /* SSID */
            size_t copy_len = tag_len;
            if (copy_len >= ssid_buf_len) copy_len = ssid_buf_len - 1;
            memcpy(ssid_out, psdu + ie_offset + 2, copy_len);
            ssid_out[copy_len] = '\0';
            return true;
        }

        ie_offset += 2 + tag_len;
    }
    return false;
}

/* ========================================================================
 * BSSID extraction from management frames
 * ======================================================================== */

static bool extract_bssid(const uint8_t *psdu, size_t psdu_len,
                          char *bssid_out)
{
    /* BSSID at bytes 16-21 (after FC(2) + Duration(2) + DA(6) + SA(6)) */
    if (psdu_len < 22) return false;
    snprintf(bssid_out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             psdu[16], psdu[17], psdu[18], psdu[19], psdu[20], psdu[21]);
    return true;
}

/* ========================================================================
 * Usage
 * ======================================================================== */

static void usage(const char *progname)
{
    fprintf(stderr, "Usage: %s [options] <input.cf32>\n", progname);
    fprintf(stderr, "  -o <file>   Output JSON to file (default: stdout)\n");
    fprintf(stderr, "  -v          Verbose: print each frame as decoded\n");
    fprintf(stderr, "  -q          Quiet: only JSON output, no progress\n");
    fprintf(stderr, "  -h          Help\n");
}

/* ========================================================================
 * Main
 * ======================================================================== */

int main(int argc, char *argv[])
{
    const char *input_file = NULL;
    const char *output_file = NULL;
    bool verbose = false;
    bool quiet = false;

    /* Parse options */
    int i;
    for (i = 1; i < argc; i++) {
        if (argv[i][0] != '-') break;
        if (strcmp(argv[i], "-o") == 0) {
            if (++i >= argc) { usage(argv[0]); return 1; }
            output_file = argv[i];
        } else if (strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "-q") == 0) {
            quiet = true;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    if (i >= argc) {
        fprintf(stderr, "Error: no input file specified\n");
        usage(argv[0]);
        return 1;
    }
    input_file = argv[i];

    /* Load input file */
    FILE *fin = fopen(input_file, "rb");
    if (!fin) {
        fprintf(stderr, "Error: cannot open '%s'\n", input_file);
        return 1;
    }

    fseek(fin, 0, SEEK_END);
    long file_size = ftell(fin);
    fseek(fin, 0, SEEK_SET);

    if (file_size <= 0 || file_size % 8 != 0) {
        fprintf(stderr, "Error: invalid file size %ld (must be multiple of 8 bytes)\n",
                file_size);
        fclose(fin);
        return 1;
    }

    size_t n_samples = (size_t)file_size / 8; /* 4 bytes I + 4 bytes Q per sample */

    float *interleaved = (float *)malloc((size_t)file_size);
    if (!interleaved) {
        fprintf(stderr, "Error: cannot allocate %ld bytes\n", file_size);
        fclose(fin);
        return 1;
    }

    size_t read_count = fread(interleaved, 1, (size_t)file_size, fin);
    fclose(fin);

    if (read_count != (size_t)file_size) {
        fprintf(stderr, "Error: short read (%zu of %ld bytes)\n", read_count, file_size);
        free(interleaved);
        return 1;
    }

    /* Deinterleave into split real[] and imag[] */
    float *real = (float *)malloc(n_samples * sizeof(float));
    float *imag = (float *)malloc(n_samples * sizeof(float));
    if (!real || !imag) {
        fprintf(stderr, "Error: cannot allocate sample buffers\n");
        free(interleaved);
        free(real);
        free(imag);
        return 1;
    }

    for (size_t s = 0; s < n_samples; s++) {
        real[s] = interleaved[2 * s];
        imag[s] = interleaved[2 * s + 1];
    }
    free(interleaved);

    /* Create FFT plan */
    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "Error: cannot create FFT plan\n");
        free(real);
        free(imag);
        return 1;
    }

    if (!quiet) {
        fprintf(stderr, "rx_file: %s (%zu samples, %.3f sec)\n",
                input_file, n_samples,
                (double)n_samples / LIB80211_SAMPLE_RATE);
    }

    /* Frame storage */
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

    /* Iterate through samples finding frames */
    size_t offset = 0;

    while (offset < n_samples && n_frames < MAX_FRAMES) {
        size_t remaining = n_samples - offset;
        if (remaining < 400) break; /* Need at least preamble + signal */

        /* Try to detect a frame */
        lib80211_sync_result sync_res;
        int sync_rc = lib80211_sync_detect(plan,
                                           real + offset, imag + offset,
                                           remaining, &sync_res);

        if (sync_rc != 0) break; /* No more frames */

        stf_detections++;
        size_t frame_pos = offset + sync_res.frame_start;

        /* Try to decode */
        size_t decode_remaining = n_samples - frame_pos;
        lib80211_rx_result rx_result;
        int rx_rc = lib80211_rx_decode(plan,
                                       real + frame_pos, imag + frame_pos,
                                       decode_remaining, &rx_result);

        if (rx_rc == 0) {
            /* Decode success */
            frame_info *fi = &frames[n_frames];
            fi->offset = frame_pos;
            fi->phy_type = rx_result.type;
            fi->rate_mbps = rx_result.rate_mbps;
            fi->mcs = rx_result.mcs;
            fi->psdu_len = rx_result.psdu_len;
            fi->fcs_ok = rx_result.fcs_valid;

            if (rx_result.fcs_valid) fcs_ok_count++;
            else fcs_fail_count++;

            /* Rate label */
            format_rate_label(fi->rate_label, sizeof(fi->rate_label),
                              rx_result.type, rx_result.rate_mbps, rx_result.mcs);

            /* Frame classification from FC bytes */
            if (rx_result.psdu_len >= 2) {
                fi->fc_type = (rx_result.psdu[0] >> 2) & 0x03;
                fi->fc_subtype = (rx_result.psdu[0] >> 4) & 0x0F;

                const char *name = frame_type_name(fi->fc_type, fi->fc_subtype);
                if (name) {
                    strncpy(fi->frame_name, name, sizeof(fi->frame_name) - 1);
                } else {
                    snprintf(fi->frame_name, sizeof(fi->frame_name),
                             "Type%d_Sub%d", fi->fc_type, fi->fc_subtype);
                }

                /* BSSID for management frames */
                if (fi->fc_type == 0) {
                    fi->has_bssid = extract_bssid(rx_result.psdu,
                                                  rx_result.psdu_len,
                                                  fi->bssid);
                }

                /* SSID for beacons (type=0,sub=8) and probe responses (type=0,sub=5) */
                if (fi->fc_type == 0 &&
                    (fi->fc_subtype == 8 || fi->fc_subtype == 5)) {
                    fi->has_ssid = extract_ssid(rx_result.psdu,
                                               rx_result.psdu_len,
                                               fi->ssid, sizeof(fi->ssid));
                }
            } else {
                strncpy(fi->frame_name, "Unknown", sizeof(fi->frame_name) - 1);
            }

            if (verbose) {
                fprintf(stderr, "  [%d] offset=%zu %s rate=%s len=%zu fcs=%s",
                        n_frames, frame_pos, fi->frame_name,
                        fi->rate_label, fi->psdu_len,
                        fi->fcs_ok ? "OK" : "FAIL");
                if (fi->has_ssid)
                    fprintf(stderr, " ssid=\"%s\"", fi->ssid);
                if (fi->has_bssid)
                    fprintf(stderr, " bssid=%s", fi->bssid);
                fprintf(stderr, "\n");
            }

            n_frames++;

            /* Advance past frame: preamble + data symbols */
            size_t frame_len = 320 + (size_t)rx_result.n_symbols * 80;
            if (frame_len < 320) frame_len = 320;
            offset = frame_pos + frame_len;
        } else {
            /* Decode failure: advance one STF period */
            offset = frame_pos + 160;
        }
    }

    if (!quiet) {
        fprintf(stderr, "rx_file: %d STF detections, %d frames decoded "
                "(%d FCS OK, %d FCS fail)\n",
                stf_detections, n_frames, fcs_ok_count, fcs_fail_count);
    }

    /* Build summary data structures */
    /* Unique SSIDs */
    char ssids[MAX_SSIDS][33];
    int n_ssids = 0;

    /* Frame type counts */
    name_count type_counts[MAX_FRAME_TYPES];
    int n_type_counts = 0;

    /* Rate distribution */
    name_count rate_counts[MAX_FRAME_TYPES];
    int n_rate_counts = 0;

    /* Category counts */
    int cat_management = 0, cat_control = 0, cat_data = 0;
    int beacon_count = 0;

    for (int f = 0; f < n_frames; f++) {
        frame_info *fi = &frames[f];

        /* Category */
        if (fi->fc_type == 0) cat_management++;
        else if (fi->fc_type == 1) cat_control++;
        else if (fi->fc_type == 2) cat_data++;

        /* Beacons */
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
            if (!found && n_type_counts < MAX_FRAME_TYPES) {
                type_counts[n_type_counts].name = fi->frame_name;
                type_counts[n_type_counts].count = 1;
                n_type_counts++;
            }
        }

        /* Rate distribution */
        {
            bool found = false;
            for (int r = 0; r < n_rate_counts; r++) {
                if (strcmp(rate_counts[r].name, fi->rate_label) == 0) {
                    rate_counts[r].count++;
                    found = true;
                    break;
                }
            }
            if (!found && n_rate_counts < MAX_FRAME_TYPES) {
                rate_counts[n_rate_counts].name = fi->rate_label;
                rate_counts[n_rate_counts].count = 1;
                n_rate_counts++;
            }
        }
    }

    /* Output JSON */
    FILE *fout = stdout;
    if (output_file) {
        fout = fopen(output_file, "w");
        if (!fout) {
            fprintf(stderr, "Error: cannot open output file '%s'\n", output_file);
            free(frames);
            lib80211_fft_plan_destroy(plan);
            free(real);
            free(imag);
            return 1;
        }
    }

    fprintf(fout, "{\n");
    fprintf(fout, "  \"file\": ");
    json_escape_string(fout, input_file, strlen(input_file));
    fprintf(fout, ",\n");
    fprintf(fout, "  \"n_samples\": %zu,\n", n_samples);
    fprintf(fout, "  \"duration_sec\": %.6f,\n",
            (double)n_samples / LIB80211_SAMPLE_RATE);

    /* Frames array */
    fprintf(fout, "  \"frames\": [\n");
    for (int f = 0; f < n_frames; f++) {
        frame_info *fi = &frames[f];
        fprintf(fout, "    {\n");
        fprintf(fout, "      \"offset\": %zu,\n", fi->offset);

        /* PHY type name */
        const char *phy_name = "legacy";
        if (fi->phy_type == LIB80211_FRAME_HT) phy_name = "ht";
        else if (fi->phy_type == LIB80211_FRAME_VHT) phy_name = "vht";
        fprintf(fout, "      \"type\": \"%s\",\n", phy_name);

        fprintf(fout, "      \"rate_mbps\": %d,\n", fi->rate_mbps);
        fprintf(fout, "      \"mcs\": %d,\n", fi->mcs);
        fprintf(fout, "      \"psdu_len\": %zu,\n", fi->psdu_len);
        fprintf(fout, "      \"fcs_ok\": %s,\n", fi->fcs_ok ? "true" : "false");
        fprintf(fout, "      \"frame_name\": \"%s\"", fi->frame_name);

        if (fi->has_ssid) {
            fprintf(fout, ",\n      \"ssid\": ");
            json_escape_string(fout, fi->ssid, strlen(fi->ssid));
        }
        if (fi->has_bssid) {
            fprintf(fout, ",\n      \"bssid\": \"%s\"", fi->bssid);
        }

        fprintf(fout, "\n    }%s\n", (f < n_frames - 1) ? "," : "");
    }
    fprintf(fout, "  ],\n");

    /* Summary */
    fprintf(fout, "  \"summary\": {\n");
    fprintf(fout, "    \"stf_detections\": %d,\n", stf_detections);
    fprintf(fout, "    \"frames_decoded\": %d,\n", n_frames);
    fprintf(fout, "    \"fcs_ok\": %d,\n", fcs_ok_count);
    fprintf(fout, "    \"fcs_fail\": %d,\n", fcs_fail_count);
    fprintf(fout, "    \"beacons\": %d,\n", beacon_count);

    /* SSIDs array */
    fprintf(fout, "    \"ssids\": [");
    for (int s = 0; s < n_ssids; s++) {
        if (s > 0) fprintf(fout, ", ");
        json_escape_string(fout, ssids[s], strlen(ssids[s]));
    }
    fprintf(fout, "],\n");

    /* Frame types */
    fprintf(fout, "    \"frame_types\": {");
    for (int t = 0; t < n_type_counts; t++) {
        if (t > 0) fprintf(fout, ", ");
        fprintf(fout, "\"%s\": %d", type_counts[t].name, type_counts[t].count);
    }
    fprintf(fout, "},\n");

    /* Rate distribution */
    fprintf(fout, "    \"rate_dist\": {");
    for (int r = 0; r < n_rate_counts; r++) {
        if (r > 0) fprintf(fout, ", ");
        fprintf(fout, "\"%s\": %d", rate_counts[r].name, rate_counts[r].count);
    }
    fprintf(fout, "},\n");

    /* Categories */
    fprintf(fout, "    \"categories\": {");
    bool first_cat = true;
    if (cat_management > 0) {
        fprintf(fout, "\"Management\": %d", cat_management);
        first_cat = false;
    }
    if (cat_control > 0) {
        if (!first_cat) fprintf(fout, ", ");
        fprintf(fout, "\"Control\": %d", cat_control);
        first_cat = false;
    }
    if (cat_data > 0) {
        if (!first_cat) fprintf(fout, ", ");
        fprintf(fout, "\"Data\": %d", cat_data);
    }
    fprintf(fout, "}\n");

    fprintf(fout, "  }\n");
    fprintf(fout, "}\n");

    if (output_file) fclose(fout);

    /* Cleanup */
    free(frames);
    lib80211_fft_plan_destroy(plan);
    free(real);
    free(imag);

    return 0;
}
