/**
 * mac.c -- MAC frame builders for OTA testing.
 *
 * Supported frames: Beacon, Deauth, Association Request, Authentication.
 * Builds raw PSDU byte sequences without FCS. Use lib80211_append_fcs()
 * to add the 4-byte FCS trailer before transmission.
 *
 * Reference: IEEE 802.11-2020 §9.3 (Frame formats).
 */

#include "lib80211/mac.h"
#include "crc32_internal.h"
#include <string.h>

/* ========================================================================
 * Constants
 * ======================================================================== */

const uint8_t LIB80211_BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t LIB80211_DEFAULT_BSSID[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

/* Supported Rates: 6*(0x8C), 9(0x12), 24*(0xB0), 36(0x48), 48(0x60), 54(0x6C) */
static const uint8_t SUPPORTED_RATES[] = {0x8C, 0x12, 0xB0, 0x48, 0x60, 0x6C};

/* RSN IE for WPA2-PSK / CCMP (used in association requests) */
static const uint8_t RSN_IE_WPA2_PSK[] = {
    0x01, 0x00,             /* Version = 1 */
    0x00, 0x0f, 0xac, 0x04, /* Group Cipher Suite: CCMP */
    0x01, 0x00,             /* Pairwise Cipher Suite Count = 1 */
    0x00, 0x0f, 0xac, 0x04, /* Pairwise Cipher Suite: CCMP */
    0x01, 0x00,             /* AKM Suite Count = 1 */
    0x00, 0x0f, 0xac, 0x02, /* AKM Suite: PSK */
    0x00, 0x00,             /* RSN Capabilities */
    0x00, 0x00,             /* PMKID Count = 0 */
};

/* ========================================================================
 * Little-endian helpers (no alignment assumptions)
 * ======================================================================== */

static inline void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static inline void put_le64(uint8_t *p, uint64_t v)
{
    for (int i = 0; i < 8; i++) {
        p[i] = (uint8_t)(v & 0xFF);
        v >>= 8;
    }
}

/* ========================================================================
 * FCS append
 * ======================================================================== */

void lib80211_append_fcs(uint8_t *buf, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc = lib80211_crc32_byte(crc, buf[i]);
    }
    crc ^= 0xFFFFFFFFu;
    /* Write 4 bytes little-endian */
    buf[len + 0] = (uint8_t)(crc & 0xFF);
    buf[len + 1] = (uint8_t)((crc >> 8) & 0xFF);
    buf[len + 2] = (uint8_t)((crc >> 16) & 0xFF);
    buf[len + 3] = (uint8_t)((crc >> 24) & 0xFF);
}

/* ========================================================================
 * Management frame header (24 bytes)
 * ======================================================================== */

static size_t write_mgmt_header(uint8_t *buf, uint16_t fc,
                                const uint8_t *da,
                                const uint8_t *sa,
                                const uint8_t *bssid)
{
    put_le16(buf, fc);          /* Frame Control */
    put_le16(buf + 2, 0);      /* Duration */
    memcpy(buf + 4, da, 6);    /* DA */
    memcpy(buf + 10, sa, 6);   /* SA */
    memcpy(buf + 16, bssid, 6);/* BSSID */
    put_le16(buf + 22, 0);     /* Sequence Control */
    return 24;
}

/* ========================================================================
 * IE (Information Element) helpers
 * ======================================================================== */

static size_t write_ie(uint8_t *buf, uint8_t tag, const uint8_t *data, uint8_t len)
{
    buf[0] = tag;
    buf[1] = len;
    if (len > 0) {
        memcpy(buf + 2, data, len);
    }
    return (size_t)(2 + len);
}

/* ========================================================================
 * Frame builders
 * ======================================================================== */

size_t lib80211_build_beacon(uint8_t *buf, size_t buf_len,
                             const char *ssid,
                             const uint8_t *bssid,
                             uint8_t channel,
                             uint16_t beacon_interval,
                             uint64_t timestamp)
{
    if (!buf || !ssid) return 0;

    const uint8_t *bss = bssid ? bssid : LIB80211_DEFAULT_BSSID;
    size_t ssid_len = strlen(ssid);
    if (ssid_len > 32) return 0;

    /* Calculate total frame size:
     * Header(24) + Timestamp(8) + Interval(2) + Capability(2)
     * + SSID IE(2+ssid_len) + Rates IE(2+6) + DS IE(2+1) + TIM IE(2+4)
     */
    size_t total = 24 + 8 + 2 + 2
                 + (2 + ssid_len)
                 + (2 + sizeof(SUPPORTED_RATES))
                 + (2 + 1)
                 + (2 + 4);

    if (buf_len < total) return 0;

    size_t pos = 0;

    /* Management header: DA=broadcast, SA=BSSID, BSSID=BSSID */
    pos += write_mgmt_header(buf + pos, LIB80211_FC_BEACON,
                             LIB80211_BROADCAST, bss, bss);

    /* Fixed fields */
    put_le64(buf + pos, timestamp);     pos += 8;
    put_le16(buf + pos, beacon_interval); pos += 2;
    put_le16(buf + pos, LIB80211_CAPABILITIES_DEFAULT); pos += 2;

    /* IEs */
    pos += write_ie(buf + pos, 0, (const uint8_t *)ssid, (uint8_t)ssid_len);
    pos += write_ie(buf + pos, 1, SUPPORTED_RATES, sizeof(SUPPORTED_RATES));
    pos += write_ie(buf + pos, 3, &channel, 1);

    uint8_t tim_data[4] = {0, 1, 0, 0};
    pos += write_ie(buf + pos, 5, tim_data, 4);

    return pos;
}

size_t lib80211_build_deauth(uint8_t *buf, size_t buf_len,
                             const uint8_t *da,
                             const uint8_t *sa,
                             const uint8_t *bssid,
                             uint16_t reason)
{
    if (!buf || !da) return 0;

    const size_t DEAUTH_SIZE = 26;  /* 24 header + 2 reason */
    if (buf_len < DEAUTH_SIZE) return 0;

    const uint8_t *bss = bssid ? bssid : LIB80211_DEFAULT_BSSID;
    const uint8_t *source = sa ? sa : bss;

    size_t pos = 0;
    pos += write_mgmt_header(buf + pos, LIB80211_FC_DEAUTH, da, source, bss);
    put_le16(buf + pos, reason); pos += 2;

    return pos;
}

size_t lib80211_build_assoc_req(uint8_t *buf, size_t buf_len,
                                const char *ssid,
                                const uint8_t *bssid,
                                const uint8_t *sta_mac)
{
    if (!buf || !ssid || !bssid || !sta_mac) return 0;

    size_t ssid_len = strlen(ssid);
    if (ssid_len > 32) return 0;

    size_t total = 24                    /* mgmt header */
                 + 2                     /* Capability Info */
                 + 2                     /* Listen Interval */
                 + (2 + ssid_len)        /* SSID IE */
                 + (2 + sizeof(SUPPORTED_RATES))   /* Supported Rates IE */
                 + (2 + sizeof(RSN_IE_WPA2_PSK));  /* RSN IE */

    if (buf_len < total) return 0;

    size_t pos = 0;

    pos += write_mgmt_header(buf + pos, LIB80211_FC_ASSOC_REQ,
                             bssid, sta_mac, bssid);

    put_le16(buf + pos, LIB80211_CAPABILITIES_DEFAULT); pos += 2;
    put_le16(buf + pos, 100);                           pos += 2;

    pos += write_ie(buf + pos, 0, (const uint8_t *)ssid, (uint8_t)ssid_len);
    pos += write_ie(buf + pos, 1, SUPPORTED_RATES, sizeof(SUPPORTED_RATES));
    pos += write_ie(buf + pos, 48, RSN_IE_WPA2_PSK, sizeof(RSN_IE_WPA2_PSK));

    return pos;
}

size_t lib80211_build_auth(uint8_t *buf, size_t buf_len,
                           const uint8_t *bssid,
                           const uint8_t *sta_mac)
{
    if (!buf || !bssid || !sta_mac) return 0;
    if (buf_len < 30) return 0;

    size_t pos = 0;
    pos += write_mgmt_header(buf + pos, LIB80211_FC_AUTH,
                             bssid, sta_mac, bssid);
    put_le16(buf + pos, 0); pos += 2;  /* Algorithm: Open System */
    put_le16(buf + pos, 1); pos += 2;  /* Seq: 1 */
    put_le16(buf + pos, 0); pos += 2;  /* Status: 0 (reserved in request) */

    return pos;
}
