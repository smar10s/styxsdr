#ifndef LIB80211_MAC_H
#define LIB80211_MAC_H

#include <stdint.h>
#include <stddef.h>

/* Frame Control values (little-endian as transmitted) */
#define LIB80211_FC_ASSOC_REQ 0x0000
#define LIB80211_FC_AUTH      0x00B0
#define LIB80211_FC_BEACON    0x0080
#define LIB80211_FC_DEAUTH    0x00C0

/* Default values */
#define LIB80211_BEACON_INTERVAL_DEFAULT  100   /* TU */
#define LIB80211_CAPABILITIES_DEFAULT     0x0431
#define LIB80211_DEAUTH_REASON_DEFAULT    7     /* Class 3 from nonassoc STA */

/* Broadcast address */
extern const uint8_t LIB80211_BROADCAST[6];

/* Locally-administered default BSSID: 02:00:00:00:00:01 */
extern const uint8_t LIB80211_DEFAULT_BSSID[6];

/**
 * Build a Beacon frame PSDU (WITHOUT FCS).
 *
 * Output structure:
 *   FC(2) + Duration(2) + DA(6) + SA(6) + BSSID(6) + SeqCtrl(2) = 24 header
 *   + Timestamp(8) + Beacon Interval(2) + Capability(2) = 12 fixed
 *   + IEs (SSID + Supported Rates + DS Parameter + TIM)
 *
 * @param buf           Output buffer
 * @param buf_len       Buffer capacity
 * @param ssid          SSID string (max 32 bytes, NULL-terminated)
 * @param bssid         6-byte BSSID (NULL = use default)
 * @param channel       DS Parameter channel number
 * @param beacon_interval   Beacon interval in TU
 * @param timestamp     64-bit TSF timestamp
 * @return Bytes written, or 0 on error
 */
size_t lib80211_build_beacon(uint8_t *buf, size_t buf_len,
                             const char *ssid,
                             const uint8_t *bssid,
                             uint8_t channel,
                             uint16_t beacon_interval,
                             uint64_t timestamp);

/**
 * Build a Deauthentication frame PSDU (WITHOUT FCS).
 *
 * Output: 24-byte header + 2-byte reason code = 26 bytes.
 *
 * @param buf       Output buffer (must be >= 26)
 * @param buf_len   Buffer capacity
 * @param da        Destination address (6 bytes)
 * @param sa        Source address (6 bytes, NULL = use bssid)
 * @param bssid     BSSID (6 bytes, NULL = use default)
 * @param reason    Reason code (IEEE 802.11-2020 Table 9-49)
 * @return Bytes written (26), or 0 on error
 */
size_t lib80211_build_deauth(uint8_t *buf, size_t buf_len,
                             const uint8_t *da,
                             const uint8_t *sa,
                             const uint8_t *bssid,
                             uint16_t reason);

/**
 * Build an Association Request frame PSDU (WITHOUT FCS).
 *
 * Output: 24-byte header + Capability(2) + Listen Interval(2)
 *         + SSID IE + Supported Rates IE + RSN IE (WPA2-PSK/CCMP).
 *
 * @param buf       Output buffer
 * @param buf_len   Buffer capacity
 * @param ssid      SSID string (max 32 bytes, NULL-terminated)
 * @param bssid     6-byte BSSID (destination)
 * @param sta_mac   6-byte STA MAC (source address)
 * @return Bytes written, or 0 on error
 */
size_t lib80211_build_assoc_req(uint8_t *buf, size_t buf_len,
                                const char *ssid,
                                const uint8_t *bssid,
                                const uint8_t *sta_mac);

/**
 * Build an Open System Authentication frame PSDU (WITHOUT FCS).
 *
 * Output: 24-byte header + Algorithm(2) + Seq(2) + Status(2) = 30 bytes.
 *
 * @param buf       Output buffer (must be >= 30)
 * @param buf_len   Buffer capacity
 * @param bssid     6-byte BSSID (destination)
 * @param sta_mac   6-byte STA MAC (source address)
 * @return Bytes written (30), or 0 on error
 */
size_t lib80211_build_auth(uint8_t *buf, size_t buf_len,
                           const uint8_t *bssid,
                           const uint8_t *sta_mac);

/**
 * Append FCS (CRC-32) to a frame buffer.
 *
 * Computes CRC-32 over buf[0..len-1] and writes 4 bytes at buf[len].
 * Caller must ensure buf has room for len+4 bytes.
 *
 * @param buf   Frame bytes (FCS will be written at buf[len])
 * @param len   Frame length before FCS
 */
void lib80211_append_fcs(uint8_t *buf, size_t len);

#endif /* LIB80211_MAC_H */
