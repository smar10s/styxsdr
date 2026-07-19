/**
 * test_mac.c -- Tests for MAC frame builders (beacon + deauth + FCS).
 */

#include "test_util.h"
#include "lib80211/mac.h"
#include "lib80211/rx.h"

#include <stdio.h>
#include <string.h>

/* ========================================================================
 * Test: Beacon frame byte-by-byte structure
 * ======================================================================== */

static void test_beacon_structure(void) {
    TEST_BEGIN("beacon_structure");

    uint8_t buf[256];
    memset(buf, 0xAA, sizeof(buf));  /* poison */

    const char *ssid = "lib80211";
    uint8_t bssid[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    uint8_t channel = 36;
    uint16_t interval = 100;
    uint64_t timestamp = 0x0000000012345678ULL;

    size_t len = lib80211_build_beacon(buf, sizeof(buf),
                                       ssid, bssid, channel,
                                       interval, timestamp);

    /* Expected size:
     * 24 header + 8 timestamp + 2 interval + 2 capability
     * + (2+8) SSID + (2+6) rates + (2+1) DS + (2+4) TIM
     * = 24 + 12 + 10 + 8 + 3 + 6 = 63
     */
    size_t expected_len = 24 + 12 + (2+8) + (2+6) + (2+1) + (2+4);
    if (len != expected_len) {
        TEST_FAIL("beacon length: got %zu, expected %zu", len, expected_len);
        return;
    }

    /* Check Frame Control = 0x0080 (LE) */
    if (buf[0] != 0x80 || buf[1] != 0x00) {
        TEST_FAIL("FC: got 0x%02X%02X, expected 0x8000", buf[1], buf[0]);
        return;
    }

    /* Duration = 0 */
    if (buf[2] != 0x00 || buf[3] != 0x00) {
        TEST_FAIL("Duration not zero");
        return;
    }

    /* DA = broadcast */
    for (int i = 0; i < 6; i++) {
        if (buf[4+i] != 0xFF) {
            TEST_FAIL("DA[%d] = 0x%02X, expected 0xFF", i, buf[4+i]);
            return;
        }
    }

    /* SA = BSSID */
    if (memcmp(buf + 10, bssid, 6) != 0) {
        TEST_FAIL("SA != BSSID");
        return;
    }

    /* BSSID field */
    if (memcmp(buf + 16, bssid, 6) != 0) {
        TEST_FAIL("BSSID mismatch");
        return;
    }

    /* SeqCtrl = 0 */
    if (buf[22] != 0x00 || buf[23] != 0x00) {
        TEST_FAIL("SeqCtrl not zero");
        return;
    }

    /* Timestamp at offset 24 (LE) */
    uint64_t got_ts = 0;
    for (int i = 0; i < 8; i++)
        got_ts |= (uint64_t)buf[24+i] << (i*8);
    if (got_ts != timestamp) {
        TEST_FAIL("timestamp mismatch");
        return;
    }

    /* Beacon interval at offset 32 */
    uint16_t got_bi = buf[32] | ((uint16_t)buf[33] << 8);
    if (got_bi != interval) {
        TEST_FAIL("beacon interval: got %u, expected %u", got_bi, interval);
        return;
    }

    /* Capability at offset 34 */
    uint16_t got_cap = buf[34] | ((uint16_t)buf[35] << 8);
    if (got_cap != 0x0431) {
        TEST_FAIL("capability: got 0x%04X, expected 0x0431", got_cap);
        return;
    }

    /* SSID IE at offset 36: tag=0, len=8, "lib80211" */
    size_t ie_pos = 36;
    if (buf[ie_pos] != 0 || buf[ie_pos+1] != 8) {
        TEST_FAIL("SSID IE header: tag=%u len=%u", buf[ie_pos], buf[ie_pos+1]);
        return;
    }
    if (memcmp(buf + ie_pos + 2, "lib80211", 8) != 0) {
        TEST_FAIL("SSID content mismatch");
        return;
    }
    ie_pos += 2 + 8;

    /* Supported Rates IE: tag=1, len=6, data={0x8C,0x12,0xB0,0x48,0x60,0x6C} */
    uint8_t expected_rates[] = {0x8C, 0x12, 0xB0, 0x48, 0x60, 0x6C};
    if (buf[ie_pos] != 1 || buf[ie_pos+1] != 6) {
        TEST_FAIL("Rates IE header: tag=%u len=%u", buf[ie_pos], buf[ie_pos+1]);
        return;
    }
    if (memcmp(buf + ie_pos + 2, expected_rates, 6) != 0) {
        TEST_FAIL("Rates IE content mismatch");
        return;
    }
    ie_pos += 2 + 6;

    /* DS Parameter IE: tag=3, len=1, channel=36 */
    if (buf[ie_pos] != 3 || buf[ie_pos+1] != 1 || buf[ie_pos+2] != 36) {
        TEST_FAIL("DS IE: tag=%u len=%u ch=%u", buf[ie_pos], buf[ie_pos+1], buf[ie_pos+2]);
        return;
    }
    ie_pos += 2 + 1;

    /* TIM IE: tag=5, len=4, data={0,1,0,0} */
    if (buf[ie_pos] != 5 || buf[ie_pos+1] != 4) {
        TEST_FAIL("TIM IE header: tag=%u len=%u", buf[ie_pos], buf[ie_pos+1]);
        return;
    }
    uint8_t expected_tim[] = {0, 1, 0, 0};
    if (memcmp(buf + ie_pos + 2, expected_tim, 4) != 0) {
        TEST_FAIL("TIM IE content mismatch");
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: Deauth frame structure
 * ======================================================================== */

static void test_deauth_structure(void) {
    TEST_BEGIN("deauth_structure");

    uint8_t buf[64];
    memset(buf, 0xAA, sizeof(buf));

    uint8_t da[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    uint8_t sa[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    uint8_t bssid[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    uint16_t reason = 7;

    size_t len = lib80211_build_deauth(buf, sizeof(buf), da, sa, bssid, reason);

    if (len != 26) {
        TEST_FAIL("deauth length: got %zu, expected 26", len);
        return;
    }

    /* FC = 0x00C0 (LE) */
    if (buf[0] != 0xC0 || buf[1] != 0x00) {
        TEST_FAIL("FC: got 0x%02X%02X, expected 0xC000", buf[1], buf[0]);
        return;
    }

    /* DA */
    if (memcmp(buf + 4, da, 6) != 0) {
        TEST_FAIL("DA mismatch");
        return;
    }

    /* SA */
    if (memcmp(buf + 10, sa, 6) != 0) {
        TEST_FAIL("SA mismatch");
        return;
    }

    /* BSSID */
    if (memcmp(buf + 16, bssid, 6) != 0) {
        TEST_FAIL("BSSID mismatch");
        return;
    }

    /* Reason code at offset 24 (LE) */
    uint16_t got_reason = buf[24] | ((uint16_t)buf[25] << 8);
    if (got_reason != reason) {
        TEST_FAIL("reason: got %u, expected %u", got_reason, reason);
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: FCS append + verify roundtrip
 * ======================================================================== */

static void test_fcs_roundtrip(void) {
    TEST_BEGIN("fcs_roundtrip");

    /* Build a small frame (deauth) and append FCS, then verify with lib80211_fcs_check */
    uint8_t buf[64];
    uint8_t da[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    size_t len = lib80211_build_deauth(buf, sizeof(buf), da, NULL, NULL,
                                       LIB80211_DEAUTH_REASON_DEFAULT);
    if (len == 0) {
        TEST_FAIL("deauth build failed");
        return;
    }

    lib80211_append_fcs(buf, len);

    /* fcs_check expects the full frame including 4-byte FCS */
    if (!lib80211_fcs_check(buf, len + 4)) {
        TEST_FAIL("FCS check failed after append");
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: FCS known vector
 * ======================================================================== */

static void test_fcs_known_vector(void) {
    TEST_BEGIN("fcs_known_vector");

    /* Use a known short byte sequence and verify FCS matches expected.
     * For "123456789" (9 bytes), CRC-32 = 0xCBF43926 */
    uint8_t data[13];
    memcpy(data, "123456789", 9);

    lib80211_append_fcs(data, 9);

    /* Expected FCS bytes (little-endian): 0x26, 0x39, 0xF4, 0xCB */
    if (data[9]  != 0x26 || data[10] != 0x39 ||
        data[11] != 0xF4 || data[12] != 0xCB) {
        TEST_FAIL("FCS for '123456789': got %02X %02X %02X %02X, "
                  "expected 26 39 F4 CB",
                  data[9], data[10], data[11], data[12]);
        return;
    }

    /* Also verify fcs_check passes on the combined buffer */
    if (!lib80211_fcs_check(data, 13)) {
        TEST_FAIL("fcs_check failed on known vector");
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: NULL bssid uses default
 * ======================================================================== */

static void test_null_bssid_default(void) {
    TEST_BEGIN("null_bssid_default");

    uint8_t buf[256];
    size_t len = lib80211_build_beacon(buf, sizeof(buf),
                                       "test", NULL, 1, 100, 0);
    if (len == 0) {
        TEST_FAIL("build_beacon with NULL bssid failed");
        return;
    }

    /* SA (offset 10) and BSSID (offset 16) should be default: 02:00:00:00:00:01 */
    if (memcmp(buf + 10, LIB80211_DEFAULT_BSSID, 6) != 0) {
        TEST_FAIL("SA not default BSSID");
        return;
    }
    if (memcmp(buf + 16, LIB80211_DEFAULT_BSSID, 6) != 0) {
        TEST_FAIL("BSSID field not default");
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: Empty SSID (hidden network)
 * ======================================================================== */

static void test_empty_ssid(void) {
    TEST_BEGIN("empty_ssid");

    uint8_t buf[256];
    size_t len = lib80211_build_beacon(buf, sizeof(buf),
                                       "", NULL, 6, 100, 0);
    if (len == 0) {
        TEST_FAIL("build_beacon with empty SSID failed");
        return;
    }

    /* SSID IE at offset 36: tag=0, len=0 */
    if (buf[36] != 0 || buf[37] != 0) {
        TEST_FAIL("SSID IE for empty: tag=%u len=%u", buf[36], buf[37]);
        return;
    }

    /* Next IE (Supported Rates) should follow immediately at offset 38 */
    if (buf[38] != 1) {
        TEST_FAIL("Expected rates IE tag=1 at offset 38, got %u", buf[38]);
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: Max SSID (32 chars)
 * ======================================================================== */

static void test_max_ssid(void) {
    TEST_BEGIN("max_ssid_32");

    uint8_t buf[256];
    const char *ssid32 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ012345";  /* 32 chars */
    size_t len = lib80211_build_beacon(buf, sizeof(buf),
                                       ssid32, NULL, 11, 100, 0);
    if (len == 0) {
        TEST_FAIL("build_beacon with 32-char SSID failed");
        return;
    }

    /* SSID IE at offset 36: tag=0, len=32 */
    if (buf[36] != 0 || buf[37] != 32) {
        TEST_FAIL("SSID IE: tag=%u len=%u, expected tag=0 len=32",
                  buf[36], buf[37]);
        return;
    }

    /* Verify SSID content */
    if (memcmp(buf + 38, ssid32, 32) != 0) {
        TEST_FAIL("SSID content mismatch for 32-char SSID");
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: SSID too long (>32) returns 0
 * ======================================================================== */

static void test_ssid_too_long(void) {
    TEST_BEGIN("ssid_too_long");

    uint8_t buf[256];
    const char *ssid33 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456";  /* 33 chars */
    size_t len = lib80211_build_beacon(buf, sizeof(buf),
                                       ssid33, NULL, 1, 100, 0);
    if (len != 0) {
        TEST_FAIL("expected 0 for 33-char SSID, got %zu", len);
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: Buffer too small returns 0
 * ======================================================================== */

static void test_buffer_too_small(void) {
    TEST_BEGIN("buffer_too_small");

    uint8_t buf[10];  /* Way too small for any frame */

    size_t len = lib80211_build_beacon(buf, sizeof(buf),
                                       "test", NULL, 1, 100, 0);
    if (len != 0) {
        TEST_FAIL("beacon: expected 0 for tiny buffer, got %zu", len);
        return;
    }

    len = lib80211_build_deauth(buf, sizeof(buf),
                                LIB80211_BROADCAST, NULL, NULL, 7);
    if (len != 0) {
        TEST_FAIL("deauth: expected 0 for tiny buffer, got %zu", len);
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: Beacon roundtrip (build + FCS + verify)
 * ======================================================================== */

static void test_beacon_fcs_roundtrip(void) {
    TEST_BEGIN("beacon_fcs_roundtrip");

    uint8_t buf[256];
    size_t len = lib80211_build_beacon(buf, sizeof(buf),
                                       "TestNetwork", NULL, 44,
                                       100, 0x00000000DEADBEEFULL);
    if (len == 0) {
        TEST_FAIL("build_beacon failed");
        return;
    }

    lib80211_append_fcs(buf, len);

    if (!lib80211_fcs_check(buf, len + 4)) {
        TEST_FAIL("FCS check failed on beacon roundtrip");
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * Test: Deauth with NULL sa uses bssid
 * ======================================================================== */

static void test_deauth_null_sa(void) {
    TEST_BEGIN("deauth_null_sa");

    uint8_t buf[64];
    uint8_t da[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    uint8_t bssid[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};

    size_t len = lib80211_build_deauth(buf, sizeof(buf), da, NULL, bssid, 1);
    if (len != 26) {
        TEST_FAIL("deauth length: got %zu, expected 26", len);
        return;
    }

    /* SA (offset 10) should equal bssid when sa=NULL */
    if (memcmp(buf + 10, bssid, 6) != 0) {
        TEST_FAIL("SA should be BSSID when sa=NULL");
        return;
    }

    TEST_PASS();
}

/* ========================================================================
 * main
 * ======================================================================== */

int main(void) {
    printf("test_mac: MAC frame builders\n");

    test_beacon_structure();
    test_deauth_structure();
    test_fcs_roundtrip();
    test_fcs_known_vector();
    test_null_bssid_default();
    test_empty_ssid();
    test_max_ssid();
    test_ssid_too_long();
    test_buffer_too_small();
    test_beacon_fcs_roundtrip();
    test_deauth_null_sa();

    TEST_SUMMARY();
    return TEST_EXIT();
}
