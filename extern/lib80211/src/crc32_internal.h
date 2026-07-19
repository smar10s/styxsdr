#ifndef LIB80211_CRC32_INTERNAL_H
#define LIB80211_CRC32_INTERNAL_H

/**
 * crc32_internal.h — Shared CRC-32 implementation (reflected poly 0xEDB88320).
 *
 * Used by both rx_common.c (FCS check) and mac.c (FCS append).
 */

#include <stdint.h>

static inline uint32_t lib80211_crc32_byte(uint32_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xEDB88320u;
        else
            crc >>= 1;
    }
    return crc;
}

#endif /* LIB80211_CRC32_INTERNAL_H */
