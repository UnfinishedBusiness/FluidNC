// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>
#include <cstddef>

// Standard CRC-32 (a.k.a. CRC-32/ISO-HDLC, as used by zlib, PNG and Ethernet):
// reflected polynomial 0xEDB88320, initial value 0xFFFFFFFF, final XOR 0xFFFFFFFF.
// A sender can reproduce this with any off-the-shelf "crc32" routine.
//
// Table-less (bitwise) on purpose: lines are short (<=255 bytes) and computed once
// each, so the loop cost is negligible, and we avoid spending ~1 KB of flash on a
// lookup table on an already-full ESP32 image.
inline uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));  // -(crc&1) without UB
        }
    }
    return ~crc;
}

inline uint32_t crc32(const char* data, size_t len) {
    return crc32(reinterpret_cast<const uint8_t*>(data), len);
}
