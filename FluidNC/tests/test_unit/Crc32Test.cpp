// Unit tests for the CRC-32 used by optional serial line-integrity framing.
// The canonical check value pins the algorithm so any standard "crc32" routine
// on the sender side (zlib, etc.) interoperates.

#include "gtest/gtest.h"
#include "Crc32.h"

#include <cstring>

static uint32_t crc_str(const char* s) {
    return crc32(s, std::strlen(s));
}

// The well-known CRC-32/ISO-HDLC check value: crc32("123456789") == 0xCBF43926.
TEST(Crc32, CanonicalCheckValue) {
    EXPECT_EQ(crc_str("123456789"), 0xCBF43926u);
}

TEST(Crc32, EmptyIsZero) {
    EXPECT_EQ(crc32((const uint8_t*)"", 0), 0x00000000u);
}

TEST(Crc32, KnownGcodeLine) {
    // Stable regression value for a representative gcode line (matches zlib).
    EXPECT_EQ(crc_str("G1 X10 Y10 F1000"), 0x46BF44C8u);
}

TEST(Crc32, DiffersOnSingleBitFlip) {
    EXPECT_NE(crc_str("G0 X1"), crc_str("G0 X2"));
}

// Round-trips the framing the firmware emits: "<line>*<8 hex>".
TEST(Crc32, HexFormatMatches) {
    char buf[16];
    snprintf(buf, sizeof(buf), "*%08lX", (unsigned long)crc_str("123456789"));
    EXPECT_STREQ(buf, "*CBF43926");
}
