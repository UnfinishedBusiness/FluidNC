#include "gtest/gtest.h"
#include "Machine/ThcManualLease.h"

using Machine::ThcManualLease;

TEST(ThcManualLease, ExpiresAndCannotBeRestartedByRenewal) {
    ThcManualLease hold;
    hold.start(1, 10);
    hold.renew(110);
    EXPECT_EQ(hold.direction(509, true), 1);
    hold.renew(510);
    EXPECT_EQ(hold.direction(510, true), 0);
    hold.renew(511);
    EXPECT_EQ(hold.direction(511, true), 0);
    hold.start(-1, 512);
    EXPECT_EQ(hold.direction(512, true), -1);
}

TEST(ThcManualLease, EndsOnLostEligibility) {
    ThcManualLease hold;
    hold.start(1, 100);
    EXPECT_EQ(hold.direction(101, false), 0);
    hold.renew(102);
    EXPECT_EQ(hold.direction(102, true), 0);
}

TEST(ThcManualLease, WrapsTimestampAndStopsExplicitly) {
    ThcManualLease hold;
    hold.start(-1, 0x3fffffffu);
    EXPECT_EQ(hold.direction(0x40000000u, true), -1);
    hold.stop();
    EXPECT_EQ(hold.direction(0x40000001u, true), 0);
}
