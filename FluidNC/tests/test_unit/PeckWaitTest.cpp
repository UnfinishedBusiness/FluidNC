// Unit tests for the pure M104 peck-pulse wait decision (Machine::peckWaitState).
// One evaluation of the "waiting for the arc to strike" loop: no hardware, so these
// run on the native test build.

#include "gtest/gtest.h"
#include "Machine/THC.h"

using Machine::PeckWait;
using Machine::peckWaitState;

TEST(PeckWait, WaitsBelowThresholdBeforeTimeout) {
    EXPECT_EQ(peckWaitState(0.0f, 40.0f, 0, 3000), PeckWait::Waiting);
    EXPECT_EQ(peckWaitState(39.9f, 40.0f, 2999, 3000), PeckWait::Waiting);
}

TEST(PeckWait, CrossesAtOrAboveThreshold) {
    EXPECT_EQ(peckWaitState(40.0f, 40.0f, 100, 3000), PeckWait::Crossed);
    EXPECT_EQ(peckWaitState(120.0f, 40.0f, 1180, 3000), PeckWait::Crossed);
}

TEST(PeckWait, TimesOutWithNoCrossing) {
    EXPECT_EQ(peckWaitState(5.0f, 40.0f, 3000, 3000), PeckWait::TimedOut);
    EXPECT_EQ(peckWaitState(0.0f, 40.0f, 9999, 3000), PeckWait::TimedOut);
}

TEST(PeckWait, CrossingWinsOverSimultaneousTimeout) {
    // The arc struck exactly as the timeout expired: the arc is REAL. Declaring a
    // misfire here would hand the retry loop a live arc — the blown-hole race M104
    // exists to close — so Crossed must win.
    EXPECT_EQ(peckWaitState(41.0f, 40.0f, 3000, 3000), PeckWait::Crossed);
    EXPECT_EQ(peckWaitState(41.0f, 40.0f, 5000, 3000), PeckWait::Crossed);
}

TEST(PeckWait, ZeroElapsedInstantCrossing) {
    // A warm PSU (post-flow still moving) can strike on the very first sample.
    EXPECT_EQ(peckWaitState(80.0f, 40.0f, 0, 3000), PeckWait::Crossed);
}
