// Tests for the Route-A jog engine's pure math + the flat-velocity invariant.
//
// Test 1 (FlatVelocityInvariant) is the acceptance gate from the firmware-jogging plan:
// a held jog must never dip below cruise after the initial accel ramp. It is a faithful
// simulation of the planner's tail-to-zero behavior (the planner always plans the last
// queued block to exit at zero, so the executing feed is decel-limited to sqrt(2 a R)
// by the queued runway R) driven by the refill strategy. The in-process refill keeps
// runway >= braking distance every tick, so the executing feed stays pinned at cruise.

#include "gtest/gtest.h"
#include "Machine/JogMath.h"

#include <algorithm>

using namespace Machine::JogMath;

namespace {

    struct SimResult {
        float min_feed_after_accel;  // lowest executing feed seen after tick 0 (mm/min)
        bool  ever_dipped;           // true if feed ever fell meaningfully below cruise
    };

    // Simulate a held jog at `cruise` (mm/min) with `accel` (mm/s^2) for `sim_s` seconds.
    // `refill_period_ms` = how often the refill loop actually runs: 0 = in-process (every
    // 1 ms tick, sub-ms latency); >0 models a serial streamer's refill latency.
    SimResult simulate_hold(float cruise, float accel, float refill_period_ms, float sim_s) {
        const float dt     = 0.001f;  // 1 ms tick
        const float block  = block_len_mm(cruise);
        const float target = target_runway_mm(cruise, accel);

        float R             = 0.0f;     // queued runway ahead of the executing point (mm)
        float t_since_fill  = 1e9f;     // force a refill on tick 0
        float min_feed      = cruise * 10.0f;
        bool  dipped        = false;
        int   ticks         = static_cast<int>(sim_s / dt);

        for (int i = 0; i < ticks; i++) {
            t_since_fill += dt * 1000.0f;
            if (refill_period_ms <= 0.0f || t_since_fill >= refill_period_ms) {
                t_since_fill = 0.0f;
                while (R < target) {
                    R += block;  // queue blocks until runway >= target
                }
            }
            float feed     = executing_feed_mm_min(R, cruise, accel);
            float consumed = feed * MM_MIN_TO_MM_S * dt;  // mm traveled this tick
            R              = std::max(0.0f, R - consumed);

            if (i > 0) {  // tick 0 is the legitimate accel ramp from R=0
                min_feed = std::min(min_feed, feed);
                if (feed < cruise - 1.0f) {
                    dipped = true;
                }
            }
        }
        return { min_feed, dipped };
    }

}

// ---- THE ACCEPTANCE GATE ----
TEST(FlatVelocityInvariant, InProcessRefillNeverDips) {
    const float cruise = 3000.0f;  // mm/min
    const float accel  = 500.0f;   // mm/s^2
    SimResult   r      = simulate_hold(cruise, accel, /*refill_period_ms=*/0.0f, /*sim_s=*/10.0f);

    EXPECT_FALSE(r.ever_dipped) << "held jog dipped below cruise with in-process refill";
    EXPECT_NEAR(r.min_feed_after_accel, cruise, 1.0f);
}

TEST(FlatVelocityInvariant, HoldsAcrossFeedAndAccelMatrix) {
    for (float cruise : { 600.0f, 3000.0f, 6000.0f, 12000.0f }) {
        for (float accel : { 100.0f, 500.0f, 2000.0f }) {
            SimResult r = simulate_hold(cruise, accel, 0.0f, 5.0f);
            EXPECT_FALSE(r.ever_dipped) << "dip at cruise=" << cruise << " accel=" << accel;
            EXPECT_NEAR(r.min_feed_after_accel, cruise, 1.0f) << "cruise=" << cruise << " accel=" << accel;
        }
    }
}

// Control: the simulation HAS teeth — a slow (serial-streamer-latency) refill DOES dip,
// which is the failure mode Route A eliminates. If this ever stops dipping the invariant
// test above would be vacuous.
TEST(FlatVelocityInvariant, SerialLatencyRefillDoesDip) {
    SimResult r = simulate_hold(3000.0f, 500.0f, /*refill_period_ms=*/80.0f, /*sim_s=*/2.0f);
    EXPECT_TRUE(r.ever_dipped) << "serial-latency model should dip below cruise (else test is vacuous)";
}

// ---- JogMath unit values ----
TEST(JogMath, BrakingDistance) {
    // v = 50 mm/s, a = 500 mm/s^2 -> 2500 / 1000 = 2.5 mm
    EXPECT_NEAR(braking_distance_mm(3000.0f, 500.0f), 2.5f, 1e-4f);
    EXPECT_FLOAT_EQ(braking_distance_mm(3000.0f, 0.0f), 0.0f);  // no accel -> guarded to 0
}

TEST(JogMath, BlockLenClamped) {
    EXPECT_NEAR(block_len_mm(3000.0f), 2.0f, 1e-4f);   // 50 mm/s * 0.04 s
    EXPECT_NEAR(block_len_mm(300.0f), 0.5f, 1e-4f);    // 5 mm/s * 0.04 = 0.2 -> floor 0.5
    EXPECT_NEAR(block_len_mm(120000.0f), 50.0f, 1e-4f);// 2000 mm/s * 0.04 = 80 -> cap 50
}

TEST(JogMath, TargetRunwayIsMaxOfBrakingAndBlocks) {
    // cruise 3000, accel 500: 1.5*2.5=3.75 vs 3*2.0=6.0 -> 6.0
    EXPECT_NEAR(target_runway_mm(3000.0f, 500.0f), 6.0f, 1e-4f);
    // cruise 3000, accel 100: braking 12.5, 1.5*12.5=18.75 vs 6.0 -> 18.75
    EXPECT_NEAR(target_runway_mm(3000.0f, 100.0f), 18.75f, 1e-3f);
}

TEST(JogMath, MaxFeedAtBrakingDistanceEqualsCruise) {
    // At runway == braking distance, the fastest holdable feed is exactly cruise.
    float brake = braking_distance_mm(3000.0f, 500.0f);
    EXPECT_NEAR(max_feed_for_runway_mm_min(brake, 500.0f), 3000.0f, 1.0f);
}

TEST(JogMath, ExecutingFeedDecelLimitedWhenRunwayShort) {
    // runway below braking distance -> decel-limited below cruise
    EXPECT_LT(executing_feed_mm_min(1.0f, 3000.0f, 500.0f), 3000.0f);
    // runway at/above braking -> pinned to cruise
    EXPECT_NEAR(executing_feed_mm_min(6.0f, 3000.0f, 500.0f), 3000.0f, 1e-3f);
}
