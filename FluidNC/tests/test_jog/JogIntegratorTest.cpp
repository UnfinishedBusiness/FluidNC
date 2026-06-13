// Tests for the LinuxCNC-style velocity-setpoint jog integrator (JogIntegrator.h).
//
// These pin the behaviors the firmware re-architecture exists to deliver, on pure math (no
// firmware stack), exactly like JogMathTest.cpp:
//   - a quick tap's move is bounded by PRESS TIME, not by any block/runway size;
//   - releasing from cruise overshoots by exactly v^2/2a (the physical minimum);
//   - a direction reversal slews the velocity vector smoothly through zero (no dwell/hard corner);
//   - the velocity vector blends so a diagonal sweep is continuous (smooth arc);
//   - machine extents are respected: an axis decelerates and parks AT its soft-limit fence;
//   - the committed lead always covers braking distance (flat-velocity hold).

#include "gtest/gtest.h"
#include "Machine/JogIntegrator.h"

#include <cmath>

using namespace Machine::JogIntegrator;

namespace {

    constexpr float DT = 0.001f;  // 1 ms tick, matches the firmware refill cadence

    float braking(float v, float a) { return (v * v) / (2.0f * a); }

    // Integrate to rest (setpoint already zero / fenced) and return the X distance traveled.
    void run_to_rest(State& s, float accel, const float* lo = nullptr, const float* hi = nullptr) {
        for (int i = 0; i < 200000 && moving(s); ++i) {
            integrate_tick(s, accel, DT, lo, hi);
        }
    }

    // Hold a setpoint for hold_s, then release and coast to rest. Returns total X distance.
    float tap_distance(float feed_mm_min, float accel, float hold_s) {
        State s;
        float zero[3] = { 0, 0, 0 };
        seed(s, zero);
        float dir[3] = { 1, 0, 0 };
        set_target(s, dir, feed_mm_min);
        int ticks = int(hold_s / DT);
        for (int i = 0; i < ticks; ++i) {
            integrate_tick(s, accel, DT, nullptr, nullptr);
        }
        release(s);
        run_to_rest(s, accel);
        return s.pos[0];
    }
}

TEST(JogIntegrator, SetTargetDistributesAlongDirection) {
    State s;
    float zero[3] = { 0, 0, 0 };
    seed(s, zero);
    float dir[3] = { 0.0f, 1.0f, 0.0f };
    set_target(s, dir, 6000.0f);  // 100 mm/s
    EXPECT_NEAR(s.targetVel[0], 0.0f, 1e-5f);
    EXPECT_NEAR(s.targetVel[1], 100.0f, 1e-3f);
    EXPECT_NEAR(s.targetVel[2], 0.0f, 1e-5f);
}

TEST(JogIntegrator, RampReachesTargetNeverOvershoots) {
    State s;
    float zero[3] = { 0, 0, 0 };
    seed(s, zero);
    float dir[3]  = { 1, 0, 0 };
    const float v = 100.0f, a = 500.0f;  // mm/s, mm/s^2  -> reaches target in 0.2 s
    set_target(s, dir, v * 60.0f);
    bool reached = false;
    for (int i = 0; i < 1000; ++i) {
        integrate_tick(s, a, DT, nullptr, nullptr);
        EXPECT_LE(s.vel[0], v + 1e-3f) << "velocity overshot the setpoint";
        if (!reached && std::fabs(s.vel[0] - v) < 1e-2f) {
            reached      = true;
            float t_secs = (i + 1) * DT;
            EXPECT_NEAR(t_secs, v / a, 0.01f);  // ~0.2 s
        }
    }
    EXPECT_TRUE(reached);
}

TEST(JogIntegrator, ReversalRampsThroughZeroSmoothly) {
    State s;
    float zero[3] = { 0, 0, 0 };
    seed(s, zero);
    float dirP[3] = { 1, 0, 0 }, dirN[3] = { -1, 0, 0 };
    const float a = 500.0f;
    set_target(s, dirP, 6000.0f);  // +100 mm/s
    for (int i = 0; i < 300; ++i) {
        integrate_tick(s, a, DT, nullptr, nullptr);  // settle at +100
    }
    ASSERT_NEAR(s.vel[0], 100.0f, 1e-2f);
    set_target(s, dirN, 6000.0f);  // reverse to -100 mm/s
    float prev      = s.vel[0];
    bool  crossed0  = false;
    float maxStep   = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        integrate_tick(s, a, DT, nullptr, nullptr);
        maxStep = std::max(maxStep, std::fabs(s.vel[0] - prev));
        if (prev > 0 && s.vel[0] <= 0) {
            crossed0 = true;
        }
        prev = s.vel[0];
    }
    EXPECT_TRUE(crossed0) << "velocity must ramp continuously through zero";
    EXPECT_NEAR(s.vel[0], -100.0f, 1e-2f);
    EXPECT_LE(maxStep, a * DT + 1e-4f) << "no velocity discontinuity — slew is accel-limited";
}

TEST(JogIntegrator, TapDistanceBoundedByPressTime) {
    const float a = 1524.0f, feed = 15240.0f;  // 254 mm/s cruise (the bench 100% case)
    // A 50 ms tap must move ~ a*t^2, FAR less than a full-cruise stop (braking ~21 mm => ~7/8").
    float d50 = tap_distance(feed, a, 0.05f);
    EXPECT_NEAR(d50, a * 0.05f * 0.05f, 0.5f);          // ~3.8 mm
    EXPECT_LT(d50, braking(254.0f, a));                  // far less than the full-cruise overshoot
    // Longer press -> proportionally larger move (scales with press time, not a fixed segment).
    float d100 = tap_distance(feed, a, 0.10f);
    EXPECT_GT(d100, d50 * 2.0f);
}

TEST(JogIntegrator, ReleaseOvershootIsBrakingDistance) {
    State s;
    float zero[3] = { 0, 0, 0 };
    seed(s, zero);
    float dir[3]  = { 1, 0, 0 };
    const float v = 200.0f, a = 1000.0f;
    set_target(s, dir, v * 60.0f);
    for (int i = 0; i < 1000 && std::fabs(s.vel[0] - v) > 1e-2f; ++i) {
        integrate_tick(s, a, DT, nullptr, nullptr);
    }
    ASSERT_NEAR(s.vel[0], v, 1e-1f);
    float p0 = s.pos[0];
    release(s);
    run_to_rest(s, a);
    EXPECT_NEAR(s.pos[0] - p0, braking(v, a), 0.5f);  // exactly v^2/2a, the physical minimum
}

TEST(JogIntegrator, ExtentDecelStopsAtFence) {
    State s;
    float zero[3] = { 0, 0, 0 };
    seed(s, zero);
    float dir[3]  = { 1, 0, 0 };
    const float a = 1000.0f;
    set_target(s, dir, 15240.0f);  // 254 mm/s toward a near fence
    float lo[3] = { -1e9f, -1e9f, -1e9f };
    float hi[3] = { 10.0f, 1e9f, 1e9f };  // X soft-limit at +10 mm
    // Integrate from rest through the accel+decel (a fixed loop, not run_to_rest, which gates on
    // moving() and would never leave standstill).
    for (int i = 0; i < 20000; ++i) {
        integrate_tick(s, a, DT, lo, hi);
    }
    EXPECT_LE(s.pos[0], 10.0f + 1e-3f);   // never crosses the fence
    EXPECT_NEAR(s.pos[0], 10.0f, 0.5f);   // parks AT the fence (within a tick's travel)
    EXPECT_NEAR(s.vel[0], 0.0f, 1e-2f);
}

TEST(JogIntegrator, DiagonalSlidesAlongFence) {
    State s;
    float zero[3] = { 0, 0, 0 };
    seed(s, zero);
    float dir[3]  = { 0.70710678f, 0.70710678f, 0.0f };  // +X+Y diagonal
    const float a = 1000.0f;
    set_target(s, dir, 12000.0f);
    float lo[3] = { -1e9f, -1e9f, -1e9f };
    float hi[3] = { 5.0f, 1e9f, 1e9f };  // only X is fenced (near)
    for (int i = 0; i < 5000; ++i) {
        integrate_tick(s, a, DT, lo, hi);
    }
    EXPECT_LE(s.pos[0], 5.0f + 1e-3f);   // X parked at its fence
    EXPECT_NEAR(s.vel[0], 0.0f, 1e-2f);
    EXPECT_GT(s.pos[1], 5.0f);            // Y kept moving — the jog slides along the fence
    EXPECT_GT(s.vel[1], 1.0f);
}

TEST(JogIntegrator, CommittedLeadCoversBrakingDistance) {
    const float a = 1524.0f;
    for (float v : { 25.0f, 50.0f, 100.0f, 200.0f }) {  // mm/s, within the unclamped band
        float lead = committed_lead_mm(v, a);
        EXPECT_GE(lead, braking(v, a) - 1e-3f) << "lead must keep >= braking distance queued (v=" << v << ")";
        EXPECT_NEAR(lead, braking(v, a) + v * LEAD_MARGIN_S, 1e-2f);
    }
}

TEST(JogIntegrator, MaxHoldableFeedIsPositiveAndHoldable) {
    const int blocks = 16;
    for (float a : { 100.0f, 500.0f, 1524.0f }) {
        float holdable = max_holdable_feed_mm_min(a, blocks);
        EXPECT_GT(holdable, 0.0f);
        float v   = holdable * MM_MIN_TO_MM_S;
        float cap = std::min(LEAD_MAX_MM, float(blocks - 2) * SEG_MAX_MM);
        EXPECT_LE(committed_lead_mm(v, a), cap + 1e-2f);  // its lead fits the queue
    }
}
