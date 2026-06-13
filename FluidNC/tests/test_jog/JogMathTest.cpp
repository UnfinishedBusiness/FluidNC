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

TEST(JogMath, TargetRunwayIsMaxOfBrakingPlusSlackAndBlocks) {
    // cruise 3000 (v=50mm/s), accel 500: brake 2.5 + slack 50*0.08=4.0 -> 6.5 vs 3 blocks 6.0 -> 6.5
    EXPECT_NEAR(target_runway_mm(3000.0f, 500.0f), 6.5f, 1e-3f);
    // cruise 3000, accel 100: brake 12.5 + 4.0 = 16.5 vs 6.0 -> 16.5
    EXPECT_NEAR(target_runway_mm(3000.0f, 100.0f), 16.5f, 1e-3f);
}

// The runway slack must cover the worst protocol-loop stall AT EVERY SPEED: the slack in TIME
// (slack distance / v) is constant by construction — this pins the 45%-clean/100%-shaky bench
// gradient closed (constant-distance margins shrink in time as v rises; v x slack does not).
TEST(JogMath, RunwaySlackIsConstantInTime) {
    for (float feed : { 3000.0f, 7620.0f, 15240.0f }) {
        float v     = feed * MM_MIN_TO_MM_S;
        float brake = braking_distance_mm(feed, 1524.0f);
        float slack = target_runway_mm(feed, 1524.0f) - brake;
        if (slack > 0.0f) {  // above the 3-block floor
            EXPECT_NEAR(slack / v, LOOP_SLACK_S, 0.02f) << "feed=" << feed;
        }
    }
}

// ── Queue-capacity sizing (the bench "jitters like $J=" defect at F15240) ────────────────────
//
// The FlatVelocityInvariant simulation above queues UNBOUNDED blocks — which is exactly why the
// capacity defect passed it. The real planner ring holds (planner_blocks - 2) usable entries; at
// high feed the plain v*0.04s block length saturates the ring BELOW the velocity-hold threshold
// (F15240 @ 250 mm/s^2, 16 blocks: capacity 14 x 10.16 = 142mm < 1.5 x braking 194mm) and the
// executing feed sags and oscillates. These tests model the cap and pin the sizing that closes it.

namespace {
    // Same tail-to-zero model as simulate_hold, but the queue holds at most `max_blocks` blocks.
    SimResult simulate_hold_capped(float cruise, float accel, float block, float target, int max_blocks, float sim_s) {
        const float dt       = 0.001f;
        float       R        = 0.0f;
        int         queued   = 0;
        float       min_feed = cruise * 10.0f;
        bool        dipped   = false;
        int         ticks    = static_cast<int>(sim_s / dt);

        for (int i = 0; i < ticks; i++) {
            // In-process refill every tick, but bounded by the planner ring.
            while (R < target && queued < max_blocks) {
                R += block;
                ++queued;
            }
            float feed     = executing_feed_mm_min(R, cruise, accel);
            float consumed = feed * MM_MIN_TO_MM_S * dt;
            if (R > 0.0f && consumed >= 0.0f) {
                float before = R;
                R            = std::max(0.0f, R - consumed);
                // Retire whole blocks as the executing point crosses them.
                while (queued > 0 && before - R >= 0.0f && R < float(queued - 1) * block) {
                    --queued;
                }
            }
            if (i > 50) {  // allow the (capped) accel ramp to finish
                min_feed = std::min(min_feed, feed);
                if (feed < cruise - 1.0f) {
                    dipped = true;
                }
            }
        }
        return { min_feed, dipped };
    }
}

TEST(JogMathCapacity, PlainSizingSagsAtBenchFeed) {
    // Documents the defect the hold-sizing closes: bench parameters, plain v*0.04s blocks.
    const float cruise = 15240.0f, accel = 250.0f;
    SimResult   r = simulate_hold_capped(cruise, accel, block_len_mm(cruise), target_runway_mm(cruise, accel),
                                         /*max_blocks=*/16 - 2, /*sim_s=*/3.0f);
    EXPECT_TRUE(r.ever_dipped) << "expected the capacity-saturated sag (else this guard is vacuous)";
}

TEST(JogMathCapacity, HoldSizedBlocksHoldBenchFeed) {
    const float cruise = 15240.0f, accel = 250.0f;
    const int   blocks = 16;
    const float vmm_s  = cruise * MM_MIN_TO_MM_S;
    const float len    = block_len_for_hold_mm(cruise, accel, blocks);
    EXPECT_GE(queue_capacity_mm(len, blocks), braking_distance_mm(cruise, accel) + vmm_s * LOOP_SLACK_S - 1e-3f);
    SimResult r = simulate_hold_capped(cruise, accel, len, target_runway_for_mm(cruise, accel, len),
                                       blocks - 2, 3.0f);
    EXPECT_FALSE(r.ever_dipped) << "hold-sized blocks must pin the bench feed flat";
    EXPECT_NEAR(r.min_feed_after_accel, cruise, 1.0f);
}

TEST(JogMathCapacity, ExtremeFeedClampsToHoldable) {
    // F30000 @ 100 mm/s^2: even 50mm blocks can't cover braking + slack — the cruise must clamp to
    // what the queue CAN hold, and at that clamped cruise the capacity covers braking + v x slack.
    const int   blocks   = 16;
    const float holdable = max_holdable_feed_mm_min(100.0f, blocks);
    EXPECT_LT(holdable, 30000.0f);
    EXPECT_GT(holdable, 0.0f);
    const float vmm_s = holdable * MM_MIN_TO_MM_S;
    const float len   = block_len_for_hold_mm(holdable, 100.0f, blocks);
    EXPECT_GE(queue_capacity_mm(len, blocks), braking_distance_mm(holdable, 100.0f) + vmm_s * LOOP_SLACK_S - 1e-2f);
    SimResult r = simulate_hold_capped(holdable, 100.0f, len, target_runway_for_mm(holdable, 100.0f, len),
                                       blocks - 2, 3.0f);
    EXPECT_FALSE(r.ever_dipped);
}

TEST(JogMath, MaxFeedAtBrakingDistanceEqualsCruise) {
    // At runway == braking distance, the fastest holdable feed is exactly cruise.
    float brake = braking_distance_mm(3000.0f, 500.0f);
    EXPECT_NEAR(max_feed_for_runway_mm_min(brake, 500.0f), 3000.0f, 1.0f);
}

TEST(JogMath, ClampFeedToAxisRates) {
    // 45deg XY move: dir = (0.707, 0.707, 0); X max 5000, Y max 3000.
    // cap = min(5000/0.707, 3000/0.707) = 4243; requested 6000 -> 4243.
    float dir[3]  = { 0.70710678f, 0.70710678f, 0.0f };
    float rate[3] = { 5000.0f, 3000.0f, 0.0f };
    EXPECT_NEAR(clamp_feed_to_axis_rates(6000.0f, dir, rate), 3000.0f / 0.70710678f, 1.0f);
    // Single-axis move is capped at that axis rate, not below.
    float dx[3] = { 1.0f, 0.0f, 0.0f };
    EXPECT_NEAR(clamp_feed_to_axis_rates(6000.0f, dx, rate), 5000.0f, 1e-3f);
    // Requested below the cap passes through.
    EXPECT_NEAR(clamp_feed_to_axis_rates(1000.0f, dir, rate), 1000.0f, 1e-3f);
}

TEST(JogMath, ExecutingFeedDecelLimitedWhenRunwayShort) {
    // runway below braking distance -> decel-limited below cruise
    EXPECT_LT(executing_feed_mm_min(1.0f, 3000.0f, 500.0f), 3000.0f);
    // runway at/above braking -> pinned to cruise
    EXPECT_NEAR(executing_feed_mm_min(6.0f, 3000.0f, 500.0f), 3000.0f, 1e-3f);
}
