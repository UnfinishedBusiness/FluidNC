// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <algorithm>
#include <cmath>

// Pure math for the Route-A self-refilling planner-feed jog engine (see docs/jogging.md
// and plans/firmware-jogging/protocol.md). Header-only and dependency-free so it is used
// unchanged by the firmware module AND unit-tested on the x86 host.
//
// Public feeds are mm/min (the wire unit); acceleration is mm/s^2 (the axes-config unit).
// Internally velocities are mm/s.
namespace Machine {
    namespace JogMath {

        constexpr float MM_MIN_TO_MM_S = 1.0f / 60.0f;

        // Distance to decelerate from `feed` to a full stop at `accel`: d = v^2 / (2a).
        // The planner always forces the last queued block to exit at zero, so the machine can
        // only HOLD `feed` while at least this much distance stays queued ahead of it.
        inline float braking_distance_mm(float feed_mm_min, float accel_mm_s2) {
            if (accel_mm_s2 <= 0.0f) {
                return 0.0f;
            }
            float v = feed_mm_min * MM_MIN_TO_MM_S;
            return (v * v) / (2.0f * accel_mm_s2);
        }

        // Length of one synthesized jog block: ~40 ms of travel, clamped to [0.5, 50] mm.
        // Short enough for fine runway/feed granularity, long enough to keep the block count low.
        inline float block_len_mm(float feed_mm_min) {
            float v = feed_mm_min * MM_MIN_TO_MM_S;
            return std::min(50.0f, std::max(0.5f, v * 0.04f));
        }

        // Runway the refill loop keeps queued ahead of the executing point so the planner's
        // tail-to-zero never reaches the front: max(1.5 x braking distance, 3 blocks).
        inline float target_runway_mm(float feed_mm_min, float accel_mm_s2) {
            float brake        = braking_distance_mm(feed_mm_min, accel_mm_s2);
            float three_blocks = 3.0f * block_len_mm(feed_mm_min);
            return std::max(1.5f * brake, three_blocks);
        }

        // ── Queue-capacity sizing (fixes the high-feed velocity sag / jitter) ────────────────
        //
        // The planner ring holds `planner_blocks` entries; the refill loop keeps one slot free and
        // one is the executing block, so the queue can hold at most (planner_blocks - 2) blocks of
        // runway. With the plain v*0.04s block length, high feed/low accel combos make the runway
        // TARGET exceed that capacity (e.g. F15240 @ 250 mm/s^2: target 193mm, capacity ~143mm) —
        // the planner saturates BELOW the velocity-hold threshold and the executing feed sags and
        // oscillates: exactly the $J= wheeze this engine exists to eliminate. Two closures:
        //   1. block_len_for_hold_mm: grow the block length until capacity covers 1.5 x braking.
        //   2. max_holdable_feed_mm_min: when even 50mm blocks can't cover it (extreme feed/accel),
        //      lower the cruise to what the queue CAN hold — a slightly slower jog that holds
        //      velocity beats a faster setpoint it can never reach.

        // Usable queue capacity for a given block length.
        inline float queue_capacity_mm(float block_len, int planner_blocks) {
            int n = planner_blocks - 2;
            return n > 0 ? float(n) * block_len : 0.0f;
        }

        // Block length sized so the queue can HOLD `feed`: the v*0.04s heuristic, raised when
        // needed so (planner_blocks - 2) x len >= 1.5 x braking distance, clamped to [0.5, 50] mm.
        inline float block_len_for_hold_mm(float feed_mm_min, float accel_mm_s2, int planner_blocks) {
            float v   = feed_mm_min * MM_MIN_TO_MM_S;
            float len = std::max(0.5f, v * 0.04f);
            int   n   = planner_blocks - 2;
            if (n > 0 && accel_mm_s2 > 0.0f) {
                float need = (1.5f * braking_distance_mm(feed_mm_min, accel_mm_s2)) / float(n);
                len        = std::max(len, need);
            }
            return std::min(50.0f, len);
        }

        // Highest cruise the queue can hold at the 50mm block-length cap: capacity >= 1.5 x brake
        // inverted for v. Cruise above this is unreachable-by-construction; clamp to it.
        inline float max_holdable_feed_mm_min(float accel_mm_s2, int planner_blocks) {
            int n = planner_blocks - 2;
            if (n <= 0 || accel_mm_s2 <= 0.0f) {
                return 0.0f;
            }
            float cap = float(n) * 50.0f;
            float v   = std::sqrt(2.0f * accel_mm_s2 * (cap / 1.5f));  // mm/s
            return v / MM_MIN_TO_MM_S;
        }

        // Runway target for a hold-sized block length (the 3-block floor uses the REAL length).
        inline float target_runway_for_mm(float feed_mm_min, float accel_mm_s2, float block_len) {
            float brake = braking_distance_mm(feed_mm_min, accel_mm_s2);
            return std::max(1.5f * brake, 3.0f * block_len);
        }

        // Fastest speed (mm/min) the machine can be moving at the front of the queue such that
        // it can still stop within `runway` (the tail exits at zero): v_max = sqrt(2 a R).
        // This is what the planner's backward pass yields at the executing block.
        inline float max_feed_for_runway_mm_min(float runway_mm, float accel_mm_s2) {
            if (runway_mm <= 0.0f || accel_mm_s2 <= 0.0f) {
                return 0.0f;
            }
            float v = std::sqrt(2.0f * accel_mm_s2 * runway_mm);  // mm/s
            return v / MM_MIN_TO_MM_S;
        }

        // Clamp `feed` (mm/min) so no active axis exceeds its per-axis max_rate when the jog
        // runs along the unit vector `dir` (XYZ): feed_max = min over active axes of
        // (max_rate[a] / |dir[a]|). Axes with dir==0 or max_rate<=0 are ignored.
        inline float clamp_feed_to_axis_rates(float feed_mm_min, const float dir[3], const float max_rate_mm_min[3]) {
            for (int a = 0; a < 3; ++a) {
                float d = std::fabs(dir[a]);
                if (d > 0.0f && max_rate_mm_min[a] > 0.0f) {
                    feed_mm_min = std::min(feed_mm_min, max_rate_mm_min[a] / d);
                }
            }
            return feed_mm_min;
        }

        // The feed the machine actually executes for a given queued runway: the cruise feed,
        // unless the runway is too short to hold it (then decel-limited). The flat-velocity
        // invariant is exactly "keep runway >= braking distance => this stays == cruise".
        inline float executing_feed_mm_min(float runway_mm, float cruise_mm_min, float accel_mm_s2) {
            return std::min(cruise_mm_min, max_feed_for_runway_mm_min(runway_mm, accel_mm_s2));
        }

    }  // namespace JogMath
}  // namespace Machine
