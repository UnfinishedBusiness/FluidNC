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

        // The feed the machine actually executes for a given queued runway: the cruise feed,
        // unless the runway is too short to hold it (then decel-limited). The flat-velocity
        // invariant is exactly "keep runway >= braking distance => this stays == cruise".
        inline float executing_feed_mm_min(float runway_mm, float cruise_mm_min, float accel_mm_s2) {
            return std::min(cruise_mm_min, max_feed_for_runway_mm_min(runway_mm, accel_mm_s2));
        }

    }  // namespace JogMath
}  // namespace Machine
