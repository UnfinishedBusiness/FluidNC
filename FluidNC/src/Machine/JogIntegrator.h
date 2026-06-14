// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <algorithm>
#include <cmath>

// LinuxCNC-style continuous-jog trajectory math: a per-axis trapezoidal velocity integrator that
// feeds the EXISTING GRBL planner (Route A). Each axis slews its velocity toward (dir * cruise) at
// the accel limit and integrates position; waypoints are emitted along that integrated path and
// queued as short jog blocks. This is a direct port of GcodePilot's proven host-side jog planner
// (jog-planner.js / jog-stream.js), moved in-process so it runs at the ~1 kHz refill tick with
// direct access to the true executing position (no status round-trip).
//
// Why this shape (vs. queueing fixed-cruise blocks and letting the planner's lookahead make the
// ramp): the velocity is ramped HERE, in time, so
//   - a quick tap only integrates the brief press -> a small move (bounded by press time);
//   - a direction change slews the velocity VECTOR per axis -> the resultant sweeps a smooth arc
//     (no hard corner / "S-curve");
//   - holding pins velocity flat once at cruise, provided the queued lead stays >= braking distance
//     (the committed-lead pacing below maintains that).
//
// Header-only and dependency-free (only <algorithm>/<cmath>) so it is shared by the firmware module
// AND unit-tested on the x86 host, exactly like JogMath.h.
namespace Machine {
    namespace JogIntegrator {

        constexpr float MM_MIN_TO_MM_S = 1.0f / 60.0f;
        constexpr float MM_S_TO_MM_MIN = 60.0f;

        // ── Emission / pacing tuning (ported from GcodePilot jog-stream.js, inch -> mm) ──────────
        // Waypoints are emitted along the integrated path; these decide WHERE. Denser through speed
        // and heading changes (capture accel ramps and corners), lazy on straight cruise.
        constexpr float SEG_MIN_MM    = 0.1f;    // smallest waypoint (sub-resolution guard) ~0.004"
        constexpr float SEG_MAX_MM    = 12.0f;   // hard cap on a cruise segment            ~0.5"
        constexpr float SEG_TIME_S    = 0.04f;   // cruise segment duration: len = |v| * SEG_TIME_S
        constexpr float EMIT_DV_MM_S  = 10.0f;   // speed-change emit trigger                ~0.4"/s
        constexpr float EMIT_DHEADING = 0.10f;   // heading-change emit trigger (rad, ~5.7 deg)

        // Committed lead = the runway kept queued ahead of the executing point. Must stay >= braking
        // distance (v^2/2a) or the planner's tail-to-zero decelerates the executing block (velocity
        // flutter). The additive v*LEAD_MARGIN_S rides out the worst protocol-loop stall (a
        // synchronous status-report TX blocks ~14 ms at 115200; CRC/acks stack). 80 ms matches the
        // value proven on the bench (JogMath::LOOP_SLACK_S). This lead does NOT cost release
        // overshoot: $Jog/Stop / 0x85 decelerate in place and FLUSH the lead (overshoot = v^2/2a),
        // so hold-robustness and stop-feel stay decoupled. Tune LEAD_MARGIN_S down if cornering lag
        // (the lead is also the corner-response delay) is felt before flutter appears.
        constexpr float LEAD_MARGIN_S = 0.08f;
        constexpr float LEAD_MIN_MM   = 0.5f;    // floor  ~0.02"
        constexpr float LEAD_MAX_MM   = 100.0f;  // ceiling ~4"

        constexpr float EPS_V_MM_S = 0.05f;  // treat |v| below this as stopped

        struct State {
            float pos[3]       = { 0, 0, 0 };  // integrated commanded position (mm) — the trajectory edge
            float vel[3]       = { 0, 0, 0 };  // current per-axis velocity (mm/s), signed
            float targetVel[3] = { 0, 0, 0 };  // commanded per-axis velocity (mm/s), signed
            // emission accumulators since the last emitted waypoint:
            float accDist       = 0.0f;        // path length traveled (mm)
            float lastDir[3]    = { 0, 0, 0 };  // unit heading at last emit
            bool  haveLastDir   = false;
            float lastEmitSpeed = 0.0f;        // |v| at last emit
            bool  primed        = false;       // pos seeded from the machine position
        };

        inline float speed(const State& s) {
            return std::sqrt(s.vel[0] * s.vel[0] + s.vel[1] * s.vel[1] + s.vel[2] * s.vel[2]);
        }
        inline bool moving(const State& s) { return speed(s) > EPS_V_MM_S; }

        // Per-axis integer DDA used by the direct-stepper jog engine (JogStepper). `inc` is the
        // signed Q16 step increment per ISR tick; `acc` is the persistent Q16 accumulator. Returns
        // -1 / 0 / +1 step for this tick. Pure integer (ISR-safe) and host-testable here.
        constexpr int32_t DDA_ONE = 1 << 16;  // fixed-point unit (must match JogStepper::ONE)
        inline int dda_step(int32_t inc, int32_t& acc) {
            int32_t mag = inc < 0 ? -inc : inc;
            if (mag == 0) {
                return 0;
            }
            acc += mag;
            if (acc >= DDA_ONE) {
                acc -= DDA_ONE;
                return inc < 0 ? -1 : 1;
            }
            return 0;
        }

        // Seed the trajectory edge at the current machine/commanded position; velocity starts at rest.
        inline void seed(State& s, const float pos[3]) {
            for (int a = 0; a < 3; ++a) {
                s.pos[a] = pos[a];
                s.vel[a] = 0.0f;
            }
            s.accDist     = 0.0f;
            s.haveLastDir = false;
            s.primed      = true;
        }

        // Set the velocity setpoint from a unit direction and a cruise feed (mm/min) that the caller
        // has ALREADY clamped to per-axis max rates / unhomed cap (JogMath::clamp_feed_to_axis_rates).
        // A direction change mid-jog just calls this again; the integrator blends toward the new vector.
        inline void set_target(State& s, const float dirUnit[3], float cruise_mm_min) {
            float v = cruise_mm_min * MM_MIN_TO_MM_S;
            for (int a = 0; a < 3; ++a) {
                s.targetVel[a] = dirUnit[a] * v;
            }
        }

        // Release: zero the setpoint so the integrator ramps velocity to rest. (The firmware uses the
        // jogCancel decel+flush for the actual $Jog/Stop, but this keeps the math self-contained and
        // host-testable for the minimal-overshoot release assertion.)
        inline void release(State& s) { s.targetVel[0] = s.targetVel[1] = s.targetVel[2] = 0.0f; }

        // Advance one tick. accel = limiting accel (mm/s^2), dt seconds.
        // limMin/limMax (nullable, parallel): per-axis absolute soft-limit envelope in machine mm.
        //   When provided, an axis proactively decelerates so it stops AT its fence (stopDist =
        //   v^2/2a vs distance-to-fence), then its position is hard-clamped to the envelope and its
        //   velocity zeroed if it lands on the fence. Pass nullptr to disable (soft limits off /
        //   unhomed free-jog carve-out). Axes with no fence get a large sentinel from the caller.
        inline void integrate_tick(State& s, float accel, float dt, const float* limMin, const float* limMax) {
            float step2 = 0.0f;
            for (int a = 0; a < 3; ++a) {
                float target = s.targetVel[a];
                if (limMin && limMax && accel > 0.0f) {
                    float stopDist = (s.vel[a] * s.vel[a]) / (2.0f * accel);
                    // Proactive decel: only when actually MOVING toward a fence and unable to stop
                    // before it. Keyed on the velocity sign — NOT a bare >=0, which at rest (vel==0)
                    // would always pick the max fence and wrongly block a jog heading the other way
                    // (e.g. parked at the max corner, jogging back into the envelope).
                    if (s.vel[a] > 0.0f && stopDist >= (limMax[a] - s.pos[a])) {
                        target = 0.0f;
                    } else if (s.vel[a] < 0.0f && stopDist >= (s.pos[a] - limMin[a])) {
                        target = 0.0f;
                    }
                    // Don't ramp INTO a fence we're already sitting on (from rest): cancel a setpoint
                    // that points further past the boundary, but still allow motion back into range.
                    if (s.pos[a] >= limMax[a] && target > 0.0f) {
                        target = 0.0f;
                    } else if (s.pos[a] <= limMin[a] && target < 0.0f) {
                        target = 0.0f;
                    }
                }
                float dv = accel * dt;
                if (s.vel[a] < target) {
                    s.vel[a] = std::min(target, s.vel[a] + dv);
                } else if (s.vel[a] > target) {
                    s.vel[a] = std::max(target, s.vel[a] - dv);
                }
                float before = s.pos[a];
                s.pos[a] += s.vel[a] * dt;
                if (limMin && limMax) {
                    if (s.pos[a] > limMax[a]) {
                        s.pos[a] = limMax[a];
                        s.vel[a] = 0.0f;
                    } else if (s.pos[a] < limMin[a]) {
                        s.pos[a] = limMin[a];
                        s.vel[a] = 0.0f;
                    }
                }
                float d = s.pos[a] - before;
                step2 += d * d;
            }
            s.accDist += std::sqrt(step2);
        }

        // The runway to keep queued ahead of the executing point at the current speed.
        inline float committed_lead_mm(float speed_mm_s, float accel) {
            float lead = (accel > 0.0f) ? (speed_mm_s * speed_mm_s) / (2.0f * accel) : 0.0f;
            lead += speed_mm_s * LEAD_MARGIN_S;
            return std::min(LEAD_MAX_MM, std::max(LEAD_MIN_MM, lead));
        }

        // Decide whether to emit a waypoint at the current trajectory edge. vmag = |vel|.
        inline bool should_emit(const State& s, float vmag) {
            if (s.accDist < SEG_MIN_MM) {
                return false;
            }
            if (s.accDist >= SEG_MAX_MM) {
                return true;
            }
            if (std::fabs(vmag - s.lastEmitSpeed) >= EMIT_DV_MM_S) {
                return true;  // capture the accel/decel ramp
            }
            if (s.haveLastDir && vmag > EPS_V_MM_S) {
                float inv = 1.0f / vmag;
                float dot = (s.vel[0] * inv) * s.lastDir[0] + (s.vel[1] * inv) * s.lastDir[1] + (s.vel[2] * inv) * s.lastDir[2];
                dot       = std::max(-1.0f, std::min(1.0f, dot));
                if (std::acos(dot) >= EMIT_DHEADING) {
                    return true;  // capture the corner — denser waypoints keep junction angles small
                }
            }
            return s.accDist >= vmag * SEG_TIME_S;  // time-scaled cruise segment
        }

        // Reset accumulators after a waypoint is emitted at speed vmag.
        inline void note_emitted(State& s, float vmag) {
            s.accDist       = 0.0f;
            s.lastEmitSpeed = vmag;
            if (vmag > EPS_V_MM_S) {
                float inv = 1.0f / vmag;
                for (int a = 0; a < 3; ++a) {
                    s.lastDir[a] = s.vel[a] * inv;
                }
                s.haveLastDir = true;
            }
        }

        // Highest cruise (mm/min) whose committed lead the planner ring can physically hold: solve
        // v^2/2a + v*LEAD_MARGIN_S = capacity for v, where capacity = (planner_blocks-2)*SEG_MAX_MM
        // (bounded by LEAD_MAX_MM). Commanding faster saturates the queue below the hold threshold
        // and the feed sags/oscillates — clamp the cruise to this instead (a slightly slower jog
        // that holds flat beats a faster setpoint it can never reach). Mirrors
        // JogMath::max_holdable_feed_mm_min for the integrator's segment sizing.
        inline float max_holdable_feed_mm_min(float accel, int planner_blocks) {
            int n = planner_blocks - 2;
            if (n <= 0 || accel <= 0.0f) {
                return 0.0f;
            }
            float cap = std::min(LEAD_MAX_MM, float(n) * SEG_MAX_MM);
            float s   = LEAD_MARGIN_S;
            float v   = accel * (-s + std::sqrt(s * s + 2.0f * cap / accel));  // mm/s, positive root
            return v > 0.0f ? v * MM_S_TO_MM_MIN : 0.0f;
        }

    }  // namespace JogIntegrator
}  // namespace Machine
