// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Config.h"  // MAX_N_AXIS, AxisMask, axis_t
#include <cstdint>

// LinuxCNC-style direct-stepper velocity jog (Route B). Drives the steppers DIRECTLY from a
// per-axis velocity via an integer DDA inside the step ISR, bypassing the GRBL planner entirely
// (no lookahead, no tail-to-zero — the things that made the planner-fed engine feel wrong).
//
// The trajectory (per-axis velocity that ramps toward direction*cruise at the accel limit and
// blends on a direction change) is produced by Machine::JogIntegrator in the ~1 kHz refill task;
// each tick that task publishes the per-axis velocity here via setVelocity(). The step ISR
// (Stepper::pulse_func) calls isr_compute() to advance a per-axis fixed-point accumulator and emit
// the next step/dir bits — exactly the per-joint stepgen model LinuxCNC uses, so multi-axis jogs
// integrate independently and blend into smooth arcs.
//
// Position truth: the emitted bits are pulsed by Stepping::step(), which advances axis_steps[]
// (the machine-position source of truth). On exit we resync the planner + gcode parser to that
// stepped position so following moves are correct.
namespace Machine {
    class JogStepper {
    public:
        // Step-ISR ceiling during a jog. Single-step-per-tick, so the max jog rate per axis is
        // JOG_STEP_HZ / steps_per_mm. The vector refill caps cruise to keep within this (so the
        // integrator's position can't outrun the steps). Raise if a high-steps/mm machine caps
        // below the desired jog speed (ISR budget permitting).
        static constexpr uint32_t JOG_STEP_HZ = 60000;

        // Step-ISR floor during a jog. The ISR tick rate tracks the fastest axis's step rate (see
        // setVelocity), but never drops below this — so at zero/low velocity the ISR idles here
        // instead of pinning JOG_STEP_HZ. Pinning 60 kHz at v=0 (a tap whose $Jog/Stop lands during
        // the ramp, the first ramp tick, or any stall) starved the main loop that runs BOTH refill()
        // and serial RX, self-sustainingly wedging the machine in <Jog> until the backlog drained.
        // 1 kHz matches the refill cadence (motion resumes within a refill tick) and is negligible
        // next to the ~50 kHz idle baseline.
        static constexpr uint32_t JOG_MIN_STEP_HZ = 1000;

        static bool active() { return _active; }

        // Timer ticks per step-ISR tick (Machine::Stepping::fStepperTimer / f_tick), published by
        // setVelocity and programmed by the jog branch of Stepper::pulse_func every tick. IRAM-safe
        // (plain volatile load; inlined, no flash call).
        static uint32_t isrPeriod() { return _isrPeriod; }

        // Current jog speed (mm/min) — the magnitude of the published velocity vector. Reported as
        // the `|FS:` feed in the `?` status while a direct jog runs (the planner, which normally
        // supplies that, is bypassed). The DDA emits steps at exactly this rate.
        static float currentRateMmMin() { return _rateMmMin; }

        // Begin direct-stepper jogging: clean stepper state, enter State::Jog, run the step timer
        // at JOG_STEP_HZ. Idempotent if already active.
        static void enter();
        // End: stop the timer and resync planner + gcode position to the stepped position. Safe to
        // call when not active.
        static void exit();

        // Publish per-axis velocity (mm/s, signed) from the 1 kHz integrator task (XYZ vector jog).
        // Converts to a fixed-point step increment per ISR tick (atomic 32-bit store per axis).
        static void setVelocity(const float vel_mm_s[3]);

        // Called from the step ISR after the pulse for the previously-computed bits is emitted.
        // Advances the per-axis DDA and writes the NEXT step/dir bits to latch for the next tick.
        // IRAM-safe: integer-only, no float, no config access.
        static void isr_compute(AxisMask& step_out, AxisMask& dir_out);

    private:
        static constexpr int32_t ONE = 1 << 16;  // fixed-point unit (Q16)

        static volatile bool     _active;
        static volatile int32_t  _inc[MAX_N_AXIS];  // signed steps per tick, Q16 (sign = travel dir)
        static int32_t           _acc[MAX_N_AXIS];   // ISR-private fractional-step accumulator (Q16)
        static volatile float    _rateMmMin;         // |velocity| (mm/min) for the |FS: status field
        static volatile uint32_t _isrPeriod;         // timer ticks/ISR tick — tracks the live step rate
    };
}
