// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "JogStepper.h"

#include "MachineConfig.h"  // config, Axes
#include "JogIntegrator.h"  // dda_step (shared, host-tested per-axis DDA)
#include "../Stepper.h"     // Stepper::reset/wake_up/go_idle
#include "../Stepping.h"    // Machine::Stepping::setTimerPeriod, fStepperTimer
#include "../System.h"      // sys, set_state, State, get_mpos
#include "../GCode.h"       // gc_sync_position
#include "../Planner.h"     // plan_sync_position

#include <cmath>

namespace Machine {

    volatile bool     JogStepper::_active          = false;
    volatile int32_t  JogStepper::_inc[MAX_N_AXIS] = { 0 };
    int32_t           JogStepper::_acc[MAX_N_AXIS] = { 0 };
    volatile float    JogStepper::_rateMmMin       = 0.0f;
    // Start at the floor period so the very first ISR tick (which fires before refill's first
    // setVelocity publishes a real rate) idles at JOG_MIN_STEP_HZ rather than storming.
    volatile uint32_t JogStepper::_isrPeriod       = Stepping::fStepperTimer / JOG_MIN_STEP_HZ;

    void JogStepper::setVelocity(const float vel_mm_s[3]) {
        // Publish the speed (mm/min) for the |FS: status field.
        float v2   = vel_mm_s[0] * vel_mm_s[0] + vel_mm_s[1] * vel_mm_s[1] + vel_mm_s[2] * vel_mm_s[2];
        _rateMmMin = std::sqrt(v2) * 60.0f;

        auto axes = config->_axes;

        // Per-axis signed step rate (steps/s), and the magnitude of the fastest-stepping axis.
        float rate[MAX_N_AXIS] = { 0.0f };
        float fastest          = 0.0f;
        for (int a = 0; a < MAX_N_AXIS; ++a) {
            float v = (a < 3) ? vel_mm_s[a] : 0.0f;  // vector jog is XYZ only; higher axes never step
            if (v != 0.0f && a < Axes::_numberAxis && axes && axes->_axis[a]) {
                rate[a] = v * axes->_axis[a]->_stepsPerMm;
                fastest = std::max(fastest, std::fabs(rate[a]));
            }
        }

        // The ISR tick rate TRACKS the fastest axis's step rate: that axis then emits ~one step per
        // tick (max DDA resolution) and — the whole point of this engine's storm fix — at zero/low
        // velocity the rate collapses toward JOG_MIN_STEP_HZ instead of pinning JOG_STEP_HZ, so the
        // step ISR stops preempting the main loop (refill() + serial RX). Floor keeps it responsive
        // to the next velocity update; ceiling guards rounding (each axis's target is already capped
        // per-axis to JOG_STEP_HZ/steps_per_mm in Jogging::computeAxisTargetVel, so the fastest axis
        // never truly exceeds JOG_STEP_HZ).
        float f_tick = fastest;
        if (f_tick < float(JOG_MIN_STEP_HZ)) {
            f_tick = float(JOG_MIN_STEP_HZ);
        } else if (f_tick > float(JOG_STEP_HZ)) {
            f_tick = float(JOG_STEP_HZ);
        }

        // Publish the timer period the jog branch of pulse_func programs each tick. Single aligned
        // 32-bit store — atomic on Xtensa; a torn read against _inc[] below costs at most one tick of
        // step mistiming, self-correcting on the next tick.
        _isrPeriod = uint32_t(lroundf(float(Stepping::fStepperTimer) / f_tick));

        // Per-axis fixed-point increment, now calibrated to f_tick (NOT the fixed JOG_STEP_HZ):
        //   steps per ISR tick (Q16) = rate[steps/s] / f_tick * ONE.   The fastest axis lands at ~ONE.
        for (int a = 0; a < MAX_N_AXIS; ++a) {
            float inc = rate[a] * (float(ONE) / f_tick);
            // Ceiling at one step/tick (the fastest axis sits here by construction; clamp guards
            // float rounding from ever owing >1 step/tick).
            if (inc > float(ONE)) {
                inc = float(ONE);
            } else if (inc < -float(ONE)) {
                inc = -float(ONE);
            }
            _inc[a] = int32_t(lroundf(inc));  // single aligned 32-bit store — atomic on Xtensa
        }
    }

    void IRAM_ATTR JogStepper::isr_compute(AxisMask& step_out, AxisMask& dir_out) {
        AxisMask steps = 0;
        AxisMask dirs  = 0;
        auto     n     = Axes::_numberAxis;
        for (axis_t a = X_AXIS; a < n; ++a) {
            int32_t inc = _inc[a];  // atomic read
            if (inc < 0) {
                set_bitnum(dirs, a);  // negative machine direction (matches Stepping::step convention)
            }
            if (JogIntegrator::dda_step(inc, _acc[a]) != 0) {
                set_bitnum(steps, a);
            }
        }
        step_out = steps;
        dir_out  = dirs;
    }

    void JogStepper::enter() {
        if (_active) {
            return;
        }
        for (int a = 0; a < MAX_N_AXIS; ++a) {
            _acc[a] = 0;
            _inc[a] = 0;
        }
        // Idle at the floor period until refill's first setVelocity publishes a real rate, so the
        // first tick after wake_up doesn't fire at the previous jog's stale (fast) period.
        _isrPeriod = Stepping::fStepperTimer / JOG_MIN_STEP_HZ;
        set_state(State::Jog);
        sys.step_control = {};   // no executeHold/executeSysMotion lingering; segment path stays dead
        Stepper::reset();        // clear st (step_outbits=0) and the segment buffer; go_idle internally
        _active = true;          // publish AFTER state is seeded + accumulators zeroed, BEFORE waking
        Stepper::wake_up();      // enable drivers, cancel pending disable, start the step timer
        // NOTE: stepTimerStart() hardcodes the first alarm to 10 ticks; the auto-reload alarm then
        // keeps that value until the ISR changes it. The pulse_func jog branch sets the real
        // JOG_STEP_HZ period on every tick, so after the first ~0.5 us fire the rate locks to
        // JOG_STEP_HZ. Do NOT rely on setting the period here (start_timer overrides it).
    }

    void JogStepper::exit() {
        if (!_active) {
            return;
        }
        _active    = false;
        _rateMmMin = 0.0f;
        Stepper::go_idle();   // stop the timer (no more isr_compute), schedule idle-disable per config
        // axis_steps[] is now final; make the planner + gcode parser agree with it so the next move
        // starts from the true stopped position (the homing / jog-cancel resync recipe).
        gc_sync_position();
        plan_sync_position();
    }
}
