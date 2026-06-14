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

    volatile bool    JogStepper::_active           = false;
    volatile int32_t JogStepper::_inc[MAX_N_AXIS]  = { 0 };
    int32_t          JogStepper::_acc[MAX_N_AXIS]  = { 0 };

    float JogStepper::maxCruiseMmMin(const float dirUnit[MAX_N_AXIS]) {
        // The fastest axis (|dir|*steps_per_mm largest) must stay <= JOG_STEP_HZ steps/s. Solve for
        // the vector feed (mm/s) at which it equals JOG_STEP_HZ, convert to mm/min.
        auto  axes  = config->_axes;
        float worst = 0.0f;  // steps per (mm of vector travel)
        for (int a = 0; a < MAX_N_AXIS && a < Axes::_numberAxis; ++a) {
            if (axes && axes->_axis[a]) {
                float s = std::fabs(dirUnit[a]) * axes->_axis[a]->_stepsPerMm;
                worst   = std::max(worst, s);
            }
        }
        if (worst <= 0.0f) {
            return 1.0e9f;  // no constraint (no active axis / no steps_per_mm)
        }
        float v_mm_s = float(JOG_STEP_HZ) / worst;  // mm/s where the fastest axis hits JOG_STEP_HZ
        return v_mm_s * 60.0f;
    }

    void JogStepper::setVelocity(const float vel_mm_s[3]) {
        auto axes = config->_axes;
        for (int a = 0; a < MAX_N_AXIS; ++a) {
            float v   = (a < 3) ? vel_mm_s[a] : 0.0f;  // vector jog is XYZ only; higher axes never step
            float inc = 0.0f;
            if (v != 0.0f && a < Axes::_numberAxis && axes && axes->_axis[a]) {
                float spmm = axes->_axis[a]->_stepsPerMm;
                // steps per ISR tick (Q16) = vel[mm/s] * steps/mm / JOG_STEP_HZ * ONE
                inc = v * spmm * (float(ONE) / float(JOG_STEP_HZ));
            }
            // Clamp below one step/tick as a safety floor (cruise is already capped to maxCruise).
            if (inc > float(ONE - 1)) {
                inc = float(ONE - 1);
            } else if (inc < -float(ONE - 1)) {
                inc = -float(ONE - 1);
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
        _active = false;
        Stepper::go_idle();   // stop the timer (no more isr_compute), schedule idle-disable per config
        // axis_steps[] is now final; make the planner + gcode parser agree with it so the next move
        // starts from the true stopped position (the homing / jog-cancel resync recipe).
        gc_sync_position();
        plan_sync_position();
    }
}
