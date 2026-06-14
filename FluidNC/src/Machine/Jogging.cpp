// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Jogging.h"

#include "MachineConfig.h"   // config, Axes, Kinematics
#include "../Protocol.h"       // lastAlarm
#include "../GCode.h"          // gc_state
#include "../System.h"         // get_mpos, state_is, sys_motion_ending
#include "../Limit.h"          // limitsMinPosition, limitsMaxPosition
#include "../Channel.h"
#include "../Logging.h"  // LogStream, log_*, setprecision
#include "JogMath.h"
#include "JogIntegrator.h"
#include "JogStepper.h"
#include "Homing.h"

#include <cstring>
#include <cmath>

namespace Machine {

    // Virtual-time step the velocity-setpoint integrator advances per refill tick. The refill loop
    // runs at ~1 kHz (vTaskDelay(1)); 1 ms is finer than GcodePilot's 2 ms host substep, so a single
    // integration per tick is accurate without sub-stepping.
    static constexpr float JOG_TICK_S = 0.001f;

    void Jogging::group(Configuration::HandlerBase& handler) {
        handler.item("allow_unhomed", _allow_unhomed);
        handler.item("unhomed_feed_cap_mm_min", _unhomed_feed_cap_mm_min, 0, 1000000);
    }

    void Jogging::init() {
        log_info("Firmware jogging: allow_unhomed=" << _allow_unhomed << " unhomed_cap=" << _unhomed_feed_cap_mm_min
                                                    << "mm/min");
    }

    bool Jogging::anyAxisUnhomed() const {
        return Homing::unhomed_axes() != 0;
    }

    bool Jogging::homingRequired() const {
        return config && config->_start && config->_start->_mustHome && Axes::homingMask != 0;
    }

    bool Jogging::canStartUnhomedJog() const {
        return _allow_unhomed && homingRequired() && anyAxisUnhomed() && state_is(State::Alarm) &&
               (lastAlarm == ExecAlarm::Unhomed);
    }

    bool Jogging::unhomedJogExceptionActive() const {
        return _entryUnhomed && _allow_unhomed && homingRequired() && anyAxisUnhomed();
    }

    bool Jogging::unhomedFeedCapActive() const {
        return unhomedJogExceptionActive() && _unhomed_feed_cap_mm_min > 0;
    }

    bool Jogging::computeDirection() {
        float len2 = 0.0f;
        for (int a = 0; a < 3; ++a) {
            len2 += float(_vec[a]) * float(_vec[a]);
        }
        if (len2 <= 0.0f) {
            return false;
        }
        float inv = 1.0f / std::sqrt(len2);
        for (int a = 0; a < 3; ++a) {
            _dirUnit[a] = float(_vec[a]) * inv;
        }
        return true;
    }

    float Jogging::vectorAccel() const {
        // Limiting (smallest) acceleration across the active axes, mm/s^2.
        float accel = 0.0f;
        auto  axes  = config->_axes;
        for (int a = 0; a < 3 && a < Axes::_numberAxis; ++a) {
            if (_dirUnit[a] != 0.0f && axes->_axis[a]) {
                float ax = axes->_axis[a]->_acceleration;
                accel    = (accel == 0.0f) ? ax : std::min(accel, ax);
            }
        }
        return accel > 0.0f ? accel : 100.0f;
    }

    float Jogging::effectiveCruise() const {
        // Clamp the requested cruise so no active axis exceeds its max_rate along the vector,
        // then apply the unhomed feed cap only for the Alarm:Unhomed jog carve-out.
        float maxRate[3] = { 0, 0, 0 };
        auto  axes       = config->_axes;
        for (int a = 0; a < 3 && a < Axes::_numberAxis; ++a) {
            if (axes->_axis[a]) {
                maxRate[a] = axes->_axis[a]->_maxRate;
            }
        }
        float cruise = JogMath::clamp_feed_to_axis_rates(_cruise_mm_min, _dirUnit, maxRate);
        if (unhomedFeedCapActive()) {
            cruise = std::min(cruise, float(_unhomed_feed_cap_mm_min));
        }
        return cruise;
    }

    Error Jogging::startVector(const JogCommand::Vector& v, Channel& out) {
        // Allowed states: Idle, Jog, or the Alarm:Unhomed carve-out.
        bool unhomedCarveout = canStartUnhomedJog();
        if (!(state_is(State::Idle) || state_is(State::Jog) || unhomedCarveout)) {
            return Error::SystemGcLock;  // error:9, disallowed state
        }

        _vec[0] = v.axis[0];
        _vec[1] = v.axis[1];
        _vec[2] = v.axis[2];
        if (!computeDirection()) {
            return Error::InvalidStatement;  // error:3
        }
        _cruise_mm_min = v.feed_mm_min;
        log_info("JogStart received: X" << int(_vec[0]) << " Y" << int(_vec[1]) << " Z" << int(_vec[2]) << " F" << v.feed_mm_min
                                        << " state=" << state_name());

        if (_phase == Phase::Stopping) {
            // Re-press during a ramp-down (rapid tap / quick reversal): resume directly. The
            // integrator still carries the current velocity and JogStepper is still active, so
            // flipping back to Jogging blends from the live velocity toward the new vector — no wait
            // for a full stop, no re-seed. (This is the LinuxCNC "tap it again and it speeds back
            // up" feel.)
            _phase = Phase::Jogging;
            refill();
            return Error::Ok;
        }

        if (_phase == Phase::Idle) {
            _entryUnhomed = unhomedCarveout;
            if (unhomedCarveout) {
                // Permit motion from Alarm:Unhomed WITHOUT clearing the unhomed flags (unlike $X,
                // which calls set_all_axes_homed()). The jog end restores Alarm:Unhomed; the unhomed
                // feed cap is the only protection while unhomed. (See docs/jogging.md.)
                set_state(State::Idle);
            }
        }
        _phase = Phase::Jogging;
        refill();  // first tick seeds the integrator + JogStepper::enter()
        return Error::Ok;
    }

    Error Jogging::changeFeed(float feed_mm_min, Channel& out) {
        if (_phase != Phase::Jogging) {
            return Error::Ok;  // no-op when not jogging (per protocol)
        }
        _cruise_mm_min = feed_mm_min;
        refill();
        return Error::Ok;
    }

    Error Jogging::stop(Channel& out) {
        // RX-proof diagnostic (bench runaway forensics): if this line appears, the command channel
        // delivered the stop; if motion continues anyway the defect is downstream. If it NEVER
        // appears while the host logged a $Jog/Stop TX, the link/channel is eating lines mid-Jog.
        log_info("JogStop received: phase=" << static_cast<int>(_phase) << " state=" << state_name());
        if (_phase != Phase::Jogging) {
            return Error::Ok;
        }
        // Release: ramp the velocity setpoint to rest at the accel limit (smooth, overshoot =
        // v^2/2a, the LinuxCNC minimum). The DDA simply decelerates; refillVectorDirect's Stopping
        // finalizer exits + resyncs position once at rest.
        _phase = Phase::Stopping;
        JogIntegrator::release(_integ);
        return Error::Ok;
    }

    void Jogging::reset() {
        onMotionTerminated();  // phase teardown (stops the step timer + resyncs position)
    }

    void Jogging::stopFromRealtime() {
        if (_phase != Phase::Jogging) {
            return;
        }
        // 0x85 panic stop: same smooth accel-limited ramp-down as $Jog/Stop.
        _phase = Phase::Stopping;
        JogIntegrator::release(_integ);
    }

    void Jogging::onMotionTerminated() {
        if (_phase == Phase::Idle) {
            return;  // idempotent — safe to call from multiple termination sites
        }
        JogStepper::exit();  // direct-stepper jog: stop the step timer + resync position (no-op if inactive)
        _phase = Phase::Idle;
        resetIntegrator();   // next vector jog re-seeds the trajectory from the machine position
        // Unhomed round-trip: restore Alarm:Unhomed only if the terminating path left us at Idle
        // (an external alarm has already set its own alarm state and must not be overwritten).
        if (_entryUnhomed) {
            _entryUnhomed = false;
            if (state_is(State::Idle)) {
                send_alarm(ExecAlarm::Unhomed);
            }
        }
    }

    void Jogging::refill() {
        // Re-entrancy guard: enter()'s Stepper::reset/wake_up and other realtime hooks can re-enter
        // protocol_execute_realtime -> refill(). The outer pass does the work; nested passes no-op.
        if (_inRefill) {
            return;
        }
        _inRefill = true;
        refillImpl();
        _inRefill = false;
    }

    void Jogging::refillImpl() {
        // The vector jog runs on the direct-stepper engine: the per-axis velocity integrator drives
        // an integer DDA in the step ISR, no planner. refillVectorDirect handles both the held jog
        // (Phase::Jogging) and the ramp-to-rest (Phase::Stopping).
        if (_phase != Phase::Idle) {
            refillVectorDirect();
        }
    }

    void Jogging::refillVectorDirect() {
        // Safety net: once the jog is live (enter() set State::Jog), any external termination — an
        // alarm, a suspend/feed-hold, a soft reset, or any state we don't own — drops everything.
        // During our own ramp-down the state is still Jog with no suspend bit, so this stays passive.
        if (JogStepper::active() && (sys_motion_ending() || !state_is(State::Jog))) {
            onMotionTerminated();  // JogStepper::exit() (stop timer + resync) + teardown
            return;
        }

        float accel = vectorAccel();

        // First tick of a fresh jog: seed the integrator at the live position and enter direct mode
        // (sets State::Jog and wakes the step timer at JOG_STEP_HZ).
        if (!_integ.primed) {
            JogIntegrator::seed(_integ, gc_state.position);
            JogStepper::enter();
        }

        if (_phase == Phase::Jogging) {
            float cruise = effectiveCruise();
            // Cap cruise so the DDA never owes more than one step/axis/tick (keeps the integrator's
            // position from outrunning the steps it commands).
            cruise        = std::min(cruise, JogStepper::maxCruiseMmMin(_dirUnit));
            JogIntegrator::set_target(_integ, _dirUnit, cruise);
            _lastTargetMm = cruise * JogMath::MM_MIN_TO_MM_S;  // |JogQ target speed (mm/s)
        } else {
            JogIntegrator::release(_integ);  // Phase::Stopping — ramp the velocity vector to rest
            _lastTargetMm = 0.0f;
        }

        // Per-axis soft-limit fences: gate on Axis::_softLimits, NOT homing, so a soft-limits-on
        // machine with no homing (known boot position) still gets extents. The only no-fence case
        // is the Alarm:Unhomed free-jog carve-out (position genuinely unknown — reach the switches).
        float        limMin[3], limMax[3];
        const float* pMin = nullptr;
        const float* pMax = nullptr;
        if (!unhomedJogExceptionActive()) {
            for (int a = 0; a < 3; ++a) {
                auto ax = (a < Axes::_numberAxis) ? config->_axes->_axis[a] : nullptr;
                if (ax && ax->_softLimits) {
                    limMin[a] = limitsMinPosition(axis_t(a));
                    limMax[a] = limitsMaxPosition(axis_t(a));
                } else {
                    limMin[a] = -1.0e9f;
                    limMax[a] = 1.0e9f;
                }
            }
            pMin = limMin;
            pMax = limMax;
        }

        JogIntegrator::integrate_tick(_integ, accel, JOG_TICK_S, pMin, pMax);
        JogStepper::setVelocity(_integ.vel);
        _lastRunwayMm = JogIntegrator::speed(_integ);  // |JogQ current speed (mm/s)

        // Ramp-to-rest complete: back to Idle, exit the stepper, resync planner+gcode position.
        if (_phase == Phase::Stopping && !JogIntegrator::moving(_integ)) {
            set_state(State::Idle);  // onMotionTerminated does the Alarm:Unhomed round-trip if needed
            onMotionTerminated();
        }
    }

    void Jogging::status_report(LogStream& msg) {
        // Live jog status while a vector jog runs: |JogQ:<speed_mm/s>,<target_speed_mm/s>.
        if (_phase == Phase::Jogging) {
            msg << "|JogQ:" << setprecision(1) << _lastRunwayMm << "," << _lastTargetMm;
        }
    }

}
