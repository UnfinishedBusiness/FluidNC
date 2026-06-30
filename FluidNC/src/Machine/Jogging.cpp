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
#include "../Stepping.h"  // Stepping::getSteps — real stepped-motion probe for the anti-wedge watchdog

#include <algorithm>
#include <cstring>
#include <cmath>

namespace Machine {

    // Virtual-time step the velocity-setpoint integrator advances per refill tick. The refill loop
    // runs at ~1 kHz (vTaskDelay(1)); 1 ms is finer than GcodePilot's 2 ms host substep, so a single
    // integration per tick is accurate without sub-stepping.
    static constexpr float JOG_TICK_S = 0.001f;

    // Snapshot the per-axis stepped-position counters (axis_steps[], advanced ONLY by the step ISR
    // via Stepping::step()) into out[3]. The anti-wedge watchdog compares this snapshot tick-to-tick
    // to tell whether the machine ACTUALLY stepped — independent of the integrator's internal
    // velocity. Per-axis (not a sum) so an opposite-direction diagonal jog can't cancel to "no
    // change". A torn read against the ISR costs at most one tick of detection latency, self-correcting.
    static inline void snapshotAxisSteps(int32_t out[3]) {
        for (int a = 0; a < 3; ++a) {
            out[a] = (a < Axes::_numberAxis) ? int32_t(Stepping::getSteps(axis_t(a))) : 0;
        }
    }

    void Jogging::group(Configuration::HandlerBase& handler) {
        handler.item("allow_unhomed", _allow_unhomed);
    }

    void Jogging::init() {
        log_info("Firmware jogging: allow_unhomed=" << _allow_unhomed);
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

    void Jogging::computeAxisTargetVel(float targetVel[3]) const {
        // Per-axis INDEPENDENT velocity (Issue 2). Every commanded axis targets the wire feed F on
        // its own, clamped only to its own max_rate and its own DDA step ceiling — there is no shared
        // cruise and no vector normalization, so a diagonal runs each axis at (up to) F and the
        // tool-tip speed legitimately exceeds F (~1.41xF for X+Y). Adding a second axis cannot slow
        // the first.
        float F_mm_s = _cruise_mm_min * JogMath::MM_MIN_TO_MM_S;
        auto  axes   = config->_axes;
        for (int a = 0; a < 3; ++a) {
            targetVel[a] = 0.0f;
            if (_vec[a] == 0) {
                continue;
            }
            float v  = F_mm_s;
            auto  ax = (a < Axes::_numberAxis) ? axes->_axis[a] : nullptr;
            if (ax) {
                // Clamp to this axis's own max_rate (mm/min -> mm/s).
                float maxRate_mm_s = ax->_maxRate * JogMath::MM_MIN_TO_MM_S;
                if (maxRate_mm_s > 0.0f) {
                    v = std::min(v, maxRate_mm_s);
                }
                // Per-axis DDA ceiling: the integer DDA emits at most one step/axis/tick, so this
                // axis can step no faster than JOG_STEP_HZ -> JOG_STEP_HZ / steps_per_mm (mm/s). This
                // replaces the old vector-wide JogStepper::maxCruiseMmMin with a true per-axis cap.
                float spmm = ax->_stepsPerMm;
                if (spmm > 0.0f) {
                    v = std::min(v, float(JogStepper::JOG_STEP_HZ) / spmm);
                }
            }
            targetVel[a] = (_vec[a] > 0 ? 1.0f : -1.0f) * v;
        }
    }

    void Jogging::computeAxisAccel(float accel[3]) const {
        // Per-axis acceleration (mm/s^2): each axis ramps/decels at its OWN configured acceleration,
        // including the integrator's proactive soft-limit decel. No single min-accel for the vector.
        auto axes = config->_axes;
        for (int a = 0; a < 3; ++a) {
            auto  ax = (a < Axes::_numberAxis) ? axes->_axis[a] : nullptr;
            float v  = (ax && ax->_acceleration > 0.0f) ? ax->_acceleration : 100.0f;
            accel[a] = v;
        }
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
        if (!anyCommandedAxis()) {
            return Error::InvalidStatement;  // error:3 (zero vector)
        }
        _cruise_mm_min = v.feed_mm_min;
        // log_debug, NOT log_info: a joystick/shuttle sender issues many $Jog/Start per second (one per
        // direction micro-change). At log_info this floods the (CRC-framed) console output on top of the
        // acks + status reports, congesting the 115200 link until acks lag seconds behind — which a host
        // watchdog then mistakes for a dead controller and resets. Keep it for forensics behind $Log/Msg.
        log_debug("JogStart received: X" << int(_vec[0]) << " Y" << int(_vec[1]) << " Z" << int(_vec[2]) << " F" << v.feed_mm_min
                                         << " state=" << state_name());

        // NON-BLOCKING ENTRY (Issue 1): the command handler does NO motion-engine work. It only
        // validates, stashes the vector/feed, flips the phase flag, and returns Ok so the ack fires
        // IMMEDIATELY. The heavy stepper setup (seed + JogStepper::enter -> Stepper::reset/wake_up +
        // set_state(Jog) + arm the LEVEL3 step-timer ISR) happens one tick (~1 ms) later in the
        // refill pump (refillVectorDirect), which runs in the motion-loop context off the
        // command/ack path. Running enter() here is what wedged the line channel in <Jog> with zero
        // motion for ~2.5 s. Do NOT call refill()/enter()/seed() from this handler.
        if (_phase == Phase::Stopping) {
            // Re-press during a ramp-down (rapid tap / quick reversal): resume directly. The
            // integrator still carries the current velocity and JogStepper is still active, so
            // flipping back to Jogging lets the pump blend from the live velocity toward the new
            // vector — no wait for a full stop, no re-seed. (The LinuxCNC "tap it again and it speeds
            // back up" feel.)
            _phase = Phase::Jogging;
            return Error::Ok;
        }

        if (_phase == Phase::Idle) {
            _entryUnhomed = unhomedCarveout;
            if (unhomedCarveout) {
                // Permit motion from Alarm:Unhomed WITHOUT clearing the unhomed flags (unlike $X,
                // which calls set_all_axes_homed()). The jog end restores Alarm:Unhomed. There is
                // no soft-limit envelope while unhomed (position is unknown). (See docs/jogging.md.)
                // This is a cheap state flag, not motion-engine work — safe on the ack path.
                set_state(State::Idle);
            }
        }
        _phase = Phase::Jogging;
        return Error::Ok;
    }

    Error Jogging::changeFeed(float feed_mm_min, Channel& out) {
        if (_phase != Phase::Jogging) {
            return Error::Ok;  // no-op when not jogging (per protocol)
        }
        // Stash only — the refill pump applies the new feed on its next tick. Like startVector, this
        // keeps all motion-engine work off the command/ack path (Issue 1).
        _cruise_mm_min = feed_mm_min;
        return Error::Ok;
    }

    Error Jogging::stop(Channel& out) {
        // RX-proof diagnostic (bench runaway forensics): if this line appears, the command channel
        // delivered the stop; if motion continues anyway the defect is downstream. If it NEVER
        // appears while the host logged a $Jog/Stop TX, the link/channel is eating lines mid-Jog.
        // log_debug, NOT log_info: paired with the per-$Jog/Start log, this fires on every jog command;
        // at log_info a fast joystick/shuttle stream floods the console and backs the link up (see startVector).
        log_debug("JogStop received: phase=" << static_cast<int>(_phase) << " state=" << state_name());
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
        _phase      = Phase::Idle;
        _stallTicks = 0;     // clear the anti-wedge watchdog on every teardown
        resetIntegrator();   // next vector jog re-seeds the trajectory from the machine position

        // The direct-stepper integrator bypasses the planner/suspend system entirely, so its teardown
        // must hand back a clean slate or a subsequent classic $J= planner jog wedges: a lingering
        // sys.step_control flag makes prep_buffer() bail (segment buffer never fills), and a lingering
        // motionCancel/jogCancel suspend bit makes protocol_do_initiate_cycle refuse to start the
        // queued block (machine sits Idle until a reset). JogStepper::enter() already clears these for
        // the *next firmware* jog; clear them here so the next *classic* jog starts clean too.
        sys.step_control = {};
        {
            auto s         = sys.suspend();
            s.bit.motionCancel = false;
            s.bit.jogCancel    = false;
            sys.set_suspend(s);
        }

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
        // ISR wedge-breaker tripped: the step ISR detected a jog firing with no motion for too long
        // (a storm, or <Jog> stuck at zero commanded velocity — the elusive intermittent freeze) and
        // halted its own timer to free the CPU. Tear the jog down to Idle now that we can run again.
        // This is the starvation-proof backstop: it works even when the wedge starved this very loop,
        // because the ISR — not this loop — is what broke it. log_warn so any recurrence is captured.
        if (JogStepper::wedgeAbort()) {
            log_warn("jog wedge breaker: ISR forced recovery (in <Jog> with no stepped motion)");
            JogStepper::clearWedgeAbort();
            set_state(State::Idle);  // onMotionTerminated does the Alarm:Unhomed round-trip if needed
            onMotionTerminated();
            return;
        }

        // Safety net: once the jog is live (enter() set State::Jog), any external termination — an
        // alarm, a suspend/feed-hold, a soft reset, or any state we don't own — drops everything.
        // During our own ramp-down the state is still Jog with no suspend bit, so this stays passive.
        if (JogStepper::active() && (sys_motion_ending() || !state_is(State::Jog))) {
            onMotionTerminated();  // JogStepper::exit() (stop timer + resync) + teardown
            return;
        }

        // Per-axis acceleration (each axis ramps/decels at its own configured accel).
        float accel[3];
        computeAxisAccel(accel);

        // First tick of a fresh jog: seed the integrator at the live position and enter direct mode
        // (sets State::Jog and wakes the step timer at JOG_STEP_HZ). This is the heavy stepper setup
        // the non-blocking handler deliberately deferred to here (Issue 1) — it runs in the
        // motion-loop context, off the command/ack path.
        if (!_integ.primed) {
            JogIntegrator::seed(_integ, gc_state.position);
            JogStepper::enter();
            _stallTicks = 0;                  // fresh entry: arm the anti-wedge watchdog from zero
            snapshotAxisSteps(_lastSteps);    // baseline the stepped-motion probe at the start position
        }

        float maxTargetVel = 0.0f;  // largest commanded per-axis target this tick (mm/s), for the watchdog
        if (_phase == Phase::Jogging) {
            // Independent per-axis target velocity: each commanded axis targets F clamped to its own
            // max_rate and its own DDA step ceiling (no vector normalization, no shared cruise).
            float targetVel[3];
            computeAxisTargetVel(targetVel);
            JogIntegrator::set_target(_integ, targetVel);
            for (int a = 0; a < 3; ++a) {
                maxTargetVel = std::max(maxTargetVel, std::fabs(targetVel[a]));
            }
            // |JogQ target speed: the commanded wire feed F (mm/s). The current speed reported
            // alongside it is the true tool-tip magnitude, which on a diagonal legitimately exceeds F.
            _lastTargetMm = _cruise_mm_min * JogMath::MM_MIN_TO_MM_S;
        } else {
            JogIntegrator::release(_integ);  // Phase::Stopping — ramp each axis's velocity to rest
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
        float curSpeed = JogIntegrator::speed(_integ);
        _lastRunwayMm  = curSpeed;  // |JogQ current speed (mm/s)

        // Anti-wedge watchdog (Issue 1). Keyed on REAL stepped motion (axis_steps[]), NOT the
        // integrator's internal velocity. If a held jog commands a real per-axis target (> EPS) yet
        // the steppers emit NO actual steps for JOG_STALL_TICKS consecutive ticks (~250 ms), force
        // teardown to Idle so the machine can NEVER sit in <Jog> with no motion.
        //
        // Why steps and not velocity: the integrator advances `_integ.vel` every refill tick in this
        // ~1 kHz loop REGARDLESS of whether the step ISR is actually firing. When an intermittent
        // engine-handoff race stops the jog step timer (the pulse_func `return state_is(Jog)` re-arm
        // dropping on a transient non-Jog, after which nothing re-wakes it), the machine sits in
        // <Jog> with frozen MPos while the integrator happily ramps `curSpeed` to FULL speed. The old
        // `curSpeed < EPS` test therefore NEVER tripped (curSpeed was large), so that wedge persisted
        // for seconds until external recovery — exactly the bug. Probing axis_steps[] catches it: a
        // dead step ISR self-heals in ~250 ms (a brief stutter; the held jog must be re-pressed), and
        // the log_warn makes the residual upstream trigger diagnosable. It also still covers the
        // original case (commanded into a soft-limit fence it's parked on → no steps possible → tear
        // down). The Phase::Stopping ramp-to-rest is excluded (it legitimately reaches zero steps).
        // Note: real jog feeds emit many steps per 250 ms, so a live jog can't false-trip; only a
        // genuinely sub-step-rate setpoint could, and that benignly drops to Idle (re-press resumes).
        int32_t steps[3];
        snapshotAxisSteps(steps);
        bool anyStep = false;
        for (int a = 0; a < 3; ++a) {
            if (steps[a] != _lastSteps[a]) {
                anyStep = true;
            }
            _lastSteps[a] = steps[a];
        }
        constexpr int JOG_STALL_TICKS = 250;
        if (_phase == Phase::Jogging && maxTargetVel > JogIntegrator::EPS_V_MM_S && !anyStep) {
            if (++_stallTicks > JOG_STALL_TICKS) {
                log_warn("jog stall watchdog: no stepped motion — forcing idle");
                set_state(State::Idle);  // onMotionTerminated does the Alarm:Unhomed round-trip if needed
                onMotionTerminated();
                return;
            }
        } else {
            _stallTicks = 0;
        }

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
