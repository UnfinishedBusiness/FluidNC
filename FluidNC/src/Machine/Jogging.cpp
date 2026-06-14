// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Jogging.h"

#include "MachineConfig.h"   // config, Axes, Kinematics
#include "../MotionControl.h"  // mc_linear
#include "../Planner.h"        // plan_get_block_buffer_available, plan_line_data_t
#include "../Protocol.h"       // protocol_auto_cycle_start, protocol_do_motion_cancel, lastAlarm
#include "../GCode.h"          // gc_state
#include "../System.h"         // get_mpos, state_is, sys_motion_ending
#include "../Limit.h"          // limitsMinPosition, limitsMaxPosition
#include "../Channel.h"
#include "../Logging.h"  // LogStream, log_*, setprecision
#include "JogMath.h"
#include "JogIntegrator.h"
#include "JogStepper.h"
#include "JogLifecycle.h"
#include "Homing.h"

#include <cstring>
#include <cmath>

namespace Machine {

    // Sentinel line number on synthesized jog blocks (kept out of the program line space).
    static constexpr int32_t JOG_LINE = -1;

    // Virtual-time step the velocity-setpoint integrator advances per sub-iteration. The refill loop
    // runs at ~1 kHz (vTaskDelay(1)); 1 ms is finer than GcodePilot's 2 ms host substep, so a single
    // integration per sub-iteration is accurate without sub-stepping.
    static constexpr float JOG_TICK_S = 0.001f;

    // Refill ticks to tolerate State::Idle after issuing a cycle start before the state flips to
    // Jog. The handoff is normally 1 tick; this bounds a start that never takes (e.g. a jog that
    // immediately parks on a soft limit) so the module terminates instead of spinning forever.
    static constexpr int PENDING_START_TICKS = 8;

    // Observe the machine state through the same state_is() seam the rest of the module uses. The
    // lifecycle decision only distinguishes Jog / Idle / "anything else", so non-Jog/non-Idle
    // states collapse to a single representative (Alarm) that maps to Terminate.
    static State observedState() {
        if (state_is(State::Jog)) {
            return State::Jog;
        }
        if (state_is(State::Idle)) {
            return State::Idle;
        }
        return State::Alarm;
    }

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
        _kind          = Kind::Vector;  // fix sticky-kind: a vector jog after a shuttle session must dispatch to refillVector
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
        if (_kind == Kind::Vector) {
            // Direct-stepper release: ramp the velocity setpoint to rest at the accel limit (smooth,
            // overshoot = v^2/2a, the LinuxCNC minimum). No planner, no flush — the DDA simply
            // decelerates. refillVectorDirect's Stopping finalizer exits + resyncs once at rest.
            _phase = Phase::Stopping;
            JogIntegrator::release(_integ);
            return Error::Ok;
        }
        // ----- Shuttle (planner-fed, unchanged): decel via jogCancel or flush the unstarted tail -----
        if (JogLifecycle::stop_action(observedState()) == JogLifecycle::StopAction::CancelInflight) {
            _phase = Phase::Stopping;
            protocol_do_motion_cancel();
        } else {
            flushQueuedJog();
            onMotionTerminated();
        }
        return Error::Ok;
    }

    void Jogging::reset() {
        onMotionTerminated();  // phase/lifecycle teardown
        _shuttleOpen = false;  // reset() additionally tears the shuttle session down
        _path.clear();
    }

    void Jogging::stopFromRealtime() {
        if (_phase != Phase::Jogging) {
            return;
        }
        // A panic byte means STOP, not "stop then continue" — drop any pending resume.
        _pendingDir        = 0;
        _pendingVecRestart = false;
        _shuttleDir        = 0;
        _phase             = Phase::Stopping;
        if (_kind == Kind::Vector) {
            JogIntegrator::release(_integ);  // same smooth accel-limited ramp-down as $Jog/Stop
            return;
        }
        if (state_is(State::Jog)) {
            protocol_do_motion_cancel();  // shuttle (planner-fed)
        }
    }

    void Jogging::beginPendingStart() {
        _sawJog            = false;
        _pendingStartTicks = PENDING_START_TICKS;
    }

    void Jogging::flushQueuedJog() {
        // Drop blocks that are queued but not yet executing (the JogCancel flush sequence). Safe
        // here because the steppers have not been woken (state is not Jog).
        protocol_flush_queued_jog();
    }

    void Jogging::onMotionTerminated() {
        if (_phase == Phase::Idle) {
            return;  // idempotent — safe to call from multiple termination sites
        }
        JogStepper::exit();  // direct-stepper jog: stop the step timer + resync position (no-op if inactive)
        _phase             = Phase::Idle;
        _sawJog            = false;
        _pendingStartTicks = 0;
        _shuttleDir        = 0;
        _pendingDir        = 0;      // a hard termination cancels any pending shuttle reversal/resume
        _pendingVecRestart = false;  // ... and any vector restart queued during a decel
        resetIntegrator();           // next vector jog re-seeds the trajectory from the machine position
        if (_shuttleOpen) {
            _reseedOnRun = true;  // re-project the commanded position on the next shuttle jog
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
        // Re-entrancy guard: mc_linear -> mc_move_motors calls protocol_execute_realtime, which calls
        // refill() again — every queued block recursed one level deeper (stack-hungry at high feeds,
        // double-queueing per tick). The outer pass does all the work; nested passes are no-ops.
        if (_inRefill) {
            return;
        }
        _inRefill = true;
        refillImpl();
        _inRefill = false;
    }

    void Jogging::refillImpl() {
        // Vector jogs run on the direct-stepper engine (Route B): the per-axis velocity integrator
        // drives an integer DDA in the step ISR, no planner. refillVectorDirect handles both the
        // held jog (Phase::Jogging) and the ramp-to-rest (Phase::Stopping).
        if (_kind == Kind::Vector) {
            if (_phase != Phase::Idle) {
                refillVectorDirect();
            }
            return;
        }

        // ── Shuttle: planner-fed (unchanged) ────────────────────────────────────────────────────
        if (_phase != Phase::Jogging) {
            // Cancel/decel finished and back to Idle: finalize and resume a pending reversal.
            if (_phase == Phase::Stopping && (state_is(State::Idle) || state_is(State::Alarm))) {
                _phase = Phase::Idle;
                if (_entryUnhomed && state_is(State::Idle)) {
                    _entryUnhomed = false;
                    send_alarm(ExecAlarm::Unhomed);
                }
                if (_shuttleOpen) {
                    _reseedOnRun = true;
                    if (_pendingDir != 0) {
                        _shuttleDir    = _pendingDir;
                        _cruise_mm_min = _pendingFeed;
                        _pendingDir    = 0;
                        beginPendingStart();
                        _phase = Phase::Jogging;
                    }
                }
            }
            return;
        }

        // Self-validating refill (the runaway guard): queue only while genuinely jogging.
        State st           = observedState();
        if (st == State::Jog) {
            _sawJog            = true;
            _pendingStartTicks = 0;
        }
        bool motionEnding = sys_motion_ending();
        bool pendingStart = _pendingStartTicks > 0 && !_sawJog;
        if (JogLifecycle::refill_action(st, motionEnding, _sawJog, pendingStart) == JogLifecycle::RefillAction::Terminate) {
            onMotionTerminated();
            return;
        }
        if (pendingStart) {
            --_pendingStartTicks;
        }
        refillShuttle();
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

    float Jogging::shuttleAccel() const {
        // Limiting acceleration over the XY axes (the shuttle plane), mm/s^2.
        float accel = 0.0f;
        auto  axes  = config->_axes;
        for (int a = 0; a < 2 && a < Axes::_numberAxis; ++a) {
            if (axes->_axis[a]) {
                float ax = axes->_axis[a]->_acceleration;
                accel    = (accel == 0.0f) ? ax : std::min(accel, ax);
            }
        }
        return accel > 0.0f ? accel : 100.0f;
    }

    void Jogging::refillShuttle() {
        if (_shuttleDir == 0) {
            return;  // released: let the planner tail-to-zero coast to a stop on the path
        }

        // Re-seed the commanded path position to where the machine actually is, after a stop.
        if (_reseedOnRun) {
            float* mpos = get_mpos();
            _cmdPos     = _path.project(mpos[0], mpos[1]);
            copyAxes(gc_state.position, mpos);  // resync commanded to actual
            _reseedOnRun = false;
        }

        float cruise = _cruise_mm_min;
        if (unhomedFeedCapActive()) {
            cruise = std::min(cruise, float(_unhomed_feed_cap_mm_min));
        }
        if (cruise <= 0.0f) {
            return;
        }
        float accel        = shuttleAccel();
        float blockLen     = JogMath::block_len_mm(cruise);
        float targetRunway = JogMath::target_runway_mm(cruise, accel);

        int queued = 0;
        while (_phase == Phase::Jogging && plan_get_block_buffer_available() > 1) {
            // Runway = straight-line distance from machine to the commanded path point. Over the
            // few queued mm the path is nearly straight, so this tracks queued arc length.
            float* mpos = get_mpos();
            float  cx, cy;
            _path.xy(_cmdPos, cx, cy);
            float dx     = cx - mpos[0];
            float dy     = cy - mpos[1];
            float runway = std::sqrt(dx * dx + dy * dy);
            if (runway >= targetRunway) {
                break;
            }

            bool             atEdge;
            ShuttlePath::Pos next = _path.advance(_cmdPos, blockLen, _shuttleDir, atEdge);
            if (next.seg == _cmdPos.seg && next.frac == _cmdPos.frac) {
                break;  // window edge / path end: no progress, planner tail-to-zero parks us here
            }

            float target[MAX_N_AXIS];
            copyAxes(target, gc_state.position);
            _path.xy(next, target[0], target[1]);  // shuttle moves in XY; Z held

            plan_line_data_t pl;
            memset(&pl, 0, sizeof(pl));
            pl.feed_rate             = cruise;
            pl.motion.noFeedOverride = 1;
            pl.is_jog                = true;
            pl.line_number           = JOG_LINE;
            pl.limits_checked        = true;  // path vertices are inside the program envelope

            if (!mc_linear(target, &pl, gc_state.position)) {
                break;  // cancelled in-flight
            }
            copyAxes(gc_state.position, target);
            _cmdPos = next;
            ++queued;
        }

        if (queued) {
            protocol_auto_cycle_start();
        }
    }

    Error Jogging::shuttleBegin(int total, Channel& out) {
        if (!(state_is(State::Idle) || (_shuttleOpen && state_is(State::Jog)))) {
            return Error::SystemGcLock;  // error:9
        }
        if (total < 2 || total > 1000000) {
            return Error::InvalidStatement;  // error:3
        }
        _path.begin(total);
        _cmdPos      = ShuttlePath::Pos {};
        _shuttleOpen = true;
        _shuttleDir  = 0;
        _pendingDir  = 0;
        _reseedOnRun = true;  // seed from the machine position on the first jog
        _kind        = Kind::Shuttle;
        return Error::Ok;
    }

    Error Jogging::shuttleData(const char* body, Channel& out) {
        if (!_shuttleOpen) {
            return Error::SystemGcLock;  // error:9, no session
        }
        JogCommand::ShuttleVertex verts[64];
        int                       first = 0;
        int                       n     = JogCommand::parse_shuttle_data(body, first, verts, 64);
        if (n < 0) {
            return Error::InvalidStatement;  // error:3
        }
        if (!_path.put(first, verts, n)) {
            return Error::InvalidStatement;  // out-of-window / gap
        }
        return Error::Ok;
    }

    Error Jogging::shuttleJog(int8_t dir, float feed_mm_min, Channel& out) {
        if (!_shuttleOpen) {
            return Error::SystemGcLock;  // error:9
        }
        if (!(state_is(State::Idle) || state_is(State::Jog))) {
            return Error::SystemGcLock;
        }

        // Entry validation: the machine must be ON the loaded path (host pre-positions it).
        if (dir != 0 && _shuttleDir == 0 && !_path.empty()) {
            float* mpos = get_mpos();
            float  d2   = 0.0f;
            _path.project(mpos[0], mpos[1], &d2);
            if (d2 > 1.0f) {  // > 1 mm off the path
                return Error::SystemGcLock;  // error:9, not on path
            }
        }

        if (dir == 0) {
            // Release: decel to rest on the path (no reversal pending).
            _shuttleDir  = 0;
            _pendingDir  = 0;
            return stop(out);
        }

        if (_shuttleDir != 0 && dir != _shuttleDir) {
            // Reversal: stop first, then resume the other way once at rest (handled in refill()).
            _pendingDir  = dir;
            _pendingFeed = feed_mm_min;
            _shuttleDir  = 0;
            return stop(out);
        }

        // Same direction (or starting from rest): hold and refill.
        _shuttleDir    = dir;
        _cruise_mm_min = feed_mm_min;
        _kind          = Kind::Shuttle;
        if (_phase != Phase::Jogging) {
            beginPendingStart();  // arm the Idle->Jog handoff when starting from rest
        }
        _phase = Phase::Jogging;
        refill();
        return Error::Ok;
    }

    Error Jogging::shuttleEnd(Channel& out) {
        if (_phase == Phase::Jogging) {
            stop(out);  // Jog -> JogCancel decel; otherwise flush queued blocks (same tap-race guarantee)
        }
        _shuttleOpen = false;
        _shuttleDir  = 0;
        _pendingDir  = 0;
        _path.clear();
        return Error::Ok;
    }

    void Jogging::status_report(LogStream& msg) {
        // Live queue health while vector-jogging: |JogQ:<runway_mm>,<target_mm>. Bench-diagnosable
        // velocity hold — runway pinned at/above target == flat feed; runway sagging below braking
        // distance == the planner is decelerating (capacity/cadence problem to investigate).
        if (_phase == Phase::Jogging && _kind == Kind::Vector) {
            msg << "|JogQ:" << setprecision(1) << _lastRunwayMm << "," << _lastTargetMm;
        }
        if (!_shuttleOpen || _path.empty()) {
            return;
        }
        float*           mpos = get_mpos();
        ShuttlePath::Pos here = _path.project(mpos[0], mpos[1]);
        msg << "|Shu:" << here.seg << "," << setprecision(3) << _path.arcFromVertex(here);
    }

}
