// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "JogCommand.h"
#include "JogIntegrator.h"  // velocity-setpoint trajectory state for vector jogs
#include "ShuttlePath.h"
#include "../Error.h"
#include "../Config.h"  // MAX_N_AXIS

class Channel;
class LogStream;

namespace Machine {

    // Route-A firmware-native jogging engine: a self-refilling planner feed. Keeps a few jog
    // blocks queued through the EXISTING planner/stepper path with sub-ms refill latency, so the
    // planner's tail-to-zero never executes and held velocity stays flat. See docs/jogging.md and
    // plans/firmware-jogging/protocol.md. Modeled on Machine::THC (a MachineConfig member).
    class Jogging : public Configuration::Configurable {
    public:
        Jogging() = default;

        // config (jogging: section)
        bool    _allow_unhomed            = false;
        int32_t _unhomed_feed_cap_mm_min  = 1000;

        void init();
        void group(Configuration::HandlerBase& handler) override;

        // $Jog/* command entry points (called from the registered handlers).
        Error startVector(const JogCommand::Vector& v, Channel& out);
        Error changeFeed(float feed_mm_min, Channel& out);
        Error stop(Channel& out);
        // Realtime 0x85 while the firmware engine is active: same stop as $Jog/Stop's live branch
        // — set _phase=Stopping (refill engine stands down) then jogCancel decel-in-place + flush.
        // Runs in the pollLine/protocol task context, so it may mutate module state and trigger the
        // cancel (set the suspend jogCancel bit) directly.
        void stopFromRealtime();

        // $Shu/* command entry points (path-window shuttle mode).
        Error shuttleBegin(int total, Channel& out);
        Error shuttleData(const char* body, Channel& out);
        Error shuttleJog(int8_t dir, float feed_mm_min, Channel& out);
        Error shuttleEnd(Channel& out);

        // Append the |Shu:<vidx>,<s_mm> status field when a shuttle session is open.
        void status_report(LogStream& msg);
        bool shuttleSession() const { return _shuttleOpen; }

        // Called from protocol_exec_rt_system (main loop, NOT an ISR) to top up the queue.
        void refill();

        // Belt-and-suspenders teardown: called from every external motion-termination site
        // (jog-cancel flush, soft reset, alarm) so the module exits Phase::Jogging immediately
        // instead of waiting for refill() to notice. Idempotent; queues nothing.
        void onMotionTerminated();

        // Full teardown incl. shuttle session (alarm/reset/disconnect).
        void reset();

        bool active() const { return _phase != Phase::Idle; }

    private:
        enum class Phase { Idle, Jogging, Stopping };
        enum class Kind { Vector, Shuttle };

        Phase  _phase            = Phase::Idle;
        Kind   _kind             = Kind::Vector;
        int8_t _vec[3]           = { 0, 0, 0 };
        float  _dirUnit[3]       = { 0, 0, 0 };  // normalized jog vector (XYZ)
        float  _cruise_mm_min    = 0.0f;         // requested cruise (pre per-axis clamp)
        bool   _entryUnhomed     = false;        // started from Alarm:Unhomed -> return there on stop

        // Lifecycle guard state (see JogLifecycle.h). Validated every refill tick so motion never
        // resurrects after an external termination (0x85, hold, reset, alarm, completion).
        bool _sawJog            = false;  // have observed State::Jog since this jog started
        int  _pendingStartTicks = 0;      // remaining ticks of the Idle->Jog start handoff window

        // Vector restart queued by a $Jog/Start that arrived during a cancel decel (Phase::Stopping).
        // Re-armed by refill()'s Stopping finalizer from a clean Idle; cancelled on hard termination.
        bool   _pendingVecRestart = false;
        int8_t _pendingVec[3]     = { 0, 0, 0 };
        float  _pendingVecFeed    = 0.0f;

        // Velocity-setpoint trajectory for vector jogs (LinuxCNC-style ramp/blend; see JogIntegrator.h).
        // Carried across direction/feed changes (the integrator blends); reset on every teardown.
        JogIntegrator::State _integ;

        // Live queue health for the |JogQ status field (set by refillVector each pass):
        // queued lead (mm in planner ahead of the executing point) and the committed-lead target.
        float _lastRunwayMm = 0.0f;
        float _lastTargetMm = 0.0f;

        // Shuttle (path-window) state.
        ShuttlePath      _path;
        ShuttlePath::Pos _cmdPos;                 // commanded position along the path
        bool             _shuttleOpen  = false;   // session open (Begin..End)
        int8_t           _shuttleDir   = 0;       // held direction (+1/-1/0=release)
        int8_t           _pendingDir   = 0;       // direction to resume after a reversal stop
        float            _pendingFeed  = 0.0f;
        bool             _reseedOnRun  = false;   // re-project _cmdPos from mpos before resuming

        // True while any homing axis is unknown.
        bool anyAxisUnhomed() const;
        // True when this machine has homing enabled and startup homing is required.
        bool homingRequired() const;
        // True before dropping from Alarm:Unhomed into Idle for an allowed unhomed jog.
        bool canStartUnhomedJog() const;
        // True while the active vector jog is the Alarm:Unhomed carve-out.
        bool unhomedJogExceptionActive() const;
        // True when the unhomed jog feed cap must be applied.
        bool unhomedFeedCapActive() const;
        // Per-axis-clamped cruise (mm/min) for the current vector, incl. the unhomed cap when active.
        float effectiveCruise() const;
        // Limiting acceleration (mm/s^2) across the active axes of the current vector.
        float vectorAccel() const;
        // Compute _dirUnit from _vec; returns false if zero vector.
        bool  computeDirection();

        // refill() is the re-entrancy-guarded shell (mc_linear -> protocol_execute_realtime re-enters);
        // refillImpl() is the real body and must only run through the shell.
        bool _inRefill = false;
        void refillImpl();

        // Vector jog (Route B): advance the per-axis velocity integrator one tick and publish it to
        // the direct-stepper DDA (JogStepper). Handles the held jog and the ramp-to-rest; the GRBL
        // planner is never used. Extents are per-axis on Axis::_softLimits (NOT homing).
        void refillVectorDirect();
        // Reset the velocity-setpoint trajectory so the next jog re-seeds from the machine position.
        void resetIntegrator() { _integ = JogIntegrator::State {}; }
        void refillShuttle();
        // Limiting acceleration over the XY axes (shuttle moves in the XY plane).
        float shuttleAccel() const;

        // Arm the Idle->Jog start-handoff window before issuing a cycle start.
        void beginPendingStart();
        // Flush queued-but-unstarted jog blocks so nothing executes (the quick-tap race).
        void flushQueuedJog();
    };

}
