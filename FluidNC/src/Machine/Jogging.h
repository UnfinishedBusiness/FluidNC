// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "JogCommand.h"
#include "JogIntegrator.h"  // velocity-setpoint trajectory state for vector jogs
#include "../Error.h"
#include "../Config.h"  // MAX_N_AXIS

class Channel;
class LogStream;

namespace Machine {

    // Firmware-native vector jogging (LinuxCNC-style): a per-axis velocity integrator drives an
    // integer DDA in the step ISR (JogStepper), bypassing the GRBL planner entirely so a tap makes
    // a small move, corners blend, and held velocity is flat. See docs/jogging.md. Modeled on
    // Machine::THC (a MachineConfig member).
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
        // Realtime 0x85 while the firmware engine is active: same smooth ramp-to-rest as $Jog/Stop
        // (set the velocity setpoint to zero; the integrator decelerates at the accel limit). Runs
        // in the pollLine/protocol task context, so it may mutate module state directly.
        void stopFromRealtime();

        // Append the |JogQ:<speed>,<target> status field while a vector jog runs.
        void status_report(LogStream& msg);

        // Called from protocol_exec_rt_system (main loop, NOT an ISR) to advance the jog trajectory.
        void refill();

        // Belt-and-suspenders teardown: called from every external motion-termination site (soft
        // reset, alarm) so the module exits immediately. Stops the step timer + resyncs position.
        // Idempotent.
        void onMotionTerminated();

        // Full teardown (alarm/reset/disconnect).
        void reset();

        bool active() const { return _phase != Phase::Idle; }

    private:
        enum class Phase { Idle, Jogging, Stopping };

        Phase  _phase            = Phase::Idle;
        int8_t _vec[3]           = { 0, 0, 0 };
        float  _dirUnit[3]       = { 0, 0, 0 };  // normalized jog vector (XYZ)
        float  _cruise_mm_min    = 0.0f;         // requested cruise (pre per-axis clamp)
        bool   _entryUnhomed     = false;        // started from Alarm:Unhomed -> return there on stop

        // Velocity-setpoint trajectory (LinuxCNC-style ramp/blend; see JogIntegrator.h). Carried
        // across direction/feed changes (the integrator blends); reset on every teardown.
        JogIntegrator::State _integ;

        // Live jog status for the |JogQ field (set by refillVectorDirect each tick): current speed
        // and target speed (mm/s).
        float _lastRunwayMm = 0.0f;
        float _lastTargetMm = 0.0f;

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

        // refill() is the re-entrancy-guarded shell; refillImpl() is the real body.
        bool _inRefill = false;
        void refillImpl();

        // Advance the per-axis velocity integrator one tick and publish it to the direct-stepper DDA
        // (JogStepper). Handles the held jog and the ramp-to-rest; the GRBL planner is never used.
        // Extents are per-axis on Axis::_softLimits (NOT homing).
        void refillVectorDirect();
        // Reset the velocity-setpoint trajectory so the next jog re-seeds from the machine position.
        void resetIntegrator() { _integ = JogIntegrator::State {}; }
    };

}
