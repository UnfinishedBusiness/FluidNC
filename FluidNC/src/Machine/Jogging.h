// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "JogCommand.h"
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

        // Dropped to idle on alarm/reset/disconnect (same as a JogCancel).
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

        // Shuttle (path-window) state.
        ShuttlePath      _path;
        ShuttlePath::Pos _cmdPos;                 // commanded position along the path
        bool             _shuttleOpen  = false;   // session open (Begin..End)
        int8_t           _shuttleDir   = 0;       // held direction (+1/-1/0=release)
        int8_t           _pendingDir   = 0;       // direction to resume after a reversal stop
        float            _pendingFeed  = 0.0f;
        bool             _reseedOnRun  = false;   // re-project _cmdPos from mpos before resuming

        // True while any homing axis is unknown (Alarm:Unhomed jog carve-out active).
        bool anyAxisUnhomed() const;
        // Per-axis-clamped cruise (mm/min) for the current vector, incl. the unhomed cap.
        float effectiveCruise() const;
        // Limiting acceleration (mm/s^2) across the active axes of the current vector.
        float vectorAccel() const;
        // Compute _dirUnit from _vec; returns false if zero vector.
        bool  computeDirection();

        void refillVector(float cruise, float accel, float blockLen, float targetRunway, bool homed);
        void refillShuttle();
        // Limiting acceleration over the XY axes (shuttle moves in the XY plane).
        float shuttleAccel() const;
    };

}
