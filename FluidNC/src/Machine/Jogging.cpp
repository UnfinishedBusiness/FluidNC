// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Jogging.h"

#include "MachineConfig.h"   // config, Axes, Kinematics
#include "../MotionControl.h"  // mc_linear
#include "../Planner.h"        // plan_get_block_buffer_available, plan_line_data_t
#include "../Protocol.h"       // protocol_auto_cycle_start, protocol_do_motion_cancel, lastAlarm
#include "../GCode.h"          // gc_state
#include "../System.h"         // get_mpos, state_is
#include "../Channel.h"
#include "JogMath.h"
#include "Homing.h"

#include <cstring>
#include <cmath>

namespace Machine {

    // Sentinel line number on synthesized jog blocks (kept out of the program line space).
    static constexpr int32_t JOG_LINE = -1;

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
        // then apply the unhomed feed cap when any axis is unhomed.
        float cruise = _cruise_mm_min;
        auto  axes   = config->_axes;
        for (int a = 0; a < 3 && a < Axes::_numberAxis; ++a) {
            float d = std::fabs(_dirUnit[a]);
            if (d > 0.0f && axes->_axis[a]) {
                float axisCap = axes->_axis[a]->_maxRate / d;  // mm/min
                cruise        = std::min(cruise, axisCap);
            }
        }
        if (anyAxisUnhomed() && _unhomed_feed_cap_mm_min > 0) {
            cruise = std::min(cruise, float(_unhomed_feed_cap_mm_min));
        }
        return cruise;
    }

    Error Jogging::startVector(const JogCommand::Vector& v, Channel& out) {
        // Allowed states: Idle, Jog, or the Alarm:Unhomed carve-out.
        bool unhomedCarveout = _allow_unhomed && state_is(State::Alarm) && (lastAlarm == ExecAlarm::Unhomed);
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

        if (_phase == Phase::Idle) {
            _entryUnhomed = unhomedCarveout;
        }
        _phase = Phase::Jogging;
        refill();
        return Error::Ok;
    }

    Error Jogging::changeFeed(float feed_mm_min, Channel& out) {
        if (_phase != Phase::Jogging) {
            return Error::Ok;  // no-op when not jogging (per protocol)
        }
        _cruise_mm_min = feed_mm_min;
        return Error::Ok;
    }

    Error Jogging::stop(Channel& out) {
        if (_phase == Phase::Jogging) {
            _phase = Phase::Stopping;
            if (state_is(State::Jog)) {
                protocol_do_motion_cancel();  // flush queued jog + decel in place (the JogCancel path)
            } else {
                _phase = Phase::Idle;
            }
        }
        return Error::Ok;
    }

    void Jogging::reset() {
        _phase        = Phase::Idle;
        _entryUnhomed = false;
    }

    void Jogging::refill() {
        if (_phase != Phase::Jogging) {
            // When the cancel/decel has finished and we are back to Idle, clear our phase.
            if (_phase == Phase::Stopping && (state_is(State::Idle) || state_is(State::Alarm))) {
                _phase = Phase::Idle;
            }
            return;
        }

        float cruise = effectiveCruise();
        if (cruise <= 0.0f) {
            return;
        }
        float accel        = vectorAccel();
        float blockLen     = JogMath::block_len_mm(cruise);
        float targetRunway = JogMath::target_runway_mm(cruise, accel);
        bool  homed        = !anyAxisUnhomed();

        int queued = 0;
        while (_phase == Phase::Jogging && plan_get_block_buffer_available() > 1) {
            // Runway = projection of (commanded - machine) onto the jog direction.
            float* mpos   = get_mpos();
            float  runway = 0.0f;
            for (int a = 0; a < 3 && a < Axes::_numberAxis; ++a) {
                runway += (gc_state.position[a] - mpos[a]) * _dirUnit[a];
            }
            if (runway >= targetRunway) {
                break;
            }

            // Next target = current commanded position + dir * blockLen.
            float target[MAX_N_AXIS];
            copyAxes(target, gc_state.position);
            for (int a = 0; a < 3 && a < Axes::_numberAxis; ++a) {
                target[a] += _dirUnit[a] * blockLen;
            }

            plan_line_data_t pl;
            memset(&pl, 0, sizeof(pl));
            pl.feed_rate             = cruise;
            pl.motion.noFeedOverride = 1;
            pl.is_jog                = true;
            pl.line_number           = JOG_LINE;

            if (homed) {
                // Clamp to the soft-limit envelope; a zero-length result means we hit the fence.
                config->_kinematics->constrain_jog(target, &pl, gc_state.position);
                float moved2 = 0.0f;
                for (int a = 0; a < 3 && a < Axes::_numberAxis; ++a) {
                    float d = target[a] - gc_state.position[a];
                    moved2 += d * d;
                }
                if (moved2 < 1e-8f) {
                    break;  // parked on the boundary; planner tail-to-zero holds us there
                }
            } else {
                pl.limits_checked = true;  // no envelope when unhomed
            }

            if (!mc_linear(target, &pl, gc_state.position)) {
                break;  // jog cancelled in-flight
            }
            copyAxes(gc_state.position, target);  // advance commanded position (mirrors jog_execute)
            ++queued;
        }

        if (queued) {
            protocol_auto_cycle_start();  // wake the stepper into the Jog state
        }
    }

}
