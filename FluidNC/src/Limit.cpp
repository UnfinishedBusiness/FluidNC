// Copyright (c) 2012 - 2016 Sungeun K. Jeon for Gnea Research LLC
// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Limit.h"

#include "Machine/MachineConfig.h"
#include "MotionControl.h"  // mc_critical
#include "System.h"         // sys.*
#include "Protocol.h"       // protocol_execute_realtime
#include "Platform.h"       // WEAK_LINK
#include "Machine/Axis.h"

#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>  // fence
#include <cmath>   // fabsf

QueueHandle_t limit_sw_queue;  // used by limit switch debouncing

void limits_init() {
#ifdef LATER  // We need to rethink debouncing
    if (Machine::Axes::limitMask) {
        if (limit_sw_queue == NULL && config->_softwareDebounceMs != 0) {
            // setup task used for debouncing
            if (limit_sw_queue == NULL) {
                limit_sw_queue = xQueueCreate(10, sizeof(int));
                xTaskCreate(limitCheckTask,
                            "limitCheckTask",
                            2048,
                            NULL,
                            5,  // priority
                            NULL);
            }
        }
    }
#endif
}

// Returns limit state as a bit-wise uint32 variable. Each bit indicates an axis limit, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is bitnum_to_mask(2), and Y_AXIS is bitnum_to_mask(1).
// The lower 16 bits are used for motor0 and the upper 16 bits are used for motor1 switches
MotorMask limits_get_state() {
    return Machine::Axes::posLimitMask | Machine::Axes::negLimitMask;
}

// Called only from Kinematics canHome() methods, hence from states allowing homing
bool ambiguousLimit() {
    if (Machine::Axes::posLimitMask & Machine::Axes::negLimitMask) {
        mc_critical(ExecAlarm::HomingAmbiguousSwitch);
        return true;
    }
    return false;
}

bool soft_limit = false;

// Performs a soft limit check. Called from mcline() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Used by jogging to limit travel within soft-limit volume.
void limit_error(axis_t axis, float coordinate) {
    log_info("Soft limit on " << Machine::Axes::axisName(axis) << " target:" << coordinate);

    limit_error();
}

void limit_error() {
    soft_limit = true;
    // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
    // workspace volume so just come to a controlled stop so position is not lost. When complete
    // enter alarm mode.
    protocol_buffer_synchronize();
    if (state_is(State::Cycle)) {
        protocol_send_event(&feedHoldEvent);
        do {
            protocol_execute_realtime();
            if (sys.abort()) {
                return;
            }
        } while (!state_is(State::Idle));
    }

    mc_critical(ExecAlarm::SoftLimit);
}

// Soft-limit envelope. `max_travel_mm` is a SIGNED extent: magnitude = travel distance, sign =
// travel direction from the home/boot position. When the axis has a homing block the direction
// comes from the cycle (homing->_positiveDirection) and the magnitude is |max_travel|. WITHOUT a
// homing block (e.g. servos that torque-home at boot, so FluidNC assumes mpos=0 at power-up) the
// SIGN of max_travel is the only direction indicator: positive => the envelope extends to +travel,
// negative => to -travel. (The old code ignored the sign without homing and always used [-|t|, 0],
// which mirrored positive-travel axes — the soft limits then blocked motion into the work area.)
float limitsMaxPosition(axis_t axis) {
    auto axisConfig = Axes::_axis[axis];
    auto homing     = axisConfig->_homing;
    auto mpos       = homing ? homing->_mpos : 0;
    auto travel     = axisConfig->_maxTravel;

    if (homing) {
        float mag = fabsf(travel);
        return homing->_positiveDirection ? mpos : mpos + mag;
    }
    return travel >= 0.0f ? mpos + travel : mpos;
}

float limitsMinPosition(axis_t axis) {
    auto axisConfig = Axes::_axis[axis];
    auto homing     = axisConfig->_homing;
    auto mpos       = homing ? homing->_mpos : 0;
    auto travel     = axisConfig->_maxTravel;

    if (homing) {
        float mag = fabsf(travel);
        return homing->_positiveDirection ? mpos - mag : mpos;
    }
    return travel >= 0.0f ? mpos : mpos + travel;
}
