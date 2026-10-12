// Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
// Copyright (c) 2009-2011 Simen Svale Skogsrud
// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
  Stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
*/

#include "EnumItem.h"

#include <cstdint>

namespace Stepper {
    void init();

    bool pulse_func();

    // Enable steppers, but cycle does not start unless called by motion control or realtime command.
    void wake_up();

    // Stops stepping and disables stepper (not ISR-safe)
    void go_idle();

    // Stops stepping (ISR-safe)
    void stop_stepping();

    // Reset the stepper subsystem variables
    void reset();

    // Changes the run state of the step segment buffer to execute the special parking motion.
    void parking_setup_buffer();

    // Restores the step segment buffer to the normal run state after a parking motion.
    void parking_restore_buffer();

    // Reloads step segment buffer. Called continuously by realtime execution system.
    void prep_buffer();

    // Called by planner_recalculate() when the executing block is updated by the new plan.
    bool update_plan_block_parameters();

    struct MotionSnapshot {
        float rate = 0.0f;        // executing segment's timer-derived path speed, mm/min
        float programmed = 0.0f;  // owning block's programmed feed, mm/min, before overrides
        bool valid = false;
        bool cutting = false;    // ordinary XY feed motion in Cycle, not hold/jog/rapid/system
    };

    // Coherent copy, never a pointer into the recyclable planner/segment rings.
    MotionSnapshot get_executing_motion();

    // Executing path speed for FS/debug reporting; direct-stepper jog uses its own engine.
    float get_realtime_rate();

    extern uint32_t isr_count;
}
