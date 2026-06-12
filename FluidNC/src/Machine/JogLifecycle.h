// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../State.h"  // enum class State (standalone, no heavy deps)

// Pure lifecycle decisions for the firmware jog engine — the guard that prevents the queue
// from being re-filled after motion has been terminated by ANY path (realtime jog-cancel 0x85,
// feed-hold, soft reset, alarm, normal completion), and the decision for how a stop must clear
// queued motion. Header-only and dependency-free (only the lightweight State enum) so it is
// shared by the firmware module AND unit-tested exhaustively on the x86 host.
//
// Background: the self-refilling jog engine keeps blocks queued while it believes it is jogging.
// Every motion-termination path in FluidNC flushes the planner WITHOUT informing the module, so
// the module must validate the live machine state on every refill tick and stop queueing the
// instant the motion is no longer genuinely ours — otherwise it resurrects motion after a panic.
namespace Machine {
    namespace JogLifecycle {

        enum class RefillAction {
            Queue,      // the machine is genuinely executing our jog — may top up the queue
            Terminate,  // motion ended/was cancelled by some path — drop to Idle, queue NOTHING
        };

        // Decide whether refill() may keep queueing while the module is in Phase::Jogging.
        //
        //  state         : sys.state()
        //  motionEnding  : any suspend bit set (jogCancel / motionCancel / hold / door / ...).
        //                  A 0x85 cancel sets jogCancel while state is STILL Jog during decel, so
        //                  this must be checked FIRST — otherwise we would refill against the cancel.
        //  sawJog        : we have observed State::Jog since this jog started (the cycle began).
        //  pendingStart  : we issued a cycle start and are within the brief Idle->Jog handoff window
        //                  (bounded by a tick budget in the module so a start that never takes expires).
        inline RefillAction refill_action(State state, bool motionEnding, bool sawJog, bool pendingStart) {
            if (motionEnding) {
                return RefillAction::Terminate;  // a cancel/hold/door is in progress — stop now
            }
            if (state == State::Jog) {
                return RefillAction::Queue;  // genuinely jogging
            }
            if (state == State::Idle && pendingStart && !sawJog) {
                return RefillAction::Queue;  // our own start has not reached Jog yet
            }
            // Idle after the jog ran (sawJog), Idle with no pending start, Alarm, Hold, Held, Sleep,
            // Homing, Cycle, SafetyDoor, Critical, ConfigAlarm, Starting, CheckMode -> not ours.
            return RefillAction::Terminate;
        }

        enum class StopAction {
            CancelInflight,  // motion is running (State::Jog) — use the JogCancel decel/flush path
            FlushQueued,     // motion not yet running — flush queued-but-unstarted blocks so NOTHING executes
        };

        // Decide how stop() must guarantee no queued jog motion executes after it returns. When the
        // machine has already entered State::Jog the queued motion is live, so cancel it through the
        // normal JogCancel path. In every other state (notably Idle during the start handoff — the
        // "quick-tap" race) the blocks are queued but not yet executing, so they must be flushed
        // directly or they would lurch the full queued runway.
        inline StopAction stop_action(State state) {
            return state == State::Jog ? StopAction::CancelInflight : StopAction::FlushQueued;
        }

    }  // namespace JogLifecycle
}  // namespace Machine
