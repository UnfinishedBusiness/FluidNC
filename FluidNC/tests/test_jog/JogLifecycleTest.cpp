// Acceptance gate for the jog-engine runaway fix: the lifecycle decision matrix.
//
// The runaway defect was that refill() kept re-queueing blocks (and re-firing cycle start)
// after motion had been terminated by a path that does not inform the module — realtime
// jog-cancel 0x85, feed-hold, soft reset, alarm, or normal completion. refill() now delegates
// the keep-going-or-stop decision to JogLifecycle::refill_action, and stop() delegates its
// flush decision to stop_action. These tests pin BOTH decisions over the full State space, so
// every termination path maps to "queue nothing" and the tap race maps to "flush".

#include "gtest/gtest.h"
#include "Machine/JogLifecycle.h"

using namespace Machine::JogLifecycle;

namespace {
    // Convenience: the common "healthy held jog" sample (seen Jog, nothing ending, not pending).
    RefillAction whileJogging(State state, bool motionEnding) {
        return refill_action(state, motionEnding, /*sawJog=*/true, /*pendingStart=*/false);
    }
}

// ---- refill_action: the genuinely-jogging cases that MAY queue ----
TEST(JogLifecycle, JogStateWithoutSuspendKeepsQueueing) {
    EXPECT_EQ(refill_action(State::Jog, false, true, false), RefillAction::Queue);
    EXPECT_EQ(refill_action(State::Jog, false, false, true), RefillAction::Queue);  // first tick
}

TEST(JogLifecycle, PendingStartHandoffMayQueueWhileIdle) {
    // After a start we issued, the cycle is queued but state is still Idle for a tick or two.
    EXPECT_EQ(refill_action(State::Idle, false, /*sawJog=*/false, /*pendingStart=*/true), RefillAction::Queue);
}

// ---- refill_action: the acceptance matrix — every termination path queues NOTHING ----
TEST(JogLifecycle, Cancel0x85MidJogTerminates) {
    // 0x85 sets jogCancel while state is still Jog during decel. Must terminate, not refill.
    EXPECT_EQ(whileJogging(State::Jog, /*motionEnding=*/true), RefillAction::Terminate);
}

TEST(JogLifecycle, FeedHoldMidJogTerminates) {
    EXPECT_EQ(whileJogging(State::Hold, true), RefillAction::Terminate);
    EXPECT_EQ(whileJogging(State::Hold, false), RefillAction::Terminate);  // state alone suffices
    EXPECT_EQ(whileJogging(State::Held, false), RefillAction::Terminate);
}

TEST(JogLifecycle, SoftResetMidJogTerminates) {
    // rt reset takes us to Alarm (AbortCycle) or Idle; both terminate.
    EXPECT_EQ(whileJogging(State::Alarm, false), RefillAction::Terminate);
    EXPECT_EQ(whileJogging(State::Idle, false), RefillAction::Terminate);  // sawJog=true => ran then ended
}

TEST(JogLifecycle, AlarmMidJogTerminates) {
    EXPECT_EQ(whileJogging(State::Alarm, false), RefillAction::Terminate);
    EXPECT_EQ(whileJogging(State::Critical, false), RefillAction::Terminate);
    EXPECT_EQ(whileJogging(State::ConfigAlarm, false), RefillAction::Terminate);
}

TEST(JogLifecycle, UnlockAfterAlarmedJogDoesNotResume) {
    // $X unlock after an alarmed jog returns to Idle. With sawJog (the jog had run) and no pending
    // start, Idle must terminate — the jog must NOT silently resume on unlock.
    EXPECT_EQ(refill_action(State::Idle, false, /*sawJog=*/true, /*pendingStart=*/false), RefillAction::Terminate);
    // Even if a stale pendingStart somehow lingered, sawJog forbids re-queueing.
    EXPECT_EQ(refill_action(State::Idle, false, /*sawJog=*/true, /*pendingStart=*/true), RefillAction::Terminate);
}

TEST(JogLifecycle, OtherNonJogStatesTerminate) {
    for (bool ending : { false, true }) {
        EXPECT_EQ(whileJogging(State::Sleep, ending), RefillAction::Terminate);
        EXPECT_EQ(whileJogging(State::Homing, ending), RefillAction::Terminate);
        EXPECT_EQ(whileJogging(State::Cycle, ending), RefillAction::Terminate);  // a program cycle, not our jog
        EXPECT_EQ(whileJogging(State::SafetyDoor, ending), RefillAction::Terminate);
        EXPECT_EQ(whileJogging(State::CheckMode, ending), RefillAction::Terminate);
    }
}

TEST(JogLifecycle, IdleWithNoPendingStartTerminates) {
    // Idle, never started, nothing pending -> not ours.
    EXPECT_EQ(refill_action(State::Idle, false, /*sawJog=*/false, /*pendingStart=*/false), RefillAction::Terminate);
}

TEST(JogLifecycle, ExpiredPendingStartTerminates) {
    // The module drops pendingStart to false once the tick budget expires (a start that never
    // reached Jog) -> Idle then terminates and the module flushes.
    EXPECT_EQ(refill_action(State::Idle, false, false, /*pendingStart=*/false), RefillAction::Terminate);
}

// ---- stop_action: guarantee no queued motion executes, in every state (the tap race) ----
TEST(JogLifecycle, StopWhileJoggingCancelsInflight) {
    EXPECT_EQ(stop_action(State::Jog), StopAction::CancelInflight);
}

TEST(JogLifecycle, StopDuringPendingStartFlushes) {
    // The quick-tap race: $Jog/Start then $Jog/Stop before the cycle start is processed (state
    // still Idle). stop() must flush the queued-but-unstarted blocks so executed distance ~= 0.
    EXPECT_EQ(stop_action(State::Idle), StopAction::FlushQueued);
}

TEST(JogLifecycle, StopInAnyOtherStateFlushes) {
    EXPECT_EQ(stop_action(State::Alarm), StopAction::FlushQueued);
    EXPECT_EQ(stop_action(State::Hold), StopAction::FlushQueued);
    EXPECT_EQ(stop_action(State::Sleep), StopAction::FlushQueued);
}
