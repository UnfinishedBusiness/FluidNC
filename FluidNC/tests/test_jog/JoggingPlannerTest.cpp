// Behavior tests for the firmware-native jog engine using a fake planner/motion
// transport. These keep the FWJOG feed decisions testable without booting the
// full firmware stack.

#include "gtest/gtest.h"

#include "EnumItem.h"
extern const EnumItem messageLevels2[];

#include "Machine/Jogging.h"
#include "Machine/MachineConfig.h"
#include "Machine/Axis.h"
#include "Machine/Homing.h"
#include "Kinematics/Kinematics.h"
#include "GCode.h"
#include "MotionControl.h"
#include "Planner.h"
#include "Protocol.h"
#include "System.h"

#include <cstring>
#include <vector>

namespace {
    struct PlannedLine {
        float feed = 0.0f;
        float target[MAX_N_AXIS] = {};
        bool  limits_checked     = false;
    };

    State                    testState = State::Idle;
    float                    testMpos[MAX_N_AXIS] = {};
    uint8_t                  plannerAvailable     = 8;
    bool                     constrainJogCalled   = false;
    bool                     testMotionEnding     = false;  // sys_motion_ending() (suspend bits)
    int                      flushCount           = 0;      // protocol_flush_queued_jog() calls
    std::vector<PlannedLine> plannedLines;

    Channel& fakeChannel() {
        return *reinterpret_cast<Channel*>(0x1);
    }
}

Machine::MachineConfig* config = nullptr;
parser_state_t          gc_state;
volatile ExecAlarm      lastAlarm = ExecAlarm::None;
const EnumItem          messageLevels2[] = { EnumItem(0) };

namespace Machine {
    AxisMask Homing::_unhomed_axes = 0;

    AxisMask Homing::unhomed_axes() {
        return _unhomed_axes;
    }

    void Homing::set_axis_homed(axis_t axis) {
        _unhomed_axes &= ~(AxisMask(1) << axis);
    }

    void Homing::set_axis_unhomed(axis_t axis) {
        _unhomed_axes |= AxisMask(1) << axis;
    }

    void Homing::set_all_axes_homed() {
        _unhomed_axes = 0;
    }

    void Homing::set_all_axes_unhomed() {
        _unhomed_axes = Axes::homingMask;
    }

    AxisMask Axes::homingMask = 0;
    axis_t   Axes::_numberAxis = A_AXIS;
    Axis*    Axes::_axis[MAX_N_AXIS] = {};

    Axes::Axes() {}
    void Axes::group(Configuration::HandlerBase&) {}
    void Axes::afterParse() {}
    Axes::~Axes() {}

    void Axis::group(Configuration::HandlerBase&) {}
    void Axis::afterParse() {}
    Axis::~Axis() {}

    void MachineConfig::group(Configuration::HandlerBase&) {}
    void MachineConfig::afterParse() {}
    MachineConfig::~MachineConfig() {}
}

namespace Kinematics {
    void Kinematics::group(Configuration::HandlerBase&) {}
    void Kinematics::afterParse() {}
    Kinematics::~Kinematics() {}

    void Kinematics::constrain_jog(float*, plan_line_data_t*, float*) {
        constrainJogCalled = true;
    }
}

void set_state(State s) {
    testState = s;
}

bool state_is(State s) {
    return testState == s;
}

float* get_mpos() {
    return testMpos;
}

uint8_t plan_get_block_buffer_available() {
    return plannerAvailable;
}

bool mc_linear(float* target, plan_line_data_t* pl_data, float*) {
    PlannedLine line;
    line.feed           = pl_data->feed_rate;
    line.limits_checked = pl_data->limits_checked;
    for (axis_t axis = X_AXIS; axis < Machine::Axes::_numberAxis; ++axis) {
        line.target[axis] = target[axis];
    }
    plannedLines.push_back(line);
    return true;
}

void protocol_auto_cycle_start() {}
void protocol_do_motion_cancel() {}

bool sys_motion_ending() {
    return testMotionEnding;
}

void protocol_flush_queued_jog() {
    ++flushCount;
    plannedLines.clear();  // model the planner buffer being emptied
}

void send_alarm(ExecAlarm alarm) {
    lastAlarm = alarm;
    set_state(State::Alarm);
}

namespace {
    class JoggingPlannerTest : public ::testing::Test {
    protected:
        Machine::MachineConfig machine;
        Machine::Axes          axes;
        Machine::Start         start;
        ::Kinematics::Kinematics kinematics;
        Machine::Axis          x { X_AXIS };
        Machine::Axis          y { Y_AXIS };
        Machine::Axis          z { Z_AXIS };
        Machine::Jogging       jogging;

        void SetUp() override {
            std::memset(&gc_state, 0, sizeof(gc_state));
            std::memset(testMpos, 0, sizeof(testMpos));
            plannedLines.clear();
            plannerAvailable   = 8;
            constrainJogCalled = false;
            testMotionEnding   = false;
            flushCount         = 0;
            lastAlarm          = ExecAlarm::None;
            testState          = State::Idle;

            config              = &machine;
            machine._axes       = &axes;
            machine._start      = &start;
            machine._kinematics = &kinematics;

            Machine::Axes::_numberAxis       = A_AXIS;
            Machine::Axes::_axis[X_AXIS]     = &x;
            Machine::Axes::_axis[Y_AXIS]     = &y;
            Machine::Axes::_axis[Z_AXIS]     = &z;
            Machine::Axes::homingMask        = 0;

            for (auto* axis : { &x, &y, &z }) {
                axis->_maxRate      = 10000.0f;
                axis->_acceleration = 500.0f;
            }

            start._mustHome                      = true;
            jogging._allow_unhomed               = false;
            jogging._unhomed_feed_cap_mm_min     = 1000;
            Machine::Homing::set_all_axes_homed();
        }

        void setHomingEnabled(bool enabled) {
            Machine::Axes::homingMask = enabled ? AxisMask(0x7) : AxisMask(0);
        }

        void forceAxesUnhomed() {
            Machine::Homing::set_axis_unhomed(X_AXIS);
            Machine::Homing::set_axis_unhomed(Y_AXIS);
            Machine::Homing::set_axis_unhomed(Z_AXIS);
        }

        Error startX(float feed) {
            Machine::JogCommand::Vector v;
            v.axis[0]     = 1;
            v.feed_mm_min = feed;
            return jogging.startVector(v, fakeChannel());
        }

        // Start a vector jog and advance it to a genuinely-running state (cycle start processed).
        void establishJog(float feed) {
            setHomingEnabled(true);
            ASSERT_EQ(startX(feed), Error::Ok);  // queues blocks; state still Idle (start handoff)
            testState = State::Jog;              // cycle start processed -> Jog
            jogging.refill();                    // observes Jog: keeps the runway topped up
            ASSERT_TRUE(jogging.active());
        }
    };

    // ---- Runaway acceptance matrix: every termination path must queue NOTHING more ----

    TEST_F(JoggingPlannerTest, Cancel0x85MidJogDoesNotResurrect) {
        establishJog(6000.0f);
        size_t before    = plannedLines.size();
        testMotionEnding = true;  // 0x85 sets jogCancel while state is still Jog during decel
        jogging.refill();
        EXPECT_EQ(plannedLines.size(), before);  // queued nothing more
        EXPECT_FALSE(jogging.active());          // module terminated
        // Resurrection check: it stays dead on subsequent ticks.
        jogging.refill();
        jogging.refill();
        EXPECT_EQ(plannedLines.size(), before);
        EXPECT_FALSE(jogging.active());
    }

    TEST_F(JoggingPlannerTest, FeedHoldMidJogTerminates) {
        establishJog(6000.0f);
        size_t before = plannedLines.size();
        testState     = State::Hold;
        jogging.refill();
        EXPECT_EQ(plannedLines.size(), before);
        EXPECT_FALSE(jogging.active());
    }

    TEST_F(JoggingPlannerTest, AlarmMidJogTerminates) {
        establishJog(6000.0f);
        size_t before = plannedLines.size();
        testState     = State::Alarm;
        jogging.refill();
        EXPECT_EQ(plannedLines.size(), before);
        EXPECT_FALSE(jogging.active());
    }

    TEST_F(JoggingPlannerTest, SoftResetMidJogTerminatesAndStaysDead) {
        establishJog(6000.0f);
        jogging.onMotionTerminated();  // what protocol_do_rt_reset()'s hook calls
        EXPECT_FALSE(jogging.active());
        size_t before = plannedLines.size();
        testState     = State::Idle;  // reset returns to Idle
        jogging.refill();
        jogging.refill();
        EXPECT_EQ(plannedLines.size(), before);  // no resurrection
        EXPECT_FALSE(jogging.active());
    }

    TEST_F(JoggingPlannerTest, UnlockAfterAlarmedJogDoesNotResume) {
        establishJog(6000.0f);
        testState = State::Alarm;
        jogging.refill();  // alarm terminates the jog
        ASSERT_FALSE(jogging.active());
        size_t before = plannedLines.size();
        testState     = State::Idle;  // $X unlock
        for (int i = 0; i < 5; ++i) {
            jogging.refill();
        }
        EXPECT_EQ(plannedLines.size(), before);  // the old jog must NOT resume
        EXPECT_FALSE(jogging.active());
    }

    TEST_F(JoggingPlannerTest, QuickTapStopFlushesQueuedRunway) {
        setHomingEnabled(true);
        // Tap race: start queues the runway, but the cycle start has NOT been processed (Idle).
        ASSERT_EQ(startX(12000.0f), Error::Ok);
        ASSERT_FALSE(plannedLines.empty());  // a runway was queued
        ASSERT_TRUE(jogging.active());
        // Stop arrives before the state flips to Jog.
        ASSERT_EQ(jogging.stop(fakeChannel()), Error::Ok);
        EXPECT_GE(flushCount, 1);            // queued-but-unstarted blocks were flushed
        EXPECT_TRUE(plannedLines.empty());   // nothing left to execute (~0 distance, not the runway)
        EXPECT_FALSE(jogging.active());
    }

    TEST_F(JoggingPlannerTest, ShuttleSessionThenVectorJogRestoresKind) {
        setHomingEnabled(true);
        // Shuttle a tiny path, which sets _kind = Shuttle.
        ASSERT_EQ(jogging.shuttleBegin(2, fakeChannel()), Error::Ok);
        ASSERT_EQ(jogging.shuttleData("0:0,0;100,0", fakeChannel()), Error::Ok);
        ASSERT_EQ(jogging.shuttleJog(1, 3000.0f, fakeChannel()), Error::Ok);
        ASSERT_EQ(jogging.shuttleEnd(fakeChannel()), Error::Ok);
        testState = State::Idle;
        jogging.refill();
        plannedLines.clear();
        // A vector jog must now dispatch to the VECTOR refill (sticky-kind bug would leave it dead).
        ASSERT_EQ(startX(6000.0f), Error::Ok);
        EXPECT_FALSE(plannedLines.empty());
        EXPECT_FLOAT_EQ(plannedLines.front().feed, 6000.0f);
    }

    TEST_F(JoggingPlannerTest, HomingDisabledDoesNotApplyUnhomedFeedCap) {
        setHomingEnabled(false);
        forceAxesUnhomed();
        jogging._allow_unhomed = true;

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        ASSERT_FALSE(plannedLines.empty());
        EXPECT_FLOAT_EQ(plannedLines.front().feed, 6000.0f);
        EXPECT_NE(plannedLines.front().feed, 1000.0f);
    }

    TEST_F(JoggingPlannerTest, AlarmUnhomedAllowedJogAppliesUnhomedFeedCap) {
        setHomingEnabled(true);
        forceAxesUnhomed();
        jogging._allow_unhomed = true;
        set_state(State::Alarm);
        lastAlarm = ExecAlarm::Unhomed;

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        ASSERT_FALSE(plannedLines.empty());
        EXPECT_FLOAT_EQ(plannedLines.front().feed, 1000.0f);
        EXPECT_TRUE(plannedLines.front().limits_checked);
    }

    TEST_F(JoggingPlannerTest, HomedIdleJogDoesNotApplyUnhomedFeedCap) {
        setHomingEnabled(true);
        jogging._allow_unhomed = true;

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        ASSERT_FALSE(plannedLines.empty());
        EXPECT_FLOAT_EQ(plannedLines.front().feed, 6000.0f);
        EXPECT_NE(plannedLines.front().feed, 1000.0f);
    }

    TEST_F(JoggingPlannerTest, LiveFeedChangeRefillsWithNewFeedForSameVector) {
        setHomingEnabled(true);

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        ASSERT_FALSE(plannedLines.empty());

        for (axis_t axis = X_AXIS; axis < Machine::Axes::_numberAxis; ++axis) {
            testMpos[axis] = gc_state.position[axis];
        }
        plannedLines.clear();

        ASSERT_EQ(jogging.changeFeed(3000.0f, fakeChannel()), Error::Ok);
        ASSERT_FALSE(plannedLines.empty());
        EXPECT_FLOAT_EQ(plannedLines.front().feed, 3000.0f);
    }
}

// Include the firmware source after the fake platform hooks above.
#include "Machine/Jogging.cpp"
