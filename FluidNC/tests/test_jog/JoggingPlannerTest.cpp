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

#include <algorithm>
#include <cmath>
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

// Mock the direct-stepper engine. Vector jogs now drive JogStepper (the integer DDA in the step
// ISR) instead of the planner, so there are no plannedLines for a vector jog. Track active state
// and the peak published velocity so tests assert the new contract. enter() mirrors the real one
// (State::Jog + active); the integer DDA itself is covered by JogIntegratorTest (dda_step).
float jogVelMaxMmS = 0.0f;
namespace Machine {
    volatile bool    JogStepper::_active          = false;
    volatile int32_t JogStepper::_inc[MAX_N_AXIS] = { 0 };
    int32_t          JogStepper::_acc[MAX_N_AXIS] = { 0 };

    float JogStepper::maxCruiseMmMin(const float*) { return 1.0e9f; }  // no DDA cap in the host test
    void  JogStepper::isr_compute(AxisMask&, AxisMask&) {}
    void  JogStepper::setVelocity(const float v[3]) {
        float m      = std::max({ std::fabs(v[0]), std::fabs(v[1]), std::fabs(v[2]) });
        jogVelMaxMmS = std::max(jogVelMaxMmS, m);
    }
    void JogStepper::enter() {
        _active = true;
        set_state(State::Jog);
    }
    void JogStepper::exit() { _active = false; }
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
            jogVelMaxMmS       = 0.0f;
            Machine::JogStepper::exit();  // ensure not active between tests
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

        // Start a vector jog. startVector's synchronous refill enters direct-stepper mode
        // (JogStepper mock sets State::Jog + active), so the jog is live on return.
        void establishJog(float feed) {
            setHomingEnabled(true);
            ASSERT_EQ(startX(feed), Error::Ok);
            ASSERT_TRUE(jogging.active());
        }

        // Pump the ~1 kHz refill tick n times (the integrator ramps velocity toward the setpoint;
        // the JogStepper mock records the peak published velocity).
        void pumpRefills(int n) {
            for (int i = 0; i < n && jogging.active(); ++i) {
                jogging.refill();
            }
        }
    };

    // ---- Runaway acceptance matrix: every termination path must queue NOTHING more ----

    // ---- Runaway acceptance matrix: every termination path must stand the engine down ----
    // Vector jogs run the direct-stepper engine; "terminated" means active()==false (JogStepper
    // exited). The DDA step math itself is covered by JogIntegratorTest (dda_step).

    TEST_F(JoggingPlannerTest, Cancel0x85MidJogDoesNotResurrect) {
        establishJog(6000.0f);
        testMotionEnding = true;  // a suspend bit set externally (e.g. a hold) during the jog
        jogging.refill();
        EXPECT_FALSE(jogging.active());  // the safety guard stood the engine down
        jogging.refill();
        jogging.refill();
        EXPECT_FALSE(jogging.active());  // stays dead
    }

    TEST_F(JoggingPlannerTest, FeedHoldMidJogTerminates) {
        establishJog(6000.0f);
        testState = State::Hold;
        jogging.refill();
        EXPECT_FALSE(jogging.active());
    }

    TEST_F(JoggingPlannerTest, AlarmMidJogTerminates) {
        establishJog(6000.0f);
        testState = State::Alarm;
        jogging.refill();
        EXPECT_FALSE(jogging.active());
    }

    TEST_F(JoggingPlannerTest, SoftResetMidJogTerminatesAndStaysDead) {
        establishJog(6000.0f);
        jogging.onMotionTerminated();  // what protocol_do_rt_reset()'s hook calls
        EXPECT_FALSE(jogging.active());
        testState = State::Idle;  // reset returns to Idle
        jogging.refill();
        jogging.refill();
        EXPECT_FALSE(jogging.active());  // no resurrection
    }

    TEST_F(JoggingPlannerTest, UnlockAfterAlarmedJogDoesNotResume) {
        establishJog(6000.0f);
        testState = State::Alarm;
        jogging.refill();  // alarm terminates the jog
        ASSERT_FALSE(jogging.active());
        testState = State::Idle;  // $X unlock
        for (int i = 0; i < 5; ++i) {
            jogging.refill();
        }
        EXPECT_FALSE(jogging.active());  // the old jog must NOT resume
    }

    // A tap: start then stop. The integrator ramps the velocity down to rest and the engine tears
    // down (JogStepper::exit). No planner queue, no flush — the move is whatever the brief ramp
    // covered (bounded by press time; the distance bound is asserted in JogIntegratorTest).
    TEST_F(JoggingPlannerTest, TapRampsDownAndTerminates) {
        setHomingEnabled(true);
        ASSERT_EQ(startX(12000.0f), Error::Ok);
        ASSERT_TRUE(jogging.active());
        ASSERT_EQ(jogging.stop(fakeChannel()), Error::Ok);  // release -> ramp to rest
        pumpRefills(5000);                                  // let the velocity ramp to zero
        EXPECT_FALSE(jogging.active());                     // tore down at rest
        EXPECT_EQ(flushCount, 0);                           // direct stepping never flushes a planner queue
    }

    TEST_F(JoggingPlannerTest, ShuttleSessionThenVectorJogRestoresKind) {
        setHomingEnabled(true);
        // Shuttle a tiny path, which sets _kind = Shuttle (planner-fed path).
        ASSERT_EQ(jogging.shuttleBegin(2, fakeChannel()), Error::Ok);
        ASSERT_EQ(jogging.shuttleData("0:0,0;100,0", fakeChannel()), Error::Ok);
        ASSERT_EQ(jogging.shuttleJog(1, 3000.0f, fakeChannel()), Error::Ok);
        ASSERT_EQ(jogging.shuttleEnd(fakeChannel()), Error::Ok);
        testState = State::Idle;
        jogging.refill();
        // A vector jog must now dispatch to the direct-stepper engine (sticky-kind bug would leave it dead).
        ASSERT_EQ(startX(6000.0f), Error::Ok);
        EXPECT_TRUE(jogging.active());
        pumpRefills(2000);
        EXPECT_NEAR(jogVelMaxMmS, 100.0f, 1.0f);  // ramped to the commanded cruise (6000 mm/min)
    }

    TEST_F(JoggingPlannerTest, HomingDisabledDoesNotApplyUnhomedFeedCap) {
        setHomingEnabled(false);
        forceAxesUnhomed();
        jogging._allow_unhomed = true;

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        pumpRefills(2000);
        EXPECT_NEAR(jogVelMaxMmS, 100.0f, 1.0f);  // no cap: ramps to the full cruise
    }

    TEST_F(JoggingPlannerTest, AlarmUnhomedAllowedJogAppliesUnhomedFeedCap) {
        setHomingEnabled(true);
        forceAxesUnhomed();
        jogging._allow_unhomed = true;
        set_state(State::Alarm);
        lastAlarm = ExecAlarm::Unhomed;

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        pumpRefills(2000);
        EXPECT_NEAR(jogVelMaxMmS, 1000.0f / 60.0f, 1.0f);  // capped to the unhomed feed cap
    }

    TEST_F(JoggingPlannerTest, HomedIdleJogDoesNotApplyUnhomedFeedCap) {
        setHomingEnabled(true);
        jogging._allow_unhomed = true;

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        pumpRefills(2000);
        EXPECT_NEAR(jogVelMaxMmS, 100.0f, 1.0f);
    }

    // A live feed change ramps the published velocity toward the new cruise.
    TEST_F(JoggingPlannerTest, LiveFeedChangeRampsToNewFeed) {
        setHomingEnabled(true);
        ASSERT_EQ(startX(6000.0f), Error::Ok);
        pumpRefills(2000);
        EXPECT_NEAR(jogVelMaxMmS, 100.0f, 1.0f);  // at 6000 mm/min

        jogVelMaxMmS = 0.0f;
        ASSERT_EQ(jogging.changeFeed(12000.0f, fakeChannel()), Error::Ok);
        pumpRefills(2000);
        EXPECT_NEAR(jogVelMaxMmS, 200.0f, 1.0f);  // ramped up to the new 12000 mm/min
        EXPECT_TRUE(jogging.active());
    }

    // A direction change mid-jog must not tear down or flush — it stays active and blends (the
    // smooth blended arc is verified at the math layer in JogIntegratorTest).
    TEST_F(JoggingPlannerTest, DirectionChangeMidJogStaysActiveNoFlush) {
        establishJog(6000.0f);
        pumpRefills(200);
        int flushBefore = flushCount;

        Machine::JogCommand::Vector v;
        v.axis[1]     = 1;  // now +Y (drop X)
        v.feed_mm_min = 6000.0f;
        ASSERT_EQ(jogging.startVector(v, fakeChannel()), Error::Ok);
        EXPECT_EQ(flushCount, flushBefore);  // direction change never flushes
        pumpRefills(400);
        EXPECT_TRUE(jogging.active());
    }
}

// Include the firmware source after the fake platform hooks above.
#include "Machine/Jogging.cpp"
