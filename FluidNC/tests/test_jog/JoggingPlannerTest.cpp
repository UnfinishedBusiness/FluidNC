// Behavior tests for the firmware-native jog engine using a fake planner/motion
// transport. These keep the FWJOG feed decisions testable without booting the
// full firmware stack.

#include "gtest/gtest.h"

#include "EnumItem.h"
extern const EnumItem messageLevels2[];

#include "Machine/Jogging.h"
#include "Machine/JogStepper.h"  // mocked below; needed for the static-member definitions
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

// Soft-limit fences. Only the stall-watchdog test enables Axis::_softLimits (default false), so for
// every other test these are never consulted. A fence at 0 (max) with a far min lets that test park
// an axis on its fence and command further into it — the integrator then produces no motion.
float limitsMaxPosition(axis_t) {
    return 0.0f;
}
float limitsMinPosition(axis_t) {
    return -1000.0f;
}

// The real planner/suspend system global. Jogging::onMotionTerminated() now hands it back clean
// (clears step_control + the jog-blocking suspend bits) so a subsequent classic $J= jog isn't wedged
// by stale state left from a firmware vector jog. System.cpp isn't in the test src filter, so define
// the global here (system_t is fully inline in System.h).
system_t sys;

// Mock the direct-stepper engine. Vector jogs now drive JogStepper (the integer DDA in the step
// ISR) instead of the planner, so there are no plannedLines for a vector jog. Track active state
// and the peak published velocity — both the overall magnitude and PER AXIS, so tests can assert
// the new per-axis-independent contract (Issue 2). enter() mirrors the real one (State::Jog +
// active); the integer DDA itself is covered by JogIntegratorTest (dda_step). The vector-wide
// maxCruiseMmMin is gone — the DDA step ceiling is now a per-axis clamp inside Jogging.
float jogVelMaxMmS  = 0.0f;
float jogVelPeak[3] = { 0.0f, 0.0f, 0.0f };  // per-axis peak |published velocity| (mm/s)
// Simulate a DEAD step ISR: when true, the mock publishes velocity but emits NO steps (axis_steps[]
// stays frozen) — the exact wedge the anti-wedge watchdog must catch. Default false: a moving axis
// advances axis_steps each tick, mirroring the real DDA, so a healthy jog never trips the watchdog.
bool fakeIsrDead = false;
namespace Machine {
    volatile bool    JogStepper::_active          = false;
    volatile int32_t JogStepper::_inc[MAX_N_AXIS] = { 0 };
    int32_t          JogStepper::_acc[MAX_N_AXIS] = { 0 };

    // axis_steps[] is the machine-position truth, advanced by the real step ISR (Stepping::step) and
    // read by Jogging's anti-wedge watchdog via Stepping::getSteps(). Stepping.cpp isn't compiled into
    // this test, so define the static member here (getSteps is inline in Stepping.h).
    steps_t Stepping::axis_steps[MAX_N_AXIS] = { 0 };

    void JogStepper::isr_compute(AxisMask&, AxisMask&) {}
    void JogStepper::setVelocity(const float v[3]) {
        for (int a = 0; a < 3; ++a) {
            jogVelPeak[a] = std::max(jogVelPeak[a], std::fabs(v[a]));
            // Model the DDA: a commanded axis emits ~one step per tick at the published rate, advancing
            // axis_steps in the travel direction (unless we're simulating a dead ISR).
            if (!fakeIsrDead && std::fabs(v[a]) > 1e-6f) {
                Stepping::setSteps(axis_t(a), Stepping::getSteps(axis_t(a)) + (v[a] > 0.0f ? 1 : -1));
            }
        }
        jogVelMaxMmS = std::max({ jogVelPeak[0], jogVelPeak[1], jogVelPeak[2] });
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
            jogVelPeak[0] = jogVelPeak[1] = jogVelPeak[2] = 0.0f;
            fakeIsrDead        = false;   // healthy step ISR unless a test opts into the wedge
            for (int a = 0; a < MAX_N_AXIS; ++a) Machine::Stepping::setSteps(axis_t(a), 0);
            Machine::JogStepper::exit();  // ensure not active between tests
            lastAlarm          = ExecAlarm::None;
            testState          = State::Idle;
            sys.reset();   // clean planner/suspend globals between tests

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
                // Generous per-axis max_rate (500 mm/s) so the new per-axis velocity clamp does NOT
                // cap the feeds these ramp tests use (up to 200 mm/s); the per-axis clamp itself is
                // asserted explicitly in PerAxisClampIsIndependent. _stepsPerMm defaults to 80, so
                // the per-axis DDA ceiling (JOG_STEP_HZ/80 = 750 mm/s) is also out of the way.
                axis->_maxRate      = 30000.0f;
                axis->_acceleration = 500.0f;
            }

            start._mustHome                      = true;
            jogging._allow_unhomed               = false;
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

        // Start a vector jog. The handler is now NON-BLOCKING (Issue 1): startVector only flips the
        // phase flag and returns Ok — it does NO stepper work. The refill pump's first tick does the
        // seed + JogStepper::enter() (State::Jog + active), exactly as in the firmware. So we pump one
        // refill here to bring the engine fully live before the test proceeds.
        void establishJog(float feed) {
            setHomingEnabled(true);
            ASSERT_EQ(startX(feed), Error::Ok);
            ASSERT_TRUE(jogging.active());               // phase flipped on the ack path
            ASSERT_FALSE(Machine::JogStepper::active());  // ...but NO stepper work in the handler
            jogging.refill();                             // pump's first tick: seed + enter()
            ASSERT_TRUE(Machine::JogStepper::active());    // now live (State::Jog)
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

    // Regression (FIX_CLASSIC_JOG_HANG): the firmware-jog teardown must hand the planner/suspend
    // system back clean, or the next classic $J= planner jog wedges — a stale step_control flag makes
    // prep_buffer() bail, and a stale motionCancel/jogCancel suspend bit makes initiate_cycle refuse
    // the queued block (machine sits Idle until a reset). onMotionTerminated() clears both.
    TEST_F(JoggingPlannerTest, TeardownHandsBackCleanPlannerStateForNextClassicJog) {
        establishJog(6000.0f);   // firmware jog live (phase != Idle)

        // Simulate the stale globals a firmware jog could leave behind.
        sys.step_control.endMotion = true;
        {
            auto s             = sys.suspend();
            s.bit.motionCancel = true;
            s.bit.jogCancel    = true;
            sys.set_suspend(s);
        }

        jogging.onMotionTerminated();   // the centralized teardown every reset/alarm path calls

        EXPECT_FALSE(jogging.active());
        EXPECT_FALSE(sys.step_control.endMotion);        // step_control fully cleared
        EXPECT_FALSE(sys.step_control.executeHold);
        EXPECT_FALSE(sys.suspend().bit.motionCancel);    // jog-blocking suspend bits cleared
        EXPECT_FALSE(sys.suspend().bit.jogCancel);
    }

    TEST_F(JoggingPlannerTest, HomingDisabledUnhomedJogRunsAtFullFeed) {
        setHomingEnabled(false);
        forceAxesUnhomed();
        jogging._allow_unhomed = true;

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        pumpRefills(2000);
        EXPECT_NEAR(jogVelMaxMmS, 100.0f, 1.0f);  // no cap: ramps to the full cruise
    }

    TEST_F(JoggingPlannerTest, AlarmUnhomedAllowedJogRunsAtFullFeed) {
        setHomingEnabled(true);
        forceAxesUnhomed();
        jogging._allow_unhomed = true;
        set_state(State::Alarm);
        lastAlarm = ExecAlarm::Unhomed;

        ASSERT_EQ(startX(6000.0f), Error::Ok);
        pumpRefills(2000);
        EXPECT_NEAR(jogVelMaxMmS, 100.0f, 1.0f);  // no cap: ramps to the full cruise (6000 mm/min)
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

    // ---- Issue 2: per-axis INDEPENDENT velocity ----

    // Adding a second axis must NOT slow the first. X alone reaches its commanded F; with Y added at
    // the same F, X reaches the SAME speed (no cartesian-vector slow-down) and Y independently
    // reaches its own F. The tool-tip speed on the diagonal is the vector sum (~1.41xF) — intended.
    TEST_F(JoggingPlannerTest, AddingSecondAxisDoesNotSlowFirst) {
        setHomingEnabled(true);
        const float feed = 12000.0f;  // 200 mm/s, well under the 500 mm/s axis max_rate

        // X alone.
        Machine::JogCommand::Vector vx;
        vx.axis[0]     = 1;
        vx.feed_mm_min = feed;
        ASSERT_EQ(jogging.startVector(vx, fakeChannel()), Error::Ok);
        pumpRefills(3000);
        EXPECT_NEAR(jogVelPeak[0], 200.0f, 1.0f);  // X reaches its own F
        float xAlone = jogVelPeak[0];

        // Re-aim to X+Y at the SAME feed. X's target/velocity is unchanged by adding Y.
        jogVelPeak[0] = jogVelPeak[1] = 0.0f;
        Machine::JogCommand::Vector vxy;
        vxy.axis[0]     = 1;
        vxy.axis[1]     = 1;
        vxy.feed_mm_min = feed;
        ASSERT_EQ(jogging.startVector(vxy, fakeChannel()), Error::Ok);
        pumpRefills(3000);
        EXPECT_NEAR(jogVelPeak[0], xAlone, 1.0f);  // X NOT slowed by the second axis
        EXPECT_NEAR(jogVelPeak[1], 200.0f, 1.0f);  // Y independently reaches its own F
    }

    // Each axis is clamped to ITS OWN max_rate, not a shared vector cruise: with X fast and Y slow,
    // an X+Y jog at F above Y's max_rate runs X at the full F while Y is capped to its own max_rate.
    TEST_F(JoggingPlannerTest, PerAxisClampIsIndependent) {
        setHomingEnabled(true);
        x._maxRate = 30000.0f;  // 500 mm/s
        y._maxRate = 6000.0f;   // 100 mm/s — below the commanded feed

        Machine::JogCommand::Vector v;
        v.axis[0]     = 1;
        v.axis[1]     = 1;
        v.feed_mm_min = 12000.0f;  // 200 mm/s commanded
        ASSERT_EQ(jogging.startVector(v, fakeChannel()), Error::Ok);
        pumpRefills(3000);
        EXPECT_NEAR(jogVelPeak[0], 200.0f, 1.0f);  // X gets the full commanded F (its max is higher)
        EXPECT_NEAR(jogVelPeak[1], 100.0f, 1.0f);  // Y clamped to ITS OWN max_rate, independently
    }

    // ---- Issue 1: anti-wedge watchdog ----

    // If a held jog commands a real per-axis target yet the integrator produces no motion (here: the
    // axis is parked exactly on its soft-limit fence and commanded further into it, so it can never
    // step), the engine must NOT sit in <Jog> at zero velocity forever — the watchdog force-tears it
    // down to Idle. This makes the "in <Jog>, no motion" wedge impossible by construction.
    TEST_F(JoggingPlannerTest, StallWatchdogForcesIdleWhenNoMotionPossible) {
        setHomingEnabled(true);
        x._softLimits = true;  // fence at 0 (limitsMaxPosition), machine parked there (mpos/gc 0)

        Machine::JogCommand::Vector v;
        v.axis[0]     = 1;  // jog +X, straight into the max fence we're already sitting on
        v.feed_mm_min = 12000.0f;
        ASSERT_EQ(jogging.startVector(v, fakeChannel()), Error::Ok);

        // The integrator can produce no motion (target zeroed at the fence every tick). Pump well past
        // the ~250-tick stall threshold; the watchdog must force teardown to Idle.
        pumpRefills(400);
        EXPECT_FALSE(jogging.active());
        EXPECT_FALSE(Machine::JogStepper::active());
        EXPECT_EQ(testState, State::Idle);
    }

    // The regression for the field bug: a DEAD step ISR. The integrator ramps its velocity to full
    // speed (curSpeed well above EPS) but the steppers emit NO steps (axis_steps[] frozen) — the
    // machine sat in <Jog> with frozen MPos for ~8 s in the field. A velocity-keyed watchdog was
    // blind to this (curSpeed was large); the step-keyed watchdog must force teardown to Idle.
    TEST_F(JoggingPlannerTest, StallWatchdogForcesIdleWhenStepIsrIsDead) {
        fakeIsrDead = true;             // velocity is published every tick, but NO steps are emitted
        establishJog(12000.0f);        // a fast jog: the integrator ramps curSpeed far above EPS
        pumpRefills(50);
        ASSERT_TRUE(jogging.active());  // still ramping; watchdog hasn't reached its threshold yet
        ASSERT_GT(jogVelMaxMmS, 1.0f);  // the integrator genuinely "thinks" it is moving (velocity-blind watchdog would never fire)
        pumpRefills(400);               // past the ~250-tick stall threshold with zero stepped motion
        EXPECT_FALSE(jogging.active());
        EXPECT_FALSE(Machine::JogStepper::active());
        EXPECT_EQ(testState, State::Idle);
    }

    // The complement: a HEALTHY jog (steps advancing each tick) must NEVER trip the watchdog, no
    // matter how long it is held.
    TEST_F(JoggingPlannerTest, StallWatchdogNeverTripsWhileStepping) {
        establishJog(6000.0f);          // fakeIsrDead=false → the mock advances axis_steps each tick
        pumpRefills(2000);              // 8x the stall threshold
        EXPECT_TRUE(jogging.active());
        EXPECT_TRUE(Machine::JogStepper::active());
        EXPECT_EQ(testState, State::Jog);
    }
}

// Include the firmware source after the fake platform hooks above.
#include "Machine/Jogging.cpp"
