// Exercises the production planner, segment preparation and pulse ISR on the host.
// Only hardware/service boundaries are stubbed. No machine connection is possible.
#include "gtest/gtest.h"
#include "EnumItem.h"
extern const EnumItem messageLevels2[];
#include "../../src/Stepper.cpp"
#include "../../src/Planner.cpp"
#include <algorithm>
#include <functional>
#include <thread>
#include <limits>
#include <cstdio>
#include <cstdlib>

Machine::MachineConfig* config = nullptr;
parser_state_t gc_state;
system_t sys;
const EnumItem messageLevels2[] = { EnumItem(0) };
const NoArgEvent cycleStopEvent(nullptr);
static uint32_t periodTicks;
static uint64_t elapsedTicks;
static bool stopped;
static constexpr float stepsPerMm = 40.0f;
static constexpr float cutFeed = 48.0f * 25.4f;
static float axisRateLimit = 6000.0f;

namespace Machine {
#ifdef ENABLE_FW_JOG
    volatile bool JogStepper::_active = false;
    volatile float JogStepper::_rateMmMin = 0;
    volatile uint32_t JogStepper::_isrPeriod = 20000;
    volatile bool JogStepper::_wedgeAbort = false;
    void JogStepper::isr_compute(AxisMask&, AxisMask&) {}
    void JogStepper::enter() {
        _active = true;
        _rateMmMin = 321;
        Stepper::reset();
        sys.set_state(State::Jog);
        Stepper::wake_up();
    }
    void JogStepper::exit() {
        _active = false;
        _rateMmMin = 0;
        Stepper::go_idle();
    }
#endif
    axis_t Axes::_numberAxis = A_AXIS;
    Axis* Axes::_axis[MAX_N_AXIS] = {};
    void Axes::set_disable(bool, bool) {}
    AxisMask Homing::unhomed_axes() { return 0; }
    std::string Axes::maskToNames(AxisMask) { return ""; }
    void MachineConfig::group(Configuration::HandlerBase&) {}
    void MachineConfig::afterParse() {}
    MachineConfig::~MachineConfig() {}
    int32_t Stepping::_segments = 12;
    steps_t Stepping::axis_steps[MAX_N_AXIS] = {};
    void Stepping::step(AxisMask mask, AxisMask dirs) {
        for (axis_t axis = X_AXIS; axis < Axes::_numberAxis; ++axis)
            if (mask & bitnum_to_mask(axis)) axis_steps[axis] += dirs & bitnum_to_mask(axis) ? -1 : 1;
    }
    void Stepping::unstep() {}
    void Stepping::setTimerPeriod(uint32_t ticks) { periodTicks = ticks; }
    void Stepping::startTimer() {}
    void Stepping::reset() {}
}
namespace Spindles {
    uint32_t Spindle::mapSpeed(SpindleState, SpindleSpeed) { return 0; }
    bool Spindle::isRateAdjusted() { return false; }
    void Spindle::init_atc() {}
    void Spindle::afterParse() {}
    bool Spindle::tool_change(uint32_t, bool, bool) { return false; }
}
class TestSpindle : public Spindles::Spindle {
public:
    TestSpindle() : Spindle("test") {}
    void init() override {}
    void setState(SpindleState, uint32_t) override {}
    void config_message() override {}
    void setSpeedfromISR(uint32_t) override {}
    void group(Configuration::HandlerBase&) override {}
};
Spindles::Spindle* spindle = nullptr;
void protocol_disable_steppers() {}
void protocol_cancel_disable_steppers() {}
void protocol_send_event_from_ISR(const Event*, void*) { stopped = true; }
bool state_is(State state) { return sys.state() == state; }
void send_alarm(ExecAlarm) { FAIL() << "Unexpected planner alarm"; }
steps_t motor_pos_to_steps(float pos, size_t) { return lroundf(pos * stepsPerMm); }
float steps_to_motor_pos(steps_t steps, size_t) { return steps / stepsPerMm; }
void get_steps(steps_t* steps) {
    for (axis_t axis = X_AXIS; axis < Machine::Axes::_numberAxis; ++axis) steps[axis] = Machine::Stepping::getSteps(axis);
}
float convert_delta_vector_to_unit_vector(float* vec) {
    float len = 0;
    for (axis_t a = X_AXIS; a < Machine::Axes::_numberAxis; ++a) len += vec[a] * vec[a];
    len = sqrtf(len);
    if (len) for (axis_t a = X_AXIS; a < Machine::Axes::_numberAxis; ++a) vec[a] /= len;
    return len;
}
float limit_acceleration_by_axis_maximum(float*) { return 203.2f * 3600.0f; }
float limit_rate_by_axis_maximum(float*) { return axisRateLimit; }

class StepperMotionTest : public ::testing::Test {
protected:
    Machine::MachineConfig machine;
    TestSpindle testSpindle;
    void SetUp() override {
        axisRateLimit = 6000.0f;
        config = &machine;
        spindle = &testSpindle;
        sys.reset();
        sys.set_state(State::Cycle);
        sys.set_f_override(100);
        sys.set_r_override(100);
        sys.set_spindle_speed_ovr(100);
        Machine::Stepping::_segments = 12;
        for (axis_t a = X_AXIS; a < 3; ++a) Machine::Stepping::setSteps(a, 0);
        Stepper::init();
        Stepper::reset();
        plan_init();
        plan_reset();
        gc_state.feed_rate = cutFeed;
        elapsedTicks = 0;
        periodTicks = 1;
        stopped = false;
    }
    void line(float x, float y, float feed = cutFeed, float z = 0, bool rapid = false) {
        float target[MAX_N_AXIS] = {x, y, z};
        plan_line_data_t data = {};
        data.feed_rate = feed;
        data.motion.rapidMotion = rapid;
        EXPECT_TRUE(plan_buffer_line(target, &data));
    }
    // Independent oracle from the timer programmed into the hardware stub and
    // Bresenham displacement, not prep.current_speed or the new telemetry fields.
    float timerFeed() {
        double lengthSteps = 0;
        for (axis_t a = X_AXIS; a < 3; ++a) {
            const double count = st.exec_block->steps[a];
            lengthSteps += count * count;
        }
        const double stepMm = sqrt(lengthSteps) / st.exec_block->step_event_count / stepsPerMm;
        // AMASS consumes 2^level ISR periods per dominant-axis step.
        const auto index = (segment_buffer_tail + Machine::Stepping::_segments - 1) % Machine::Stepping::_segments;
        const auto* seg = st.exec_segment ? st.exec_segment : &segment_buffer[index];
        return stepMm * Machine::Stepping::fStepperTimer * 60.0 / (periodTicks * (1u << seg->amass_level));
    }
    void run(const std::function<void(float)>& observe) {
        Stepper::prep_buffer();
        Stepper::wake_up();
        unsigned ticks = 0;
        while (!stopped && ++ticks < 2000000) {
            elapsedTicks += periodTicks;
            if (Stepper::pulse_func()) observe(timerFeed());
            Stepper::prep_buffer();
        }
        ASSERT_TRUE(stopped);
    }
};

TEST_F(StepperMotionTest, ReportedFeedFollowsExecutingTimerThroughCorner) {
    line(25, 0);
    line(0, 0); // reversal forces a stop, then acceleration
    double maxError = 0;
    run([&](float actual) { maxError = std::max(maxError, double(fabsf(Stepper::get_realtime_rate() - actual))); });
    EXPECT_LT(maxError, 0.01);
}

TEST_F(StepperMotionTest, FutureAccelerationCannotReleaseCornerAntiDive) {
    line(25, 0);
    line(0, 0);
    int unsafe90 = 0, unsafe99 = 0;
    // Optional bounded diagnostic trace of this real queue run; no firmware logging.
    const char* tracePath = std::getenv("THC_MOTION_TRACE");
    FILE* trace = tracePath ? std::fopen(tracePath, "w") : nullptr;
    ASSERT_TRUE(!tracePath || trace != nullptr);
    if (trace) std::fputs("time_s,x_steps,y_steps,prepared_mm_min,timer_mm_min,reported_mm_min,reference_mm_min,percent,antidive,down\n", trace);
    uint32_t lastPeriod = 0;
    run([&](float actual) {
        const bool traceSample = trace && periodTicks != lastPeriod;
        lastPeriod = periodTicks;
        // Same acquisition and percentage gate as the firmware THC service.
        for (int pct : {90, 99}) {
            Machine::ThcInputs in = {};
            in.avg_volts = 135.3f;
            in.target_volts = 128;
            in.threshold_volts = 3;
            in.min_volts = 30;
            in.enabled = in.arc_ok = in.stabilized = true;
            in.antidive = Machine::thcVelocityAntidive(Stepper::get_executing_motion(), pct);
            float rate;
            auto action = Machine::thcDecide(in, 150, 1, 8000, rate);
            if (traceSample) {
                std::fprintf(trace, "%.6f,%d,%d,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n",
                             double(elapsedTicks) / Machine::Stepping::fStepperTimer,
                             Machine::Stepping::getSteps(X_AXIS), Machine::Stepping::getSteps(Y_AXIS),
                             double(prep.current_speed), double(actual), double(Stepper::get_realtime_rate()),
                             double(Stepper::get_executing_motion().programmed), pct, int(in.antidive),
                             int(action == Machine::ThcAction::Down));
            }
            if (actual < cutFeed * pct / 100.0f && action == Machine::ThcAction::Down)
                ++(pct == 90 ? unsafe90 : unsafe99);
        }
    });
    if (trace) std::fclose(trace);
    EXPECT_EQ(unsafe90, 0);
    EXPECT_EQ(unsafe99, 0);
}

TEST_F(StepperMotionTest, ReferenceBelongsToExecutingBlockNotFutureParserFeed) {
    line(25, 0, cutFeed);
    line(50, 0, 254);
    line(75, 0, 1800);
    gc_state.feed_rate = 0; // parser already consumed a later word, even zero
    int first = 0, second = 0, third = 0;
    run([&](float) {
        const auto motion = Stepper::get_executing_motion();
        ASSERT_TRUE(motion.valid);
        const int x = Machine::Stepping::getSteps(X_AXIS);
        gc_state.feed_rate = (x % 3 == 0) ? 0 : ((x % 3 == 1) ? 254 : 2540);
        // At a geometric endpoint AMASS may still have substeps to complete
        // before activating the next block. Compare strictly inside each move.
        if (x < 999) { EXPECT_FLOAT_EQ(motion.programmed, cutFeed); ++first; }
        else if (x > 1001 && x < 1999) { EXPECT_FLOAT_EQ(motion.programmed, 254); ++second; }
        else if (x > 2001) { EXPECT_FLOAT_EQ(motion.programmed, 1800); ++third; }
    });
    EXPECT_GT(first, 0); EXPECT_GT(second, 0); EXPECT_GT(third, 0);
}

TEST_F(StepperMotionTest, SnapshotSurvivesPlannerDiscardAndFinalPendingPulse) {
    line(0.05f, 0);
    Stepper::prep_buffer();
    ASSERT_EQ(plan_get_current_block(), nullptr); // already discarded before first motor pulse
    EXPECT_FALSE(Stepper::get_executing_motion().valid);
    Stepper::wake_up();
    unsigned ticks = 0;
    while (!stopped && ++ticks < 10000) {
        bool running = Stepper::pulse_func();
        if (running) {
            EXPECT_FLOAT_EQ(Stepper::get_executing_motion().programmed, cutFeed);
            EXPECT_GT(Stepper::get_realtime_rate(), 0);
        }
    }
    ASSERT_TRUE(stopped);
    EXPECT_EQ(Machine::Stepping::getSteps(X_AXIS), 2);
    EXPECT_FALSE(Stepper::get_executing_motion().valid);
    EXPECT_EQ(Stepper::get_realtime_rate(), 0);
}

TEST_F(StepperMotionTest, SegmentCountsShortCornersAndDiagonalTimerAgreement) {
    for (int segments : {6, 12, 20}) {
        SetUp();
        Machine::Stepping::_segments = segments;
        Stepper::init();
        line(25, 0);
        line(25, 25); // right angle
        line(25.025f, 25);
        line(25.025f, 25.025f);
        line(0, 0); // diagonal
        double maxError = 0;
        run([&](float actual) { maxError = std::max(maxError, double(fabsf(Stepper::get_realtime_rate() - actual))); });
        EXPECT_LT(maxError, 0.01) << segments;
    }
}

TEST_F(StepperMotionTest, LowFeedAndPartialQueueReportExecutedTimer) {
    for (float feed : {3.0f, 60.0f}) {
        SetUp();
        line(0.5f, 0, feed);
        int samples = 0;
        run([&](float actual) {
            ++samples;
            EXPECT_NEAR(Stepper::get_realtime_rate(), actual, 0.001);
            EXPECT_FLOAT_EQ(Stepper::get_executing_motion().programmed, feed);
        });
        EXPECT_GT(samples, 0);
    }
}

TEST_F(StepperMotionTest, OverridesAndAxisLimitsDoNotRewriteProgrammedReference) {
    for (int overridePct : {50, 100, 150}) {
        SetUp();
        sys.set_f_override(overridePct);
        line(25, 0);
        float peak = 0;
        int held = 0;
        run([&](float actual) {
            const auto motion = Stepper::get_executing_motion();
            EXPECT_FLOAT_EQ(motion.programmed, cutFeed);
            peak = std::max(peak, actual);
            if (Machine::thcVelocityAntidive(motion, 99)) ++held;
        });
        EXPECT_NEAR(peak, cutFeed * overridePct / 100.0f, 1.0f);
        EXPECT_GT(held, 0);
    }
    SetUp();
    axisRateLimit = 600;
    line(25, 0);
    run([&](float) { EXPECT_TRUE(Machine::thcVelocityAntidive(Stepper::get_executing_motion(), 90)); });
}

TEST_F(StepperMotionTest, HoldResetAndUnderrunNeverLeaveAnActiveReference) {
    line(25, 0);
    Stepper::prep_buffer();
    Stepper::wake_up();
    ASSERT_TRUE(Stepper::pulse_func());
    ASSERT_TRUE(Stepper::get_executing_motion().valid);
    sys.set_state(State::Hold);
    EXPECT_GT(Stepper::get_realtime_rate(), 0); // status still reports deceleration
    EXPECT_TRUE(Machine::thcVelocityAntidive(Stepper::get_executing_motion(), 0));
    sys.set_state(State::Cycle);
    sys.step_control.executeHold = true;
    EXPECT_TRUE(Machine::thcVelocityAntidive(Stepper::get_executing_motion(), 0));
    sys.step_control.executeHold = false;
    // Stop refilling the queue, leaving plenty of unprepared planner motion.
    unsigned ticks = 0;
    while (!stopped && ++ticks < 100000) Stepper::pulse_func();
    ASSERT_TRUE(stopped);
    EXPECT_NE(plan_get_current_block(), nullptr);
    EXPECT_EQ(Stepper::get_realtime_rate(), 0);
    Stepper::prep_buffer(); // refill and restart: no stale reference before activation
    Stepper::wake_up();
    EXPECT_FALSE(Stepper::get_executing_motion().valid);
    EXPECT_TRUE(Stepper::pulse_func());
    EXPECT_TRUE(Stepper::get_executing_motion().valid);
    Stepper::go_idle();
    EXPECT_FALSE(Stepper::get_executing_motion().valid);
    Stepper::reset();
    EXPECT_EQ(Stepper::get_realtime_rate(), 0);
}

TEST_F(StepperMotionTest, RapidZOnlyHomingAndJogAreNotCuttingReferences) {
    line(25, 0, cutFeed, 0, true);
    run([&](float) {
        EXPECT_GT(Stepper::get_realtime_rate(), 0);
        EXPECT_TRUE(Machine::thcVelocityAntidive(Stepper::get_executing_motion(), 0));
    });
    SetUp();
    line(0, 0, cutFeed, 5);
    run([&](float) { EXPECT_FALSE(Stepper::get_executing_motion().cutting); });
    SetUp();
    sys.set_state(State::Homing);
    line(25, 0);
    run([&](float) {
        EXPECT_GT(Stepper::get_realtime_rate(), 0);
        EXPECT_FALSE(Stepper::get_executing_motion().cutting);
    });
#ifdef ENABLE_FW_JOG
    Machine::JogStepper::enter();
    EXPECT_FLOAT_EQ(Stepper::get_realtime_rate(), 321);
    EXPECT_FALSE(Stepper::get_executing_motion().valid);
    Machine::JogStepper::exit();
    EXPECT_EQ(Stepper::get_realtime_rate(), 0);
#endif
}

TEST_F(StepperMotionTest, AtomicSnapshotNeverMixesSpeedAndReference) {
    Stepper::wake_up();
    std::atomic<bool> done {false};
    std::thread writer([&] {
        for (int i = 0; i < 20000; ++i)
            publish_motion(motion_float_bits(float(i)), motion_float_bits(float(i * 2)), MotionValid | MotionCutting);
        done = true;
    });
    int mixed = 0;
    while (!done) {
        const auto m = Stepper::get_executing_motion();
        if (m.valid && m.programmed != m.rate * 2) ++mixed;
    }
    writer.join();
    EXPECT_EQ(mixed, 0);
}

TEST_F(StepperMotionTest, InjectedZStepsDoNotInflateCuttingFeed) {
    line(25, 0);
    run([&](float actual) {
        const float before = Stepper::get_realtime_rate();
        Machine::Stepping::setSteps(Z_AXIS, Machine::Stepping::getSteps(Z_AXIS) - 1);
        EXPECT_FLOAT_EQ(Stepper::get_realtime_rate(), before);
        EXPECT_NEAR(before, actual, 0.001);
    });
}

TEST_F(StepperMotionTest, AlarmAndAbortSuppressOldSnapshot) {
    line(25, 0);
    Stepper::prep_buffer();
    Stepper::wake_up();
    ASSERT_TRUE(Stepper::pulse_func());
    ASSERT_TRUE(Stepper::get_executing_motion().valid);
    sys.set_abort(true);
    EXPECT_FALSE(Stepper::get_executing_motion().valid);
    sys.set_abort(false);
    for (State state : {State::Alarm, State::Idle, State::CheckMode}) {
        sys.set_state(state);
        EXPECT_FALSE(Stepper::get_executing_motion().valid);
        EXPECT_EQ(Stepper::get_realtime_rate(), 0);
    }
}

TEST_F(StepperMotionTest, InverseTimeReferenceIsPlannerNormalizedFeed) {
    float target[MAX_N_AXIS] = {25, 0};
    plan_line_data_t data = {};
    data.feed_rate = 2; // 1/2 minute for 25 mm => 50 mm/min
    data.motion.inverseTime = true;
    ASSERT_TRUE(plan_buffer_line(target, &data));
    run([&](float) { EXPECT_FLOAT_EQ(Stepper::get_executing_motion().programmed, 50); });
}

TEST(ThcVelocity, InvalidReferencesAndDisabledPercentage) {
    Stepper::MotionSnapshot m {20, cutFeed, true, true};
    EXPECT_TRUE(Machine::thcVelocityAntidive(m, 90));
    EXPECT_FALSE(Machine::thcVelocityAntidive(m, 0));
    for (float bad : {0.0f, -1.0f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        m.programmed = bad;
        EXPECT_TRUE(Machine::thcVelocityAntidive(m, 99));
    }
    m.programmed = cutFeed;
    m.rate = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(Machine::thcVelocityAntidive(m, 99));
    EXPECT_TRUE(Machine::thcVelocityAntidive({}, 0));
}

TEST(ThcVelocity, MetricNormalizedInchAndMetricFeedsHaveIdenticalThresholds) {
    // Planner boundary contract: GCode.cpp normalizes G20 F48 to 1219.2 mm/min.
    // This is not a replacement G-code parser or an end-to-end parser test.
    for (float fraction : {0.1f, 0.899f, 0.9f, 0.989f, 0.99f, 1.0f}) {
        Stepper::MotionSnapshot inch {48.0f * 25.4f * fraction, 48.0f * 25.4f, true, true};
        Stepper::MotionSnapshot metric {1219.2f * fraction, 1219.2f, true, true};
        EXPECT_EQ(Machine::thcVelocityAntidive(inch, 90), Machine::thcVelocityAntidive(metric, 90));
        EXPECT_EQ(Machine::thcVelocityAntidive(inch, 99), Machine::thcVelocityAntidive(metric, 99));
    }
}

TEST(ThcVelocity, RecoveryDoesNotActOnCornerVoltageRemainingInAverage) {
    Machine::ThcVelocityGate gate;
    Stepper::MotionSnapshot m {cutFeed, cutFeed, true, true};
    float samples[16];
    std::fill_n(samples, 16, 136.0f); // corner voltage stored in the existing average
    m.rate = 0.5f * cutFeed;
    EXPECT_TRUE(gate.update(m, 99, 16));
    m.rate = cutFeed;
    for (unsigned i = 0; i < 16; ++i) {
        samples[i] = 128; // voltage physically settles at recovery
        float avg = 0;
        for (float v : samples) avg += v / 16;
        Machine::ThcInputs in {avg, 128, 3, 30, true, true, true, gate.update(m, 99, 16)};
        float rate;
        EXPECT_EQ(Machine::thcDecide(in, 150, 1, 8000, rate), Machine::ThcAction::Hold);
        EXPECT_EQ(in.antidive, i < 15);
    }
    // A genuine persistent height error is allowed after fresh qualified samples.
    Machine::ThcInputs realError {136, 128, 3, 30, true, true, true, gate.update(m, 99, 16)};
    float rate;
    EXPECT_EQ(Machine::thcDecide(realError, 150, 1, 8000, rate), Machine::ThcAction::Down);
    m.rate = 0.5f * cutFeed;
    EXPECT_TRUE(gate.update(m, 99, 16)); // immediate re-lock
    m.rate = cutFeed;
    EXPECT_TRUE(gate.update(m, 99, 16)); // must earn a fresh window again
}

TEST(ThcVelocity, OneSampleWindowHasNoExtraDelay) {
    Machine::ThcVelocityGate gate;
    EXPECT_TRUE(gate.update({}, 99, 1));
    EXPECT_FALSE(gate.update({cutFeed, cutFeed, true, true}, 99, 1));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
