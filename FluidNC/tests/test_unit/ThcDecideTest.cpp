// Unit tests for the pure THC control decision (Machine::thcDecide).
// These exercise the gating + direction + proportional-rate logic without any
// ESP32 hardware, so they run on the native test build.

#include "gtest/gtest.h"
#include "Machine/THC.h"

using Machine::ThcAction;
using Machine::ThcInputs;
using Machine::thcDecide;

namespace {

    // A baseline "actively cutting, voltage low" input that should produce an Up
    // correction; individual tests flip one field to check each gate.
    ThcInputs goodInputs() {
        ThcInputs in;
        in.avg_volts       = 100.0f;  // measured
        in.target_volts    = 120.0f;  // want higher -> raise torch (Up)
        in.threshold_volts = 2.0f;
        in.min_volts       = 30.0f;
        in.enabled         = true;
        in.arc_ok          = true;
        in.stabilized      = true;
        in.antidive        = false;
        return in;
    }

    const float kP       = 10.0f;
    const float kMinRate = 1.0f;
    const float kMaxRate = 5000.0f;

    ThcAction decide(const ThcInputs& in, float& rate) { return thcDecide(in, kP, kMinRate, kMaxRate, rate); }

}

TEST(ThcDecide, RaisesWhenVoltageLow) {
    float     rate = -1.0f;
    ThcAction act  = decide(goodInputs(), rate);
    EXPECT_EQ(act, ThcAction::Up);
    // error = 120-100 = 20V; rate = 10*20 = 200, within [1,5000]
    EXPECT_FLOAT_EQ(rate, 200.0f);
}

TEST(ThcDecide, LowersWhenVoltageHigh) {
    ThcInputs in = goodInputs();
    in.avg_volts = 140.0f;  // above target -> lower torch
    float rate   = 0.0f;
    EXPECT_EQ(decide(in, rate), ThcAction::Down);
    EXPECT_FLOAT_EQ(rate, 200.0f);
}

TEST(ThcDecide, HoldsWithinDeadband) {
    ThcInputs in = goodInputs();
    in.avg_volts = 121.0f;  // 1V error < 2V threshold
    float rate   = 99.0f;
    EXPECT_EQ(decide(in, rate), ThcAction::Hold);
    EXPECT_FLOAT_EQ(rate, 0.0f);
}

TEST(ThcDecide, HoldsWhenDisabled) {
    ThcInputs in = goodInputs();
    in.enabled   = false;
    float rate   = 0.0f;
    EXPECT_EQ(decide(in, rate), ThcAction::Hold);
}

TEST(ThcDecide, HoldsWhenNoArc) {
    ThcInputs in = goodInputs();
    in.arc_ok    = false;
    float rate   = 0.0f;
    EXPECT_EQ(decide(in, rate), ThcAction::Hold);
}

TEST(ThcDecide, HoldsWhenTargetBelowMin) {
    ThcInputs in    = goodInputs();
    in.target_volts = 25.0f;  // <= min_volts (30) means THC off
    float rate      = 0.0f;
    EXPECT_EQ(decide(in, rate), ThcAction::Hold);
}

TEST(ThcDecide, HoldsBeforeStabilized) {
    ThcInputs in  = goodInputs();
    in.stabilized = false;
    float rate    = 0.0f;
    EXPECT_EQ(decide(in, rate), ThcAction::Hold);
}

TEST(ThcDecide, HoldsDuringAntiDive) {
    ThcInputs in = goodInputs();
    in.antidive  = true;
    float rate   = 0.0f;
    EXPECT_EQ(decide(in, rate), ThcAction::Hold);
}

TEST(ThcDecide, RateClampedToMax) {
    ThcInputs in = goodInputs();
    in.avg_volts = 0.0f;  // huge error -> 10*120 = 1200 (still < 5000)
    in.target_volts = 600.0f;  // error 600 -> 6000, clamped to 5000
    float rate   = 0.0f;
    EXPECT_EQ(decide(in, rate), ThcAction::Up);
    EXPECT_FLOAT_EQ(rate, kMaxRate);
}

TEST(ThcDecide, RateFlooredToMin) {
    // Tiny error just past the deadband; pid_p small so proportional rate < min.
    ThcInputs in       = goodInputs();
    in.threshold_volts = 0.5f;
    in.avg_volts       = 119.0f;  // 1V error
    float rate         = 0.0f;
    ThcAction act      = thcDecide(in, 0.1f /*pid_p*/, kMinRate, kMaxRate, rate);
    EXPECT_EQ(act, ThcAction::Up);
    EXPECT_FLOAT_EQ(rate, kMinRate);  // 0.1*1 = 0.1 -> floored to 1
}
