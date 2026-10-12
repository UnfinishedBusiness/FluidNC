// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

// The PURE pieces of the THC feature: control decision, override resolution, and the
// M104 peck-pulse wait decision. Same inputs always yield the same result — no hardware,
// no FreeRTOS, no logging — so this translation unit compiles on the native test build
// (it is whitelisted in [tests_common] build_src_filter; THC.cpp with its service loop
// and ESP32 timers is not).

#include "THC.h"

#include <cmath>
#include <cstdint>

namespace Machine {

    ThcAction thcDecide(const ThcInputs& in, float pid_p, float min_rate_hz, float max_rate_hz, float& rate_hz_out) {
        rate_hz_out = 0.0f;

        // Gates that suppress all correction (the torch holds its commanded Z):
        //  - disabled (M102, or never M101'd)
        //  - no arc established
        //  - target at/below the configured minimum (the "THC off" setpoint)
        //  - measured voltage below the configured minimum: no valid arc to regulate.
        //    At end-of-cut the arc-OK input typically lingers a few ms after the arc
        //    voltage has already collapsed toward 0; without this gate `error` would be
        //    full-scale and the THC would slam Z at max rate off a non-physical reading.
        //  - arc not yet stable (hasn't traveled the engage distance yet)
        //  - velocity anti-dive active (feed dropped through a corner)
        if (!in.enabled || !in.arc_ok || in.target_volts <= in.min_volts || in.avg_volts < in.min_volts ||
            !in.stabilized || in.antidive) {
            return ThcAction::Hold;
        }

        // error > 0  => measured voltage is low => arc gap too small => raise torch (+Z)
        float error = in.target_volts - in.avg_volts;
        if (std::fabs(error) <= in.threshold_volts) {
            return ThcAction::Hold;  // inside the deadband
        }

        float rate = pid_p * std::fabs(error);
        if (rate < min_rate_hz) {
            rate = min_rate_hz;
        }
        if (rate > max_rate_hz) {
            rate = max_rate_hz;
        }
        rate_hz_out = rate;

        return (error > 0.0f) ? ThcAction::Up : ThcAction::Down;
    }

    bool thcVelocityAntidive(const Stepper::MotionSnapshot& motion, int threshold_pct) {
        if (!motion.valid || !motion.cutting || !std::isfinite(motion.rate) || !std::isfinite(motion.programmed) ||
            motion.rate <= 0.0f || motion.programmed <= 0.0f) {
            return true;
        }
        return threshold_pct > 0 && motion.rate < motion.programmed * (float(threshold_pct) / 100.0f);
    }

    bool ThcVelocityGate::update(const Stepper::MotionSnapshot& motion, int threshold_pct, unsigned avg_samples) {
        if (thcVelocityAntidive(motion, threshold_pct)) {
            _qualified_samples = 0;
            return true;
        }
        const unsigned window = avg_samples < 1 ? 1 : (avg_samples > 16 ? 16 : avg_samples);
        if (_qualified_samples < window) {
            ++_qualified_samples;
        }
        return _qualified_samples < window;
    }

    ThcResolved thcResolve(ThcOvrMode mode, float m103_target, float ovr_volts, bool m101_enabled) {
        switch (mode) {
            case ThcOvrMode::Disabled:
                // Forced off: M101/M102 are no-ops. Report the M103 target so the DRO
                // still shows what the program asked for.
                return { m103_target, false };
            case ThcOvrMode::Override:
                // GcodePilot's volts replace M103; arming still follows M101/M102.
                return { ovr_volts, m101_enabled };
            case ThcOvrMode::Gcode:
            default:
                return { m103_target, m101_enabled };
        }
    }

    PeckWait peckWaitState(float peak_volts, float threshold_volts, uint32_t elapsed_ms, uint32_t timeout_ms) {
        // Crossed is checked FIRST: on the tick where the arc strikes exactly as the
        // timeout expires, the arc is real — declaring a misfire would hand the retry
        // loop a live arc (the blown-hole race this M-code exists to close).
        if (peak_volts >= threshold_volts) {
            return PeckWait::Crossed;
        }
        if (elapsed_ms >= timeout_ms) {
            return PeckWait::TimedOut;
        }
        return PeckWait::Waiting;
    }

}
