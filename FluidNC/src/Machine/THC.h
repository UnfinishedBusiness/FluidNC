// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "Pin.h"

#include <atomic>

class LogStream;

namespace Machine {

    // ---- Pure control decision (no hardware; unit-tested on the native build) ----

    enum class ThcAction {
        Hold = 0,  // do not move Z
        Up,        // raise the torch (+Z): increases the arc gap, raising arc voltage
        Down,      // lower the torch (-Z)
    };

    struct ThcInputs {
        float avg_volts;        // measured arc voltage, moving-averaged
        float target_volts;     // setpoint (from M103 Q)
        float threshold_volts;  // deadband half-width
        float min_volts;        // a target at/below this means THC is off
        bool  enabled;          // M101/M102 run state
        bool  arc_ok;           // arc established (arc-ok input asserted)
        bool  stabilized;       // arc has been ok for longer than the stabilization delay
        bool  antidive;         // velocity anti-dive is suppressing correction
    };

    // Returns the direction to move and, via rate_hz_out, the proportional step rate.
    // Pure: the same inputs always yield the same result, so it is unit-testable
    // without any ESP32 hardware.
    ThcAction thcDecide(const ThcInputs& in, float pid_p, float min_rate_hz, float max_rate_hz, float& rate_hz_out);

    // ---------------------------- The THC feature ----------------------------

    // Torch Height Control.  When a `thc:` section is present in the machine config
    // this object is created; otherwise it stays null and the feature is inert (so
    // existing non-plasma setups are completely unaffected).  The control loop runs
    // on the ESP32 in its own service context; on the native test build only the
    // pure pieces compile.
    class THC : public Configuration::Configurable {
    public:
        THC() = default;
        ~THC();

        void init();
        void group(Configuration::HandlerBase& handler) override;

        // Appends the always-on plasma status field to a `?` status report:
        //   |Arc:<volts>,<arc_ok 0|1>,<active 0|1>
        void status_report(LogStream& msg);

        // ---- GCode-facing API (M101/M102/M103) ----
        void  enable() { _enabled.store(true); }
        void  disable() { _enabled.store(false); }
        void  setTargetVoltage(float volts) { _target_volts.store(volts); }
        float targetVoltage() const { return _target_volts.load(); }

        bool  arcOk() const { return _arc_ok.load(); }
        float arcVoltage() const { return _arc_volts.load(); }
        bool  active() const { return _active.load(); }

    private:
        // ---- Configuration (YAML) ----
        Pin   _arc_voltage_pin;          // analog input carrying (divided) arc voltage
        Pin   _arc_ok_pin;               // digital input, asserted when the arc is established
        float _arc_voltage_scale = 1.0f;   // multiplies the pin voltage (the divider ratio)
        float _arc_voltage_offset = 0.0f;  // added after scaling (volts)
        float _min_arc_voltage   = 0.0f;   // target <= this disables correction
        float _default_target    = 0.0f;   // seed target until M103 sets one
        float _threshold_volts   = 2.0f;   // deadband
        float _pid_p             = 10.0f;  // proportional gain (volts -> step rate)
        int   _vad_threshold_pct = 0;      // velocity anti-dive; 0 disables it
        int   _thc_delay_ms      = 300;    // stabilization delay after arc-ok
        float _max_z_rate_mm_min = 600.0f; // caps the injected Z rate
        int   _avg_samples       = 5;      // moving-average window
        bool  _invert_z          = false;  // set if +Z lowers the torch on this machine

        // ---- Runtime state shared across contexts ----
        std::atomic<bool>  _enabled { false };
        std::atomic<float> _target_volts { 0.0f };
        std::atomic<float> _arc_volts { 0.0f };
        std::atomic<bool>  _arc_ok { false };
        std::atomic<bool>  _active { false };

        // ---- Control loop (ESP32 only) ----
        void service();    // 1 kHz: read voltage, decide direction + rate
        void stepTick();   // high rate: emit paced Z steps

        // moving average ring
        float    _samples[16] = { 0 };
        int      _sample_idx  = 0;
        int      _sample_count = 0;

        bool     _arc_ok_prev   = false;
        int64_t  _arc_ok_since_us = 0;  // when arc-ok last rose

        std::atomic<int>   _step_dir { 0 };       // +1 up, -1 down, 0 hold
        std::atomic<float> _step_rate_hz { 0.0f };
        float    _step_phase = 0.0f;              // phase accumulator for pacing

        float    _max_rate_hz = 0.0f;             // derived from _max_z_rate_mm_min and Z steps/mm

#ifdef __FLUIDNC
        static void serviceTimerCb(void* arg);
        static void stepTimerCb(void* arg);
        void*       _service_timer = nullptr;
        void*       _step_timer    = nullptr;
#endif
    };
}
