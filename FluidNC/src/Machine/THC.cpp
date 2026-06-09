// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "THC.h"

#include "MachineConfig.h"  // config, Axes, Stepping
#include "../Stepper.h"     // Stepper::get_realtime_rate
#include "../GCode.h"       // gc_state
#include "../Logging.h"     // LogStream, log_*

#include <cmath>

#ifdef __FLUIDNC
#    include <Arduino.h>    // analogReadMilliVolts
#    include <esp_timer.h>  // esp_timer_*
#endif

namespace Machine {

    // The high-rate timer that paces Z steps.  8 kHz (125 us) matches the original
    // Grbl_Esp32 plasma implementation and is the ceiling on the injected step rate.
    static const int   StepTimerHz = 8000;
    static const float MinRateHz   = 1.0f;

    // ----------------------------- pure decision -----------------------------

    ThcAction thcDecide(const ThcInputs& in, float pid_p, float min_rate_hz, float max_rate_hz, float& rate_hz_out) {
        rate_hz_out = 0.0f;

        // Gates that suppress all correction (the torch holds its commanded Z):
        //  - disabled (M102, or never M101'd)
        //  - no arc established
        //  - target at/below the configured minimum (the "THC off" setpoint)
        //  - arc not yet stable (still within the post-pierce delay)
        //  - velocity anti-dive active (feed dropped through a corner)
        if (!in.enabled || !in.arc_ok || in.target_volts <= in.min_volts || !in.stabilized || in.antidive) {
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

    // ------------------------------- lifecycle -------------------------------

    THC::~THC() {}

    void THC::group(Configuration::HandlerBase& handler) {
        handler.item("arc_voltage_pin", _arc_voltage_pin);
        handler.item("arc_ok_pin", _arc_ok_pin);
        handler.item("arc_voltage_scale", _arc_voltage_scale);
        handler.item("arc_voltage_offset", _arc_voltage_offset);
        handler.item("min_arc_voltage", _min_arc_voltage);
        handler.item("target_voltage", _default_target);
        handler.item("threshold_volts", _threshold_volts);
        handler.item("pid_p", _pid_p);
        handler.item("vad_threshold_pct", _vad_threshold_pct, 0, 100);
        handler.item("thc_delay_ms", _thc_delay_ms, 0, 10000);
        handler.item("max_z_rate_mm_min", _max_z_rate_mm_min);
        handler.item("avg_samples", _avg_samples, 1, 16);
        handler.item("invert_z", _invert_z);
    }

    void THC::init() {
        if (_arc_voltage_pin.undefined()) {
            log_warn("THC: arc_voltage_pin not configured; THC inactive");
            return;
        }
        if (!Stepping::thcCanStep(Z_AXIS)) {
            log_error("THC: Z must use the 'timed' step engine with a motor assigned; THC inactive");
            return;
        }

        // The arc-voltage pin is read with the ADC; getNative validates ADC capability.
        // It is intentionally not setAttr'd as a digital input so the ADC driver owns it.
        (void)_arc_voltage_pin.getNative(Pin::Capabilities::ADC);
        if (_arc_ok_pin.defined()) {
            _arc_ok_pin.setAttr(Pin::Attr::Input);
        }

        float zStepsPerMm = (config->_axes && config->_axes->_axis[Z_AXIS]) ? config->_axes->_axis[Z_AXIS]->_stepsPerMm : 80.0f;
        _max_rate_hz      = (_max_z_rate_mm_min / 60.0f) * zStepsPerMm;
        if (_max_rate_hz > (float)StepTimerHz) {
            _max_rate_hz = (float)StepTimerHz;
        }
        _target_volts.store(_default_target);

        log_info("THC: arc_voltage_pin:" << _arc_voltage_pin.name() << " arc_ok_pin:" << _arc_ok_pin.name()
                                         << " max Z rate:" << _max_z_rate_mm_min << "mm/min");

#ifdef __FLUIDNC
        esp_timer_create_args_t serviceArgs = {};
        serviceArgs.callback                = &THC::serviceTimerCb;
        serviceArgs.arg                     = this;
        serviceArgs.name                    = "thc_service";
        esp_timer_handle_t serviceHandle;
        if (esp_timer_create(&serviceArgs, &serviceHandle) == ESP_OK) {
            esp_timer_start_periodic(serviceHandle, 1000);  // 1 kHz
            _service_timer = serviceHandle;
        }

        esp_timer_create_args_t stepArgs = {};
        stepArgs.callback                = &THC::stepTimerCb;
        stepArgs.arg                     = this;
        stepArgs.name                    = "thc_step";
        esp_timer_handle_t stepHandle;
        if (esp_timer_create(&stepArgs, &stepHandle) == ESP_OK) {
            esp_timer_start_periodic(stepHandle, 1000000 / StepTimerHz);
            _step_timer = stepHandle;
        }
#endif
    }

    // ------------------------------ control loop ------------------------------

    void THC::service() {
        float v = 0.0f;
#ifdef __FLUIDNC
        pinnum_t gpio = _arc_voltage_pin.getNative(Pin::Capabilities::ADC);
        uint32_t mv   = analogReadMilliVolts((uint8_t)gpio);
        v             = (mv * 0.001f) * _arc_voltage_scale + _arc_voltage_offset;
        if (v < 0.0f) {
            v = 0.0f;
        }
#endif

        // moving average
        _samples[_sample_idx] = v;
        _sample_idx           = (_sample_idx + 1) % _avg_samples;
        if (_sample_count < _avg_samples) {
            _sample_count++;
        }
        float sum = 0.0f;
        for (int i = 0; i < _sample_count; i++) {
            sum += _samples[i];
        }
        float avg = _sample_count ? (sum / _sample_count) : 0.0f;
        _arc_volts.store(avg);

        bool ok = _arc_ok_pin.defined() ? _arc_ok_pin.read() : false;
        _arc_ok.store(ok);

        int64_t now = 0;
#ifdef __FLUIDNC
        now = esp_timer_get_time();
#endif
        if (ok && !_arc_ok_prev) {
            _arc_ok_since_us = now;
        }
        _arc_ok_prev    = ok;
        bool stabilized = ok && (now - _arc_ok_since_us) >= (int64_t)_thc_delay_ms * 1000;

        // velocity anti-dive: suppress while the feed has dropped through a corner
        bool antidive = false;
        if (_vad_threshold_pct > 0) {
            float programmed = gc_state.feed_rate;
            float realtime   = Stepper::get_realtime_rate();
            if (programmed > 0.0f && realtime < programmed * (_vad_threshold_pct / 100.0f)) {
                antidive = true;
            }
        }

        ThcInputs in;
        in.avg_volts       = avg;
        in.target_volts    = _target_volts.load();
        in.threshold_volts = _threshold_volts;
        in.min_volts       = _min_arc_voltage;
        in.enabled         = _enabled.load();
        in.arc_ok          = ok;
        in.stabilized      = stabilized;
        in.antidive        = antidive;

        float     rate = 0.0f;
        ThcAction act  = thcDecide(in, _pid_p, MinRateHz, _max_rate_hz, rate);

        int dir = (act == ThcAction::Up) ? 1 : (act == ThcAction::Down ? -1 : 0);
        if (_invert_z) {
            dir = -dir;
        }
        _step_dir.store(dir);
        _step_rate_hz.store(rate);
        _active.store(act != ThcAction::Hold);
    }

    void THC::stepTick() {
        int dir = _step_dir.load();
        if (dir == 0) {
            return;
        }
        float rate = _step_rate_hz.load();
        if (rate <= 0.0f) {
            return;
        }
        _step_phase += rate / (float)StepTimerHz;
        if (_step_phase >= 1.0f) {
            _step_phase -= 1.0f;
            if (_step_phase > 1.0f) {
                _step_phase = 0.0f;  // rate exceeded the timer; never queue a backlog
            }
            Stepping::thcStep(Z_AXIS, dir > 0);
        }
    }

    // ------------------------------- reporting -------------------------------

    void THC::status_report(LogStream& msg) {
        msg << "|Arc:" << setprecision(1) << _arc_volts.load() << "," << (_arc_ok.load() ? 1 : 0) << ","
            << (_active.load() ? 1 : 0);
    }

#ifdef __FLUIDNC
    void THC::serviceTimerCb(void* arg) {
        static_cast<THC*>(arg)->service();
    }
    void THC::stepTimerCb(void* arg) {
        static_cast<THC*>(arg)->stepTick();
    }
#endif
}
