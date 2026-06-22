// Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/HandlerBase.h"
#include "Configuration/Configurable.h"

#include <cstdint>

// Selects which of the probe's two physical inputs a single G38 cycle watches. Set transiently by
// mc_probe_cycle from the per-block `G38.n D<sel>` word and always reset to Both at cycle end, so the
// mask never leaks past the one probe move. Both = today's hardcoded OR of both inputs.
enum class ProbeSource : uint8_t {
    Both           = 0,  // probe.pin OR toolsetter_pin (default / D omitted / D0)
    ProbeOnly      = 1,  // probe.pin (floating head) only — D1
    ToolsetterOnly = 2,  // toolsetter_pin (ohmic) only — D2
};

class Probe : public Configuration::Configurable {
    // Inverts the probe pin state depending on user settings and probing cycle mode.
    bool _away = false;

    // Which input(s) get_state() honors for the current cycle. Transient, per-block (see ProbeSource).
    ProbeSource _source = ProbeSource::Both;

    class ProbeEventPin : public EventPin {
    public:
        ProbeEventPin(const char* legend);

        // Differs from the EventPin version by sending the event on either edge
        void trigger(bool active) override {
            InputPin::trigger(active);
            protocol_send_event(_event, this);
        }
    };

    ProbeEventPin _probePin;
    ProbeEventPin _toolsetterPin;

public:
    bool _hard_stop        = false;
    bool _probe_hard_limit = false;
    // Configurable
    bool _check_mode_start = true;
    // _check_mode_start configures the position after a probing cycle
    // during check mode. false sets the position to the probe target,
    // true sets the position to the start position.

    Probe() : _probePin("Probe"), _toolsetterPin("Toolsetter") {}

    // Configurable
    bool exists() { return _probePin.defined() || _toolsetterPin.defined(); }

    void init();

    // setup probing direction G38.2 vs. G38.4
    void set_direction(bool away);

    // Returns probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
    bool get_state();

    // Returns true if the probe pin is tripped, depending on the direction (away or not)
    bool tripped();

    // Select which input(s) get_state() watches for the duration of one probe cycle. Per-block: the
    // probe cycle sets it at start and restores Both on every exit path.
    void setSource(ProbeSource s) { _source = s; }

    // True if the pin backing the given source is configured (for validating a `D<sel>` word).
    bool sourcePinDefined(ProbeSource s) {
        switch (s) {
            case ProbeSource::ProbeOnly:
                return _probePin.defined();
            case ProbeSource::ToolsetterOnly:
                return _toolsetterPin.defined();
            default:  // Both
                return _probePin.defined() || _toolsetterPin.defined();
        }
    }

    ProbeEventPin& probePin() { return _probePin; }
    ProbeEventPin& toolsetterPin() { return _toolsetterPin; }

    // Configuration handlers.
    void validate() override;
    void group(Configuration::HandlerBase& handler) override;

    ~Probe() = default;
};
