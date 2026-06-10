// Copyright (c) 2021 -  Stefan de Bruijn
// Copyright (c) 2021 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "GCode.h"  // MaxUserDigitalPin MaxUserAnalogPin

namespace Machine {
    class UserOutputs : public Configuration::Configurable {
        uint32_t _current_value[MaxUserAnalogPin];

        // Last commanded state, kept so it can be surfaced in the status report.
        bool  _digitalState[MaxUserDigitalPin] = { false };
        float _analogPercent[MaxUserAnalogPin] = { 0 };

    public:
        UserOutputs();

        Pin     _analogOutput[MaxUserAnalogPin];
        int32_t _analogFrequency[MaxUserAnalogPin];
        Pin     _digitalOutput[MaxUserDigitalPin];

        void init();
        void all_off();

        void group(Configuration::HandlerBase& handler) override;
        bool setDigital(size_t io_num, bool isOn);
        bool setAnalogPercent(size_t io_num, float percent);

        // Live state accessors for status reporting.
        bool  getDigital(size_t io_num) const { return _digitalState[io_num]; }
        float getAnalogPercent(size_t io_num) const { return _analogPercent[io_num]; }

        virtual ~UserOutputs();
    };
}
