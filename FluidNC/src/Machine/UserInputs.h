// Copyright (c) 2024 - Dylan Knutson
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "GCode.h"
#include "InputPin.h"

namespace Machine {

    class UserInputs : public Configuration::Configurable {
        // Per-analog-input scaling: readAnalog() returns (pin_volts * scale + offset),
        // so the value reported by M66 E / #5399 / |UIO: is in real engineering units.
        float _analogScale[MaxUserAnalogPin];
        float _analogOffset[MaxUserAnalogPin];

    public:
        UserInputs();
        virtual ~UserInputs();

        static InputPin digitalInput[];
        // Read as analog via readAnalog(); the InputPin itself only does a digital read.
        static InputPin analogInput[];

        void init();
        void group(Configuration::HandlerBase& handler) override;

        // Reads analog input `i`, scaled to engineering units. Returns 0 on the
        // native (non-hardware) build. Out-of-range or undefined pins return 0.
        float readAnalog(size_t i);
    };

}  // namespace Machine
