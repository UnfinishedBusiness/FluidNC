// Copyright (c) 2024 - Dylan Knutson
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "UserInputs.h"

#ifdef __FLUIDNC
#    include <Arduino.h>  // analogReadMilliVolts
#endif

namespace Machine {
    UserInputs::UserInputs() {
        for (size_t i = 0; i < MaxUserAnalogPin; i++) {
            _analogScale[i]  = 1.0f;
            _analogOffset[i] = 0.0f;
        }
    }
    UserInputs::~UserInputs() {}

    // clang-format off
    InputPin UserInputs::digitalInput[MaxUserDigitalPin] = {
        InputPin { "digital0_pin" },
        InputPin { "digital1_pin" },
        InputPin { "digital2_pin" },
        InputPin { "digital3_pin" },
        InputPin { "digital4_pin" },
        InputPin { "digital5_pin" },
        InputPin { "digital6_pin" },
        InputPin { "digital7_pin" },
    };
    InputPin UserInputs::analogInput[MaxUserAnalogPin] = {
        InputPin { "analog0_pin" },
        InputPin { "analog1_pin" },
        InputPin { "analog2_pin" },
        InputPin { "analog3_pin" },
    };
    // clang-format on

    void UserInputs::group(Configuration::HandlerBase& handler) {
        for (size_t i = 0; i < MaxUserDigitalPin; i++) {
            auto& pin = digitalInput[i];
            handler.item(pin.legend(), pin);
        }
        for (size_t i = 0; i < MaxUserAnalogPin; i++) {
            auto& pin = analogInput[i];
            handler.item(pin.legend(), pin);
        }
        handler.item("analog0_scale", _analogScale[0]);
        handler.item("analog1_scale", _analogScale[1]);
        handler.item("analog2_scale", _analogScale[2]);
        handler.item("analog3_scale", _analogScale[3]);
        handler.item("analog0_offset", _analogOffset[0]);
        handler.item("analog1_offset", _analogOffset[1]);
        handler.item("analog2_offset", _analogOffset[2]);
        handler.item("analog3_offset", _analogOffset[3]);
    }

    float UserInputs::readAnalog(size_t i) {
        if (i >= MaxUserAnalogPin) {
            return 0.0f;
        }
        auto& pin = analogInput[i];
        if (pin.undefined()) {
            return 0.0f;
        }
        float volts = 0.0f;
#ifdef __FLUIDNC
        pinnum_t gpio = pin.getNative(Pin::Capabilities::ADC);
        volts         = analogReadMilliVolts((uint8_t)gpio) * 0.001f;
#endif
        return volts * _analogScale[i] + _analogOffset[i];
    }

    void UserInputs::init() {
        for (size_t i = 0; i < MaxUserDigitalPin; i++) {
            auto& pin = digitalInput[i];
            if (pin.defined()) {
                pin.init();
            }
        }
        for (size_t i = 0; i < MaxUserAnalogPin; i++) {
            auto& pin = analogInput[i];
            if (pin.defined()) {
                pin.init();
            }
        }
    }

}  // namespace Machine
