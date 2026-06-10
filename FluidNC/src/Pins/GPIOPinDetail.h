// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "PinDetail.h"
#include "Driver/PwmPin.h"

namespace Pins {
    class GPIOPinDetail : public PinDetail {
        PinCapabilities _capabilities;
        PinAttributes   _attributes;

        static PinCapabilities GetDefaultCapabilities(pinnum_t index);

        static std::vector<bool> _claimed;

        // True for a ":shared" declaration: a second (or later) use of a GPIO that is
        // already owned by a non-shared declaration.  Shared pins do not claim the GPIO
        // and do not (re)configure its hardware mode; they only add an event handler.
        bool _shared = false;

        bool _lastWrittenValue = false;

        PwmPin* _pwm;

        int8_t _driveStrength = -1;

        void setDriveStrength(uint8_t n, PinAttributes attr);

    public:
        GPIOPinDetail(pinnum_t index, PinOptionsParser options);

        PinCapabilities capabilities() const override;

        // I/O:
        void          write(bool high) override;
        bool          read() override;
        void          setAttr(PinAttributes value, uint32_t frequency) override;
        PinAttributes getAttr() const override;

        void     setDuty(uint32_t duty) override;
        uint32_t maxDuty() override { return _pwm->period(); };

        int8_t driveStrength() override { return _driveStrength; }

        bool canStep() override { return true; }

        void registerEvent(InputPin* obj) override;

        // A shared declaration never claimed the GPIO, so it must not release the
        // owner's claim when it is destroyed.
        ~GPIOPinDetail() override {
            if (!_shared) {
                _claimed[_index] = false;
            }
        }
    };
}
