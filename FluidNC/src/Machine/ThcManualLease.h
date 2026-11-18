// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <atomic>
#include <cstdint>

namespace Machine {
    // Direction and timestamp share one 32-bit atomic so expiry cannot clear a
    // concurrent new command. Timestamps wrap in the upper 30 bits.
    class ThcManualLease {
    public:
        static constexpr uint32_t TimeoutMs = 400;

        void start(int dir, uint32_t now) {
            _command.store(pack(now, dir > 0 ? 1u : 2u));
        }

        void renew(uint32_t now) {
            uint32_t cmd = _command.load();
            if (cmd && elapsed(now, cmd) < TimeoutMs) {
                _command.compare_exchange_strong(cmd, pack(now, cmd & 3u));
            }
        }

        void stop() { _command.store(0); }

        int direction(uint32_t now, bool eligible) {
            uint32_t cmd = _command.load();
            if (!cmd) {
                return 0;
            }
            if (!eligible || elapsed(now, cmd) >= TimeoutMs) {
                _command.compare_exchange_strong(cmd, 0);
                return 0;
            }
            return (cmd & 3u) == 1u ? 1 : -1;
        }

    private:
        static constexpr uint32_t TimestampMask = 0x3fffffffu;
        static uint32_t pack(uint32_t now, uint32_t dir) { return (now << 2) | dir; }
        static uint32_t elapsed(uint32_t now, uint32_t cmd) { return (now - (cmd >> 2)) & TimestampMask; }

        std::atomic<uint32_t> _command { 0 };
    };
}
