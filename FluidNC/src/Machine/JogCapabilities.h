// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Logging.h"

#include <cstdio>

namespace Machine {
    // The exact legacy line a sender parses byte-for-byte to feature-detect the firmware vector-jog
    // engine. Both brackets are part of the literal. (Path shuttle was removed; shuttle now runs
    // host-side via $J=, so FWSHU is no longer advertised.)
    inline constexpr const char* FwJogCapabilitiesReport = "[CAP:FWJOG=1]";

    // Emit the capabilities line. We MUST use sendLine (raw write), NOT log_stream/log_*: those go
    // through LogStream, whose destructor appends a closing ']' to any line starting with '[', which
    // would turn this into "[CAP:FWJOG=1]]". That doubled-bracket bug has recurred several times;
    // JogCapabilities.ReportsExactContractLine pins the exact output so it can't come back.
    //
    // `build_info` (pass `git_info`) appends a `VER=<identity>` token — the firmware build identity —
    // so the host can compare the RUNNING firmware against its bundled images and badge "flash
    // needed" (the $Cap query is anyState + retried, unlike the boot welcome, which is missed
    // whenever the port opens after boot). Tokens are ADDITIVE ONLY (k=v, comma-separated); the value
    // is sanitized so it can never carry ',', '[', ']' or spaces, which would break token parsing.
    template <typename ChannelLike>
    inline void reportFwJogCapabilities(ChannelLike& channel, const char* build_info = nullptr) {
        if (build_info == nullptr || *build_info == '\0') {
            channel.sendLine(MsgLevelNone, FwJogCapabilitiesReport);
            return;
        }
        char   ver[64];
        size_t n = 0;
        for (const char* p = build_info; *p != '\0' && n < sizeof(ver) - 1; ++p) {
            char c = *p;
            if (c == ' ' || c == ',' || c == '[' || c == ']') {
                continue;
            }
            ver[n++] = c;
        }
        ver[n] = '\0';
        char line[96];
        snprintf(line, sizeof(line), "[CAP:FWJOG=1,VER=%s]", ver);
        channel.sendLine(MsgLevelNone, line);
    }
}
