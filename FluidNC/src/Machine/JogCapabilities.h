// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Logging.h"

namespace Machine {
    // The exact line a sender parses byte-for-byte to feature-detect the firmware jog/shuttle
    // engines. Both brackets are part of the literal.
    inline constexpr const char* FwJogCapabilitiesReport = "[CAP:FWJOG=1,FWSHU=1]";

    // Emit it verbatim. We MUST use sendLine (raw write), NOT log_stream/log_*: those go through
    // LogStream, whose destructor appends a closing ']' to any line starting with '[', which would
    // turn this into "[CAP:FWJOG=1,FWSHU=1]]". That doubled-bracket bug has recurred several times;
    // JogCapabilities.ReportsExactContractLine pins the exact output so it can't come back.
    template <typename ChannelLike>
    inline void reportFwJogCapabilities(ChannelLike& channel) {
        channel.sendLine(MsgLevelNone, FwJogCapabilitiesReport);
    }
}
