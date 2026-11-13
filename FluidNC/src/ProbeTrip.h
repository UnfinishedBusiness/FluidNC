// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

// Pure helpers for the probe trip-source latch (`[PRB:x,y,z:<ok>:<src>]`). No hardware, no
// config — header-only so the native unit tests can exercise them directly.
//
// Letters match the `|Pn:` status field (Report.cpp report_pin_string):
//   P = probe.pin            (floating head on the JetCad plasma configs)
//   T = probe.toolsetter_pin (ohmic ring on the JetCad plasma configs)

#include <cstdint>
#include <string>

enum ProbeTripBits : uint8_t {
    ProbeTripNone       = 0,
    ProbeTripProbe      = 1,  // probe.pin satisfied the trip condition
    ProbeTripToolsetter = 2,  // probe.toolsetter_pin satisfied the trip condition
};

// Which inputs satisfied the trip condition at the moment the probe fired. An input counts only if
// it is configured AND watched by this cycle's `G38.n D<sel>` selector AND its state matches the
// cycle direction (asserted for G38.2/.3 "toward", released for G38.4/.5 "away").
inline uint8_t probeTripMask(bool probeDefined,
                             bool probeWatched,
                             bool probeActive,
                             bool toolsetterDefined,
                             bool toolsetterWatched,
                             bool toolsetterActive,
                             bool away) {
    uint8_t mask = ProbeTripNone;
    if (probeDefined && probeWatched && (probeActive != away)) {
        mask |= ProbeTripProbe;
    }
    if (toolsetterDefined && toolsetterWatched && (toolsetterActive != away)) {
        mask |= ProbeTripToolsetter;
    }
    return mask;
}

// "P", "T", "PT", or "" (nothing latched). Order matches the `|Pn:` field.
inline std::string probeTripLetters(uint8_t mask) {
    std::string s;
    if (mask & ProbeTripProbe) {
        s += 'P';
    }
    if (mask & ProbeTripToolsetter) {
        s += 'T';
    }
    return s;
}

// The optional trailing field appended to `[PRB:x,y,z:<ok>`: ":<letters>" only on a successful probe
// with a latched source, else "" — so a failed probe / no latch keeps the stock 3-field + flag form.
inline std::string probeReportSourceSuffix(bool succeeded, uint8_t mask) {
    if (!succeeded || mask == ProbeTripNone) {
        return std::string();
    }
    return ":" + probeTripLetters(mask);
}
