// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>
#include <cstdlib>
#include <cctype>

// Pure parsers for the firmware-jogging `$Jog/*` and `$Shu/*` command arguments
// (see plans/firmware-jogging/protocol.md). Header-only + dependency-free so they are
// shared by the firmware module and unit-tested on the x86 host. All feeds are mm/min,
// all coordinates mm (the wire units), independent of G20/G21.
namespace Machine {
    namespace JogCommand {

        struct Vector {
            int8_t axis[3]     = { 0, 0, 0 };  // X,Y,Z, each -1 / 0 / +1
            float  feed_mm_min = 0.0f;
            bool   anyAxis() const { return axis[0] || axis[1] || axis[2]; }
        };

        // Parse "X1 Y0 Z-1 F1000" (any order/subset of X/Y/Z, F required). Returns false on
        // a malformed token, an axis value other than -1/0/+1, a missing/non-positive feed,
        // or no nonzero axis. The argument is the text AFTER `$Jog/Start=`.
        inline bool parse_vector(const char* s, Vector& out) {
            out          = Vector {};
            bool haveF   = false;
            const char* p = s;
            while (*p) {
                while (*p == ' ' || *p == '\t') {
                    ++p;
                }
                if (!*p) {
                    break;
                }
                char  letter = static_cast<char>(toupper(static_cast<unsigned char>(*p++)));
                char* end    = nullptr;
                float val    = strtof(p, &end);
                if (end == p) {
                    return false;  // letter not followed by a number
                }
                p = end;
                switch (letter) {
                    case 'X':
                    case 'Y':
                    case 'Z': {
                        if (val != -1.0f && val != 0.0f && val != 1.0f) {
                            return false;
                        }
                        out.axis[letter - 'X'] = static_cast<int8_t>(val);
                        break;
                    }
                    case 'F':
                        if (val <= 0.0f) {
                            return false;
                        }
                        out.feed_mm_min = val;
                        haveF           = true;
                        break;
                    default:
                        return false;
                }
            }
            return haveF && out.anyAxis();
        }

        // Parse a bare positive feed "<mm/min>" for `$Jog/Feed=` / `$Shu/Jog` cruise.
        inline bool parse_feed(const char* s, float& feed) {
            char* end = nullptr;
            float v   = strtof(s, &end);
            if (end == s || v <= 0.0f) {
                return false;
            }
            while (*end == ' ' || *end == '\t') {
                ++end;
            }
            if (*end) {
                return false;  // trailing garbage
            }
            feed = v;
            return true;
        }

        // Parse a shuttle direction command "<+1|-1|0> F<mm/min>" for `$Shu/Jog=`.
        inline bool parse_shuttle(const char* s, int8_t& dir, float& feed) {
            char* end = nullptr;
            float d   = strtof(s, &end);
            if (end == s || (d != -1.0f && d != 0.0f && d != 1.0f)) {
                return false;
            }
            dir       = static_cast<int8_t>(d);
            const char* p = end;
            while (*p == ' ' || *p == '\t') {
                ++p;
            }
            // dir 0 (release) needs no feed; a held direction requires F.
            if (dir == 0) {
                feed = 0.0f;
                return *p == '\0';
            }
            if (toupper(static_cast<unsigned char>(*p)) != 'F') {
                return false;
            }
            return parse_feed(p + 1, feed);
        }

    }  // namespace JogCommand
}  // namespace Machine
