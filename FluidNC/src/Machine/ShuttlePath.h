// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "JogCommand.h"  // ShuttleVertex
#include <algorithm>
#include <cmath>

namespace Machine {

    // A sliding window of the program's XY polyline. The host streams chunks of vertices; the
    // firmware never stores the whole path. The shuttle commands a position = (segment, frac)
    // along the polyline and walks it by arc length. Ring-buffered by absolute index (a window
    // of <= CAP vertices, so absIdx % CAP never aliases). Pure + unit-tested.
    class ShuttlePath {
    public:
        static constexpr int CAP = 1024;

        struct Pos {
            int   seg  = 0;     // absolute index of the segment's start vertex (segment seg -> seg+1)
            float frac = 0.0f;  // [0,1) along that segment
        };

        void begin(int total) {
            _total = total;
            _first = 0;
            _count = 0;
        }
        void clear() {
            _total = 0;
            _first = 0;
            _count = 0;
        }

        int  total() const { return _total; }
        int  firstStored() const { return _first; }
        int  lastStored() const { return _first + _count - 1; }
        bool empty() const { return _count == 0; }
        bool has(int absIdx) const { return _count > 0 && absIdx >= _first && absIdx <= lastStored(); }

        const JogCommand::ShuttleVertex& at(int absIdx) const { return _v[absIdx % CAP]; }

        // Overwrite n consecutive vertices at absolute index firstIdx. Must be contiguous with the
        // current window (no gaps). Slides the window forward past CAP. Returns false on a gap,
        // out-of-total index, or empty write.
        bool put(int firstIdx, const JogCommand::ShuttleVertex* v, int n) {
            if (n <= 0 || firstIdx < 0 || firstIdx + n > _total) {
                return false;
            }
            int end = _count ? (_first + _count) : firstIdx;  // one past last stored
            if (_count && (firstIdx > end || firstIdx + n < _first)) {
                return false;  // gap from the existing window
            }
            for (int i = 0; i < n; ++i) {
                _v[(firstIdx + i) % CAP] = v[i];
            }
            int newFirst = _count ? std::min(_first, firstIdx) : firstIdx;
            int newEnd   = std::max(_count ? end : firstIdx, firstIdx + n);
            if (newEnd - newFirst > CAP) {
                newFirst = newEnd - CAP;  // slide, dropping the oldest
            }
            _first = newFirst;
            _count = newEnd - newFirst;
            return true;
        }

        float segLen(int seg) const {
            const auto& a = at(seg);
            const auto& b = at(seg + 1);
            return std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
        }

        void xy(const Pos& p, float& x, float& y) const {
            const auto& a = at(p.seg);
            if (p.seg + 1 > lastStored()) {  // at the last stored vertex
                x = a.x;
                y = a.y;
                return;
            }
            const auto& b = at(p.seg + 1);
            x = a.x + (b.x - a.x) * p.frac;
            y = a.y + (b.y - a.y) * p.frac;
        }

        // Arc length from vertex p.seg to the position (for the |Shu: s_mm field).
        float arcFromVertex(const Pos& p) const { return has(p.seg + 1) ? p.frac * segLen(p.seg) : 0.0f; }

        // Closest point on the loaded polyline to (x,y), as a Pos. Used to re-seed the commanded
        // position to where the machine actually stopped (after a release / direction reversal /
        // entry validation). Scans the loaded segments; cheap (one-shot, not per refill).
        Pos project(float x, float y, float* outDist2 = nullptr) const {
            Pos   best;
            best.seg     = _first < 0 ? 0 : _first;
            best.frac    = 0.0f;
            float bestD2 = 1e30f;
            for (int s = _first; s < lastStored(); ++s) {
                const auto& a    = at(s);
                const auto& b    = at(s + 1);
                float       dx   = b.x - a.x;
                float       dy   = b.y - a.y;
                float       len2 = dx * dx + dy * dy;
                float       t    = len2 > 0.0f ? ((x - a.x) * dx + (y - a.y) * dy) / len2 : 0.0f;
                t                = std::max(0.0f, std::min(1.0f, t));
                float px         = a.x + dx * t;
                float py         = a.y + dy * t;
                float d2         = (x - px) * (x - px) + (y - py) * (y - py);
                if (d2 < bestD2) {
                    bestD2    = d2;
                    best.seg  = s;
                    best.frac = t;
                }
            }
            if (outDist2) {
                *outDist2 = bestD2;
            }
            return best;
        }

        // Advance pos by `dist` mm in `dir` (+1 forward / -1 back), walking segments. Stops at the
        // loaded window edge in the travel direction OR the path end (sets atEdge). Requires the
        // vertices it walks over to be stored; runway exhaustion = stop with atEdge=true.
        Pos advance(Pos pos, float dist, int dir, bool& atEdge) const {
            atEdge          = false;
            float remaining = dist;
            const int lastSeg = _total - 2;  // last valid segment index

            while (remaining > 1e-6f) {
                if (dir > 0) {
                    if (pos.seg > lastSeg || !has(pos.seg) || !has(pos.seg + 1)) {
                        atEdge = true;
                        break;
                    }
                    float L    = segLen(pos.seg);
                    float rest = (1.0f - pos.frac) * L;  // distance to the end of this segment
                    if (L <= 0.0f) {                     // degenerate segment, step over it
                        pos.seg += 1;
                        pos.frac = 0.0f;
                        continue;
                    }
                    if (rest > remaining) {
                        pos.frac += remaining / L;
                        remaining = 0.0f;
                    } else {
                        remaining -= rest;
                        pos.seg += 1;
                        pos.frac = 0.0f;
                        if (pos.seg > lastSeg) {  // reached the path end
                            pos.seg  = lastSeg < 0 ? 0 : lastSeg;
                            pos.frac = (lastSeg < 0) ? 0.0f : 1.0f;
                            atEdge   = true;
                            break;
                        }
                    }
                } else {  // dir < 0
                    if (pos.frac <= 0.0f) {
                        if (pos.seg <= 0 || !has(pos.seg - 1)) {
                            atEdge = true;
                            break;
                        }
                        pos.seg -= 1;
                        pos.frac = 1.0f;
                    }
                    float L    = segLen(pos.seg);
                    float rest = pos.frac * L;  // distance back to this segment's start
                    if (L <= 0.0f) {
                        pos.frac = 0.0f;
                        continue;
                    }
                    if (rest > remaining) {
                        pos.frac -= remaining / L;
                        remaining = 0.0f;
                    } else {
                        remaining -= rest;
                        pos.frac = 0.0f;  // landed on this segment's start vertex; loop drops to prev
                    }
                }
            }
            return pos;
        }

    private:
        JogCommand::ShuttleVertex _v[CAP];
        int                       _total = 0;
        int                       _first = 0;
        int                       _count = 0;
    };

}
