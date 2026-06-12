// Tests for the shuttle path-window ring buffer + arc-length walk (Machine/ShuttlePath.h).
// The host streams the program's XY polyline in chunks; the firmware walks a commanded
// position along it by arc length. This validates the pure geometry: vertex storage with
// absolute indices, forward/backward walk across segments, window-edge runway exhaustion,
// path-end clamping, and the (vidx, s_mm) status derivation.

#include "gtest/gtest.h"
#include "Machine/ShuttlePath.h"

using Machine::ShuttlePath;
using JogCommand_ShuttleVertex = Machine::JogCommand::ShuttleVertex;

namespace {
    // L-shaped path: (0,0) -> (10,0) -> (10,10). Two 10mm segments.
    ShuttlePath makeL() {
        ShuttlePath p;
        p.begin(3);
        JogCommand_ShuttleVertex v[3] = { { 0, 0 }, { 10, 0 }, { 10, 10 } };
        EXPECT_TRUE(p.put(0, v, 3));
        return p;
    }
}

TEST(ShuttlePath, StoresAbsoluteIndices) {
    ShuttlePath p = makeL();
    EXPECT_EQ(p.total(), 3);
    EXPECT_EQ(p.firstStored(), 0);
    EXPECT_EQ(p.lastStored(), 2);
    EXPECT_TRUE(p.has(0));
    EXPECT_TRUE(p.has(2));
    EXPECT_FALSE(p.has(3));
    EXPECT_FLOAT_EQ(p.at(1).x, 10.0f);
    EXPECT_FLOAT_EQ(p.segLen(0), 10.0f);
}

TEST(ShuttlePath, ForwardWalkWithinAndAcrossSegments) {
    ShuttlePath p = makeL();
    bool        edge;
    float       x, y;

    auto pos = p.advance({ 0, 0.0f }, 5.0f, +1, edge);  // mid first segment
    EXPECT_FALSE(edge);
    EXPECT_EQ(pos.seg, 0);
    EXPECT_NEAR(pos.frac, 0.5f, 1e-5f);
    p.xy(pos, x, y);
    EXPECT_NEAR(x, 5.0f, 1e-4f);
    EXPECT_NEAR(y, 0.0f, 1e-4f);

    pos = p.advance({ 0, 0.0f }, 12.0f, +1, edge);  // across the corner into segment 1
    EXPECT_FALSE(edge);
    EXPECT_EQ(pos.seg, 1);
    EXPECT_NEAR(pos.frac, 0.2f, 1e-5f);
    p.xy(pos, x, y);
    EXPECT_NEAR(x, 10.0f, 1e-4f);
    EXPECT_NEAR(y, 2.0f, 1e-4f);
}

TEST(ShuttlePath, ForwardClampsAtPathEnd) {
    ShuttlePath p = makeL();
    bool        edge;
    auto        pos = p.advance({ 0, 0.0f }, 1000.0f, +1, edge);
    EXPECT_TRUE(edge);
    EXPECT_EQ(pos.seg, 1);  // last segment
    EXPECT_NEAR(pos.frac, 1.0f, 1e-5f);
    float x, y;
    p.xy(pos, x, y);
    EXPECT_NEAR(x, 10.0f, 1e-4f);
    EXPECT_NEAR(y, 10.0f, 1e-4f);
}

TEST(ShuttlePath, BackwardWalkAcrossCorner) {
    ShuttlePath p = makeL();
    bool        edge;
    auto        pos = p.advance({ 1, 0.5f }, 7.0f, -1, edge);  // from (10,5) back 7mm
    EXPECT_FALSE(edge);
    EXPECT_EQ(pos.seg, 0);
    EXPECT_NEAR(pos.frac, 0.8f, 1e-5f);
    float x, y;
    p.xy(pos, x, y);
    EXPECT_NEAR(x, 8.0f, 1e-4f);
    EXPECT_NEAR(y, 0.0f, 1e-4f);
}

TEST(ShuttlePath, BackwardStopsAtPathStart) {
    ShuttlePath p = makeL();
    bool        edge;
    auto        pos = p.advance({ 0, 0.3f }, 100.0f, -1, edge);
    EXPECT_TRUE(edge);
    EXPECT_EQ(pos.seg, 0);
    EXPECT_NEAR(pos.frac, 0.0f, 1e-5f);
}

TEST(ShuttlePath, ForwardStopsAtLoadedWindowEdge) {
    // Only the first two vertices are loaded; total claims three.
    ShuttlePath              p;
    p.begin(3);
    JogCommand_ShuttleVertex v[2] = { { 0, 0 }, { 10, 0 } };
    ASSERT_TRUE(p.put(0, v, 2));
    bool edge;
    auto pos = p.advance({ 0, 0.0f }, 100.0f, +1, edge);
    EXPECT_TRUE(edge);          // runway exhausted at the loaded edge
    EXPECT_EQ(pos.seg, 1);      // parked on the last loaded vertex (seg start)
    float x, y;
    p.xy(pos, x, y);
    EXPECT_NEAR(x, 10.0f, 1e-4f);
    EXPECT_NEAR(y, 0.0f, 1e-4f);
}

TEST(ShuttlePath, StatusVertexAndArcLength) {
    ShuttlePath p   = makeL();
    bool        edge;
    auto        pos = p.advance({ 0, 0.0f }, 13.0f, +1, edge);  // 3mm into segment 1
    EXPECT_EQ(pos.seg, 1);                                      // vidx = vertex at/behind position
    EXPECT_NEAR(p.arcFromVertex(pos), 3.0f, 1e-4f);            // s_mm from vertex 1
}

TEST(ShuttlePath, ProjectsMachinePositionOntoPath) {
    ShuttlePath p = makeL();
    float       d2;
    // A point near the middle of segment 0 (slightly off the line).
    auto pos = p.project(4.0f, 0.3f, &d2);
    EXPECT_EQ(pos.seg, 0);
    EXPECT_NEAR(pos.frac, 0.4f, 1e-4f);
    EXPECT_NEAR(d2, 0.09f, 1e-4f);  // 0.3mm off -> dist^2 = 0.09
    // A point near the vertical segment.
    pos = p.project(10.2f, 7.0f, &d2);
    EXPECT_EQ(pos.seg, 1);
    EXPECT_NEAR(pos.frac, 0.7f, 1e-4f);
}

TEST(ShuttlePath, AppendChunksAndRejectGaps) {
    ShuttlePath              p;
    p.begin(5);
    JogCommand_ShuttleVertex a[2] = { { 0, 0 }, { 1, 0 } };
    JogCommand_ShuttleVertex b[2] = { { 2, 0 }, { 3, 0 } };
    ASSERT_TRUE(p.put(0, a, 2));
    ASSERT_TRUE(p.put(2, b, 2));  // contiguous append
    EXPECT_EQ(p.lastStored(), 3);
    JogCommand_ShuttleVertex c[1] = { { 9, 0 } };
    EXPECT_FALSE(p.put(4, c, 0));  // empty
    EXPECT_TRUE(p.put(1, a, 2));   // overlapping overwrite is allowed
    // A gap (index beyond one-past-end) is rejected.
    ShuttlePath              q;
    q.begin(100);
    JogCommand_ShuttleVertex d[1] = { { 0, 0 } };
    ASSERT_TRUE(q.put(0, d, 1));
    EXPECT_FALSE(q.put(5, d, 1));  // gap from index 1
}
