// Command-parser tests (plan test category 7: parsing fuzz -> reject, never crash).

#include "gtest/gtest.h"
#include "Machine/JogCapabilities.h"
#include "Machine/JogCommand.h"

#include <string>

using namespace Machine::JogCommand;

TEST(JogCapabilities, ReportsExactContractLine) {
    struct CapturingChannel {
        MsgLevel    level = MsgLevelVerbose;
        std::string line;

        void sendLine(MsgLevel l, const char* s) {
            level = l;
            line  = s;
        }
    };

    CapturingChannel out;
    Machine::reportFwJogCapabilities(out);
    EXPECT_EQ(out.level, MsgLevelNone);
    EXPECT_EQ(out.line, "[CAP:FWJOG=1,FWSHU=1]");
}

TEST(JogVector, ParsesFullVector) {
    Vector v;
    ASSERT_TRUE(parse_vector("X1 Y0 Z-1 F1000", v));
    EXPECT_EQ(v.axis[0], 1);
    EXPECT_EQ(v.axis[1], 0);
    EXPECT_EQ(v.axis[2], -1);
    EXPECT_FLOAT_EQ(v.feed_mm_min, 1000.0f);
}

TEST(JogVector, OrderAndSubsetIndependent) {
    Vector v;
    ASSERT_TRUE(parse_vector("F500 Z1", v));
    EXPECT_EQ(v.axis[0], 0);
    EXPECT_EQ(v.axis[2], 1);
    EXPECT_FLOAT_EQ(v.feed_mm_min, 500.0f);
}

TEST(JogVector, RejectsBadInputs) {
    Vector v;
    EXPECT_FALSE(parse_vector("X1 Y0 Z-1", v));   // no feed
    EXPECT_FALSE(parse_vector("F1000", v));       // no axis
    EXPECT_FALSE(parse_vector("X0 Y0 Z0 F1000", v)); // all-zero vector
    EXPECT_FALSE(parse_vector("X2 F1000", v));    // axis not -1/0/1
    EXPECT_FALSE(parse_vector("X1 F-5", v));      // non-positive feed
    EXPECT_FALSE(parse_vector("X F1000", v));     // letter w/o number
    EXPECT_FALSE(parse_vector("Q1 F1000", v));    // unknown letter
    EXPECT_FALSE(parse_vector("", v));            // empty
}

TEST(JogFeed, ParsesAndRejects) {
    float f = 0;
    EXPECT_TRUE(parse_feed("2500", f));
    EXPECT_FLOAT_EQ(f, 2500.0f);
    EXPECT_TRUE(parse_feed("750  ", f));
    EXPECT_FALSE(parse_feed("0", f));
    EXPECT_FALSE(parse_feed("-1", f));
    EXPECT_FALSE(parse_feed("12x", f));
    EXPECT_FALSE(parse_feed("", f));
}

TEST(Shuttle, ParsesDirectionAndFeed) {
    int8_t d = 9;
    float  f = 0;
    ASSERT_TRUE(parse_shuttle("1 F3000", d, f));
    EXPECT_EQ(d, 1);
    EXPECT_FLOAT_EQ(f, 3000.0f);

    ASSERT_TRUE(parse_shuttle("-1 F800", d, f));
    EXPECT_EQ(d, -1);

    ASSERT_TRUE(parse_shuttle("0", d, f));  // release needs no feed
    EXPECT_EQ(d, 0);
    EXPECT_FLOAT_EQ(f, 0.0f);
}

TEST(Shuttle, RejectsBad) {
    int8_t d;
    float  f;
    EXPECT_FALSE(parse_shuttle("1", d, f));      // held dir needs feed
    EXPECT_FALSE(parse_shuttle("2 F100", d, f)); // dir not -1/0/1
    EXPECT_FALSE(parse_shuttle("1 X100", d, f)); // missing F
    EXPECT_FALSE(parse_shuttle("1 F0", d, f));   // non-positive feed
    EXPECT_FALSE(parse_shuttle("", d, f));
}

TEST(ShuttleData, ParsesVerticesAtAbsoluteIndex) {
    ShuttleVertex v[8];
    int           first = -1;
    int           n     = parse_shuttle_data("12:1.5,2.0;3.25,-4.5;0,0", first, v, 8);
    ASSERT_EQ(n, 3);
    EXPECT_EQ(first, 12);
    EXPECT_FLOAT_EQ(v[0].x, 1.5f);
    EXPECT_FLOAT_EQ(v[0].y, 2.0f);
    EXPECT_FLOAT_EQ(v[1].x, 3.25f);
    EXPECT_FLOAT_EQ(v[1].y, -4.5f);
    EXPECT_FLOAT_EQ(v[2].x, 0.0f);
}

TEST(ShuttleData, RejectsMalformedAndOverflow) {
    ShuttleVertex v[2];
    int           first;
    EXPECT_EQ(parse_shuttle_data("5 1,2", first, v, 2), -1);     // missing colon
    EXPECT_EQ(parse_shuttle_data("5:1", first, v, 2), -1);       // missing y
    EXPECT_EQ(parse_shuttle_data("5:1,2;3,x", first, v, 2), -1); // bad number
    EXPECT_EQ(parse_shuttle_data("5:1,2;3,4;5,6", first, v, 2), -1); // overflow maxN=2
    EXPECT_EQ(parse_shuttle_data(":1,2", first, v, 2), -1);      // missing index
}
