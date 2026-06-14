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
    EXPECT_EQ(out.line, "[CAP:FWJOG=1]");
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
