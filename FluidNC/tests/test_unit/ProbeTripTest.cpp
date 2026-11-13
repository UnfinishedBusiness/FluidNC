// Unit tests for the probe trip-source latch formatting (`[PRB:x,y,z:<ok>:<src>]`, ProbeTrip.h).

#include "gtest/gtest.h"
#include "ProbeTrip.h"

TEST(ProbeTrip, LettersMatchPnField) {
    EXPECT_EQ(probeTripLetters(ProbeTripNone), "");
    EXPECT_EQ(probeTripLetters(ProbeTripProbe), "P");
    EXPECT_EQ(probeTripLetters(ProbeTripToolsetter), "T");
    EXPECT_EQ(probeTripLetters(ProbeTripProbe | ProbeTripToolsetter), "PT");
}

TEST(ProbeTrip, TowardCountsAssertedInputs) {
    // G38.2/.3 D0: ohmic (toolsetter) asserted, floating head (probe pin) not.
    EXPECT_EQ(probeTripMask(true, true, false, true, true, true, false), ProbeTripToolsetter);
    // Both asserted -> ambiguous "PT".
    EXPECT_EQ(probeTripMask(true, true, true, true, true, true, false), ProbeTripProbe | ProbeTripToolsetter);
}

TEST(ProbeTrip, SelectorMasksUnwatchedInput) {
    // G38.3 D1 (probe pin only) with a stuck-ON ohmic: report only the floating head.
    EXPECT_EQ(probeTripMask(true, true, true, true, false, true, false), ProbeTripProbe);
}

TEST(ProbeTrip, UndefinedPinNeverReported) {
    // Away probe: an unconfigured pin reads inactive, which must not count as "released".
    EXPECT_EQ(probeTripMask(true, true, false, false, true, false, true), ProbeTripProbe);
}

TEST(ProbeTrip, AwayCountsReleasedInputs) {
    EXPECT_EQ(probeTripMask(true, true, false, true, true, true, true), ProbeTripProbe);
}

TEST(ProbeTrip, ReportSuffixIsBackwardCompatible) {
    // Success with a latched source -> 4th field.
    EXPECT_EQ(probeReportSourceSuffix(true, ProbeTripToolsetter), ":T");
    EXPECT_EQ(probeReportSourceSuffix(true, ProbeTripProbe | ProbeTripToolsetter), ":PT");
    // Failure or nothing latched -> stock `[PRB:x,y,z:0]` / `[PRB:x,y,z:1]` form.
    EXPECT_EQ(probeReportSourceSuffix(false, ProbeTripProbe), "");
    EXPECT_EQ(probeReportSourceSuffix(true, ProbeTripNone), "");

    // Full line as a host sees it, split on ':' like JetCad3's controller.js.
    std::string line  = std::string("PRB:1.000,2.000,-12.345:1") + probeReportSourceSuffix(true, ProbeTripToolsetter);
    size_t      first = line.find(':');
    size_t      second = line.find(':', first + 1);
    size_t      third  = line.find(':', second + 1);
    EXPECT_EQ(line.substr(second + 1, 1), "1");        // success flag still field [2]
    EXPECT_EQ(line.substr(third + 1), "T");            // source is field [3]
    EXPECT_EQ(line.substr(first + 1, second - first - 1), "1.000,2.000,-12.345");
}
