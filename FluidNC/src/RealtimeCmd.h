#pragma once

#include <cstdint>

// Define realtime command special characters. These characters are 'picked-off' directly from the
// serial read data stream and are not passed to the grbl line execution parser. Select characters
// that do not and must not exist in the streamed GCode program. ASCII control characters may be
// used, if they are available per user setup. Also, extended ASCII codes (>127), which are never in
// GCode programs, maybe selected for interface programs.
// NOTE: If changed, manually update help message in report.c.

// NOTE: All override realtime commands must be in the extended ASCII character set, starting
// at character value 128 (0x80) and up to 255 (0xFF). If the normal set of realtime commands,
// such as status reports, feed hold, reset, and cycle start, are moved to the extended set
// space, serial.c's RX ISR will need to be modified to accommodate the change.

enum class Cmd : uint8_t {
    None                  = 0,
    Reset                 = 0x18,  // Ctrl-X
    StatusReport          = '?',
    CycleStart            = '~',
    FeedHold              = '!',
    SafetyDoor            = 0x84,
    JogCancel             = 0x85,
    DebugReport           = 0x86,  // Only when DEBUG_REPORT_REALTIME enabled, sends debug report in '{}' braces.
    Macro0                = 0x87,
    Macro1                = 0x88,
    Macro2                = 0x89,
    Macro3                = 0x8a,
    FeedOvrReset          = 0x90,  // Restores feed override value to 100%.
    FeedOvrCoarsePlus     = 0x91,
    FeedOvrCoarseMinus    = 0x92,
    FeedOvrFinePlus       = 0x93,
    FeedOvrFineMinus      = 0x94,
    RapidOvrReset         = 0x95,  // Restores rapid override value to 100%.
    RapidOvrMedium        = 0x96,
    RapidOvrLow           = 0x97,
    RapidOvrExtraLow      = 0x98,  // *NOT SUPPORTED*
    SpindleOvrReset       = 0x99,  // Restores spindle override value to 100%.
    SpindleOvrCoarsePlus  = 0x9A,  // 154
    SpindleOvrCoarseMinus = 0x9B,
    SpindleOvrFinePlus    = 0x9C,
    SpindleOvrFineMinus   = 0x9D,
    SpindleOvrStop        = 0x9E,
    CoolantFloodOvrToggle = 0xA0,
    CoolantMistOvrToggle  = 0xA1,
    // Plasma AVTHC override (GcodePilot). All ack-free, safe to send during a cut.
    ThcVoltPlus           = 0xA2,  // override target +1V (seeds from effective target, enters Override)
    ThcVoltMinus          = 0xA3,  // override target -1V
    ThcGcode              = 0xA4,  // mode -> Gcode Controlled (clear override)
    ThcDisable            = 0xA5,  // mode -> Disabled (M101/M102 become no-ops)
    ThcManualUp           = 0xA6,  // manual Z comp up (continuous, while running)
    ThcManualDown         = 0xA7,  // manual Z comp down
    ThcManualStop         = 0xA8,  // stop manual Z comp
    // Channel Extender uses the Bx range; see Channel.h
};

class Channel;

bool is_realtime_command(uint8_t data);
void execute_realtime_command(Cmd command, Channel& channel);
