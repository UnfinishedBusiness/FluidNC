#include "RealtimeCmd.h"
#include "Config.h"
#include "Channel.h"
#include "Protocol.h"
#include "Report.h"
#include "System.h"
#include "State.h"                     // State, state_is
#include "Machine/Macros.h"           // macroNEvent
#include "Machine/MachineConfig.h"    // config->_thc
#ifdef ENABLE_FW_JOG
#    include "Machine/MachineConfig.h"  // config->_jogging
#    include "Machine/Jogging.h"
#endif

// Act upon a realtime character
void execute_realtime_command(Cmd command, Channel& channel) {
    switch (command) {
        case Cmd::Reset:
            protocol_send_event(&rtResetEvent);
            break;
        case Cmd::StatusReport:
            report_realtime_status(channel);  // direct call instead of setting flag
            // protocol_send_event(&reportStatusEvent, int(&channel));
            break;
        case Cmd::CycleStart:
            protocol_send_event(&cycleStartEvent);
            break;
        case Cmd::FeedHold:
            protocol_send_event(&feedHoldEvent);
            break;
        case Cmd::SafetyDoor:
            protocol_send_event(&safetyDoorEvent);
            break;
        case Cmd::JogCancel:
#ifdef ENABLE_FW_JOG
            // A firmware-engine jog stops via stopFromRealtime(): identical to $Jog/Stop's live
            // branch — set _phase=Stopping so the refill engine stops queuing, then jogCancel
            // decel-in-place + flush (overshoot = v^2/2a, the physical minimum). Do NOT send the
            // stock motionCancelEvent here: that cancels the in-flight motion WITHOUT telling the
            // refill engine to stand down, so it keeps queuing blocks against the cancel — that
            // (not the jogCancel decel itself) was the bench channel-dead runaway, since fully
            // fixed by also registering $Jog/* as AsyncUserCommand. Legacy $J= jogs (module not
            // active) keep the stock event.
            if (config && config->_jogging && config->_jogging->active()) {
                config->_jogging->stopFromRealtime();
                break;
            }
#endif
            if (state_is(State::Jog)) {  // Block all other states from invoking motion cancel.
                protocol_send_event(&motionCancelEvent);
            }
            break;
        case Cmd::DebugReport:
            protocol_send_event(&debugEvent);
            break;
        case Cmd::SpindleOvrStop:
            protocol_send_event(&accessoryOverrideEvent, AccessoryOverride::SpindleStopOvr);
            break;
        case Cmd::FeedOvrReset:
            protocol_send_event(&feedOverrideEvent, FeedOverride::Default);
            break;
        case Cmd::FeedOvrCoarsePlus:
            protocol_send_event(&feedOverrideEvent, FeedOverride::CoarseIncrement);
            break;
        case Cmd::FeedOvrCoarseMinus:
            protocol_send_event(&feedOverrideEvent, -FeedOverride::CoarseIncrement);
            break;
        case Cmd::FeedOvrFinePlus:
            protocol_send_event(&feedOverrideEvent, FeedOverride::FineIncrement);
            break;
        case Cmd::FeedOvrFineMinus:
            protocol_send_event(&feedOverrideEvent, -FeedOverride::FineIncrement);
            break;
        case Cmd::RapidOvrReset:
            protocol_send_event(&rapidOverrideEvent, RapidOverride::Default);
            break;
        case Cmd::RapidOvrMedium:
            protocol_send_event(&rapidOverrideEvent, RapidOverride::Medium);
            break;
        case Cmd::RapidOvrLow:
            protocol_send_event(&rapidOverrideEvent, RapidOverride::Low);
            break;
        case Cmd::RapidOvrExtraLow:
            protocol_send_event(&rapidOverrideEvent, RapidOverride::ExtraLow);
            break;
        case Cmd::SpindleOvrReset:
            protocol_send_event(&spindleOverrideEvent, SpindleSpeedOverride::Default);
            break;
        case Cmd::SpindleOvrCoarsePlus:
            protocol_send_event(&spindleOverrideEvent, SpindleSpeedOverride::CoarseIncrement);
            break;
        case Cmd::SpindleOvrCoarseMinus:
            protocol_send_event(&spindleOverrideEvent, -SpindleSpeedOverride::CoarseIncrement);
            break;
        case Cmd::SpindleOvrFinePlus:
            protocol_send_event(&spindleOverrideEvent, SpindleSpeedOverride::FineIncrement);
            break;
        case Cmd::SpindleOvrFineMinus:
            protocol_send_event(&spindleOverrideEvent, -SpindleSpeedOverride::FineIncrement);
            break;
        case Cmd::CoolantFloodOvrToggle:
            protocol_send_event(&accessoryOverrideEvent, AccessoryOverride::FloodToggle);
            break;
        case Cmd::CoolantMistOvrToggle:
            protocol_send_event(&accessoryOverrideEvent, AccessoryOverride::MistToggle);
            break;
        case Cmd::Macro0:
            protocol_send_event(&macro0Event);
            break;
        case Cmd::Macro1:
            protocol_send_event(&macro1Event);
            break;
        case Cmd::Macro2:
            protocol_send_event(&macro2Event);
            break;
        case Cmd::Macro3:
            protocol_send_event(&macro3Event);
            break;
        // ---- Plasma AVTHC override (atomic stores; safe to call from the channel task) ----
        case Cmd::ThcVoltPlus:
            if (config && config->_thc) {
                config->_thc->ovrNudge(+1);
            }
            break;
        case Cmd::ThcVoltMinus:
            if (config && config->_thc) {
                config->_thc->ovrNudge(-1);
            }
            break;
        case Cmd::ThcGcode:
            if (config && config->_thc) {
                config->_thc->ovrSetGcode();
            }
            break;
        case Cmd::ThcDisable:
            if (config && config->_thc) {
                config->_thc->ovrSetDisabled();
            }
            break;
        case Cmd::ThcManualUp:
            // Manual comp only moves Z while a program is running, so a stray byte can't drift Z when idle.
            if (config && config->_thc && state_is(State::Cycle)) {
                config->_thc->manualCompUp();
            }
            break;
        case Cmd::ThcManualDown:
            if (config && config->_thc && state_is(State::Cycle)) {
                config->_thc->manualCompDown();
            }
            break;
        case Cmd::ThcManualStop:
            if (config && config->_thc) {
                config->_thc->manualCompStop();
            }
            break;
        default:  // None
            break;
    }
}

// checks to see if a character is a realtime character
bool is_realtime_command(uint8_t data) {
    if (data >= 0x80) {
        return true;
    }
    auto cmd = static_cast<Cmd>(data);
    return cmd == Cmd::Reset || cmd == Cmd::StatusReport || cmd == Cmd::CycleStart || cmd == Cmd::FeedHold;
}
