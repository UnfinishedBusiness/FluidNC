# THC executing-motion regression and validation

The planner and preparation queue run ahead of motor execution. Stepper::get_executing_motion()
copies the active segment's final timer-derived path rate and its owning block's normalized
programmed feed (mm/min, before feed override). Each segment retains those values after its
planner block is discarded. The same speed supplies FS status and THC engage-distance integration;
THC-injected Z pulses do not increase it.

## Publication and lifecycle

- Segment preparation computes speed using final isrPeriod, AMASS scaling, and steps/mm.
- The pulse ISR emits the preceding pulse before publishing the newly activated segment.
  Do not invalidate on segment retirement: its last computed pulse is still pending.
- Publication/read uses a short ESP32 critical section containing only three integer words;
  floats are carried as bit patterns so publication does not invoke the FPU in the ISR.
  The task reader masks interrupts while owning the same mux; no sequence-lock retry loop
  can strand an ISR waiting for the task it interrupted.
- Stop/empty queue clears the snapshot; wake-up stays invalid until segment activation.
  Reset, non-motion states and direct jog never expose stale cutting references.
- Programmed reference never comes from gc_state or the current planner/preparation pointer.
- THC requires finite positive speed/reference and ordinary XY feed in Cycle. Percentage 0
  bypasses only the percentage comparison. Manual compensation remains independent.
- Re-arm requires a full avg_samples window of qualified samples; there is no guessed
  100 ms delay. Persistent voltage error at recovered speed can still command correction.
  This change cannot infer whether that persistent error is plate height or plasma behavior.

## Native regression

Run from the firmware repository, without connecting any hardware:

    PATH="$HOME/.platformio/penv/bin:$PATH" pio test -e tests_stepper -e tests

On the Mac with incomplete CLT C++ headers, add:

    PLATFORMIO_BUILD_FLAGS="-isystem $(xcrun --show-sdk-path)/usr/include/c++/v1"

The test_stepper suite compiles the actual Planner.cpp and Stepper.cpp in its test
translation unit, plus the production THC gate/decision implementation. Hardware pulse/timer
and service boundaries are stubs; emitted step counts and programmed timer periods are observable.
The speed oracle is computed independently from the timer and Bresenham displacement, not copied
from the new telemetry fields. This is not a host connection to the machine.

The initial red run on October 4, 2026 failed both core regressions: maximum reported-vs-timer
error 1219.512 mm/min; unsafe DOWN decisions on 544 callback observations at 90% and 656 at 99%.
Those counts are callback observations, not separate crashes or milliseconds.
Set THC_MOTION_TRACE to a CSV path when running the native test executable to capture
corner observations at timer-period changes: prepared speed, executing timer/report speed,
owning feed, percentage gate and real THC DOWN decision at both 90% and 99%.
The trace checks the immediate percentage gate; the separate recovery tests cover the
additional averaging-window qualification. Do not run PlatformIO tests and firmware
builds concurrently: its test cleanup may remove other environment artifacts.

Coverage includes reversed/right-angle/short/diagonal paths, 6/12/20 segment slots, low rates,
planner-block disposal, final pending pulse, future parser feed, feed overrides/axis limits,
hold/reset/underrun, non-cutting motion, direct-jog handoff, coherent concurrent publication,
invalid values and voltage-filter recovery. Unit-equivalence checks exercise the normalized
planner input boundary, not the entire G20/G21 parser. Parser conversion is unchanged.

Getter audit: Report.cpp, THC.cpp (including engage distance), and the debug-message methods in
TMC2130/2209/5160/5160Pro. The driver uses are diagnostic only, not motor-control policies.
The existing FS and Arc formats, configuration keys, CRC and M-code interfaces remain unchanged.

## Rig acceptance

Canonical card: cmqzmy0dm001q01pc0fdv5c5v, jetcad3 / gcode-pilot.
Evidence: /Users/travis/Documents/JetCad3-Diagnostics/2026-10-04-thc-antidive/.

All 347 relevant host cases and all five firmware targets passed. The test bundle
`20261004-thc-antidive-test1` was installed in the rig's firmware overlay and read back through
JetCad3's flasher with matching image hashes. Its ESP32 no-radio merged image SHA-256 is
`d66a7798549116baf975bd034684deb4d2fce0c069d4fa184c35b72590d4a0e1`.

On October 12, 2026, Travis flashed and tested the fix on his plasma rig and reported:
"Worked perfect, pretty sure this is fixed." He accepted the result and requested card closure.
That completes the physical cut acceptance for this rig's timed stepping engine. It does not
constitute separate oscilloscope/jitter measurement or validation of I2S streaming latency.
The prior bundled images and controller configuration remain in the evidence directory.
Public firmware publication is a separate release action.
