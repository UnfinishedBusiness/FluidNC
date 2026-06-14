# Firmware-native jogging & shuttle

A built-in motion engine that lets a sender drive **continuous, smoothly-held jogs** and a
**path shuttle** (jog the tool back and forth *along the loaded toolpath*) without streaming
a string of `$J=` blocks. The sender sends one start command; the firmware keeps the motion
fed until told to stop.

It is **optional and gated at build time.** It is compiled in only when
`-DENABLE_FW_JOG` is set — that flag is on for the `noradio` / `noradio_s3` targets and off
for the `wifi` / `bt` targets, which build byte-for-byte as before (the module's source is
also excluded from those link steps). Senders feature-detect it from the capability line
(below) rather than assuming it is present.

A sender talks to it over the same line protocol it already uses; the full wire contract is
[`plans/firmware-jogging/protocol.md`](../plans/firmware-jogging/protocol.md).

## Why not just stream `$J=`?

`$J=` jogs are discrete planner blocks. To hold a button down the sender must keep refilling the
planner over the serial link, and the planner's lookahead/**tail-to-zero** owns the velocity
profile — so cornering decelerates at every junction and held velocity is inconsistent. Feeding
the planner *in process* (an earlier "Route A" attempt) does not fix this: the planner is still
in charge. The firmware engine instead bypasses the planner entirely and drives the steppers at a
commanded per-axis velocity (below).

### Route B: direct-stepper velocity jog (LinuxCNC equivalent)

The vector-jog engine **drives the stepper motors directly and does not use the GRBL planner at
all** — the planner's lookahead and "last queued block exits at zero" rule are exactly what made
a planner-fed jog feel wrong (cornering chokes, motion is inconsistent). Instead it is a per-joint
**stepgen**, the same model LinuxCNC uses:

- A per-axis **velocity integrator** (`JogIntegrator.h`, ported from GcodePilot's proven host-side
  jog planner) runs in the ~1 kHz refill tick: each axis slews its velocity toward
  `direction × cruise` at the accel limit and integrates independently.
- Each tick it publishes the per-axis velocity (mm/s) to `JogStepper`, which converts it to a
  fixed-point step increment and runs an **integer DDA inside the step ISR** (`Stepper::pulse_func`
  → `JogIntegrator::dda_step`): each axis accumulates and emits a step when it crosses one full
  step. Pulses go out through the universal `Stepping::step()`, so it works on every stepping
  engine (RMT, I2S, timed) and `axis_steps[]` — the machine-position source of truth — stays exact.

This is what delivers the RC-car feel:

- **A quick tap makes a small move.** Velocity only ramps while the key is held; the steppers
  only ever move at the integrated velocity, so a tap covers exactly the brief ramp — bounded by
  press time, with no committed runway to overshoot.
- **Cornering sweeps a smooth arc.** A direction change re-aims the setpoint; each axis slews its
  own velocity (ramping through zero on a reversal), so the resultant velocity *vector* rotates
  smoothly and the path is a blended arc, never an angular "S-curve."
- **Flat velocity at any speed.** Once an axis reaches cruise the DDA emits steps at a constant
  rate — there is no queue to starve, so no flutter. Cruise is capped to `JogStepper::JOG_STEP_HZ
  / steps_per_mm` so the DDA never owes more than one step/axis/tick.
- **Smooth, minimal stop.** `$Jog/Stop` and realtime `0x85` set the velocity setpoint to zero; the
  integrator ramps each axis to rest at the accel limit (overshoot `v²/2a`, the physical minimum).
  A re-press during the ramp-down resumes from the live velocity (tap-and-speed-back-up).

When the jog ends, `JogStepper::exit()` resyncs the planner and gcode parser to the stepped
position (`gc_sync_position(); plan_sync_position();`, the same recipe homing uses) so the next
program/`$J=` move starts from the true machine position. The integer DDA and the velocity
trajectory are host-tested in `tests/test_jog` (`JogIntegratorTest`: `dda_step`, plus tap-bound,
reversal blend, release overshoot, soft-limit park).

## Configuration

A `jogging:` section is optional; defaults are safe.

```yaml
jogging:
  allow_unhomed: false           # permit jogging from Alarm:Unhomed (default false)
  unhomed_feed_cap_mm_min: 1000  # feed ceiling for the allowed Alarm:Unhomed jog exception
```

- **`allow_unhomed`** — when true, a vector jog may start from `Alarm:Unhomed`. The machine
  drops to `Idle` to run the move **without clearing the unhomed flags** (unlike `$X`), and
  re-asserts `Alarm:Unhomed` when the jog ends, so program-start homing enforcement and the
  sender's homing badge stay truthful. No soft-limit envelope is applied while unhomed, so
  the feed cap is the only protection — keep it conservative.
- **`unhomed_feed_cap_mm_min`** — the feed ceiling applied only to the allowed
  `Alarm:Unhomed` jog exception on machines with required homing enabled.

## Vector jog (`$Jog/*`)

All feeds are **mm/min** and coordinates **mm**, independent of `G20/G21`.

| Command | Meaning |
|---------|---------|
| `$Jog/Start=X<±1\|0> Y<±1\|0> Z<±1\|0> F<mm/min>` | Begin/redirect a held jog. Re-issued mid-jog with a new vector, the velocity **blends** to the new direction (smooth arc). |
| `$Jog/Feed=<mm/min>` | Change the held feed in place; velocity ramps to the new cruise (no-op if not jogging). |
| `$Jog/Stop` | Decelerate to a stop (smooth, acceleration-limited; overshoot `v²/2a`). |
| realtime `0x85` | Immediate cancel — same smooth in-place decel as `$Jog/Stop` (panic path). |

Allowed states: `Idle`, `Jog` (re-issue `Start` to change direction mid-jog), or the
`Alarm:Unhomed` carve-out above. A disallowed state returns `error:9`; a malformed
vector/feed returns `error:3`.

The cruise feed is clamped so no active axis exceeds its `max_rate` along the vector
(`feed ≤ min(max_rate[a]/|dir[a]|)`), then the unhomed cap (if applicable) and the
queue-holdable cap are applied.

### Machine extents

Extent enforcement is keyed on each axis's **`soft_limits`** setting — **not** on homing.
A machine with `soft_limits` on but homing not required (known boot position) gets full
envelope protection; the integrator **proactively decelerates and parks an axis exactly at its
soft-limit boundary** (`v²/2a` vs. distance-to-fence), so a jog into a limit stops smoothly at
the fence instead of slamming or overrunning. On a diagonal, the axis that reaches its limit
parks while the others keep moving — the jog slides along the fence. The **sole** no-envelope
case is the `allow_unhomed` `Alarm:Unhomed` carve-out, where the position is genuinely unknown
and you must be able to jog toward the homing switches; there the feed cap is the only guard.

## Path shuttle (`$Shu/*`)

Shuttle mode jogs the tool **along the program's XY toolpath** — useful for inspecting or
re-running a section. The sender streams the path as a sliding window of vertices; the
firmware never holds the whole program. It walks a commanded position along that polyline by
arc length, forward or backward.

| Command | Meaning |
|---------|---------|
| `$Shu/Begin=<N>` | Open a session for a path of `N` total vertices. |
| `$Shu/Data=<firstIdx>:x,y;x,y;…` | Load consecutive vertices (mm) at absolute index `firstIdx`. Chunks of ≤64; must be contiguous with the loaded window. |
| `$Shu/Jog=<±1\|0> F<mm/min>` | Hold a direction along the path (`+1` forward / `−1` back) at cruise `F`; `0` releases. |
| `$Shu/End` | Close the session and free the window. |

- The vertex window is a 1024-entry ring keyed by absolute index, so the sender can stream
  ahead and let old vertices fall out behind the tool.
- **Entry** requires the machine to already be on the loaded path (within 1 mm) — the sender
  positions it there first; otherwise `error:9`.
- **Release** (`$Shu/Jog=0`) lets the planner decelerate to rest *on the path*. A
  **direction reversal** stops first, then resumes the other way from where it actually came
  to rest (the planner forces the stop at the reversal point; you cannot reverse with
  infinite jerk).
- **Window/path edges:** if the tool reaches the end of the loaded window (runway not yet
  streamed) or the end of the path, it decelerates and parks there; streaming more `Data`
  resumes motion.
- Per-block axis limits are enforced by the planner; the unhomed feed cap still applies.

### Shuttle status field

While a session is open, the realtime status report (`?` and the `$Report/Interval`
auto-report) carries:

```
|Shu:<vidx>,<s_mm>
```

- **`vidx`** — absolute index of the path vertex at or behind the tool's current position.
- **`s_mm`** — arc length (mm, 3 dp) from that vertex to the tool.

The sender uses this to highlight the live position along the toolpath.

## Lifecycle & termination (safety)

A vector jog drives the steppers directly while the engine believes it owns the motion. It
**must stop the instant the motion is no longer genuinely its own**, or it would keep stepping
after a panic action. Two layers guarantee that:

**1. Self-validating tick (primary).** Every ~1 kHz tick the engine checks the live machine state
and tears down — stopping the step timer and dropping to idle — unless the machine is in `Jog`
with no cancel/hold/door (suspend) condition active. Any other observation — `Alarm`, `Hold`,
`Sleep`, `Homing`, a program `Cycle`, `SafetyDoor`, or a suspend bit set — terminates the engine.

**2. Belt-and-suspenders teardown (immediate).** `onMotionTerminated()` is also called the moment
the system terminates motion (soft-reset and alarm handlers, gated to FWJOG builds); it calls
`JogStepper::exit()` to stop the step timer and resync position immediately rather than at the
next tick.

Termination matrix:

| Action while jogging | Result |
|----------------------|--------|
| Realtime `0x85` / `$Jog/Stop` | velocity ramps to rest at the accel limit (`v²/2a`), engine exits |
| Feed-hold / suspend | engine terminates (step timer stopped) |
| Soft reset `0x18` | engine terminates |
| Alarm (limit, etc.) | engine terminates |
| `$X` unlock *after* an alarmed jog | does **not** resume the old jog |
| Quick tap | only the brief ramp is stepped — bounded by press time, no committed runway |

On exit the engine **resyncs the planner + gcode position** to the stepped position
(`gc_sync_position(); plan_sync_position();`), so the next program/`$J=` move starts correctly.

**Vector and shuttle modes are mutually exclusive and reset on each start** — a vector jog after a
shuttle session always runs as a vector jog. (Shuttle `$Shu/*` still uses the planner-fed path; it
is a path-constrained motion that does not map onto free per-axis velocity.)

## Capability advertisement

When the engine is compiled in, the firmware advertises it on the **boot banner** and in
the `$I` build-info response:

```
[CAP:FWJOG=1,FWSHU=1]
```

`FWJOG` = the `$Jog/*` vector engine; `FWSHU` = the `$Shu/*` shuttle engine. A sender should
feature-detect from this line and fall back to streamed `$J=` jogs when it is absent.

**`$Cap` (anyState)** emits the same line on demand. Use it: the boot banner is missed whenever
the sender opens the port after boot, and `$I` is unusable on a `must_home` board parked in
Alarm:Unhomed — passive detection alone made the jog source nondeterministic across reboots.

## Velocity hold & step rate

There is no planner queue to starve, so held velocity is flat by construction — once at cruise the
DDA emits steps at a constant rate. The cap is the step-ISR rate: `JogStepper::JOG_STEP_HZ`
(default 60 kHz) bounds the max jog rate per axis to `JOG_STEP_HZ / steps_per_mm`, and the
commanded cruise is clamped to that (`JogStepper::maxCruiseMmMin`) so the DDA never owes more than
one step/axis/tick. Raise `JOG_STEP_HZ` (ISR budget permitting) if a high-`steps_per_mm` machine
caps below the desired jog speed. The `?` status reports `|JogQ:<speed>,<target-speed>` (mm/s)
while a vector jog runs.

A re-press during the ramp-down resumes directly from the live velocity (no wait for a full stop),
so rapid taps and quick reversals feel continuous.

## Build

The engine is on for the radio-less targets. To produce the full tester bundle (all five
targets + manifest) run [`agent-build-firmware.sh`](../agent-build-firmware.sh); see
[`AGENT-firmware-build.md`](../AGENT-firmware-build.md). For a single target:

```sh
pio run -e noradio     # engine compiled in
pio run -e wifi        # engine gated out (unchanged build)
```

The pure motion math, command parsers, the path ring-buffer + arc-length walk, and the
flat-velocity invariant are unit-tested on the host:

```sh
pio test -e tests -f test_jog
```
