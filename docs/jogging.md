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

`$J=` jogs are discrete blocks. To hold a button down you must keep refilling the planner
over the serial link, and any latency in that refill lets the planner's **tail-to-zero**
reach the front of the queue — the machine decelerates mid-jog and the felt velocity sags.
The firmware engine refills *in process* from the realtime loop with sub-millisecond
latency, so the queued runway never runs dry and held velocity stays ruler-flat.

The engine does **not** bypass the planner or drive the steppers directly — it feeds the
**existing planner/stepper path** (compatible with RMT and I2S stepping). This is "Route A":
a self-refilling planner feed.

### The flat-velocity invariant

The planner always plans the last queued block to exit at zero velocity. The machine can
therefore only *hold* feed `v` while at least the braking distance `v²/2a` stays queued
ahead of the executing point. The refill loop keeps the queued runway at
`max(1.5·v²/2a, 3 block-lengths)` every realtime tick, so the decel tail is never reached
and the executing feed stays pinned at cruise. This is enforced as the acceptance test
`FlatVelocityInvariant` in `tests/test_jog`.

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
| `$Jog/Start=X<±1\|0> Y<±1\|0> Z<±1\|0> F<mm/min>` | Begin/redirect a held jog along the given direction (each axis −1/0/+1; ≥1 nonzero; F required). |
| `$Jog/Feed=<mm/min>` | Change the held feed in place (no-op if not jogging). |
| `$Jog/Stop` | Decelerate to a stop. |
| realtime `0x85` | Cancel (same as `$Jog/Stop`, immediate). |

Allowed states: `Idle`, `Jog` (re-issue `Start` to change direction mid-jog), or the
`Alarm:Unhomed` carve-out above. A disallowed state returns `error:9`; a malformed
vector/feed returns `error:3`.

The cruise feed is clamped so no active axis exceeds its `max_rate` along the vector
(`feed ≤ min(max_rate[a]/|dir[a]|)`), then the unhomed cap is applied. When homed, each
queued block is constrained to the soft-limit envelope; on reaching a fence the engine
parks against it (the planner tail-to-zero holds position) rather than overrunning.

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

## Capability advertisement

When the engine is compiled in, the firmware advertises it on the **boot banner** and in
the `$I` build-info response:

```
[CAP:FWJOG=1,FWSHU=1]
```

`FWJOG` = the `$Jog/*` vector engine; `FWSHU` = the `$Shu/*` shuttle engine. A sender should
feature-detect from this line and fall back to streamed `$J=` jogs when it is absent.

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
