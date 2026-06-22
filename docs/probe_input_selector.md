# Probe-input selector word (`G38.n D<sel>`)

An optional, **per-block** word on the probe canned cycles (`G38.2/.3/.4/.5`) that selects **which of the
probe's two configured inputs** the cycle watches. It exists so a host g-code program can probe with a
single input on demand — for example, to fall back to the floating-head switch when an ohmic ring is
slagged — without any modal state or a custom M-code.

## Why

`probe:` can declare two inputs:

```yaml
probe:
  pin: gpio.34            # e.g. floating-head switch
  toolsetter_pin: gpio.35 # e.g. ohmic ring
```

By default the probe trips on **either** input — `Probe::get_state()` returns `pin OR toolsetter_pin`.
That OR is hardcoded and, until now, could not be masked from g-code. On a plasma torch a worn electrode can
stick to the shield and **latch the ohmic input ON** even when not touching the work, which makes every
`G38` false-trigger at the current height. There is no standard RS274/LinuxCNC g-code for selecting a probe
input (LinuxCNC does it in HAL), so this fork adds one minimal, backward-compatible word.

## Syntax

```
G38.n D<sel>
```

| `D` value      | Input watched for this probe move                          |
|----------------|------------------------------------------------------------|
| `0` or omitted | **Both** — `pin OR toolsetter_pin` (the default OR; unchanged) |
| `1`            | `probe.pin` **only** (e.g. floating head)                  |
| `2`            | `probe.toolsetter_pin` **only** (e.g. ohmic)               |

- **Per-block.** The selection applies to that one `G38` move only. The cycle sets it at start and restores
  *Both* on every exit path (success, failure, abort), so it can never leak to the next move or job — there
  is no modal state and no fail-safe reset to manage.
- **Backward-compatible.** Omit `D` (or use `D0`) → exactly today's OR behavior. Standard single-probe
  setups are unaffected, and a generic RS274 interpreter that doesn't know the word is the only thing that
  needs the fork.
- A `D` word on any non-`G38` block is ignored (it is not an error).

## Errors

- `D` is not `0`, `1`, or `2` (or not an integer) → **error 181** (`Gcode invalid word value`).
- The pin backing the selected source is not configured (e.g. `D2` with no `toolsetter_pin`) → **error 28**
  (`Gcode value word missing`).

## Verifying

The two inputs are reported independently in the `?` status report's `|Pn:` field (`P` = probe pin,
`T` = toolsetter pin) when [user-I/O reporting](user_io_reporting.md) is enabled, so you can bench-confirm:

- `G38.3 D1 …` triggers on the floating-head pin only,
- `G38.3 D2 …` on the ohmic pin only,
- omitted / `D0` on either (the original OR).

## Implementation

- `src/Probe.h` / `src/Probe.cpp` — `enum class ProbeSource { Both, ProbeOnly, ToolsetterOnly }`, a transient
  `_source` member with `setSource()`, and `get_state()` honoring it. `sourcePinDefined()` validates a `D`
  word against the configured pins.
- `src/GCode.cpp` — parses the `D` word into `gc_values_t::d`, validates it in the `G38` block, and passes
  the decoded `ProbeSource` into `mc_probe_cycle()`.
- `src/MotionControl.cpp` — `mc_probe_cycle()` calls `setSource(sel)` before the pre-trigger check and
  restores `Both` on every return path.

This is consumed by JetCad3's GcodePilot plasma macro (`fire_torch`) for stuck-ohmic auto-fallback.
