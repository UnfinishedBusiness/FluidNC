# User-I/O status reporting (`:report`)

FluidNC can include the live state of user-defined I/O (the pins driven by
`M62`–`M68`) in its `?` status report, so a sender or pendant can light indicators
and gauges directly from the report. It is **opt-in per pin** and **off by default** —
pins without `:report` are not reported and the status line is unchanged.

## Enabling it

Add the `:report` attribute to any user-I/O pin in `config.yaml`:

```yaml
user_outputs:
  digital0_pin: gpio.16:report      # torch / relay -- report its on/off state
  analog0_pin:  gpio.25:report      # PWM output -- report its commanded percent

user_inputs:
  digital0_pin: gpio.34:report      # e.g. a "material present" sensor
```

`:report` combines with the usual attributes (`:low`, `:pu`, `:pd`, …), e.g.
`gpio.34:low:pu:report`.

## What the report looks like

When at least one user-I/O pin is flagged, the report gains a `|UIO:` field:

```
<Idle|MPos:0.000,0.000,0.000|FS:0,0|UIO:DO0=1,AO0=40,DI0=0>
```

Each entry is `<type><index>=<value>`:

| Prefix | I/O | Set/read by | Value |
|--------|-----|-------------|-------|
| `DO` | digital output | `M62`/`M63`/`M64`/`M65` | `0` / `1` |
| `AO` | analog output | `M67`/`M68` | commanded percent `0`–`100` |
| `DI` | digital input | `M66 P` | `0` / `1` |
| `AI` | analog input | `M66 E` | scaled value (see [m66_analog_input.md](m66_analog_input.md)) |

Only flagged pins appear; the field is omitted entirely when none are flagged. The
index matches the config pin number (`digital0_pin` → `DO0`/`DI0`,
`analog1_pin` → `AO1`/`AI1`).

## Consuming it

A sender reads `|UIO:` from each `?` report (FluidNC is typically polled at ~10 Hz)
and updates indicators by name — e.g. map `DO0` to a "torch on" LED, `DI0` to a
"material present" lamp, `AO0` to a level bar. Values reflect the live state at the
moment of the report (digital outputs reflect the last commanded state; inputs
reflect the live pin).

## Limitations / notes

- **Analog input** (`AI<n>`) reports a real, scaled value (volts × `analogN_scale` +
  `analogN_offset`), the same reading `M66 E<n>` returns — see
  [m66_analog_input.md](m66_analog_input.md). Use an ADC1 GPIO so it coexists with WiFi.
- **`:report` is meaningful only on user-I/O pins.** It is accepted (and ignored) on
  other pins.
- **Latency** is the sender's poll interval. A future enhancement could push an
  immediate report when a reported input changes (as limit pins do) for snappier UI.
- **Default-off:** a config with no `:report` produces no `|UIO:` field — no change for
  existing setups or report parsers.

## Implementation

- `:report` is parsed in the pin detail (`GPIOPinDetail`, `I2SOPinDetail`) and exposed
  via `Pin::reportInStatus()`.
- `UserOutputs` tracks the last commanded digital/analog state so it can be reported.
- `report_user_io_string()` in `Report.cpp` builds the `|UIO:` field each report from
  live state + the per-pin flags.
