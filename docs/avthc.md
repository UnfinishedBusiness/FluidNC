# AVTHC — Automatic Voltage Torch Height Control

AVTHC is plasma torch height control built into FluidNC. During a cut it reads the
plasma arc voltage from an analog input and nudges the Z axis up or down to hold a
target voltage (which corresponds to a constant torch-to-work height), following the
LinuxCNC/plasmac model.

It is **optional and inert unless configured** — with no `thc:` section in
`config.yaml`, nothing changes for existing machines.

A complete machine config is in
[`example_configs/plasma_thc.yaml`](../example_configs/plasma_thc.yaml).

## G-code interface

Three dedicated M-codes in the LinuxCNC user-defined band (M100–M199):

| Code | Action |
|------|--------|
| `M101` | Enable THC (motion-synchronized — engages at the next motion boundary) |
| `M102` | Disable THC (motion-synchronized) |
| `M103 Q<volts>` | Set the target arc voltage. `Q` at or below the configured `min_arc_voltage` turns correction off. |

LinuxCNC has no M-code that sets a numeric arc voltage (it uses material files), so
`M103 Q` is a purpose-built FluidNC command rather than a reused one.

Typical program shape:

```gcode
M103 Q120     ; target 120 V for this material
... pierce, then start the cut ...
M101          ; enable height control once the arc is established
G1 X... Y...  ; cut
M102          ; disable before the lead-out / rapids
```

### Torch firing is NOT M3/M5

The torch (and any oxy/engraver heads) is fired with **digital I/O (M62–M65)** from
the program or an `M6` tool-change subroutine. The THC is torch-agnostic and only
watches the **arc-ok** input to know a cut is live. `M3`/`M5`/`S` stay dedicated to a
real router spindle that can run alongside on the same machine.

## Configuration

```yaml
thc:
  arc_voltage_pin: gpio.36        # ADC input carrying the (divided) arc voltage
  arc_voltage_scale: 230.0        # multiplier: pin volts -> arc volts (divider ratio)
  arc_voltage_offset: 0.0         # added after scaling (volts)
  arc_ok_pin: gpio.39:low         # asserted when the arc is established (active-low here)
  min_arc_voltage: 30             # M103 Q at/below this disables correction
  target_voltage: 0               # seed target until M103 sets one
  threshold_volts: 2.0            # deadband around the target (no motion inside it)
  pid_p: 10.0                     # proportional gain (volts -> step rate)
  vad_threshold_pct: 65           # velocity anti-dive (0 disables); see below
  thc_delay_ms: 300               # stabilization delay after arc-ok before correcting
  max_z_rate_mm_min: 600          # caps the injected Z correction rate
  avg_samples: 5                  # arc-voltage moving-average window
  invert_z: false                 # set true if +Z lowers the torch on your machine
```

## How the control loop behaves

A background loop samples the arc voltage (moving-averaged) and moves Z only when
**all** of these hold; otherwise it holds height:

- THC is enabled (`M101`) and the **arc-ok** input is asserted.
- The target is above `min_arc_voltage`.
- The post-arc **stabilization delay** (`thc_delay_ms`) has elapsed.
- **Velocity anti-dive** is not active. When the actual feed drops below
  `vad_threshold_pct` of the programmed feed (e.g. decelerating into a corner), the
  arc voltage rises for reasons unrelated to height, so correction is suspended to
  stop the torch diving.
- The voltage error exceeds the `threshold_volts` deadband.

Inside those gates a proportional term (`pid_p`) sets the Z step rate, capped by
`max_z_rate_mm_min`. Voltage low → raise the torch (more arc gap); voltage high →
lower it.

## Status reporting

When `thc:` is present, **every** `?` status report carries an arc field so a sender
can drive a live DRO and AVTHC indicators:

```
|Arc:<voltage>,<arc_ok>,<active>,<target>,<enabled>
```

| idx | field   | type        | meaning |
|-----|---------|-------------|---------|
| 0   | voltage | float (1dp) | measured arc voltage (scaled engineering volts) |
| 1   | arc_ok  | `0`/`1`     | arc-OK input asserted |
| 2   | active  | `0`/`1`     | THC is **currently correcting Z** (all control gates passed) |
| 3   | target  | float (1dp) | **set target voltage** (`M103 Q`; `0` until set) |
| 4   | enabled | `0`/`1`     | **AVTHC armed** (`M101`=1 / `M102`=0) |

Example: `|Arc:123.4,1,1,120.0,1`.

**`enabled` vs `active`:** `enabled` is the operator's intent (armed via `M101`); `active`
is true only while the loop is *actually moving Z* (enabled **and** arc-OK **and** past the
stabilization delay **and** not in anti-dive **and** outside the deadband). So a torch can be
`enabled=1, active=0` (armed but idle). Drive an "AVTHC ON" indicator off `enabled` and an
"adjusting" cue off `active`.

**`target` vs `min_arc_voltage`:** a `target` at or below the configured `min_arc_voltage`
means correction is effectively off even when `enabled=1` (the firmware's THC-OFF floor).

### Sender mapping

Parse the `|Arc:` CSV → `{ voltage, arcOk, active, target, enabled }`. Positions 0-2 are
unchanged from earlier firmware, and 3-4 are additive — a 3-field `|Arc:` from an older build
still parses (treat missing `target` as unset and `enabled` as `0`).

## Z motion injection — requirements

The THC moves Z by pulsing the Z step/dir pins directly while the program holds Z
(only X/Y move during the cut), and it tracks the executed position so the DRO/MPos
stay exact.

- **The Z axis must use the direct-GPIO `timed` step engine.** RMT/I2S step engines
  buffer/stream their pulses and cannot be driven out-of-band; THC will refuse to
  arm (logged at startup) if Z is on one of them.
- Use an **ADC1** GPIO for `arc_voltage_pin` so it coexists with WiFi (ADC2 is shared
  with the WiFi radio on the ESP32).
- Provide proper electrical isolation/scaling for the arc voltage (e.g. a divider
  board); `arc_voltage_scale` is the divider ratio.

## Scope / status

This is the pragmatic-core feature set (voltage mode + threshold + arc-ok gate +
stabilization delay + velocity anti-dive + proportional rate). Full plasmac parity
(PID I/D terms, void anti-dive, auto-volts sampling, ohmic probing, material files
via `M190`) is possible future work. Must be validated on a machine before relying
on it for cutting.
