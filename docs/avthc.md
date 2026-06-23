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
  manual_comp_rate_mm_min: 600    # Z rate for manual comp (RT comp up/down) — see Operator override
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

## Operator override (real-time)

A sender (GcodePilot) can override the program's AVTHC live during a cut, without
disturbing the streamed G-code, using **ack-free real-time command bytes**. These are
picked off the serial stream like the feed/spindle override bytes — they never produce an
`ok`, so they're safe to send while a program is running.

| byte   | command | effect |
|--------|---------|--------|
| `0xA2` | THC volt + | override target **+1 V**. The first nudge out of Gcode/Disabled mode seeds the override from the current effective target and switches to **Override** mode. |
| `0xA3` | THC volt − | override target **−1 V** (same seeding). |
| `0xA4` | THC Gcode | mode → **Gcode Controlled** — clears the override; `M101/M102/M103` are back in charge. |
| `0xA5` | THC Disable | mode → **Disabled** — THC is forced off; `M101/M102` become no-ops for the rest of the run. |
| `0xA6` | Manual comp up | move Z **up** continuously at `manual_comp_rate_mm_min` (only while `State == Cycle`). |
| `0xA7` | Manual comp down | move Z **down** continuously. |
| `0xA8` | Manual comp stop | stop manual comp. |

**Override modes** (reported as `ovrMode` in the status field below):

- **Gcode Controlled (0)** — default. Effective target = `M103 Q`; armed = `M101/M102`.
- **Override (1)** — effective target = the operator's override volts; arming still follows
  `M101/M102`. Entered automatically by the first `+/−1 V` nudge or a verbatim set (a burst
  of `+/−1 V` bytes walking to the target — there is no value-carrying byte, so resolution is
  1 V).
- **Disabled (2)** — THC forced off; `M101`/`M102` ignored.

**Manual compensation** drives Z directly (reusing the THC step-injection path), independent
of arc voltage — for finishing a cut on warped stock or a nearly-dead consumable. It only
moves Z while a program is running (`State == Cycle`). GcodePilot switches the override to
**Disabled** before sending comp commands so the control loop is already idle and the handoff
is seamless; the operator clicks back to auto when done.

> ⚠️ Manual comp and `Disabled` mode remove automatic height control mid-cut — the torch can
> dive into or climb off the plate. Manual comp requires the same direct-GPIO `timed` Z step
> engine as THC itself (see *Z motion injection* below).

## Status reporting

When `thc:` is present, **every** `?` status report carries an arc field so a sender
can drive a live DRO and AVTHC indicators:

```
|Arc:<voltage>,<arc_ok>,<active>,<target>,<enabled>,<ovrMode>
```

| idx | field   | type        | meaning |
|-----|---------|-------------|---------|
| 0   | voltage | float (1dp) | measured arc voltage (scaled engineering volts) |
| 1   | arc_ok  | `0`/`1`     | arc-OK input asserted |
| 2   | active  | `0`/`1`     | THC is **currently correcting Z** (all control gates passed) |
| 3   | target  | float (1dp) | **effective target voltage** — override volts when overriding, else `M103 Q` (`0` until set) |
| 4   | enabled | `0`/`1`     | **effective armed** — `0` when override mode is Disabled, else `M101`=1 / `M102`=0 |
| 5   | ovrMode | `0`/`1`/`2` | operator override mode: `0` Gcode Controlled, `1` Override, `2` Disabled |

Example: `|Arc:123.4,1,1,120.0,1,0`.

`target` and `enabled` carry the **effective** values (after the override mode is applied), so a
sender DRO that already reads them follows the override automatically; `ovrMode` tells the sender
which mode the firmware is in so it can reflect the override UI state.

**`enabled` vs `active`:** `enabled` is the operator's intent (armed via `M101`); `active`
is true only while the loop is *actually moving Z* (enabled **and** arc-OK **and** past the
stabilization delay **and** not in anti-dive **and** outside the deadband). So a torch can be
`enabled=1, active=0` (armed but idle). Drive an "AVTHC ON" indicator off `enabled` and an
"adjusting" cue off `active`.

**`target` vs `min_arc_voltage`:** a `target` at or below the configured `min_arc_voltage`
means correction is effectively off even when `enabled=1` (the firmware's THC-OFF floor).

### Sender mapping

Parse the `|Arc:` CSV → `{ voltage, arcOk, active, target, enabled, ovrMode }`. Positions 0-2
are unchanged from earlier firmware, and 3-5 are additive — a 3-field `|Arc:` from an older build
still parses (treat missing `target`/`enabled`/`ovrMode` as unset). Send the operator-override
bytes from the *Operator override* table above to drive it live.

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
