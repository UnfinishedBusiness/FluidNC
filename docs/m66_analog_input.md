# M66 analog/digital input read (`#5399`, `[AIR:]`)

`M66` reads a user input and makes the value available to the g-code program,
following the LinuxCNC convention: the result is stored in parameter **`#5399`**, and
the program copies it out immediately.

```gcode
M66 E0 L0          ; read analog input 0 (immediate)
#<arcv> = #5399    ; copy the value out (it only updates on an M66)
```

- `P<n>` reads digital input *n*; `E<n>` reads analog input *n*.
- `L` is the wait mode; analog supports only `L0` (immediate), matching LinuxCNC.
- Result lands in `#5399`. (For future digital wait modes, a timeout yields `#5399 = -1`.)

## Analog values are scaled to real units

Each analog input has optional scale/offset so `#5399` carries an engineering value
rather than a raw pin reading:

```yaml
user_inputs:
  analog0_pin:    gpio.36        # use an ADC1 GPIO so it works with WiFi on
  analog0_scale:  230.0          # value = pin_volts * scale + offset
  analog0_offset: 0.0
```

`readAnalog(n) = (pin_volts × analogN_scale) + analogN_offset`, where `pin_volts` is
the ADC reading (0–3.3 V). Defaults are `scale = 1.0`, `offset = 0.0` (raw volts).

## Capturing the value on a host-side g-code engine

LinuxCNC runs the g-code interpreter in the controller, so `#5399` is resolved
internally. Senders that run the RS274NGC engine **themselves** (expanding variables
host-side and streaming flat g-code) need the value reported back — exactly like
probing returns `[PRB:…]`.

On every `M66`, FluidNC therefore **broadcasts** the result so the sender can store it
in its own `#5399`:

```
M66 E0 L0   ->   [AIR:E0=120.480]
M66 P2 L0   ->   [AIR:P2=1]
```

Format: `[AIR:<E|P><index>=<value>]` ("Analog/Input Read"). `E` values are the scaled
analog reading; `P` values are `0`/`1`. The read is **synchronized with motion** (the
planner buffer drains first), so the value reflects the program point where the `M66`
appears. FluidNC also still sets its own `#5399` for on-board execution.

A sender's flow (e.g. JetCad3 GcodePilot): on `M66`, drain/sync, read the `[AIR:…]`
line, set its `#5399`, then resolve the following `#5399` references while expanding.

## Notes

- Analog `E` is immediate-only; digital wait modes (rise/fall/high/low + `Q` timeout)
  are not yet implemented.
- The same scaled reading drives the `|UIO:AI<n>` status field for live gauges — see
  [user_io_reporting.md](user_io_reporting.md).
- `readAnalog()` is hardware (ESP32 ADC); on the native test build it returns 0.
