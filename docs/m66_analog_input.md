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

## Sender implementation (host-side g-code engine, e.g. GcodePilot)

LinuxCNC runs the interpreter in the controller, so `#5399` is resolved internally.
A sender that runs the RS274NGC engine **itself** — expanding variables host-side and
streaming flat g-code — must get the value reported back, exactly like it already does
for probing (`[PRB:…]` → `#5061–#5070`). This section is the contract to implement.

### Wire message

On every `M66`, FluidNC **broadcasts** one line to all channels (it appears on the
sender's connection interleaved with `ok`/status/etc.):

```
M66 E0 L0   ->   [AIR:E0=120.480]
M66 P2 L0   ->   [AIR:P2=1]
```

Grammar — `[AIR:<kind><index>=<value>]`:

| Field | Values |
|-------|--------|
| `kind` | `E` = analog input, `P` = digital input |
| `index` | the pin number from the `E`/`P` word (`0`-based) |
| `value` | analog: a decimal with **3 fractional digits** (e.g. `120.480`); digital: `0` or `1` |

Suggested parser regex: `^\[AIR:([EP])(\d+)=(-?\d+(?:\.\d+)?)\]$`.
(A future digital-wait timeout will report `=-1`; analog is immediate-only today.)

### Association model (why this is simple)

`M66` is a **queue-buster**: FluidNC synchronizes the planner (drains motion) before
reading. A correct sender already drains its own pipeline at `M66` and streams it
alone, so the exchange is strictly **1:1 and in order** — send one `M66`, receive
exactly one `[AIR:…]`. The `kind`+`index` in the message let you assert it matches the
`M66` you issued; you do not need to correlate concurrent reads.

### Algorithm

When the NGC engine reaches an `M66`:

1. **Drain / sync** the stream (the same point you already drain for probe / `M0` /
   `MSG`): stop expanding ahead, let outstanding lines `ok`.
2. **Send** the `M66 …` line; wait for its terminating `ok` **and** the `[AIR:…]` line
   (either order — the `[AIR:]` is emitted just before the `ok`).
3. **Store** the parsed value into the engine's `#5399`.
4. **Resume** expansion; the following `#<var> = #5399` (or any `#5399` reference) now
   resolves to the captured value.

### Pseudocode

```js
// In the controller line handler, alongside the existing [PRB:...] capture:
const m = line.match(/^\[AIR:([EP])(\d+)=(-?\d+(?:\.\d+)?)\]$/);
if (m) {
  ngc.setParam(5399, parseFloat(m[3]));   // #5399, same registry probe writes #5061+
  pendingM66?.resolve();                   // unblock the drained M66 step
  return;
}

// In the NGC runner, when the next op is an M66 (a queue-buster):
await drainAndSync();                       // reuse the probe/M0 drain path
const air = waitForLine(/^\[AIR:/);         // resolved by the handler above
sendLine(op.text);                          // "M66 E0 L0"
await Promise.all([air, waitForOk()]);      // [AIR:] + ok
// #5399 is now set; continue expanding -- a later "#<v> = #5399" reads it.
```

### Error / edge cases

- **Undefined pin:** if the referenced input is not configured, FluidNC does **not**
  emit `[AIR:]` and instead `error:`s the `M66` line. Treat the `error:` on a drained
  `M66` as a fault (and time out the `[AIR:]` wait) rather than hanging.
- **Don't confuse tags:** `[AIR:]` is distinct from `[PRB:]`, `[MSG:…]`, `[GC:…]`; match
  the full `[AIR:…]` shape.
- FluidNC also sets its own `#5399` for on-board execution; the host copy is what your
  expansion uses.

The same scaled reading also drives the periodic `|UIO:AI<n>` status field for live
gauges (see [user_io_reporting.md](user_io_reporting.md)) — that path is independent of
the `M66`/`[AIR:]` capture.

## Notes

- Analog `E` is immediate-only; digital wait modes (rise/fall/high/low + `Q` timeout)
  are not yet implemented.
- The same scaled reading drives the `|UIO:AI<n>` status field for live gauges — see
  [user_io_reporting.md](user_io_reporting.md).
- `readAnalog()` is hardware (ESP32 ADC); on the native test build it returns 0.
