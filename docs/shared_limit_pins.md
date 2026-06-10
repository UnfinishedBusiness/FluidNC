# Shared limit pins (`:shared`)

The `:shared` pin attribute lets the **same physical GPIO be used as a limit input
for more than one motor/axis**. It is intended for two-motor (squared) gantries on
boards that are short on inputs, where two home switches are wired onto one input.

## The problem it solves

A squared gantry has four home switches — X, Y1 (left), Y2 (right) and Z — and
normally needs four limit inputs. When a board does not have four free GPIOs, the
switches are paired onto two inputs, for example:

```
   GPIO 13  <---+--- X  home switch
                +--- Y1 home switch        (X and Y1 share one input)

   GPIO  5  <---+--- Z  home switch
                +--- Y2 home switch        (Z and Y2 share one input)
```

Without `:shared`, FluidNC rejects this config two ways:

1. **Pin claiming** — a GPIO may only appear once; the second use throws
   *"Pin is already used."*
2. **Event dispatch** — only one handler is stored per GPIO, so only the
   last-declared motor would ever see the switch.

`:shared` removes both restrictions for the additional declarations.

## Syntax

Declare the GPIO **once as the owner** (a normal pin), and **again with `:shared`**
for each additional motor that uses the same input:

```yaml
x:
  motor0:
    limit_all_pin: gpio.13:low          # owner of GPIO 13
y:
  motor0:
    limit_all_pin: gpio.13:low:shared   # same input as X -> fires X AND Y1 limits
  motor1:
    limit_all_pin: gpio.5:low:shared    # same input as Z -> fires Z AND Y2 limits
z:
  motor0:
    limit_all_pin: gpio.5:low           # owner of GPIO 5
```

`:shared` combines with the usual attributes (`:low`, `:high`, `:pu`, `:pd`) and
works on `limit_neg_pin`, `limit_pos_pin` and `limit_all_pin`.

A complete machine config is in
[`example_configs/gantry_shared_limits.yaml`](../example_configs/gantry_shared_limits.yaml).

## How it works

- The **first, non-`:shared`** declaration *owns* the GPIO: it claims the pin and
  configures the hardware (input mode, pulls, inversion).
- Each **`:shared`** declaration does **not** re-claim the pin and does **not**
  reconfigure the hardware — it only **adds its handler** to that GPIO.
- When the input changes state, FluidNC fires **every** handler registered for the
  GPIO, so all of the sharing motors see the limit at once.

Declaration order does not matter: a `:shared` line may appear before or after its
owner. (In the example, `gpio.5:shared` on Y/motor1 is declared before its owner
`gpio.5` on Z/motor0.)

## Rules and constraints

- **Exactly one owner per shared GPIO.** Declare the pin once without `:shared`.
  Two non-`:shared` declarations of the same GPIO is still an error (this is the
  safety check that catches accidental pin reuse). If you declare *only* `:shared`
  uses and no owner, the pin is never configured and the limit will not work.
- **Shared switches must use the same polarity.** A GPIO has a single inversion
  setting shared by all its handlers, so put the same `:low`/`:high` on every
  declaration of that pin (and wire the paired switches the same way).
- **Axes that share an input must home in separate homing cycles.** If two sharing
  axes homed in the same `cycle`, they would move toward the shared input together,
  and the first switch to close would stop *both* axes. Give each sharing axis its
  own cycle (e.g. `Z: cycle 1`, `X: cycle 2`, `Y: cycle 3`). The two motors of a
  squared axis still home **together** in one cycle — that is what squares the
  gantry — because Y1 and Y2 are on *different* inputs.
- **Scope: limit pins only.** `:shared` is for `limit_*_pin` inputs. It is not for
  output pins, step/dir pins, or other functions.
- **Maximum handlers per GPIO:** 4 (more than enough for a gantry). Extra `:shared`
  declarations beyond that are ignored.

## Behavior during homing and running

- **Homing the owner's axis:** only that axis moves; when its switch closes the
  GPIO fires both handlers, but the non-moving axis's limit has no effect, so only
  the moving axis stops. Correct.
- **Homing a squared (two-motor) axis:** both motors move; each stops when *its*
  switch closes (Y1 on one input, Y2 on the other), squaring the gantry.
- **Hard limits while running:** a trip on a shared input flags the limit for both
  of its axes and raises the alarm — the safe behavior. Because the two switches
  are indistinguishable on one input, automatic pull-off can be ambiguous; pull
  off / clear the alarm manually if needed.

## Compatibility

Configs that do not use `:shared` are completely unaffected — each GPIO still has a
single handler and the duplicate-pin check is unchanged. The feature adds no runtime
cost to non-shared pins.
