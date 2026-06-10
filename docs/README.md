# FluidNC feature docs

Documentation for features added on top of the main
[FluidNC wiki](http://wiki.fluidnc.com/). Each page covers one optional, off-by-default
feature: what it does, the `config.yaml` it uses, and the sender-facing behavior.

| Feature | Doc | Example config |
|---------|-----|----------------|
| **CRC32 serial line integrity** — optional checksummed line protocol for robust streaming over noisy USB. | [crc32_serial.md](crc32_serial.md) | (config snippet in the doc) |
| **Shared limit pins (`:shared`)** — let one GPIO serve several motors' limits, for two-input gantry squaring. | [shared_limit_pins.md](shared_limit_pins.md) | [gantry_shared_limits.yaml](../example_configs/gantry_shared_limits.yaml) |
| **User-I/O status reporting (`:report`)** — surface user I/O (M62–M68) state in the `?` report to drive sender indicators. | [user_io_reporting.md](user_io_reporting.md) | [user_io_reporting.yaml](../example_configs/user_io_reporting.yaml) |
| **M66 analog input read** — scaled analog reads into `#5399`, broadcast as `[AIR:]` for host-side g-code engines. | [m66_analog_input.md](m66_analog_input.md) | [user_io_reporting.yaml](../example_configs/user_io_reporting.yaml) |

All of these are inert unless explicitly enabled in `config.yaml`, so they do not
change behavior for existing machines or senders.

## New `config.yaml` pin attributes

These pages add pin attributes that combine with the existing ones (`:low`, `:high`,
`:pu`, `:pd`, drive-strength):

- `:shared` — on a limit pin, allow the GPIO to be declared for an additional motor
  (see [shared_limit_pins.md](shared_limit_pins.md)).
- `:report` — on a user-I/O pin, include its live state in the `?` status report
  (see [user_io_reporting.md](user_io_reporting.md)).
