# CRC32 serial line integrity (`checksum:`)

Optional CRC32 framing for the primary serial console, for robust g-code streaming
over noisy USB links. It is **off by default** — when disabled, the protocol is
byte-for-byte the classic Grbl/FluidNC line protocol, so existing senders and tools
are unaffected.

When enabled, each line carries a CRC32 so corruption is detected instead of being
silently executed. A sender (e.g. JetCad3 GcodePilot) appends a checksum to every
line it sends and verifies the checksum on every reply.

## Configuration

A single top-level key in `config.yaml` controls the primary serial console:

```yaml
checksum: required     # off | optional | required   (default: off)
```

| Mode | Inbound (host → FluidNC) | Outbound (FluidNC → host) |
|------|--------------------------|---------------------------|
| `off` | no checksum (classic behavior) | no checksum |
| `optional` | verify a checksum **when present**; bare lines still accepted | framed |
| `required` | every line **must** carry a valid checksum | framed |

`optional` is the friendly mode for bring-up: a checksum-aware sender gets full
integrity while a plain terminal (fluidterm, the WebUI console) still works on the
same port. `required` is the strict mode for production streaming.

> The toggle applies to the **primary serial console** (UART0 / USB-CDC), which is
> the connection a g-code sender uses. It is a top-level key because the console
> channel is not a configurable `uart_channel` section.

## Wire format

```
<line>*<8 hex digits>\n
```

- The checksum is the standard **CRC-32 / ISO-HDLC** (the same CRC used by zlib,
  PNG and Ethernet: polynomial `0xEDB88320`, init `0xFFFFFFFF`, final XOR). Canonical
  check value: `crc32("123456789") == 0xCBF43926`.
- It is computed over the line content **before** the `*`, and written as 8
  uppercase hex digits.
- Only a trailing `*` followed by **exactly 8 hex digits** is treated as a checksum,
  so a `*` inside a comment or expression does not get mistaken for one.
- **Realtime characters** (`?`, `~`, `!`, `0x18`, feed/spindle overrides, …) are
  never framed — they bypass line assembly and are sent/received raw, exactly as
  before.

Example: `G1 X10 Y10 F1000` → `G1 X10 Y10 F1000*46BF44C8`.

## Error handling and resend

If an inbound line fails its checksum (or, in `required` mode, has none), FluidNC
replies `error:182` ("Line checksum mismatch") and **does not execute** the line. No
line numbers are needed: because FluidNC acknowledges lines in order, the sender
knows the rejected line is the oldest un-acked one and simply resends it. This rides
on the normal Grbl streaming/flow-control model.

Outbound replies (`ok`, `error:N`, `<…>` status, `[MSG:…]`) are framed in
`optional`/`required` mode so the sender can verify them too.

## Sender requirements

A sender that enables checksum mode must:

1. Append `*<8 hex CRC32>` to every line it sends (any standard `crc32` routine
   produces a matching value — verify against `crc32("123456789") == 0xCBF43926`).
2. Treat `error:182` as "resend the last line."
3. In `required` mode, expect and verify the `*<8 hex>` suffix on every reply.

## Scope and notes

- **Polarity of safety:** default `off` means zero change for everyone who does not
  opt in.
- **Per-channel / native USB-CDC:** v1 applies to the primary `Console`. Extending
  the toggle to individual `uart_channel` sections (pendants) and to a separate
  ESP32-S3 native USB-CDC channel object is a straightforward follow-up.
- **Implementation:** `Crc32.h` (table-less, flash-friendly); inbound verify/strip in
  `Channel::pollLine` → `verifyChecksum`; outbound framing in `Channel::print_msg`;
  toggle applied to `Console` in `main.cpp`. Unit tests pin the canonical CRC value.
