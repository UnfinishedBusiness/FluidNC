#!/usr/bin/env python3
"""
Bench harness for the firmware-jog "wedge" bug: <Jog>/RUNNING with no motion for
seconds on the first jog after an idle gap.

Root cause (see backlog card cms7wyhkn009801o4rl87dnbd): the Timed step engine's
start_unstep() spun on _stepPulseEndTime, a deadline on the wrapping 32-bit CPU
cycle counter (period ~17.895 s at 240 MHz), and Stepper::stop_stepping() reaches
it with no preceding step().  JogStepper::enter() therefore busy-spun the protocol
main task for up to ~9 s whenever the last step pulse was stale by
T mod 17.895 in (8.95, 17.895) seconds.

This harness exercises exactly that window against a real board:

  for each idle gap T in the sweep:
      sit quiet for T seconds (stepper timer stopped, deadline going stale)
      send $Jog/Start, measure ack latency and press-to-first-motion latency
      $Jog/Stop, wait for Idle

On PRE-fix firmware, trials with T mod 17.895 in (8.95, 17.895) wedge with a
press-to-motion latency of roughly (17.895 - T mod 17.895) seconds; trials in the
lower half never do.  On FIXED firmware every trial moves within tens of ms.
The first trial after boot is a warmup (deadline static-init phase is arbitrary)
and is excluded from the pass/fail verdict.

Usage:
  python3 bench/jog_wedge_bench.py [--port /dev/cu.usbserial-0001]
      [--idles 4,10,12,14,16] [--feed 1000] [--wedge-ms 500]

Exit code 0 = no wedges (bug squashed), 1 = at least one wedge.
The board's config must have the timed engine; checksum mode off/optional/required
all work (every line is sent CRC-framed).
"""
import argparse
import re
import sys
import time
import zlib

import serial

WRAP_S = 2**32 / 240e6  # ccount wrap period at 240 MHz


def crc_frame(line: str) -> bytes:
    return f"{line}*{zlib.crc32(line.encode()) & 0xFFFFFFFF:08X}\n".encode()


STATUS_RE = re.compile(r"<([^|>]+)\|MPos:([-\d.]+),([-\d.]+),([-\d.]+)[^>]*?\|FS:([-\d.]+),")


class Board:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.buf = b""

    def send(self, line: str):
        self.ser.write(crc_frame(line))

    def poll_status(self):
        self.ser.write(b"?")

    def drain(self, seconds=0.3):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ser.read(4096)
        self.buf = b""


def read_available(bd: Board):
    bd.buf += bd.ser.read(4096)
    lines = []
    while b"\n" in bd.buf:
        raw, bd.buf = bd.buf.split(b"\n", 1)
        lines.append(raw.decode("latin1", "replace").strip())
    return lines


def wait_for(bd: Board, pred, timeout, poll=False, poll_every=0.02):
    """Wait until pred(line) returns a truthy value for some received line."""
    end = time.monotonic() + timeout
    next_poll = 0.0
    while time.monotonic() < end:
        if poll and time.monotonic() >= next_poll:
            bd.poll_status()
            next_poll = time.monotonic() + poll_every
        for ln in read_available(bd):
            r = pred(ln)
            if r:
                return r, ln
        time.sleep(0.002)
    return None, None


def parse_status(ln):
    m = STATUS_RE.search(ln)
    if not m:
        return None
    return {
        "state": m.group(1),
        "mpos": (float(m.group(2)), float(m.group(3)), float(m.group(4))),
        "feed": float(m.group(5)),
    }


def get_status(bd: Board, timeout=2.0):
    st, _ = wait_for(bd, parse_status, timeout, poll=True)
    return st


def jog_trial(bd: Board, axis_cmd: str, feed: int, wedge_ms: float):
    """Returns (ack_ms, motion_ms, stop_ms).  motion_ms None = never moved."""
    start_st = get_status(bd)
    if start_st is None:
        raise RuntimeError("no status from board")
    start_pos = start_st["mpos"]
    bd.drain(0.05)

    t0 = time.monotonic()
    bd.send(f"$Jog/Start={axis_cmd} F{feed}")
    ok, _ = wait_for(bd, lambda ln: ln.startswith("ok") or ln.startswith("error"), 20.0)
    ack_ms = (time.monotonic() - t0) * 1000.0
    if not ok:
        raise RuntimeError("no ack for $Jog/Start")

    def moved(ln):
        st = parse_status(ln)
        if not st:
            return None
        if st["feed"] > 0.5:
            return st
        if any(abs(a - b) > 0.01 for a, b in zip(st["mpos"], start_pos)):
            return st
        return None

    st, _ = wait_for(bd, moved, max(15.0, wedge_ms / 1000.0 + 12.0), poll=True)
    motion_ms = (time.monotonic() - t0) * 1000.0 if st else None

    t1 = time.monotonic()
    bd.send("$Jog/Stop")
    idle, _ = wait_for(
        bd, lambda ln: (lambda s: s and s["state"].startswith("Idle"))(parse_status(ln)), 25.0, poll=True
    )
    stop_ms = (time.monotonic() - t1) * 1000.0 if idle else None
    return ack_ms, motion_ms, stop_ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-0001")
    ap.add_argument("--idles", default="4,10,12,14,16", help="comma list of idle gaps (s); sweep repeats in order")
    ap.add_argument("--feed", type=int, default=1000)
    ap.add_argument("--wedge-ms", type=float, default=500.0, help="press-to-motion latency that counts as a wedge")
    args = ap.parse_args()

    idles = [float(x) for x in args.idles.split(",")]
    bd = Board(args.port)
    bd.drain(0.5)

    # Direction probe: find a jog direction that actually moves inside the soft-limit
    # envelope, then alternate it so the position stays near the start.
    st = get_status(bd)
    print(f"board: state={st['state']} mpos={st['mpos']}")
    direction = None
    for cand in ("X-1 Y0 Z0", "X+1 Y0 Z0", "X0 Y-1 Z0", "X0 Y+1 Z0"):
        ack, motion, stop = jog_trial(bd, cand, args.feed, 400)
        print(f"probe {cand}: ack={ack:.0f}ms motion={'%.0fms' % motion if motion else 'NONE'}")
        if motion is not None:
            direction = cand
            break
    if direction is None:
        sys.exit("no jog direction produced motion — check config/soft limits")
    flipped = direction.replace("-1", "#").replace("+1", "-1").replace("#", "+1")
    print(f"using directions: {direction} / {flipped}  (probe trial was the warmup — excluded from verdict)\n")

    wedges = 0
    print(f"{'trial':>5} {'idle_s':>7} {'phase_s':>8} {'predict':>8} {'ack_ms':>7} {'motion_ms':>10} {'stop_ms':>8}  verdict")
    for i, idle in enumerate(idles):
        bd.drain(0.2)
        time.sleep(idle)  # the idle gap: stepper timer stopped, deadline going stale
        phase = idle % WRAP_S
        predict = "WEDGE" if phase > WRAP_S / 2 else "clean"
        cmd = direction if i % 2 == 0 else flipped
        ack, motion, stop = jog_trial(bd, cmd, args.feed, args.wedge_ms)
        wedged = motion is None or motion > args.wedge_ms
        wedges += wedged
        print(
            f"{i:>5} {idle:>7.1f} {phase:>8.2f} {predict:>8} {ack:>7.0f} "
            f"{motion if motion is None else round(motion):>10} "
            f"{stop if stop is None else round(stop):>8}  {'WEDGED' if wedged else 'ok'}"
        )

    print(f"\n{wedges} wedge(s) in {len(idles)} trials -> {'FAIL (bug present)' if wedges else 'PASS (no wedges)'}")
    sys.exit(1 if wedges else 0)


if __name__ == "__main__":
    main()
