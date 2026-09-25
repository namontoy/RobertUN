#!/usr/bin/env python3
"""
Serial link to a RobertUN wheel node's USART1 console.

This is the transport and the protocol, and nothing else — no experiments, no
policy about what is safe to command. bench.py owns those.

THE ONE HARD PART
-----------------
When `telem on` is running, telemetry lines and command replies share one wire
and interleave freely. A naive "send, then read the reply" loop will swallow
telemetry lines as if they were part of the reply, or worse, mistake a telemetry
line for a reply and return nonsense.

So there is exactly one reader. Every complete line it sees is classified once:
a line starting with "T," goes to the telemetry sink, everything else goes to
the pending-reply buffer. command() drains the same pump while it waits for its
prompt, which is why a command issued mid-stream loses no samples.

THE CONSOLE'S WIRE PROTOCOL
---------------------------
Printable characters are echoed as typed. On CR the firmware emits "\\r\\n",
runs the command, then prints the prompt "> " with no trailing newline
(execute_line() in console.c). So one exchange looks like:

    host:  drv duty 20\\r
    board: drv duty 20\\r\\n  duty +20%\\r\\n  > 

Reading until the accumulated text ends in "> " is therefore the frame
boundary, and the first line back is always the echo of what was sent — which
is worth checking rather than discarding, because a mismatch means bytes were
lost on the way out.

RAW LOG FIRST, PARSE SECOND
---------------------------
Every received byte is appended to the raw log before anything looks at it. A
bug in this file can then cost an analysis, but never a bench run — the run can
be re-parsed from the log offline. Bench time is the expensive thing here.
"""

from __future__ import annotations

import glob
import re
import time
from dataclasses import dataclass

import serial

BAUD = 115200
PROMPT = "> "

# A complete telemetry record, matched ANYWHERE in a line rather than only at
# the start. It has to be this way: the console echoes each typed character as
# its own one-byte write, while a telemetry line is one atomic write, so a
# record routinely lands in the MIDDLE of the echo of a command -
# "drv t" + "T,226,575448,200,...\r\n" + "imeout 2000". Anchoring at the line
# start loses that record and corrupts the command echo behind it.
TELEM_RE = re.compile(r"T,(\d+),(\d+),(-?\d+),(-?\d+),(-?\d+),(\d+),(\d+)")

# Bit meanings in the telemetry `flags` field. Must match print_telem_line()
# in Core/Src/console.c — that is the authority, this is the mirror.
FLAG_SYNC = 0x01      # current is Imotor (phase-synchronised), not Isup
FLAG_ENABLED = 0x02   # nSLEEP high
FLAG_FAULT = 0x04     # nFAULT has been seen low since the last clear
FLAG_SATURATED = 0x08 # the ADC reading hit its ceiling
FLAG_WATCHDOG = 0x10  # the command watchdog has expired since arming


class NodeError(RuntimeError):
    """The board did not answer, or answered something unusable."""


@dataclass(frozen=True)
class Telem:
    """One telemetry line: T,seq,ms,duty,count,mrpm,mA,flags"""

    seq: int
    ms: int             # board time, HAL_GetTick() at emission
    duty: int           # signed per-mille, as commanded
    count: int          # encoder position — the real measurement
    mrpm: int           # rpm x 1000, FILTERED by `enc window`; see .rpm
    ma: int             # Imotor if sync else Isup — check .sync before using
    flags: int
    host_t: float       # host monotonic time at arrival, for cross-checking only

    @property
    def duty_pct(self) -> float:
        return self.duty / 10.0

    @property
    def rpm(self) -> float:
        """Convenience only. This is a boxcar average over `enc window` ticks
        and therefore LAGS the true speed. Differentiate .count instead when
        fitting anything time-dependent — see the note in console.c."""
        return self.mrpm / 1000.0

    @property
    def sync(self) -> bool:
        return bool(self.flags & FLAG_SYNC)

    @property
    def enabled(self) -> bool:
        return bool(self.flags & FLAG_ENABLED)

    @property
    def fault(self) -> bool:
        return bool(self.flags & FLAG_FAULT)

    @property
    def saturated(self) -> bool:
        return bool(self.flags & FLAG_SATURATED)

    @property
    def watchdog(self) -> bool:
        return bool(self.flags & FLAG_WATCHDOG)

    @classmethod
    def parse(cls, line: str, host_t: float) -> "Telem | None":
        parts = line.split(",")
        if len(parts) != 8 or parts[0] != "T":
            return None
        try:
            f = [int(x) for x in parts[1:]]
        except ValueError:
            # A truncated line — the TX ring dropped bytes mid-line. Discarding
            # it is right, and the seq gap it leaves is the point: the host finds
            # out rather than fitting a corrupted sample.
            return None
        return cls(f[0], f[1], f[2], f[3], f[4], f[5], f[6], host_t)


def find_port() -> str:
    """Guess the node's port. Raises if it is not a single obvious choice —
    picking one of several silently is how you drive the wrong board."""
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not candidates:
        raise NodeError(
            "no /dev/ttyUSB* or /dev/ttyACM* found. Is the USB-serial adapter "
            "plugged in? (USART1 is PA9 TX / PA10 RX — the ST-Link/V2 on this "
            "board has no virtual COM port, so a separate adapter is required.)"
        )
    if len(candidates) > 1:
        raise NodeError(
            f"several ports present: {', '.join(candidates)}. Pass --port."
        )
    return candidates[0]


class Node:
    """One open console session. Use as a context manager."""

    def __init__(self, port: str | None = None, raw_log=None, on_telem=None):
        self.port = port or find_port()
        self.raw_log = raw_log          # file object, or None
        self.on_telem = on_telem        # callable(Telem), or None
        self.ser: serial.Serial | None = None
        self._buf = ""                  # bytes seen but not yet a complete line
        self._replies: list[str] = []   # non-telemetry lines awaiting a reader
        self._prompt_seen = False       # a "> " has arrived since the last send
        self._partial = ""              # text a telemetry record cut in half
        self.echo_mismatches = 0        # see command(); not fatal, but counted
        self.telem_count = 0
        self.telem_gaps = 0             # missing seq numbers — dropped lines
        self._last_seq: int | None = None

    # -- lifecycle ---------------------------------------------------------

    def __enter__(self) -> "Node":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def open(self) -> None:
        self.ser = serial.Serial(self.port, BAUD, timeout=0)
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    # -- the single reader -------------------------------------------------

    def pump(self) -> None:
        """Read whatever has arrived, classify every complete line. Cheap and
        safe to call in a tight loop; it never blocks."""
        assert self.ser is not None
        chunk = self.ser.read(4096)
        if not chunk:
            return

        text = chunk.decode("utf-8", errors="replace")
        if self.raw_log is not None:
            self.raw_log.write(text)

        now = time.monotonic()
        self._buf += text

        # Consume the buffer strictly in arrival order. The prompt has to be
        # recognised HERE and not in command(), because it carries no newline:
        # if a telemetry line lands right behind it, the two sit in the buffer
        # as one run of text and an endswith() test would never see the prompt
        # again. Stripping it the moment it reaches the front keeps the line
        # splitting below honest.
        while True:
            if self._buf.startswith(PROMPT):
                self._buf = self._buf[len(PROMPT):]
                self._prompt_seen = True
                continue
            if "\n" not in self._buf:
                break
            line, self._buf = self._buf.split("\n", 1)
            self._consume(line.rstrip("\r"), now)

    def _consume(self, raw: str, now: float) -> None:
        """Pull every telemetry record out of one received line, and put back
        together whatever text those records interrupted.

        The newline that ended this line belongs to the LAST telemetry record
        whenever that record runs to the very end - a record always emits its
        own "\r\n", so the surrounding text was cut in half by it and its other
        half is at the front of the next line. Carrying the remainder forward is
        what reassembles "drv t" + "imeout 2000" into one command echo.
        """
        full = self._partial + raw
        self._partial = ""

        matches = list(TELEM_RE.finditer(full))
        if not matches:
            if full:
                self._replies.append(full)
            return

        residual, last_end = [], 0
        for m in matches:
            residual.append(full[last_end:m.start()])
            last_end = m.end()
            f = [int(g) for g in m.groups()]
            sample = Telem(f[0], f[1], f[2], f[3], f[4], f[5], f[6], now)
            self._note_seq(sample.seq)
            self.telem_count += 1
            if self.on_telem is not None:
                self.on_telem(sample)

        text = "".join(residual) + full[last_end:]
        if last_end == len(full):
            self._partial = text          # the line's newline was the record's
        elif text:
            self._replies.append(text)


    def _note_seq(self, seq: int) -> None:
        # seq restarts at 0 on every `telem on`, so a decrease is a restart,
        # not a gap.
        if self._last_seq is not None and seq > self._last_seq + 1:
            self.telem_gaps += seq - self._last_seq - 1
        self._last_seq = seq

    def idle(self, seconds: float) -> None:
        """Pump for a while, doing nothing else. This is how a dwell is spent —
        never time.sleep(), which would let the OS buffer fill and lose the
        arrival timing of every sample in the gap."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pump()
            time.sleep(0.002)

    # -- commands ----------------------------------------------------------

    def command(self, text: str, timeout: float = 2.0) -> list[str]:
        """Send one command line, return its reply lines (echo removed).

        Telemetry arriving during the exchange is routed to on_telem, not
        returned here and not lost.
        """
        assert self.ser is not None
        self._replies.clear()
        self._prompt_seen = False
        if text.startswith("telem on"):
            self._last_seq = None        # seq restarts at 0 on every `telem on`

        self.ser.write((text + "\r").encode())
        self.ser.flush()
        if self.raw_log is not None:
            self.raw_log.write(f"\n[host] {text}\n")

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            # The prompt is the frame boundary: the firmware prints it only
            # after the command has finished producing output.
            if self._prompt_seen:
                break
            time.sleep(0.002)
        else:
            raise NodeError(
                f"no prompt within {timeout:.1f} s after {text!r}. "
                f"Partial reply: {self._replies!r}"
            )

        lines = self._replies[:]
        self._replies.clear()

        # The first line back should be the echo. Counted, not fatal: an abort
        # here costs a bench run, and the replies are not the measurement - the
        # telemetry stream is. A nonzero count at the end of a run means bytes
        # were lost on the way out and the run deserves a second look.
        if lines and lines[0].strip() == text.strip():
            lines.pop(0)
        elif lines:
            self.echo_mismatches += 1

        return lines

    def ask(self, text: str, timeout: float = 2.0) -> str:
        """command(), joined into one string. For output meant to be read or
        regex'd rather than iterated."""
        return "\n".join(self.command(text, timeout))

    # -- the stop that must always happen ----------------------------------

    def safe_stop(self) -> None:
        """Bring the motor to rest. Coast, never brake — braking from speed
        drives I = E/R through the low-side FETs (drive.h has the table: 50 rpm
        is 3.6 A). Best-effort: each step is attempted even if an earlier one
        raised, because a half-executed stop is the worst outcome."""
        for cmd in ("drv duty 0", "drv coast", "drv disable", "telem off"):
            try:
                self.command(cmd, timeout=1.0)
            except Exception:
                pass
