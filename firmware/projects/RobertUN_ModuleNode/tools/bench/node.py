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
a "T," record goes to the telemetry sink, a "V," record to the velocity sink,
everything else goes to the pending-reply buffer. command() drains the same
pump while it waits for its prompt, which is why a command issued mid-stream
loses no samples.

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
_TELEM_PAT = r"T,(\d+),(\d+),(-?\d+),(-?\d+),(-?\d+),(\d+),(\d+)"

# The velocity channel — one record per CONTROL STEP, emitted by
# print_velocity_line() when `telem vel on`. A separate record type rather than
# more columns on T: Telem.parse() below rejects anything without exactly eight
# fields, and every run directory already committed holds a telemetry.csv with
# the seven-field header, so a widened T would mean two incompatible things
# depending on which reader saw it.
_VELOC_PAT = (r"V,(\d+),(\d+),(-?\d+),(-?\d+),(-?\d+),"
              r"(-?\d+),(-?\d+),(-?\d+),(-?\d+),(\d+)")

# ONE alternation, not two finditers. _dispatch() rebuilds the command echo from
# the text BETWEEN matches, which requires them to arrive ordered and
# non-overlapping; a single regex guarantees that, and merging two independent
# iterators only appears to. Dispatch on the leading letter.
RECORD_RE = re.compile(f"(?:{_TELEM_PAT})|(?:{_VELOC_PAT})")

# Kept as its own name because analysis scripts import it.
TELEM_RE = re.compile(_TELEM_PAT)
VELOC_RE = re.compile(_VELOC_PAT)

# Bit meanings in the telemetry `flags` field. Must match print_telem_line()
# in Core/Src/console.c — that is the authority, this is the mirror.
FLAG_SYNC = 0x01      # current is Imotor (phase-synchronised), not Isup
FLAG_ENABLED = 0x02   # nSLEEP high
FLAG_FAULT = 0x04     # nFAULT has been seen low since the last clear
FLAG_SATURATED = 0x08 # the ADC reading hit its ceiling
FLAG_WATCHDOG = 0x10  # the command watchdog has expired since arming
FLAG_DECAY = 0x20     # current sampled in the slow-decay brake phase (<14.5% duty)

# Bit meanings in the VELOCITY `flags` field — a different set on a different
# record. print_velocity_line() in console.c is the authority; this is the
# mirror. VFLAG_FROZEN without SLEWING or NOBRIDGE means the integrator was
# held because the output was clamped and the error pushed further into the
# clamp: that is anti-windup working, and it wants no correction. FROZEN with
# either of those two means something outside the loop held it off.
VFLAG_SATURATED = 0x01  # output hit min(vel_max, drv limit)
VFLAG_FROZEN = 0x02     # the integrator did not advance this step
VFLAG_SLEWING = 0x04    # ...because drive_slewing() — drv ramp is fighting it
VFLAG_NOBRIDGE = 0x08   # ...because the bridge is disabled or faulted
VFLAG_WATCHDOG = 0x10   # the SETPOINT watchdog has expired since arming
VFLAG_RAMPING = 0x20    # ramped setpoint has not reached the commanded one
VFLAG_MISSED = 0x40     # a control step was lost before this line — see below


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

    @property
    def decay(self) -> bool:
        """mA came from the brake phase, scaled to motor current (+/-4%)."""
        return bool(self.flags & FLAG_DECAY)

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


@dataclass(frozen=True)
class Veloc:
    """One control step: V,seq,ms,sp_mrpm,meas_mrpm,out,ff,p,i,d,flags

    This is the CONTROLLER's record, and it is paced by the control loop
    (1000 / `enc window` Hz), not by `telem rate`. Join it to the Telem stream
    on `.ms` when you need current or encoder count alongside it.
    """

    seq: int
    ms: int             # board time, HAL_GetTick() at the step
    sp_mrpm: int        # the RAMPED setpoint — what the loop actually chased
    meas_mrpm: int      # what the loop measured; see .meas_rpm
    out: int            # per-mille written to drive_set_duty()
    ff: int             # feedforward contribution, per-mille
    p: int              # proportional
    i: int              # integrator
    d: int              # derivative
    flags: int
    host_t: float       # host monotonic time at arrival, for cross-checking only

    @property
    def sp_rpm(self) -> float:
        return self.sp_mrpm / 1000.0

    @property
    def meas_rpm(self) -> float:
        """What the LOOP saw — deliberately the boxcar-filtered figure, because
        that is what the controller acted on. Judge the loop with this; fit the
        PLANT from Telem.count, which is exact and unfiltered."""
        return self.meas_mrpm / 1000.0

    @property
    def error_rpm(self) -> float:
        return (self.sp_mrpm - self.meas_mrpm) / 1000.0

    @property
    def saturated(self) -> bool:
        return bool(self.flags & VFLAG_SATURATED)

    @property
    def frozen(self) -> bool:
        return bool(self.flags & VFLAG_FROZEN)

    @property
    def slewing(self) -> bool:
        return bool(self.flags & VFLAG_SLEWING)

    @property
    def nobridge(self) -> bool:
        return bool(self.flags & VFLAG_NOBRIDGE)

    @property
    def watchdog(self) -> bool:
        return bool(self.flags & VFLAG_WATCHDOG)

    @property
    def ramping(self) -> bool:
        return bool(self.flags & VFLAG_RAMPING)

    @property
    def missed(self) -> bool:
        """A control step was published and overwritten before the board's main
        loop could send it. The HOST is not at fault — this happens inside the
        firmware — but the stream is decimated, which looks exactly like a slow
        control loop if you do not check this."""
        return bool(self.flags & VFLAG_MISSED)

    @property
    def freeze_reason(self) -> str | None:
        if not self.frozen:
            return None
        if self.nobridge:
            return "no_bridge"
        if self.slewing:
            return "drv_slewing"
        return "anti_windup"     # clamped, and the error pushed further in


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

    def __init__(self, port: str | None = None, raw_log=None, on_telem=None,
                 on_veloc=None):
        self.port = port or find_port()
        self.raw_log = raw_log          # file object, or None
        self.on_telem = on_telem        # callable(Telem), or None
        self.on_veloc = on_veloc        # callable(Veloc), or None
        self.ser: serial.Serial | None = None
        self._buf = ""                  # bytes seen but not yet a complete line
        self._replies: list[str] = []   # non-telemetry lines awaiting a reader
        self._prompt_seen = False       # a "> " has arrived since the last send
        self._partial = ""              # text a telemetry record cut in half
        self.echo_mismatches = 0        # see command(); not fatal, but counted
        self.telem_count = 0
        self.telem_gaps = 0             # missing seq numbers — dropped lines
        self._last_seq: int | None = None
        self.veloc_count = 0
        self.veloc_gaps = 0             # missing V seq — lines lost on the wire
        self.veloc_steps_missed = 0     # V lines carrying VFLAG_MISSED — steps
                                        # lost inside the firmware, a different
                                        # failure from a gap and worth its own
                                        # counter
        self._last_vseq: int | None = None

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

        matches = list(RECORD_RE.finditer(full))
        if not matches:
            if full:
                self._replies.append(full)
            return

        residual, last_end = [], 0
        for m in matches:
            residual.append(full[last_end:m.start()])
            last_end = m.end()
            # One alternation matched, so exactly one arm's groups are non-None.
            # The leading letter says which, and is cheaper and clearer than
            # counting Nones.
            f = [int(g) for g in m.groups() if g is not None]
            if m.group(0)[0] == "T":
                sample = Telem(f[0], f[1], f[2], f[3], f[4], f[5], f[6], now)
                self._note_seq(sample.seq)
                self.telem_count += 1
                if self.on_telem is not None:
                    self.on_telem(sample)
            else:
                v = Veloc(f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8],
                          f[9], now)
                self._note_vseq(v.seq)
                self.veloc_count += 1
                if v.missed:
                    self.veloc_steps_missed += 1
                if self.on_veloc is not None:
                    self.on_veloc(v)

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

    def _note_vseq(self, seq: int) -> None:
        # Same rule as _note_seq: V's seq restarts at 0 on `telem on` too, so a
        # decrease is a restart and not a gap.
        if self._last_vseq is not None and seq > self._last_vseq + 1:
            self.veloc_gaps += seq - self._last_vseq - 1
        self._last_vseq = seq

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
            self._last_vseq = None       # ...and so does the velocity channel's

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
        raised, because a half-executed stop is the worst outcome.

        `vel off` COMES FIRST, and that ordering is not cosmetic. With the
        velocity loop armed, drive_set_duty() is called fifty times a second
        from the board's own tick, so "drv duty 0" and "drv coast" are both
        overwritten roughly 20 ms after they land. `drv disable` would still
        cut nSLEEP and stop the motor, but a stop sequence whose first three
        steps are silently undone is a stop sequence that works by accident.

        The general form of this, worth remembering at the next layer up:
        arming a control loop invalidates every stop sequence that addresses
        the layer below it. The loop has to be disarmed first, or not at all.
        """
        for cmd in ("vel off", "drv duty 0", "drv coast", "drv disable",
                    "telem off"):
            try:
                self.command(cmd, timeout=1.0)
            except Exception:
                pass
