#!/usr/bin/env python3
"""
Bench runner for the RobertUN wheel node.

    ./bench.py list
    ./bench.py run sweep --duty 5,10,15,20,25,30 --dwell 4
    ./bench.py run sweep --dir ccw
    ./bench.py status                      # last run
    ./bench.py status runs/2026-...._sweep

WHY IT IS SHAPED LIKE THIS
--------------------------
The point of this tool is the run you are NOT watching. So there is no daemon
and no IPC: the runner owns the port for the duration, and everything an
observer wants is a file on disk. `status.json` is rewritten once a second and
is the cheap thing to read when checking in — one small file, rather than
tailing a CSV that is still growing.

SAFETY, IN THE ORDER IT MATTERS
-------------------------------
1. try/finally around everything, plus a SIGTERM handler, so a normal exit, an
   exception, Ctrl-C, or `kill` all end at the same stop sequence.
2. `drv timeout` armed on the board and kicked while dwelling. This is the part
   try/finally CANNOT do: SIGKILL and a yanked USB cable run no Python at all.
   The board stops itself.
3. A duty ceiling, defaulting to the rover's stated operating band.
4. Pre-flight refusal on a latched fault, rather than a run whose conditions
   were already wrong at the start.
5. Post-flight integrity: tx_dropped and seq gaps are recorded, and a run with
   either is marked SUSPECT. A stream with holes must not be quietly fitted.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from node import Node, NodeError, Telem  # noqa: E402

COUNTS_PER_REV = 8403.2          # TIM2 quadrature, at the output shaft
RUNS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")

DEFAULT_MAX_DUTY = 30.0          # percent; the rover runs slow — 30% is the stated ceiling
WATCHDOG_MS = 2000
KICK_INTERVAL = 0.5


class Aborted(Exception):
    """SIGTERM arrived. Exists so the finally-block stop runs for a kill the
    same way it does for Ctrl-C."""


def _on_sigterm(signum, frame):
    raise Aborted("SIGTERM")


# -- run directory ---------------------------------------------------------


class Run:
    """One run directory and the files in it."""

    def __init__(self, profile: str, args: argparse.Namespace):
        stamp = datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
        self.dir = os.path.join(RUNS_DIR, f"{stamp}_{profile}")
        os.makedirs(self.dir, exist_ok=True)
        self.profile = profile
        self.args = args
        self.t0 = time.monotonic()
        self.started_iso = datetime.datetime.now().isoformat(timespec="seconds")

        self.raw = open(os.path.join(self.dir, "console.log"), "w", buffering=1)

        self._telem_f = open(os.path.join(self.dir, "telemetry.csv"), "w", newline="")
        self.telem = csv.writer(self._telem_f)
        self.telem.writerow(
            ["host_s", "seq", "board_ms", "duty_permille", "count", "mrpm", "ma", "flags"]
        )

        self._events_f = open(os.path.join(self.dir, "events.csv"), "w", newline="")
        self.events = csv.writer(self._events_f)
        self.events.writerow(["host_s", "kind", "detail"])

        self.samples: list[Telem] = []
        self.state = "starting"
        self._status_due = 0.0

    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def sink(self, s: Telem) -> None:
        """Called by Node for every telemetry line. Writes through to disk
        immediately AND keeps the sample, so a profile can compute a settled
        figure without re-reading its own file."""
        self.samples.append(s)
        self.telem.writerow(
            [f"{s.host_t - self.t0:.4f}", s.seq, s.ms, s.duty, s.count, s.mrpm, s.ma, s.flags]
        )

    def event(self, kind: str, detail: str = "") -> None:
        self.events.writerow([f"{self.elapsed():.4f}", kind, detail])
        self._events_f.flush()

    def write_meta(self, meta: dict) -> None:
        with open(os.path.join(self.dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

    def write_status(self, node: Node, force: bool = False, **extra) -> None:
        """Rewritten once a second. Deliberately tiny — this is what gets read
        by someone checking in mid-run."""
        now = time.monotonic()
        if not force and now < self._status_due:
            return
        self._status_due = now + 1.0

        last = self.samples[-1] if self.samples else None
        status = {
            "profile": self.profile,
            "state": self.state,
            "elapsed_s": round(self.elapsed(), 1),
            "samples": node.telem_count,
            "seq_gaps": node.telem_gaps,
            "echo_mismatches": node.echo_mismatches,
            "last": None
            if last is None
            else {
                "duty_pct": last.duty_pct,
                "rpm_filtered": last.rpm,
                "mA": last.ma,
                "count": last.count,
                "sync": last.sync,
                "fault": last.fault,
                "saturated": last.saturated,
                "watchdog_expired": last.watchdog,
            },
        }
        status.update(extra)
        # Flush the telemetry buffer on the same beat. A SIGKILL runs no Python,
        # so whatever is still sitting in stdio is lost; at 8 kB of default
        # buffering that was ~2 s of samples. console.log is line-buffered and
        # keeps them regardless, but the CSV should not need recovering from it.
        self._telem_f.flush()
        # Write-then-rename, so a reader never catches a half-written file.
        tmp = os.path.join(self.dir, "status.json.tmp")
        with open(tmp, "w") as f:
            json.dump(status, f, indent=2)
        os.replace(tmp, os.path.join(self.dir, "status.json"))

    def close(self) -> None:
        self._telem_f.flush()
        self._telem_f.close()
        self._events_f.close()
        self.raw.close()


# -- shared mechanics ------------------------------------------------------


def dwell(node: Node, run: Run, seconds: float, **status_extra) -> None:
    """Spend time. Pumps the serial link, kicks the board watchdog, and keeps
    status.json current. Never time.sleep() alone — that would let the OS
    buffer fill and destroy the arrival timing of everything in the gap."""
    end = time.monotonic() + seconds
    next_kick = time.monotonic() + KICK_INTERVAL
    while True:
        node.pump()
        now = time.monotonic()
        if now >= end:
            break
        if now >= next_kick:
            node.command(f"drv timeout {WATCHDOG_MS}", timeout=1.0)
            next_kick = now + KICK_INTERVAL
        run.write_status(node, **status_extra)
        time.sleep(0.002)


def ramp(node: Node, run: Run, start: float, end: float, rate: float,
         hold_s: float = 0.0) -> None:
    """Walk duty from `start` to `end` at `rate` percent per second.

    A host-side stand-in for the firmware slew-rate limiter, which is still
    owed. Two honest limitations: `drv duty` takes integer PERCENT, so the
    finest available step is 1% and this is a staircase rather than a ramp;
    and each step costs a console round-trip, so the achieved rate is a little
    slower than the one asked for. The firmware could do both properly on the
    1 kHz tick at per-mille resolution.

    It exists because a step to a high duty from rest demands 12 V / 1.87 Ohm
    at the instant of the step, which is what fired the trip on every duty step
    in the Sep 23 session.

    `start` is stepped to directly and held: below breakaway the wheel does not
    move at all, so there is nothing for a ramp to do down there — it has to be
    crossed in one step, and then the ramp begins from a turning wheel.

    Telemetry through the ramp is recorded like any other. A ramp slow against
    the mechanical time constant is a quasi-static sweep in its own right, and
    can be read back against the settled points.
    """
    run.state = f"ramp {start:+.0f}% -> {end:+.0f}%"
    node.command(f"drv duty {int(round(start))}")
    run.event("ramp_start", f"{start:+.0f} -> {end:+.0f} @ {rate:g}%/s")
    if hold_s > 0:
        dwell(node, run, hold_s, point=f"ramp hold {start:+.0f}%")

    d, target = int(round(start)), int(round(end))
    step = 1 if target >= d else -1
    interval = 1.0 / rate if rate > 0 else 0.0
    while d != target:
        d += step
        node.command(f"drv duty {d}")
        if interval:
            dwell(node, run, interval, point=f"ramp {d:+.0f}%")
    run.event("ramp_end", f"{end:+.0f}")


def rpm_from_counts(samples: list[Telem]) -> float | None:
    """Velocity by least-squares slope of count against board time.

    `count` is the real measurement: exact and unfiltered. The `mrpm` column is
    a boxcar average over `enc window` ticks and lags the truth, so fitting it
    would partly measure the filter. Board `ms` is used rather than host arrival
    time because it is stamped at emission, before any USB or OS latency.
    """
    pts = [(s.ms, s.count) for s in samples]
    if len(pts) < 3:
        return None
    n = len(pts)
    mt = sum(p[0] for p in pts) / n
    mc = sum(p[1] for p in pts) / n
    num = sum((t - mt) * (c - mc) for t, c in pts)
    den = sum((t - mt) ** 2 for t, c in pts)
    if den == 0:
        return None
    counts_per_ms = num / den
    return counts_per_ms * 1000.0 * 60.0 / COUNTS_PER_REV


def preflight(node: Node, run: Run, expect_duty_zero: bool = True) -> dict:
    """Record the conditions rather than trusting them, and refuse to start on
    a latched fault.

    The fault check reads the telemetry `flags` bit instead of parsing the human
    `drv` text — the bit is defined by the line format this tool owns, so it
    cannot drift with a wording change in console.c.

    THE ORDER HERE IS THE WHOLE POINT. drive_faulted() reads the nFAULT pin
    directly, and the DRV8874 holds nFAULT low the entire time nSLEEP is low.
    So a freshly reset board ALWAYS reports a latched fault, and a check made
    before waking the driver would refuse every run on an artifact. Wake it,
    clear the latch, and only then look: a fault that re-asserts while the
    driver is awake and the duty is zero is a real one.
    """
    node.command("monitor off")      # CAN frame lines would interleave
    node.command("heartbeat off")

    conditions = {name: node.ask(name) for name in ("info", "cfg", "drv", "enc")}
    run.event("preflight", "captured info/cfg/drv/enc")

    node.command("drv enable")
    node.command("drv clearfault")
    run.event("preflight", "driver awake, fault latch cleared")

    node.command("telem rate 10")
    node.command("telem on")
    dwell_end = time.monotonic() + 0.8
    while time.monotonic() < dwell_end:
        node.pump()
        time.sleep(0.002)
    node.command("telem off")

    if not run.samples:
        raise NodeError(
            "no telemetry received during pre-flight. Either the firmware "
            "predates the `telem` command (reflash needed) or RX is not getting "
            "through."
        )
    s = run.samples[-1]
    if s.fault:
        raise NodeError(
            "nFAULT re-asserted with the driver awake and the duty at zero. "
            "That is a real fault, not the sleep-mode artifact — find out why "
            "before running. A run started into a fault produces data whose "
            "conditions were already wrong."
        )
    if expect_duty_zero and s.duty != 0:
        raise NodeError(f"motor is already commanded to {s.duty_pct:+.1f}% — stop it first.")

    run.samples.clear()             # pre-flight samples are not run data
    return conditions


# -- profiles --------------------------------------------------------------


def profile_sweep(node: Node, run: Run, a: argparse.Namespace) -> dict:
    """Duty list, dwell at each, record settled speed and current.

    This is the task-17 table, automated — and it is how the CCW direction gets
    taken. Its acceptance test is that its CW numbers reproduce the hand-taken
    Sep 23 table.
    """
    duties = [float(x) for x in a.duty.split(",")]
    sign = -1.0 if a.dir == "ccw" else 1.0

    over = [d for d in duties if abs(d) > a.max_duty]
    if a.ramp_from is not None and abs(a.ramp_from) > a.max_duty:
        over.append(a.ramp_from)
    if over:
        raise NodeError(
            f"duty {over} exceeds the {a.max_duty:.0f}% ceiling. Raise it "
            f"explicitly with --max-duty if that is really intended."
        )

    node.command(f"enc window {a.window}")
    # The boot default is 999 mA, which is BELOW the from-rest stall current
    # (12 V / 1.87 Ohm = 6.4 A demanded at the instant of a step), so current
    # regulation would chop every breakaway. Set it explicitly and record it,
    # rather than depending on whatever the last session left in RAM.
    node.command(f"drv trip {a.trip}")
    node.command(f"drv timeout {WATCHDOG_MS}")
    node.command("drv decay slow")   # the driver was woken during pre-flight
    node.command(f"telem rate {a.rate}")
    node.command("telem on")
    run.event("telem_on", f"{a.rate} Hz")

    # The entry ramp runs after `telem on`, so the ramp is in the data rather
    # than in the gap before it.
    if a.ramp_from is not None:
        ramp(node, run, sign * a.ramp_from, sign * duties[0], a.ramp, hold_s=1.0)

    results = []
    for duty in duties:
        commanded = sign * duty
        run.state = f"duty {commanded:+.0f}%"
        # `drv duty` takes integer PERCENT and multiplies by 10 internally, so
        # the console's resolution is 1% even though drive_set_duty() is
        # per-mille. Fractional duties are not commandable today.
        node.command(f"drv duty {int(round(commanded))}")
        run.event("duty", f"{commanded:+.0f}")

        mark = len(run.samples)
        dwell(node, run, a.dwell, point=f"{commanded:+.0f}%")

        # Fit only the settled tail. The leading fraction is the transient,
        # and including it would bias the slope low on every ascending step.
        window = run.samples[mark:]
        tail = window[int(len(window) * a.settle):]
        rpm = rpm_from_counts(tail)
        ma = [s.ma for s in tail]
        sync = all(s.sync for s in tail) if tail else False

        row = {
            "duty_pct": commanded,
            "rpm": None if rpm is None else round(rpm, 3),
            "rpm_filtered": round(sum(s.rpm for s in tail) / len(tail), 3) if tail else None,
            "mA_mean": round(sum(ma) / len(ma), 1) if ma else None,
            "mA_min": min(ma) if ma else None,
            "mA_max": max(ma) if ma else None,
            # Below 14.5% duty the phase-synchronised window does not exist, so
            # the current figure is supply current, not motor current. Recorded
            # per point because it changes partway up any sweep that starts low.
            "current_is_motor": sync,
            "samples": len(tail),
        }
        results.append(row)
        run.event("point", json.dumps(row))
        print(
            f"  {commanded:+6.1f}%  {row['rpm'] if row['rpm'] is not None else float('nan'):7.2f} rpm"
            f"  {row['mA_mean']:6.1f} mA  {'Imot' if sync else 'Isup'}  n={row['samples']}"
        )

    run.state = "stopping"
    # The braking half. safe_stop() in the runner's finally block stays a hard
    # stop on purpose — that is the emergency path, and it must not be slow.
    if a.ramp and duties:
        ramp(node, run, sign * duties[-1], 0.0, a.ramp)
    node.command("drv duty 0")
    node.command("telem off")

    with open(os.path.join(run.dir, "sweep.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)

    return {"points": results}


PROFILES = {
    "sweep": (
        profile_sweep,
        "duty list, dwell at each, settled rpm + current — the task-17 table, automated",
    ),
}


# -- driver ----------------------------------------------------------------


def _meta(a, run, node, outcome, error, conditions, dropped, suspect, result=None) -> dict:
    """The run's own record of itself. Written once at connect and again at the
    end, so an interrupted run still says what it was and what it ran under."""
    return {
        "profile": a.profile,
        "args": {k: v for k, v in vars(a).items() if k != "func"},
        "note": getattr(a, "note", None),
        "started": run.started_iso,
        "duration_s": round(run.elapsed(), 1),
        "outcome": outcome,
        "error": error,
        "counts_per_rev": COUNTS_PER_REV,
        "samples": node.telem_count,
        "seq_gaps": node.telem_gaps,
        "echo_mismatches": node.echo_mismatches,
        "tx_dropped": dropped,
        "suspect": suspect,
        "conditions_at_connect": conditions,
        "result": result or {},
    }


def cmd_run(a: argparse.Namespace) -> int:
    fn, _ = PROFILES[a.profile]
    run = Run(a.profile, a)
    print(f"run: {run.dir}")

    signal.signal(signal.SIGTERM, _on_sigterm)

    node = Node(port=a.port, raw_log=run.raw, on_telem=run.sink)
    outcome, error = "ok", None
    result: dict = {}
    conditions: dict = {}
    suspect = False

    try:
        node.open()
        run.event("connect", node.port)
        conditions = preflight(node, run)
        # Provisional meta, written before the first duty command. The end-of-run
        # write below supersedes it; this one exists so that a run that never
        # reaches the end — SIGKILL, yanked cable, power loss — still carries the
        # conditions it ran under. Conditions are known now; the outcome is not.
        run.write_meta(_meta(a, run, node, "running", None, conditions, None, False))
        run.state = "running"
        result = fn(node, run, a)
    except (Aborted, KeyboardInterrupt) as e:
        outcome, error = "aborted", str(e) or type(e).__name__
        print(f"\naborted: {error}", file=sys.stderr)
    except Exception as e:
        outcome, error = "error", f"{type(e).__name__}: {e}"
        print(f"\nerror: {error}", file=sys.stderr)
    finally:
        # The stop runs on every path. Best-effort by design: if the link is
        # already gone, the board watchdog is what stops the motor.
        run.state = "stopped"
        try:
            node.safe_stop()
            run.event("safe_stop", "duty 0, coast, disable")
        except Exception as e:
            run.event("safe_stop_failed", str(e))

        # Post-flight integrity. A run whose stream had holes is marked, not
        # silently accepted.
        dropped = None
        try:
            stats = node.ask("stats")
            run.event("poststats", stats.replace("\n", " | "))
            for line in stats.splitlines():
                if "dropped" in line.lower():
                    digits = "".join(c if c.isdigit() else " " for c in line).split()
                    if digits:
                        dropped = int(digits[-1])
        except Exception:
            pass

        suspect = bool(node.telem_gaps) or bool(dropped)
        run.write_meta(
            _meta(a, run, node, outcome, error, conditions, dropped, suspect, result)
        )
        run.write_status(node, force=True, outcome=outcome, suspect=suspect)
        run.close()
        node.close()

    if suspect:
        print(
            f"\nSUSPECT: {node.telem_gaps} seq gaps, tx_dropped={dropped}. "
            "The stream has holes — do not fit this run without looking at why.",
            file=sys.stderr,
        )
    print(f"{outcome}: {run.dir}")
    return 0 if outcome == "ok" else 1


def cmd_status(a: argparse.Namespace) -> int:
    path = a.run
    if path is None:
        runs = sorted(d for d in os.listdir(RUNS_DIR)) if os.path.isdir(RUNS_DIR) else []
        if not runs:
            print("no runs yet", file=sys.stderr)
            return 1
        path = os.path.join(RUNS_DIR, runs[-1])
    f = os.path.join(path, "status.json")
    if not os.path.exists(f):
        print(f"no status.json in {path}", file=sys.stderr)
        return 1
    print(os.path.basename(path))
    print(open(f).read())
    return 0


def cmd_list(a: argparse.Namespace) -> int:
    print("profiles:")
    for name, (_, doc) in PROFILES.items():
        print(f"  {name:12s} {doc}")
    if os.path.isdir(RUNS_DIR):
        runs = sorted(os.listdir(RUNS_DIR))
        if runs:
            print("\nruns:")
            for r in runs[-10:]:
                meta = os.path.join(RUNS_DIR, r, "meta.json")
                tag = ""
                if os.path.exists(meta):
                    m = json.load(open(meta))
                    tag = f"  [{m.get('outcome')}{', SUSPECT' if m.get('suspect') else ''}]"
                print(f"  {r}{tag}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="run a profile")
    r.add_argument("profile", choices=sorted(PROFILES))
    r.add_argument("--port", default=None, help="serial device (autodetected if omitted)")
    r.add_argument("--note", default=None,
                   help="free text recorded in meta.json — the physical setup this run "
                        "was taken under (rig, load, wheel mounting, rail). The board "
                        "cannot know any of it, so if it is not typed it is not recorded.")
    # The rover runs slow: characterisation stays at or below 30% duty, walked
    # in 2% steps so breakaway and the band's curvature are resolved rather than
    # bracketed. Above ~31% the wheel starts to bounce on the rig belt, which
    # makes those points a measurement of the rig and not of the plant.
    r.add_argument("--duty", default="5,7,9,11,13,15,17,19,21,23,25,27,29",
                   help="comma-separated duty percents (default: 5-29%% in 2%% steps)")
    r.add_argument("--ramp", type=float, default=0.0,
                   help="duty slew rate, percent per second, for the entry and exit "
                        "ramps. 0 steps straight to the setpoint")
    r.add_argument("--ramp-from", type=float, default=None,
                   help="duty the entry ramp starts at. Stepped to directly and held "
                        "1 s, because the wheel must break away before a ramp means "
                        "anything — set it just above breakaway")
    r.add_argument("--dir", choices=("cw", "ccw"), default="cw")
    r.add_argument("--dwell", type=float, default=4.0, help="seconds at each point")
    r.add_argument("--settle", type=float, default=0.5,
                   help="fraction of each dwell discarded as transient")
    r.add_argument("--rate", type=int, default=50, help="telemetry Hz (1..100)")
    r.add_argument("--window", type=int, default=100, help="enc window, in 1 ms ticks")
    r.add_argument("--trip", type=int, default=1580,
                   help="current-regulation trip in mA (boot default 999 is below stall)")
    r.add_argument("--max-duty", type=float, default=DEFAULT_MAX_DUTY,
                   help="refuse any duty above this (percent)")
    r.set_defaults(func=cmd_run)

    s = sub.add_parser("status", help="print a run's status.json (default: latest)")
    s.add_argument("run", nargs="?", default=None)
    s.set_defaults(func=cmd_status)

    l = sub.add_parser("list", help="list profiles and recent runs")
    l.set_defaults(func=cmd_list)

    a = p.parse_args()
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
