#!/usr/bin/env python3
"""
Bench runner for the RobertUN wheel node.

    ./bench.py list
    ./bench.py run sweep --duty 5,10,15,20,25,30 --dwell 4
    ./bench.py run sweep --dir ccw
    ./bench.py run step --rpm 20           # velocity-loop step response
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
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from node import Node, NodeError, Telem, Veloc  # noqa: E402

COUNTS_PER_REV = 8403.2          # TIM2 quadrature, at the output shaft
RUNS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")

DEFAULT_MAX_DUTY = 30.0          # percent; the rover runs slow — 30% is the stated ceiling
# The setpoint ceiling is the duty ceiling pushed through the plant fit:
# 0.7993 x 300 o/oo / 10 - 2.42 = 21.6 rpm at 30% duty. 22 is that, rounded up.
# Ask for more only the way --max-duty is asked for: explicitly.
DEFAULT_MAX_RPM = 22.0
WATCHDOG_MS = 2000
KICK_INTERVAL = 0.5
# How long after the setpoint stops moving an overshoot peak is still an
# overshoot. The loaded plant's slow pole is 2.75 s (figures/plot_plant_12v_tau.py),
# so everything the loop is going to do it has done by 3 s. The bound matters
# because the peak is a MAXIMUM: search a longer window and the +/-1 rpm
# mechanical ripple alone hands back a larger "overshoot", which would make the
# number a function of --dwell rather than of the gains.
OVERSHOOT_WINDOW_S = 3.0


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

        # The V channel gets its own file rather than more columns on
        # telemetry.csv: the two streams are sampled on different clocks (T on
        # the telem timer, V once per control step) and every committed run
        # directory holds a 7-field telemetry.csv. Joining them is the
        # analysis's job, on board_ms, and only when an analysis wants both.
        self._veloc_f = open(os.path.join(self.dir, "velocity.csv"), "w", newline="")
        self.veloc = csv.writer(self._veloc_f)
        self.veloc.writerow(
            ["host_s", "seq", "board_ms", "sp_mrpm", "meas_mrpm",
             "out", "ff", "p", "i", "d", "flags"]
        )

        self._events_f = open(os.path.join(self.dir, "events.csv"), "w", newline="")
        self.events = csv.writer(self._events_f)
        self.events.writerow(["host_s", "kind", "detail"])

        self.samples: list[Telem] = []
        self.velocs: list[Veloc] = []
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

    def veloc_sink(self, v: Veloc) -> None:
        """Called by Node for every V line. Same contract as sink(): written
        through immediately, and kept so a profile can compute its metrics
        without re-reading its own file."""
        self.velocs.append(v)
        self.veloc.writerow(
            [f"{v.host_t - self.t0:.4f}", v.seq, v.ms, v.sp_mrpm, v.meas_mrpm,
             v.out, v.ff, v.p, v.i, v.d, v.flags]
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
            "veloc_samples": node.veloc_count,
            "veloc_gaps": node.veloc_gaps,
            "veloc_steps_missed": node.veloc_steps_missed,
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
        self._veloc_f.flush()
        # Write-then-rename, so a reader never catches a half-written file.
        tmp = os.path.join(self.dir, "status.json.tmp")
        with open(tmp, "w") as f:
            json.dump(status, f, indent=2)
        os.replace(tmp, os.path.join(self.dir, "status.json"))

    def close(self) -> None:
        self._telem_f.flush()
        self._telem_f.close()
        self._veloc_f.flush()
        self._veloc_f.close()
        self._events_f.close()
        self.raw.close()


# -- shared mechanics ------------------------------------------------------


def dwell(node: Node, run: Run, seconds: float, vel_kick: int | None = None,
          **status_extra) -> None:
    """Spend time. Pumps the serial link, kicks the board watchdogs, and keeps
    status.json current. Never time.sleep() alone — that would let the OS
    buffer fill and destroy the arrival timing of everything in the gap.

    `vel_kick` is the current setpoint in MILLI-rpm, and must be passed for
    every dwell taken with the velocity loop armed. The loop carries its own
    setpoint watchdog (`vel_tmo`, 1000 ms by default) and **only `vel target`
    refreshes it** — `drv timeout` kicks the layer below and does nothing for
    it. Without this a 6 s dwell coasts the wheel one second in, in the middle
    of the measurement, and the data looks like a plant that cannot hold speed.
    """
    end = time.monotonic() + seconds
    next_kick = time.monotonic() + KICK_INTERVAL
    while True:
        node.pump()
        now = time.monotonic()
        if now >= end:
            break
        if now >= next_kick:
            node.command(f"drv timeout {WATCHDOG_MS}", timeout=1.0)
            if vel_kick is not None:
                # Re-commanding the same setpoint is a no-op to the loop other
                # than refreshing the countdown: velocity_set_setpoint() writes the
                # target and refreshes the countdown, and touches nothing else.
                node.command(f"vel target {vel_kick}m", timeout=1.0)
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
    run.velocs.clear()
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

    window = 100 if a.window is None else a.window
    a.window = window               # so meta.json records what actually ran
    node.command(f"enc window {window}")
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
        run.state = f"duty {commanded:+.1f}%"
        # `drv duty <n>p` commands per-mille directly, so fractional percent
        # duties (0.1% resolution) reach drive_set_duty() unrounded.
        node.command(f"drv duty {int(round(commanded * 10))}p")
        run.event("duty", f"{commanded:+.1f}")

        mark = len(run.samples)
        dwell(node, run, a.dwell, point=f"{commanded:+.1f}%")

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


def step_metrics(rows: list[Veloc], from_rpm: float, to_rpm: float) -> dict:
    """Reduce one step segment to the four numbers a tuning decision is made
    from, plus the three that say whether those numbers mean anything.

    READ THE CAVEAT BEFORE READING THE NUMBERS. Every timing here is computed
    from `meas_mrpm`, which is encoder_rpm() — the boxcar over `enc window`
    ticks. That is deliberate: it is what the controller actually saw and acted
    on, so these figures describe the closed loop as the loop experienced it,
    which is the right frame for choosing gains. It is NOT the frame for a
    plant time constant: the same warning that governs rpm_from_counts() applies
    with more force inside a loop, because the filter's lag is now inside the
    feedback path. Plant-side timing comes from the T rows and `count`.

    Time is board `ms`, relative to the first row after the step command, so
    host scheduling and USB latency are outside every figure below.

    THE SLEW ANCHOR — why overshoot and settling are not measured from the
    command. `vel_slew` ramps the SETPOINT, and it ships at 4 rpm/s, so a
    0 -> 10 rpm "step" spends its first 2.5 s with the loop tracking a moving
    target. Nothing in that window is a step response: there is no step for the
    loop to overshoot, and a rise time measured across it is a division of the
    step size by the slew rate with the controller barely involved. The first
    version of this function anchored everything at the command instant and so
    reported the slew limiter's properties under the loop's name — a 0 -> 10 rpm
    step returned "rise 2.0 s" no matter what Kp was, which is the tell.

    So the segment is split at the instant the RAMPING flag clears, which is
    when the setpoint stops moving and the loop is first regulating to a fixed
    number:

      during the ramp   `track_lag_rpm` — how far behind the moving setpoint
                        the loop runs. This is the real measure of loop
                        bandwidth when a ramp is in force, and it is the same
                        estimator the Sep 25 tau figure used on the entry ramp.
      after the ramp    `overshoot_pct`, `settle_s` — anchored at ramp end,
                        because that is the only part that is a regulation
                        problem.

    `rise_s` is still reported from the command instant, because that is the
    conventional definition and it is a true statement about the response. It
    is reported NEXT TO `rise_slew_floor_s`, the rise time the ramp alone would
    produce against an infinitely fast loop: when the two are close, `rise_s`
    measured the ramp, and `ramp_limited` says so outright rather than leaving
    it to be noticed. Run with `--slew 0` to measure the loop's own rise.
    """
    out: dict = {
        "from_rpm": round(from_rpm, 3),
        "to_rpm": round(to_rpm, 3),
        "samples": len(rows),
    }
    if not rows:
        return out

    t0 = rows[0].ms
    t = [(r.ms - t0) / 1000.0 for r in rows]
    y = [r.meas_rpm for r in rows]
    change = to_rpm - from_rpm
    sign = 1.0 if change >= 0 else -1.0

    # Health of the segment first. These are fractions of the steps taken, and
    # they are what says whether the timing figures are describing the
    # controller or describing something that got in its way.
    n = float(len(rows))
    frozen = [r for r in rows if r.frozen]
    out["sat_fraction"] = round(sum(1 for r in rows if r.saturated) / n, 4)
    out["freeze_fraction"] = round(len(frozen) / n, 4)
    # Split by reason, because they want opposite corrections. Anti-windup
    # freezing under saturation is the mechanism working; freezing because the
    # bridge is unavailable or drv's own slew limiter is active means the run
    # measured the obstruction, not the loop.
    out["freeze_antiwindup"] = round(
        sum(1 for r in frozen if not r.slewing and not r.nobridge) / n, 4)
    out["freeze_drv_slewing"] = round(sum(1 for r in frozen if r.slewing) / n, 4)
    out["freeze_no_bridge"] = round(sum(1 for r in frozen if r.nobridge) / n, 4)
    out["watchdog_expired"] = any(r.watchdog for r in rows)
    out["steps_missed"] = sum(1 for r in rows if r.missed)

    # The settled tail: the last quarter of the segment.
    tail = rows[int(len(rows) * 0.75):] or rows[-1:]
    mean_tail = sum(r.meas_rpm for r in tail) / len(tail)
    out["final_rpm"] = round(mean_tail, 3)
    out["ss_error_rpm"] = round(mean_tail - to_rpm, 3)
    # How much the integrator is carrying once everything has settled is how
    # much the feedforward missed by, in the units the output is written in.
    # A large steady i with a small ss error means ff_a/ff_b want re-fitting,
    # not that Ki wants raising.
    out["i_at_rest_permille"] = round(sum(r.i for r in tail) / len(tail), 1)
    out["ff_at_rest_permille"] = round(sum(r.ff for r in tail) / len(tail), 1)
    out["out_at_rest_permille"] = round(sum(r.out for r in tail) / len(tail), 1)
    # The noise the overshoot figure has to be judged against. On this rig it is
    # ~1 rpm of mechanical ripple at 12 events per output revolution, which is
    # not the loop's doing and must not be read as the loop's overshoot.
    if len(tail) > 1:
        m = sum(r.meas_rpm for r in tail) / len(tail)
        out["tail_sd_rpm"] = round(
            (sum((r.meas_rpm - m) ** 2 for r in tail) / len(tail)) ** 0.5, 3)
    else:
        out["tail_sd_rpm"] = None

    # -- the slew anchor -----------------------------------------------------
    # Where the commanded setpoint stopped moving. The firmware says so
    # directly via VFLAG_RAMPING, which is better than comparing sp to target
    # here: the flag is set by the same code that does the ramping, so it
    # cannot disagree with it the way a host-side epsilon can.
    ramp_rows = [i for i, r in enumerate(rows) if r.ramping]
    if ramp_rows:
        i0, i1 = ramp_rows[0], min(ramp_rows[-1] + 1, len(rows) - 1)
        out["ramp_s"] = round(t[i1] - t[i0], 4)
        dt = t[i1] - t[i0]
        out["slew_rpm_s"] = (round((rows[i1].sp_rpm - rows[i0].sp_rpm) / dt, 3)
                             if dt > 1e-9 else None)
        # Skip the first fifth of the ramp: the lag needs roughly a plant time
        # constant to reach the constant value the estimator assumes, and
        # averaging the approach into it biases the answer low.
        lag_rows = rows[i0 + max(1, (i1 - i0) // 5):i1 + 1]
        out["track_lag_rpm"] = (
            round(sum(sign * (r.sp_rpm - r.meas_rpm) for r in lag_rows)
                  / len(lag_rows), 4) if len(lag_rows) >= 5 else None)
    else:
        i1 = 0
        out["ramp_s"] = 0.0
        out["slew_rpm_s"] = None
        out["track_lag_rpm"] = None
    out["anchor_s"] = round(t[i1], 4)

    if abs(change) < 1e-6:
        # A zero-magnitude step — `--rpm 0` from rest, which is the parser and
        # plumbing check. There is no rise time or overshoot to report, and
        # inventing one from noise would be worse than saying so.
        return out

    def first_at(frac: float) -> float | None:
        level = from_rpm + frac * change
        for ti, yi in zip(t, y):
            if sign * (yi - level) >= 0.0:
                return ti
        return None

    t10, t90 = first_at(0.10), first_at(0.90)
    out["t10_s"] = None if t10 is None else round(t10, 4)
    out["t90_s"] = None if t90 is None else round(t90, 4)
    out["rise_s"] = None if (t10 is None or t90 is None) else round(t90 - t10, 4)

    # What the ramp alone costs: the commanded setpoint's own 10->90% time. A
    # loop with infinite bandwidth cannot beat this, so a `rise_s` near it is a
    # measurement of `vel_slew` wearing the loop's name.
    floor = 0.8 * out["ramp_s"]
    out["rise_slew_floor_s"] = round(floor, 4)
    out["ramp_limited"] = bool(floor > 1e-6 and out["rise_s"] is not None
                               and out["rise_s"] < 1.3 * floor)

    # -- regulation, measured from the anchor --------------------------------
    after = rows[i1:]
    ta = [(r.ms - rows[i1].ms) / 1000.0 for r in after]
    ya = [r.meas_rpm for r in after]

    wt = [(tv, v) for tv, v in zip(ta, ya) if tv <= OVERSHOOT_WINDOW_S] or \
         list(zip(ta[:1], ya[:1]))
    peak = max((v for _, v in wt), key=lambda v: sign * v)
    out["peak_rpm"] = round(peak, 3)
    out["overshoot_rpm"] = round(max(0.0, sign * (peak - to_rpm)), 3)
    out["overshoot_pct"] = round(out["overshoot_rpm"] / abs(change) * 100.0, 2)
    # A maximum over a SHORTER window is a smaller maximum. If --dwell did not
    # leave OVERSHOOT_WINDOW_S of data after the ramp, this run's overshoot is
    # not comparable with one that did, and saying which window was actually
    # searched is the only way a reader can tell.
    out["overshoot_window_s"] = round(wt[-1][0], 3)
    # Truncated means the DATA ran out early enough to matter, not that the
    # last sample landed at 2.98 s instead of 3.00. A 50 Hz stream never has a
    # sample exactly on the boundary, so the test allows one control period —
    # otherwise every run in existence is flagged and the flag means nothing.
    period = (ta[-1] - ta[0]) / max(1, len(ta) - 1)
    out["overshoot_window_truncated"] = bool(
        ta[-1] < OVERSHOOT_WINDOW_S - 2.0 * period)
    # A single maximum drawn from a rippling signal overshoots by construction.
    # The peak has to beat the settled tail's OWN worst excursion in the same
    # direction, plus one encoder count, before it is the controller's. The
    # first version compared it with 2 sd of the tail, which assumes the ripple
    # is roughly Gaussian — it is not: the 12-per-rev feature is a train of
    # sharp dips whose extremes sit well past 2 sd, so every down-step was
    # flagged for a dip the settled hold also shows (Sep 26, runs 19-31-06 and
    # 19-32-19: flagged minima one count from the tail's own). Comparing like
    # with like — an extreme against an extreme — is what separates "14%
    # overshoot, lower Kp" from "no overshoot resolvable on this rig".
    tail_exc = max(sign * (r.meas_rpm - to_rpm) for r in tail)
    out["tail_excursion_rpm"] = round(max(0.0, tail_exc), 3)
    quantum = 60000.0 / (COUNTS_PER_REV * 20.0)
    out["overshoot_above_ripple"] = (
        None if len(tail) < 2
        else bool(out["overshoot_rpm"] > out["tail_excursion_rpm"] + quantum))

    # Settling: the last moment it was outside the band, not the first moment
    # it was inside one. A response that dips back out is not settled, and the
    # first-crossing definition would call it settled anyway.
    ref = abs(to_rpm) if abs(to_rpm) > 1e-6 else abs(change)
    band = 0.02 * ref
    out["settle_band_rpm"] = round(band, 4)

    def settled_at(times, vals):
        last_out = None
        for ti, yi in zip(times, vals):
            if abs(yi - to_rpm) > band:
                last_out = ti
        if last_out is None:
            return 0.0                  # inside the band from the first sample
        if last_out >= times[-1] - 1e-9:
            return None                 # never settled within the dwell
        return round(last_out, 4)

    out["settle_s"] = settled_at(ta, ya)            # from the anchor: the loop
    out["settle_from_command_s"] = settled_at(t, y)  # end to end: the operator's

    # The band is a fixed 2% of the target, but the encoder resolves
    # 60000/(counts_per_rev * window) rpm — at window 20 that is 0.36 rpm,
    # wider than the +/-0.2 rpm band a 10 rpm target asks for. Where that is
    # true `settle_s` cannot be computed honestly and this says why.
    out["settle_band_below_quantum"] = bool(
        band < 60000.0 / (COUNTS_PER_REV * 20.0) / 2.0)
    return out


def profile_step(node: Node, run: Run, a: argparse.Namespace) -> dict:
    """Command a setpoint step and measure what the loop does about it.

    This is the profile the velocity PID exists to be tuned against. The three
    setup lines below are not boilerplate and each has a reason:

    `enc window` defaults to 20 here, NOT the sweep's 100. The loop advances
    once per encoder window, so window 100 is a 10 Hz control loop with 100 ms
    of measurement lag — it would dominate the very response being measured,
    and the gains chosen against it would be gains for a different plant.

    `cfg ramp_pmps 0` disarms drive.c's slew limiter. `vel_slew` is the loop's
    own setpoint ramp and the two must not both be armed: with drv's limiter
    running, drive_slewing() is true almost continuously, the integrator is
    frozen for essentially the whole run, and the step measures the limiter.

    `vel_tmo` is set to WATCHDOG_MS so the loop's setpoint watchdog and the
    drive watchdog have the same period, both refreshed by the same 0.5 s kick
    in dwell().
    """
    sign = -1.0 if a.dir == "ccw" else 1.0
    to_rpm = sign * a.rpm
    from_rpm = sign * a.step_from

    over = [r for r in (a.rpm, a.step_from) if abs(r) > a.max_rpm]
    if over:
        raise NodeError(
            f"setpoint {over} rpm exceeds the {a.max_rpm:.0f} rpm ceiling, which is "
            f"the {DEFAULT_MAX_DUTY:.0f}% duty ceiling pushed through the plant fit. "
            f"Raise it explicitly with --max-rpm if that is really intended."
        )

    window = 20 if a.window is None else a.window
    a.window = window               # so meta.json records what actually ran
    if window > 40:
        print(f"  warning: enc window {window} gives a {1000.0 / window:.0f} Hz loop — "
              f"the measurement lag will dominate the step response", file=sys.stderr)
    node.command(f"enc window {window}")
    node.command("cfg ramp_pmps 0")
    for key, val in (("vel_kp", a.kp), ("vel_ki", a.ki), ("vel_kd", a.kd),
                     ("vel_ff_a", a.ff_a), ("vel_ff_b", a.ff_b),
                     ("vel_slew", a.slew)):
        if val is not None:
            node.command(f"cfg {key} {val}")
            run.event("gain", f"{key}={val}")
    node.command(f"cfg vel_tmo {WATCHDOG_MS}")

    node.command(f"drv trip {a.trip}")
    node.command(f"drv timeout {WATCHDOG_MS}")
    node.command("drv decay slow")
    node.command("drv enable")
    node.command("drv clearfault")

    node.command(f"telem rate {a.rate}")
    node.command("telem on")
    node.command("telem vel on")
    run.event("telem_on", f"T {a.rate} Hz + V per control step")

    gains = node.ask("vel gains")
    run.event("gains", gains.replace("\n", " | "))

    node.command("vel on")
    run.event("vel_on", "")

    segments = []

    # Baseline at rest, with the loop armed. Worth its own dwell: it shows the
    # loop holding zero, and it is where a stiction-driven limit cycle would be
    # visible before any step muddies it.
    run.state = "baseline"
    dwell(node, run, a.baseline, vel_kick=0, point="baseline 0 rpm")

    if abs(from_rpm) > 1e-6:
        run.state = f"pre-step {from_rpm:+.1f} rpm"
        node.command(f"vel target {int(round(from_rpm * 1000))}m")
        run.event("pre_step", f"{from_rpm:+.3f}")
        dwell(node, run, a.pre, vel_kick=int(round(from_rpm * 1000)),
              point=f"pre-step {from_rpm:+.1f} rpm")

    run.state = f"step {from_rpm:+.1f} -> {to_rpm:+.1f} rpm"
    mark = len(run.velocs)
    node.command(f"vel target {int(round(to_rpm * 1000))}m")
    run.event("step", f"{from_rpm:+.3f} -> {to_rpm:+.3f}")
    dwell(node, run, a.dwell, vel_kick=int(round(to_rpm * 1000)),
          point=f"step {to_rpm:+.1f} rpm")
    seg = step_metrics(run.velocs[mark:], from_rpm, to_rpm)
    seg["segment"] = "up" if abs(to_rpm) >= abs(from_rpm) else "down"
    segments.append(seg)

    # The return step is not symmetry-checking for its own sake. The
    # feedforward carries a friction offset applied with the sign of the
    # setpoint, so the down-step is the one place a wrong ff_b shows up as a
    # different response rather than as a constant error.
    if a.ret:
        run.state = f"return {to_rpm:+.1f} -> {from_rpm:+.1f} rpm"
        mark = len(run.velocs)
        node.command(f"vel target {int(round(from_rpm * 1000))}m")
        run.event("step", f"{to_rpm:+.3f} -> {from_rpm:+.3f} (return)")
        dwell(node, run, a.dwell, vel_kick=int(round(from_rpm * 1000)),
              point=f"return {from_rpm:+.1f} rpm")
        seg = step_metrics(run.velocs[mark:], to_rpm, from_rpm)
        seg["segment"] = "return"
        segments.append(seg)

    run.state = "stopping"
    # `vel stop` ramps the setpoint down and then coasts. The dwell after it is
    # for the ramp, and it still needs the kick: the watchdog is running until
    # the loop is disarmed.
    node.command("vel stop")
    dwell(node, run, a.stop_dwell, vel_kick=0, point="ramp down")
    node.command("vel off")
    node.command("telem vel off")
    node.command("telem off")

    order = ["segment", "from_rpm", "to_rpm", "samples",
             # the ramp, and whether it ate the measurement
             "ramp_s", "slew_rpm_s", "track_lag_rpm", "anchor_s",
             "rise_s", "rise_slew_floor_s", "ramp_limited", "t10_s", "t90_s",
             # regulation, anchored at ramp end
             "overshoot_pct", "overshoot_rpm", "overshoot_above_ripple",
             "overshoot_window_s", "overshoot_window_truncated",
             "tail_sd_rpm", "tail_excursion_rpm", "peak_rpm", "settle_s",
             "settle_from_command_s", "settle_band_rpm", "settle_band_below_quantum",
             "final_rpm", "ss_error_rpm", "i_at_rest_permille",
             "ff_at_rest_permille", "out_at_rest_permille", "sat_fraction",
             "freeze_fraction", "freeze_antiwindup", "freeze_drv_slewing",
             "freeze_no_bridge", "watchdog_expired", "steps_missed"]
    fields = order + [k for s in segments for k in s if k not in order]
    with open(os.path.join(run.dir, "step.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, restval="")
        w.writeheader()
        w.writerows(segments)

    for seg in segments:
        print(f"  {seg['segment']:<7s} {seg['from_rpm']:+6.2f} -> {seg['to_rpm']:+6.2f} rpm"
              f"  ramp {_fmt(seg.get('ramp_s'), 's')}"
              f"  lag {_fmt(seg.get('track_lag_rpm'), ' rpm')}"
              f"  rise {_fmt(seg.get('rise_s'), 's')}"
              f"  over {_fmt(seg.get('overshoot_pct'), '%')}"
              f"{'' if seg.get('overshoot_above_ripple') is not False else '*'}"
              f"  settle {_fmt(seg.get('settle_s'), 's')}"
              f"  sserr {_fmt(seg.get('ss_error_rpm'), ' rpm')}"
              f"  sat {seg['sat_fraction'] * 100:.0f}%")
        # Say it at the point of use, not only in the CSV. A number that
        # measured the wrong thing is worse than a missing one, and the
        # only defence is that the run itself says so while it is on screen.
        if seg.get("ramp_limited"):
            print(f"  warning: rise {seg['rise_s']:.2f} s is within 30% of the "
                  f"{seg['rise_slew_floor_s']:.2f} s the {seg['slew_rpm_s']:.1f} rpm/s "
                  f"setpoint ramp costs on its own — this segment measured "
                  f"vel_slew, not the loop.\n"
                  f"           Read track_lag_rpm for loop bandwidth under the "
                  f"ramp, or re-run with --slew 0 for a true step.",
                  file=sys.stderr)
        if seg.get("overshoot_window_truncated"):
            print(f"  warning: only {seg['overshoot_window_s']:.2f} s of the "
                  f"{OVERSHOOT_WINDOW_S:.0f} s overshoot window fell inside this "
                  f"segment — the peak was searched over less time than usual and "
                  f"reads low. Give --dwell at least "
                  f"{seg['ramp_s'] + OVERSHOOT_WINDOW_S:.1f} s.", file=sys.stderr)
        if seg.get("overshoot_above_ripple") is False and seg.get("overshoot_rpm"):
            print(f"  note: the {seg['overshoot_rpm']:.2f} rpm peak (marked *) is within "
                  f"one count of the settled hold's own {seg['tail_excursion_rpm']:.2f} rpm "
                  f"excursion — it is a ripple peak, not resolvable controller "
                  f"overshoot.", file=sys.stderr)
        if seg.get("settle_band_below_quantum"):
            print(f"  warning: the +/-{seg['settle_band_rpm']:.2f} rpm settle band is "
                  f"narrower than one encoder count at this window — settle_s "
                  f"cannot be computed honestly and is not meaningful.",
                  file=sys.stderr)

    return {"segments": segments, "gains": gains}


def _fmt(v, unit: str) -> str:
    return "  n/a" if v is None else f"{v:5.2f}{unit}"


def profile_stair(node: Node, run: Run, a: argparse.Namespace) -> dict:
    """Hold a series of setpoints for a long dwell each, one arming, one sweep.

    Why a staircase and not N separate `run step` invocations: the question is
    long-run behaviour, so the loop must stay ARMED across the whole sweep.
    Tearing down and re-arming between points would reset the integrator at
    every point and throw away exactly the slow drift — thermal, friction,
    integrator wind — that a long run exists to capture.

    Why a long dwell buys the resolution: at `enc window 20` one encoder count
    is 0.36 rpm, which alone cannot resolve a 0.5 rpm increment. But the ripple
    decorrelates in ~0.4 s, so a 60 s dwell holds ~150 independent looks and
    the standard error of the mean lands near 0.08 rpm. The long dwell IS the
    measurement, and `--hold-settle` seconds are discarded at the head of each
    point so the ramp between setpoints is not averaged into it.

    THE SIGN. `--lo`/`--hi` are MAGNITUDES and `--dir` carries the sign, the
    same split `profile_step` uses. Getting this wrong is not cosmetic: a
    literal `--lo -10 --hi -20` would make the step count negative and produce
    an empty setpoint list, and a ceiling guard written as `max(setpoints)`
    would read -10 for a reverse run and wave through any speed at all. The
    guard below tests MAGNITUDE for that reason.
    """
    sign = -1.0 if a.dir == "ccw" else 1.0
    lo, hi, stride = abs(a.lo), abs(a.hi), abs(a.stair_step)
    if stride < 1e-6:
        raise NodeError("--stair-step must be non-zero")
    walk = 1.0 if hi >= lo else -1.0
    setpoints = [round(sign * (lo + walk * stride * i), 3)
                 for i in range(int(round(abs(hi - lo) / stride)) + 1)]

    worst = max(abs(s) for s in setpoints)
    if worst > a.max_rpm:
        raise NodeError(
            f"setpoint {worst} rpm exceeds the {a.max_rpm:.0f} rpm ceiling, which is "
            f"the {DEFAULT_MAX_DUTY:.0f}% duty ceiling pushed through the plant fit. "
            f"Raise it explicitly with --max-rpm if that is really intended."
        )

    window = 20 if a.window is None else a.window
    a.window = window               # so meta.json records what actually ran
    node.command(f"enc window {window}")
    node.command("cfg ramp_pmps 0")          # drv slew and vel_slew must not both be armed
    for key, val in (("vel_kp", a.kp), ("vel_ki", a.ki), ("vel_kd", a.kd),
                     ("vel_ff_a", a.ff_a), ("vel_ff_b", a.ff_b),
                     ("vel_slew", a.slew)):
        if val is not None:
            node.command(f"cfg {key} {val}")
            run.event("gain", f"{key}={val}")
    node.command(f"cfg vel_tmo {WATCHDOG_MS}")
    node.command(f"drv trip {a.trip}")
    node.command(f"drv timeout {WATCHDOG_MS}")
    node.command("drv decay slow")
    node.command("drv enable")
    node.command("drv clearfault")
    node.command(f"telem rate {a.rate}")
    node.command("telem on")
    node.command("telem vel on")
    gains = node.ask("vel gains")
    run.event("gains", gains.replace("\n", " | "))
    node.command("vel on")
    run.event("vel_on", f"staircase {setpoints[0]:+.1f} -> {setpoints[-1]:+.1f} rpm, "
                        f"{a.hold:.0f}s each")

    rows = []
    t_run0 = run.elapsed()
    print(f"  {len(setpoints)} points x {a.hold:.0f} s = "
          f"{len(setpoints) * a.hold / 60:.0f} min\n")
    print("    sp    mean    sd    err   out   ff    i   sat%  mA   n")

    for sp in setpoints:
        mrpm = int(round(sp * 1000))
        run.state = f"hold {sp:+.1f} rpm"
        vmark, tmark = len(run.velocs), len(run.samples)
        t_start = run.elapsed()
        node.command(f"vel target {mrpm}m")
        run.event("setpoint", f"{sp:+.2f}")
        dwell(node, run, a.hold, vel_kick=mrpm, point=f"{sp:+.1f} rpm")

        # Statistics over the settled tail only. The head of each point is the
        # vel_slew ramp from the previous setpoint plus whatever the loop does
        # about it, which belongs to the step response, not to the hold.
        vs = [v for v in run.velocs[vmark:]
              if (v.host_t - run.t0) - t_start >= a.hold_settle]
        ts = [s for s in run.samples[tmark:]
              if (s.host_t - run.t0) - t_start >= a.hold_settle]
        if not vs:
            raise NodeError(
                f"no velocity samples in the settled tail at {sp:+.1f} rpm — "
                f"--hold {a.hold} must exceed --hold-settle {a.hold_settle}")
        meas = [v.meas_rpm for v in vs]
        ma = [s.ma for s in ts] or [0]
        row = {
            "sp_rpm": sp,
            "t_start_s": round(t_start - t_run0, 1),
            "n": len(vs),
            "mean_rpm": round(statistics.fmean(meas), 4),
            "sd_rpm": round(statistics.pstdev(meas), 4),
            "min_rpm": round(min(meas), 3),
            "max_rpm": round(max(meas), 3),
            "err_rpm": round(statistics.fmean(meas) - sp, 4),
            "sem_rpm": round(statistics.pstdev(meas) / (len(vs) ** 0.5), 4),
            "out_pm": round(statistics.fmean([v.out for v in vs]), 1),
            "ff_pm": round(statistics.fmean([v.ff for v in vs]), 1),
            "p_pm": round(statistics.fmean([v.p for v in vs]), 2),
            "i_pm": round(statistics.fmean([v.i for v in vs]), 2),
            "d_pm": round(statistics.fmean([v.d for v in vs]), 2),
            "sat_frac": round(sum(v.saturated for v in vs) / len(vs), 4),
            "freeze_frac": round(sum(v.frozen for v in vs) / len(vs), 4),
            "wd_frac": round(sum(v.watchdog for v in vs) / len(vs), 4),
            "ma_mean": round(statistics.fmean(ma), 1),
            "ma_max": max(ma),
        }
        rows.append(row)
        print(f"  {sp:+5.1f} {row['mean_rpm']:7.3f} {row['sd_rpm']:5.3f} "
              f"{row['err_rpm']:+6.3f} {row['out_pm']:5.0f} {row['ff_pm']:4.0f} "
              f"{row['i_pm']:5.1f} {row['sat_frac'] * 100:4.0f} {row['ma_mean']:5.0f} "
              f"{row['n']:5d}")

        # A staircase is 20+ minutes unattended. Both of these end the run
        # rather than logging a note, because the remaining points would be
        # taken under a condition the header does not describe.
        if row["ma_max"] > a.abort_ma:
            raise NodeError(f"current {row['ma_max']} mA exceeded the "
                            f"{a.abort_ma} mA soft guard at {sp:+.1f} rpm")
        if row["wd_frac"] > 0:
            raise NodeError(f"setpoint watchdog expired during the {sp:+.1f} rpm hold")

    run.state = "stopping"
    node.command("vel stop")
    dwell(node, run, a.stop_dwell, vel_kick=0, point="ramp down")
    node.command("vel off")
    node.command("telem vel off")
    node.command("telem off")

    with open(os.path.join(run.dir, "stair.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    sat = [r["sp_rpm"] for r in rows if r["sat_frac"] > 0]
    if sat:
        print(f"  note: output saturated at {len(sat)} of {len(rows)} setpoints, "
              f"from {sat[0]:+.1f} rpm — the {DEFAULT_MAX_DUTY:.0f}% duty ceiling "
              f"is binding, so those points measure the ceiling, not the loop.",
              file=sys.stderr)

    return {"setpoints": rows, "settle_excluded_s": a.hold_settle,
            "hold_s": a.hold, "window": window, "dir": a.dir, "gains": gains}


PROFILES = {
    "sweep": (
        profile_sweep,
        "duty list, dwell at each, settled rpm + current — the task-17 table, automated",
    ),
    "step": (
        profile_step,
        "setpoint step against the velocity loop — rise, overshoot, settling, ss error",
    ),
    "stair": (
        profile_stair,
        "long-run staircase — one arming, a long dwell at each setpoint, settled stats",
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
        "veloc_samples": node.veloc_count,
        "veloc_gaps": node.veloc_gaps,
        # Steps the firmware computed but never published, because the main
        # loop did not drain the slot in time. Distinct from veloc_gaps, which
        # is lines lost on the wire — this one is a hole in the control record
        # with no missing seq to reveal it, which is exactly why it is counted.
        "veloc_steps_missed": node.veloc_steps_missed,
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

    node = Node(port=a.port, raw_log=run.raw, on_telem=run.sink,
                on_veloc=run.veloc_sink)
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

        suspect = (bool(node.telem_gaps) or bool(dropped)
                   or bool(node.veloc_gaps) or bool(node.veloc_steps_missed))
        run.write_meta(
            _meta(a, run, node, outcome, error, conditions, dropped, suspect, result)
        )
        run.write_status(node, force=True, outcome=outcome, suspect=suspect)
        run.close()
        node.close()

    if suspect:
        print(
            f"\nSUSPECT: {node.telem_gaps} T seq gaps, {node.veloc_gaps} V seq gaps, "
            f"{node.veloc_steps_missed} unpublished control steps, tx_dropped={dropped}. "
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
    r.add_argument("--window", type=int, default=None,
                   help="enc window, in 1 ms ticks. Defaults PER PROFILE: 100 for "
                        "sweep, 20 for step — the loop advances once per window, so "
                        "100 would make step a 10 Hz loop and measure the filter")
    r.add_argument("--trip", type=int, default=1580,
                   help="current-regulation trip in mA (boot default 999 is below stall)")
    r.add_argument("--max-duty", type=float, default=DEFAULT_MAX_DUTY,
                   help="refuse any duty above this (percent)")

    # -- step profile --
    g = r.add_argument_group("step profile")
    g.add_argument("--rpm", type=float, default=10.0,
                   help="setpoint stepped TO, rpm (default 10)")
    g.add_argument("--from", dest="step_from", type=float, default=0.0,
                   help="setpoint stepped FROM. Non-zero gives a small-signal step "
                        "from a turning wheel, which is a different plant from rest")
    g.add_argument("--return", dest="ret", action="store_true",
                   help="step back down afterwards. The feedforward's friction offset "
                        "is applied with the sign of the setpoint, so the down-step is "
                        "where a wrong ff_b shows as a different response")
    g.add_argument("--max-rpm", type=float, default=DEFAULT_MAX_RPM,
                   help="refuse any setpoint above this (rpm)")
    g.add_argument("--baseline", type=float, default=1.0,
                   help="seconds held at 0 rpm with the loop armed, before the step")
    g.add_argument("--pre", type=float, default=3.0,
                   help="seconds held at --from before the step (ignored if --from 0)")
    g.add_argument("--stop-dwell", type=float, default=2.0,
                   help="seconds after `vel stop`, for the setpoint ramp to run out")
    # Gains are in MILLI-units because config.c stores int32 only: --kp 5000 is
    # Kp = 5.0. Left at None, the board keeps whatever it has, which is what a
    # repeat run of the same gains wants.
    g.add_argument("--kp", type=int, default=None, help="cfg vel_kp, milli-units")
    g.add_argument("--ki", type=int, default=None, help="cfg vel_ki, milli-units")
    g.add_argument("--kd", type=int, default=None, help="cfg vel_kd, milli-units")
    g.add_argument("--ff-a", type=int, default=None,
                   help="cfg vel_ff_a, milli o/oo per rpm (feedforward slope)")
    g.add_argument("--ff-b", type=int, default=None,
                   help="cfg vel_ff_b, o/oo (feedforward friction offset)")
    g.add_argument("--slew", type=int, default=None,
                   help="cfg vel_slew, milli-rpm/s. 0 makes it a true step")

    # -- stair profile --  (also uses --dir, --window, --rate, --trip,
    #                       --max-rpm, --stop-dwell and the gain options above)
    h = r.add_argument_group("stair profile")
    h.add_argument("--lo", type=float, default=10.0,
                   help="first setpoint, rpm MAGNITUDE — --dir carries the sign")
    h.add_argument("--hi", type=float, default=20.0,
                   help="last setpoint, rpm MAGNITUDE. Below --lo walks the staircase down")
    h.add_argument("--stair-step", type=float, default=0.5,
                   help="rpm between setpoints (default 0.5)")
    h.add_argument("--hold", type=float, default=60.0,
                   help="seconds at each setpoint. This is the measurement: the "
                        "standard error of the mean falls as its square root")
    h.add_argument("--hold-settle", type=float, default=10.0,
                   help="seconds discarded at the head of each hold, so the ramp "
                        "in from the previous setpoint is not averaged into it")
    h.add_argument("--abort-ma", type=int, default=1200,
                   help="end the run if any point's peak current exceeds this. A soft "
                        "guard under --trip, because a staircase runs for 20+ minutes")
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
