# Bench tooling for the wheel node

Drives experiments against a RobertUN wheel node over its USART1 console,
logs everything to a run directory, and can be checked on mid-run without
being attached to it.

It lives under the firmware tree on purpose: the parser is coupled to the
console's exact output format, so a console change and its parser change land
in the same commit.

## Requirements

**Host** — see `requirements.txt`, which carries the reasoning per line:

```sh
python3 -m pip install -r requirements.txt
```

| package | why | needed to take a run? |
|---|---|---|
| `pyserial` | the serial link to USART1 | **yes — and it is the only one** |
| `numpy` | array maths in the analysis and figure scripts | no |
| `matplotlib` | figures | no |
| `scipy` | exponential fits for the planned `coastdown` / `step` profiles | not yet |

Specs are **floors, not pins**, so a distro's own packages normally satisfy
them and no virtualenv is needed. Verified on Python 3.10.12 with pyserial
3.5, numpy 2.2.6, matplotlib 3.10.8, scipy 1.15.3.

**`bench.py` and `node.py` import nothing outside the standard library except
pyserial.** The least-squares slope in `rpm_from_counts()` is hand-rolled
rather than handed to numpy, deliberately: a bench box stays minimal, and a
missing analysis library can never cost you a run. So on a machine that only
needs to *take* data:

```sh
python3 -m pip install pyserial
```

The user must be in the `dialout` group or `/dev/ttyUSB*` is unreadable:

```sh
sudo usermod -aG dialout $USER     # then log out and back in
```

**Firmware** — an image with the `telem` and `drv timeout` commands. Older
images fail pre-flight with "no telemetry received" — that means reflash, not
a bug.

**No Node.js.** Nothing here is JavaScript.

## Wiring

USART1, **115200 8N1**, on a USB-serial adapter:

| board | adapter |
|---|---|
| PA9 (TX) | RX |
| PA10 (RX) | TX |
| GND | GND |

The ST-Link/V2 used for flashing has no virtual COM port, so this is a second,
separate cable. Do not connect the adapter's Vcc.

## Use

```sh
./bench.py list                                   # profiles and recent runs
./bench.py run sweep                              # CW, 5-29% in 2% steps, 4 s/point
./bench.py run sweep --dir ccw                    # the other direction
./bench.py status                                 # latest run's status.json
```

The port is autodetected when exactly one `/dev/ttyUSB*` or `/dev/ttyACM*` is
present; otherwise pass `--port`.

### Checking in on a run

The runner owns the port for the duration, so there is no second connection to
make. Everything is a file:

```sh
./bench.py status          # or: cat runs/<latest>/status.json
```

`status.json` is rewritten once a second and holds the current duty, speed,
current, elapsed time and the running dropped-sample count. That is the whole
"run it, come back in five minutes" story — no daemon, no IPC.

## Profiles

| profile | what it does | what it yields |
|---|---|---|
| `sweep` | duty list, dwell at each, settled speed + current | the steady-state plant table, automated |

More profiles (`coastdown`, `step`, `hold`, `stiction`) are planned; the first
three are what actually unblock PID gain selection.

### `sweep` options

| flag | default | |
|---|---|---|
| `--duty` | `5,7,9,11,13,15,17,19,21,23,25,27,29` | comma-separated percents, always positive. **The rover runs slow: 30% duty is the ceiling for characterisation work**, and the default walks the band in 2% steps. Anything above it is calibration only and must be asked for explicitly. |
| `--dir` | `cw` | `ccw` negates every duty |
| `--dwell` | `4.0` | seconds held at each point |
| `--settle` | `0.5` | leading fraction of each dwell discarded as transient |
| `--rate` | `50` | telemetry Hz, 1..100 |
| `--window` | `100` | `enc window`, in 1 ms ticks |
| `--max-duty` | `30` | refuses anything above it. **Raise it only for deliberate calibration** — the rover's working band tops out well below this, and above ~31% the wheel begins to bounce on the rig belt, which makes those points a property of the rig rather than of the plant. |
| `--trip` | `1580` | mA; **set at run start**, so the trip is a recorded run condition rather than whatever the board happened to boot with |

### Echo mismatches are counted, not fatal

The console echoes **each typed character as its own one-byte write**, while a
telemetry line is **one atomic write**. A `T,` record therefore lands *inside*
the echo of a command being typed:

```
sent:    drv timeout 2000
echoed:  drv tT,226,575448,200,11047,14780,230,3imeout 2000
```

`node.py` matches a telemetry record **anywhere** in a line, extracts every
match, and rejoins the residue into the echo - so both the record and the
command survive. When the reassembled echo still does not match what was sent,
that is **counted** (`echo_mismatches`, reported in `meta.json` and
`status.json`) rather than raised: the echo is a convenience, the telemetry is
the measurement, and a run should not die because a character interleaved. A
non-zero count is worth a look; it does not by itself invalidate the data.
`seq` gaps and `tx_dropped` are the checks that do.

## Run directory

```
runs/2026-09-25T14-03-11_sweep/
    meta.json       profile, arguments, outcome, and full dumps of
                    info / cfg / drv / enc taken at connect
    console.log     raw byte-for-byte transcript
    telemetry.csv   one row per T-line, plus host arrival time
    events.csv      every command sent, with host time
    sweep.csv       the settled result per duty point
    status.json     rewritten every second
```

**The raw log is written before anything is parsed.** A parser bug can then
cost an analysis but never a bench run — the run is re-parsable offline. Bench
time is the expensive thing here.

## Reading the data correctly

`count` is the measurement. `mrpm` is a convenience column.

`enc window` is a boxcar average over N × 1 ms ticks, so at window 100 the rpm
figure lags the true speed by ~50 ms and is smoothed over 100 ms. Sampling that
at 50 Hz and fitting an exponential to it would measure the *filter's* time
constant and report it as the plant's. So the tool derives speed from the slope
of `count` against the board's own `ms` stamp, and keeps `mrpm` only as a
cross-check.

Two more things the columns are telling you:

- **`current_is_motor`**: below **14.5% duty** the on-phase is shorter than the
  IPROPI settle + aperture window, so there is no synchronised sample and the
  current figure is *supply* current, not motor current. It flips partway up
  any sweep that starts low.
- **`seq`**: the board's own line counter. A gap means the TX ring overflowed
  and lines were dropped. The run is marked `suspect` in `meta.json` and
  `status.json` if any gap or any `tx_dropped` is seen — do not fit a run with
  holes in it without first finding out why.

## Safety

In the order it matters:

1. **`try` / `finally` plus a SIGTERM handler.** A normal exit, an exception,
   Ctrl-C and `kill` all end at the same stop: `drv duty 0`, `drv coast`,
   `drv disable`, `telem off`. Best-effort — each step is attempted even if an
   earlier one raised, because a half-executed stop is the worst outcome.
2. **The board's own watchdog.** `drv timeout 2000` is armed at run start and
   refreshed while dwelling. This is the part `finally` *cannot* cover:
   `kill -9` and a yanked USB cable run no Python at all, so the board has to
   be able to stop itself. It **coasts** rather than brakes — braking from
   speed drives I = E/R through the low-side FETs, and a dead host is exactly
   when nobody is watching the driver dissipate it.
3. **A duty ceiling**, 40% by default, matching the rover's stated operating
   band. Anything higher has to be asked for explicitly with `--max-duty`.
4. **Pre-flight refusal** on a latched nFAULT or a motor already commanded
   non-zero — better than a run whose conditions were wrong from the start.
   The fault check reads the telemetry `flags` bit rather than parsing the
   human-readable `drv` text, so it cannot drift with a wording change.
5. **Post-flight integrity**, as above.

**Every run so far is with a free-rotating wheel on the motor shaft and no
rig.** That is a condition of the data, not a detail — record it when the rig
goes on.

## Verifying the watchdog

This is the one safety property that has to be tested deliberately rather than
assumed, because it only ever fires when everything else has already failed:

```sh
./bench.py run sweep --duty 20 --dwell 600 &
sleep 10
kill -9 %1                      # no Python runs after this
```

The wheel must coast to a stop within ~2 s. If it keeps spinning, stop it by
hand at the console and do not run anything unattended until it is fixed.

## Telemetry line format

```
T,<seq>,<ms>,<duty>,<count>,<mrpm>,<ma>,<flags>
```

| field | |
|---|---|
| `seq` | uint32, +1 per line; restarts at 0 on `telem on` |
| `ms` | `HAL_GetTick()` at emission — board time, stamped before any USB latency |
| `duty` | signed per-mille, as commanded |
| `count` | int32 encoder position at the output shaft, 8403.2 counts/rev |
| `mrpm` | rpm × 1000 (filtered — see above) |
| `ma` | Imotor if flag bit 0, else Isup |
| `flags` | 1 sync · 2 enabled · 4 fault latched · 8 ADC saturated · 16 watchdog expired |

Integer fields only: `%f` pulls in newlib's float formatter, which is far too
slow to run at 100 Hz.

`print_telem_line()` in `Core/Src/console.c` is the authority for this format;
the constants in `node.py` are the mirror.
