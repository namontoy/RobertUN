---
name: wheel-fw
description: Build, flash and talk to the RobertUN wheel-node firmware (STM32F446RE, CubeMX CMake project in firmware/projects/RobertUN_ModuleNode) with scripts that print a short verdict instead of the full tool output. Use whenever the firmware needs to be compiled, checked for errors or warnings, measured for flash/RAM use, written to the board over the ST-Link, or sent console commands over the serial port — instead of running cmake, ninja, STM32_Programmer_CLI or ad-hoc pyserial code directly.
---

# Wheel firmware: build, flash, console

Both scripts live in this skill's `scripts/` folder and are run from anywhere
inside the repo. They print a few lines; the full tool output goes to a log
file under `firmware/projects/RobertUN_ModuleNode/build/`, which is git-ignored.

## Build

```bash
.claude/skills/wheel-fw/scripts/build.sh          # Debug (default)
.claude/skills/wheel-fw/scripts/build.sh Release
```

- Prints `BUILD OK` with the warning count, any warning lines (max 10), and
  flash/RAM use; or `BUILD FAILED` with the unique error lines (max 20).
- Builds are incremental: the warning count covers only the files recompiled
  this time. For a full warning audit, delete `build/<preset>` first and say so.
- Configures the preset automatically on the first build.
- If the verdict isn't enough, `grep` the log for the file or message in
  question. Don't print the whole log.

## Flash

```bash
.claude/skills/wheel-fw/scripts/flash.sh          # flashes the Debug .elf
```

- Flashes over SWD with the ST-Link, verifies, and resets the target. Prints
  one line: `FLASH OK` or `FLASH FAILED` with the relevant error lines.
- Refuses to run while a debug server (a VS Code Cortex-Debug session) holds
  the ST-Link. Ask the user to stop the debug session; don't kill it yourself.
- Only flash after a `BUILD OK` from the same code. The OK line shows when the
  `.elf` was built, as a check against flashing a stale image.
- After flashing, give the board about a second to boot before talking to
  its console.

## Console

```bash
.claude/skills/wheel-fw/scripts/console.py "cfg"
.claude/skills/wheel-fw/scripts/console.py "vel off" "drv duty 0" "drv"
.claude/skills/wheel-fw/scripts/console.py --wait 1 "cfg"     # right after flashing
```

- Sends each command in order and prints `> command` followed by its reply.
  Telemetry records are not printed. Replies longer than 40 lines are cut;
  raise `--max-lines` only when the whole reply is really needed.
- Uses `tools/bench/node.py` for the serial port, echo handling and prompt
  detection. Don't write new pyserial code for console access.
- `--timeout` (default 2 s) is per command; raise it for slow commands.
- Prints `CONSOLE ERROR: ...` and exits 1 if the port is missing or busy, or
  a prompt never comes back. Only one program can hold the port: if the error
  says it's busy, ask the user whether a terminal or bench run has it open.
- Bench runs (profiles, telemetry capture) use `tools/bench/bench.py`
  (`run`, `status`, `list`), not this script. Report a run from
  `bench.py status`, never by printing its CSV or console log.

## Rules

- Use these scripts, not raw `cmake`, `ninja` or `STM32_Programmer_CLI`
  commands. If a script fails in a way its output doesn't explain, report it
  to the user rather than working around it.
- Flashing changes what is running on the bench. If a motor could be powered,
  say so before flashing.
- Don't send commands that move a motor (`drv duty`, `vel` setpoints) unless
  the user asked for that motion. To stop, send `vel off` first, then
  `drv duty 0`: with the velocity loop armed, a duty command alone is
  overridden at its next step.
