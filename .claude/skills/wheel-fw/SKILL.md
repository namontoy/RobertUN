---
name: wheel-fw
description: Build and flash the RobertUN wheel-node firmware (STM32F446RE, CubeMX CMake project in firmware/RobertUN_ModuleNode) with scripts that print a short verdict instead of the full tool output. Use whenever the firmware needs to be compiled, checked for errors or warnings, measured for flash/RAM use, or written to the board over the ST-Link — instead of running cmake, ninja or STM32_Programmer_CLI directly.
---

# Wheel firmware: build and flash

Both scripts live in this skill's `scripts/` folder and are run from anywhere
inside the repo. They print a few lines; the full tool output goes to a log
file under `firmware/RobertUN_ModuleNode/build/`, which is git-ignored.

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

## Rules

- Use these scripts, not raw `cmake`, `ninja` or `STM32_Programmer_CLI`
  commands. If a script fails in a way its output doesn't explain, report it
  to the user rather than working around it.
- Flashing changes what is running on the bench. If a motor could be powered,
  say so before flashing.
