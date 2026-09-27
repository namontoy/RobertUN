# RobertUN — working rules for Claude Code

## Session start
- Use the `project-context` skill. Read only the hot file for the track:
  wheel firmware → `docs/environment/PROJECT_CONTEXT_WHEEL_FW.md`;
  machines, ROS 2, power → `docs/environment/PROJECT_CONTEXT_REST.md`.
  If the track is unclear, ask. Never read `_REF_*` or `_LOG` files whole.

## Replies
- Short. Lead with the result or the number; no restating output the user
  just saw, no recap of the plan at the end of every turn.
- At most 3–4 commands per batch, then stop and check the result.

## Command output — keep it out of the conversation
- Builds: show only errors, warnings and the memory summary, e.g.
  `... 2>&1 | grep -E 'error|warning|FLASH|RAM' | tail -40`.
- Never `cat` a large file or print a whole log. Use `grep -n`, `head`, `tail`,
  or Read with offset/limit.
- Bench runs write their data to `tools/bench/runs/` (local-only, git-ignored).
  Report a run from `./bench.py status` or a short script that prints only the
  numbers asked for — never print `console.log` or a CSV.
- Serial/console transcripts: save to a file, then grep it.

## Reading code
- Find the function with `grep -n`, then read that line range. Don't read a
  whole `.c` file to change one function.

## Subagents
- Use a subagent for heavy reading: analysing a run's CSVs, comparing runs,
  searching a `_LOG` file, surveying unfamiliar code. It returns only the
  answer (the numbers, the file:line), not the material it read.

## End of a task
- Append the full account to the track's `_LOG` from the shell (no read),
  update the hot file (one line in Recent progress, Active work), commit.
  Then tell the user it's a good point to `/clear`.

# Compact instructions
When compacting, preserve: the current task and its goal, files changed,
commands and bench runs done with their key numbers, decisions made, open
questions, and the next step. Drop: raw command output, logs, CSV contents,
file contents already read, and exploration that didn't lead anywhere.
