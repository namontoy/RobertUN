# RobertUN — Project Context (router)
**Last updated:** September 17, 2026 (the single 4500-line context file was split into three scoped files; this is now only a signpost)

This file used to hold everything. It grew to ~4500 lines, which meant every
session — whatever it was about — paid for the whole project's history. On
**Sep 17, 2026** it was split. **Read the one file that matches the work, not
all three.**

| Working on | Read | Lines |
|---|---|---|
| **Wheel-node STM32 firmware** — CAN peripheral, pin allocation, firmware modules, SERVO42C, drive motor / DRV8874 / encoder | **`PROJECT_CONTEXT_WHEEL_FW.md`** | ~2340 |
| Machines, network, SSH, ROS 2 / Jetson / Isaac Sim, bus-wide CAN architecture, power distribution, tooling | `PROJECT_CONTEXT_REST.md` | ~1820 |
| The exact numbers, register values or reasoning behind one past wheel-firmware session | `PROJECT_CONTEXT_WHEEL_FW_LOG.md` — **on demand only** | ~470 |

`PROJECT_CONTEXT_WHEEL_FW.md` carries a one-line-per-session brief log; the
`_LOG` file carries the full entry behind each line. Open the log only when a
specific entry's detail is actually needed.

**Writing at the end of a session:** detailed entry → the `_LOG` file; one
summary line → the brief log in `PROJECT_CONTEXT_WHEEL_FW.md`; durable rules →
that file's KEY LEARNINGS; open work → its NEXT TASKS. The point of the split is
that the file read every session stays roughly flat in length.

The pre-split file is preserved in git history at commit `271b20a`.
