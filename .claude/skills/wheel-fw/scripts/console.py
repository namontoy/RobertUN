#!/usr/bin/env python3
"""console.py — send one or more console commands to the wheel node, print the replies.

Usage:
    console.py "cfg"
    console.py "vel off" "drv duty 0" "drv"
    console.py --timeout 5 --wait 1 "cfg save"

Reuses tools/bench/node.py (port discovery, echo handling, prompt detection),
so this file adds no serial logic of its own. Telemetry records are handled
by node.py and are not printed.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

root = Path(subprocess.check_output(["git", "rev-parse", "--show-toplevel"],
                                    text=True).strip())
sys.path.insert(0, str(root / "firmware/projects/RobertUN_ModuleNode/tools/bench"))
from node import Node, NodeError  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Send console commands to the wheel node.")
    ap.add_argument("commands", nargs="+", help="console commands, sent in order")
    ap.add_argument("--timeout", type=float, default=2.0,
                    help="seconds to wait for the prompt after each command (default 2)")
    ap.add_argument("--wait", type=float, default=0.0,
                    help="seconds to wait before connecting, e.g. 1 right after flashing")
    ap.add_argument("--port", default=None, help="serial port (default: node.py finds it)")
    ap.add_argument("--max-lines", type=int, default=40,
                    help="reply lines printed per command before truncating (default 40)")
    args = ap.parse_args()

    if args.wait > 0:
        time.sleep(args.wait)

    try:
        with Node(port=args.port) as node:
            for cmd in args.commands:
                lines = node.command(cmd, timeout=args.timeout)
                print(f"> {cmd}")
                for line in lines[:args.max_lines]:
                    print(line)
                if len(lines) > args.max_lines:
                    print(f"... {len(lines) - args.max_lines} more line(s) not shown "
                          f"(raise --max-lines, or narrow the command)")
    except NodeError as e:
        print(f"CONSOLE ERROR: {e}")
        return 1
    except Exception as e:  # serial port missing, busy, permission denied
        print(f"CONSOLE ERROR ({type(e).__name__}): {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
