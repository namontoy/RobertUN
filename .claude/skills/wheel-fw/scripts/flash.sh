#!/usr/bin/env bash
# flash.sh [Debug|Release] — flash the built .elf over SWD (ST-Link), verify, reset.
# Prints one verdict line; the programmer's full output goes to build/flash.log.
set -uo pipefail

preset="${1:-Debug}"
root="$(git rev-parse --show-toplevel)"
proj="$root/firmware/RobertUN_ModuleNode"
elf="$proj/build/$preset/RobertUN_ModuleNode.elf"
cli=/opt/st/stm32cubeclt_1.22.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI
log="$proj/build/flash.log"

[ -f "$elf" ] || { echo "FLASH FAILED: no $preset .elf — run build.sh first"; exit 1; }

# Only one program can hold the ST-Link: a VS Code debug session blocks flashing.
holders=$(pgrep -fa 'ST-LINK_gdbserver|openocd|st-util' || true)
if [ -n "$holders" ]; then
  echo "FLASH BLOCKED: a debug server holds the ST-Link — stop the debug session first:"
  echo "$holders" | cut -c1-120
  exit 1
fi

"$cli" -c port=SWD -w "$elf" -v -rst > "$log" 2>&1
rc=$?

if [ $rc -eq 0 ] && grep -qi 'verified successfully' "$log"; then
  echo "FLASH OK ($preset): verified and reset — $(date -r "$elf" '+built %H:%M:%S')"
else
  echo "FLASH FAILED (exit $rc):"
  grep -iE 'error|fail|not found|no st-link|unable' "$log" | head -8
  echo "full log: firmware/RobertUN_ModuleNode/build/flash.log"
  exit 1
fi
