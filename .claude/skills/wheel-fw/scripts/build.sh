#!/usr/bin/env bash
# build.sh [Debug|Release] — build the wheel firmware and print only a verdict.
# The full compiler output goes to build/build-<preset>.log.
set -uo pipefail

preset="${1:-Debug}"
root="$(git rev-parse --show-toplevel)"
proj="$root/firmware/projects/RobertUN_ModuleNode"
clt=/opt/st/stm32cubeclt_1.22.0

cmake_bin=cmake
[ -x "$clt/CMake/bin/cmake" ] && cmake_bin="$clt/CMake/bin/cmake"
[ -d "$clt/Ninja/bin" ] && PATH="$clt/Ninja/bin:$PATH"
[ -d "$clt/GNU-tools-for-STM32/bin" ] && PATH="$clt/GNU-tools-for-STM32/bin:$PATH"

cd "$proj" || exit 1
mkdir -p build
log="build/build-$preset.log"
: > "$log"

if [ ! -f "build/$preset/CMakeCache.txt" ]; then
  if ! "$cmake_bin" --preset "$preset" >> "$log" 2>&1; then
    echo "CONFIGURE FAILED ($preset) — last lines of $log:"
    tail -15 "$log"
    exit 1
  fi
fi

"$cmake_bin" --build --preset "$preset" >> "$log" 2>&1
rc=$?

warnings=$(grep -c 'warning:' "$log")
errpat='error:|undefined reference|ld returned|FAILED:'

if [ $rc -ne 0 ]; then
  echo "BUILD FAILED ($preset): $(grep -cE "$errpat" "$log") error line(s), $warnings warning(s)"
  grep -E "$errpat" "$log" | sort -u | head -20
  echo "full log: firmware/projects/RobertUN_ModuleNode/$log"
  exit 1
fi

echo "BUILD OK ($preset): $warnings warning(s) in the files recompiled this time"
grep 'warning:' "$log" | sort -u | head -10
arm-none-eabi-size "build/$preset/RobertUN_ModuleNode.elf" | LC_ALL=C awk 'NR==2 {
  fl = $1 + $2; ram = $2 + $3
  printf "flash %.1f KB (%.0f%% of 512 KB), RAM %.1f KB (%.0f%% of 128 KB)\n",
         fl/1024, 100*fl/524288, ram/1024, 100*ram/131072 }'
echo "full log: firmware/projects/RobertUN_ModuleNode/$log"
