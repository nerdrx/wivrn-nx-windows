#!/bin/sh
# Cross compile-check the whole tree on Linux with llvm-mingw.
set -e
cd "$(dirname "$0")/.."
CMAKE=$(command -v cmake || echo /run/media/nerdrx/Lex/claude/tools/cmake-3.31.10-linux-x86_64/bin/cmake)
"$CMAKE" --preset mingw-check
"$CMAKE" --build --preset mingw-check "$@"
