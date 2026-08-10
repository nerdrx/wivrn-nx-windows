# Cross toolchain for compile-checking on Linux with llvm-mingw.
# Point LLVM_MINGW_ROOT at an extracted llvm-mingw release
# (default: the Lex/claude/tools checkout used on the dev box).

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

if(NOT DEFINED LLVM_MINGW_ROOT)
    set(LLVM_MINGW_ROOT "/run/media/nerdrx/Lex/claude/tools/llvm-mingw")
endif()

set(CMAKE_C_COMPILER   ${LLVM_MINGW_ROOT}/bin/x86_64-w64-mingw32-clang)
set(CMAKE_CXX_COMPILER ${LLVM_MINGW_ROOT}/bin/x86_64-w64-mingw32-clang++)
set(CMAKE_RC_COMPILER  ${LLVM_MINGW_ROOT}/bin/x86_64-w64-mingw32-windres)

set(CMAKE_FIND_ROOT_PATH ${LLVM_MINGW_ROOT}/x86_64-w64-mingw32)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
