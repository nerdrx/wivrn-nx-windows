#!/bin/sh
# Build OpenSSL (libcrypto) as a static Windows x86_64 library with llvm-mingw.
#
# common/crypto.cpp, common/smp.cpp and common/secrets.cpp are pulled straight
# from the Linux tree, so the Windows helper needs the same libcrypto they link
# against there. This produces a cross-built static libcrypto.a + headers that
# helper/net/CMakeLists.txt picks up through WIVRNNX_OPENSSL_ROOT.
#
# Only libcrypto is used (EVP, BN, RAND, PEM). libssl is built too because
# Configure has no supported way to drop it, but nothing links it.
#
# Usage:  sh tools/build_openssl_mingw.sh [-f]
#   -f   force a rebuild even if the prefix already has libcrypto.a
#
# Environment overrides:
#   OPENSSL_VERSION   openssl release to build          (default 3.6.3)
#   OPENSSL_PREFIX    install prefix                    (default below)
#   LLVM_MINGW_ROOT   llvm-mingw toolchain root         (default below)
#   OPENSSL_WORKDIR   scratch dir for download + build
set -eu

OPENSSL_VERSION="${OPENSSL_VERSION:-3.6.3}"
OPENSSL_PREFIX="${OPENSSL_PREFIX:-/run/media/nerdrx/Lex/claude/tools/openssl-mingw}"
LLVM_MINGW_ROOT="${LLVM_MINGW_ROOT:-/run/media/nerdrx/Lex/claude/tools/llvm-mingw}"
OPENSSL_WORKDIR="${OPENSSL_WORKDIR:-${TMPDIR:-/tmp}/wivrnnx-openssl-build}"

FORCE=0
[ "${1:-}" = "-f" ] && FORCE=1

TRIPLE=x86_64-w64-mingw32
BIN="$LLVM_MINGW_ROOT/bin"

if [ ! -x "$BIN/$TRIPLE-clang" ]; then
    echo "error: no llvm-mingw clang at $BIN/$TRIPLE-clang" >&2
    echo "       set LLVM_MINGW_ROOT to an extracted llvm-mingw release" >&2
    exit 1
fi

if [ "$FORCE" -eq 0 ] && [ -f "$OPENSSL_PREFIX/lib/libcrypto.a" ]; then
    echo "openssl already installed at $OPENSSL_PREFIX (pass -f to rebuild)"
    exit 0
fi

TARBALL="openssl-$OPENSSL_VERSION.tar.gz"
SRCDIR="$OPENSSL_WORKDIR/openssl-$OPENSSL_VERSION"

mkdir -p "$OPENSSL_WORKDIR"
cd "$OPENSSL_WORKDIR"

if [ ! -f "$TARBALL" ]; then
    echo "==> downloading $TARBALL from openssl.org"
    curl -fL --retry 3 -o "$TARBALL.part" \
        "https://www.openssl.org/source/$TARBALL"
    mv "$TARBALL.part" "$TARBALL"
fi

# openssl.org publishes a detached sha256 next to the tarball; use it when it is
# reachable, but do not make the build depend on it being served.
if curl -fsL -o "$TARBALL.sha256" "https://www.openssl.org/source/$TARBALL.sha256" 2>/dev/null; then
    # The published file is BSD/coreutils form: "<hash> *<filename>"
    want=$(awk '{print $1; exit}' "$TARBALL.sha256")
    got=$(sha256sum "$TARBALL" | awk '{print $1}')
    if [ "$want" != "$got" ]; then
        echo "error: sha256 mismatch for $TARBALL" >&2
        echo "  expected $want" >&2
        echo "  got      $got" >&2
        exit 1
    fi
    echo "==> sha256 ok ($got)"
else
    echo "==> no published sha256 fetched, skipping checksum verification"
fi

if [ ! -f "$SRCDIR/Configure" ]; then
    echo "==> extracting"
    rm -rf "$SRCDIR"
    tar xf "$TARBALL"
    [ -f "$SRCDIR/Configure" ] || { echo "error: $TARBALL did not extract to $SRCDIR" >&2; exit 1; }
fi

cd "$SRCDIR"

# The mingw64 target assembles its x86_64 asm with nasm. Without nasm in PATH,
# fall back to the portable C implementations: correct, just slower AES.
ASM_OPT=no-asm
if command -v nasm >/dev/null 2>&1; then
    ASM_OPT=
    echo "==> nasm found, building with assembly"
else
    echo "==> nasm not found, building with no-asm (slower AES/SHA)"
fi

echo "==> configuring for mingw64, prefix $OPENSSL_PREFIX"
# Full compiler paths rather than --cross-compile-prefix: llvm-mingw drives the
# whole toolchain through the clang driver, and mixing the two makes Configure
# prepend the prefix to an already absolute CC.
./Configure mingw64 \
    no-shared \
    no-tests \
    no-apps \
    no-docs \
    $ASM_OPT \
    --prefix="$OPENSSL_PREFIX" \
    --openssldir="$OPENSSL_PREFIX/ssl" \
    --libdir=lib \
    CC="$BIN/$TRIPLE-clang" \
    CXX="$BIN/$TRIPLE-clang++" \
    AR="$BIN/$TRIPLE-ar" \
    RANLIB="$BIN/$TRIPLE-ranlib" \
    RC="$BIN/$TRIPLE-windres"

echo "==> building"
make -j"$(nproc)"

echo "==> installing (install_sw: libs + headers, no man pages)"
rm -rf "$OPENSSL_PREFIX"
make install_sw

echo
echo "openssl $OPENSSL_VERSION installed to $OPENSSL_PREFIX"
ls -la "$OPENSSL_PREFIX/lib"/libcrypto.a "$OPENSSL_PREFIX/include/openssl/evp.h"
