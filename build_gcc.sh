#!/bin/bash
# Build rnx2rtkp with GCC (MinGW-w64) on Windows
# Usage: bash build_gcc.sh
# Output: build/gcc/rnx2rtkp.exe
#
# All intermediate .o files go into build/gcc/ to keep the project clean.

set -e

# Add WinLibs GCC to PATH if not already available
if ! command -v gcc &>/dev/null; then
    WINLIBS_DIR="$(ls -d /c/Users/*/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_*/mingw64/bin 2>/dev/null | tail -1)"
    if [ -n "$WINLIBS_DIR" ]; then
        export PATH="$WINLIBS_DIR:$PATH"
    else
        echo "ERROR: gcc not found. Install WinLibs: winget install BrechtSanders.WinLibs.POSIX.UCRT"
        exit 1
    fi
fi

RTKLIB_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$RTKLIB_DIR/src"
BUILD="$RTKLIB_DIR/build/gcc"

mkdir -p "$BUILD"

# Compile flags — match CMakeLists.txt definitions
OPTS="-DWIN32 -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=3 -DNEXOBS=3"
CFLAGS="-std=c99 -Wall -O3 -Wno-unused-but-set-variable -Wno-stringop-truncation -I$SRC $OPTS"

# Source files for rnx2rtkp (post-processing only, no stream/server)
SRCS=(
    "$RTKLIB_DIR/app/consapp/rnx2rtkp/rnx2rtkp.c"
    "$SRC/rtkcmn.c"
    "$SRC/trace.c"
    "$SRC/rinex.c"
    "$SRC/rtkpos.c"
    "$SRC/postpos.c"
    "$SRC/solution.c"
    "$SRC/lambda.c"
    "$SRC/geoid.c"
    "$SRC/sbas.c"
    "$SRC/preceph.c"
    "$SRC/pntpos.c"
    "$SRC/ephemeris.c"
    "$SRC/options.c"
    "$SRC/ppp.c"
    "$SRC/ppp_ar.c"
    "$SRC/rtcm.c"
    "$SRC/rtcm2.c"
    "$SRC/rtcm3.c"
    "$SRC/rtcm3e.c"
    "$SRC/ionex.c"
    "$SRC/tides.c"
    "$SRC/sofa.c"
)

# Compile each source to object file (only if changed)
OBJS=()
NEED_LINK=0

for src in "${SRCS[@]}"; do
    name="$(basename "${src%.c}")"
    obj="$BUILD/$name.o"
    OBJS+=("$obj")

    if [ ! -f "$obj" ] || [ "$src" -nt "$obj" ] || [ "$SRC/rtklib.h" -nt "$obj" ]; then
        echo "  CC  $name.c"
        gcc $CFLAGS -c "$src" -o "$obj"
        NEED_LINK=1
    fi
done

# Link
EXE="$BUILD/rnx2rtkp.exe"
if [ $NEED_LINK -eq 1 ] || [ ! -f "$EXE" ]; then
    echo "  LD  rnx2rtkp.exe"
    gcc "${OBJS[@]}" -o "$EXE" -lm -lwinmm -lws2_32
fi

echo "Build complete: $EXE"
echo "$(gcc --version 2>&1 | head -1)"
