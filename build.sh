#!/bin/bash
# Iris FM data modem build script
# Usage: bash build.sh [debug|release]

set -e

MODE="${1:-release}"
CXX="${CXX:-g++}"
OUTDIR="build"
BINARY="iris"

CXXFLAGS="-std=c++17 -Wall -Wextra -Wpedantic -I include"

if [ "$MODE" = "debug" ]; then
    CXXFLAGS="$CXXFLAGS -g -O0 -DDEBUG"
else
    CXXFLAGS="$CXXFLAGS -O2 -DNDEBUG"
fi

# Platform detection
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "mingw"* || "$OSTYPE" == "cygwin" ]]; then
    BINARY="iris.exe"
    CXXFLAGS="$CXXFLAGS -D_WIN32_WINNT=0x0601"
    LDFLAGS="-lws2_32"
else
    LDFLAGS="-lpthread"
fi

mkdir -p "$OUTDIR"

SOURCES=$(find source -name '*.cc' -o -name '*.cpp' | sort)

echo "=== Building Iris ($MODE) ==="
echo "  CXX: $CXX"
echo "  Sources: $(echo $SOURCES | wc -w) files"

$CXX $CXXFLAGS $SOURCES -o "$OUTDIR/$BINARY" $LDFLAGS

echo "=== Build complete: $OUTDIR/$BINARY ==="
ls -la "$OUTDIR/$BINARY"
