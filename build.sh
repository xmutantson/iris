#!/bin/bash
# Iris FM data modem build script
# Usage: bash build.sh [debug|release|gui|gui-debug|gui-sdl|gui-sdl-debug]

set -e

MODE="${1:-release}"
CXX="${CXX:-g++}"
OUTDIR="build"
BINARY="iris"

CXXFLAGS="-std=c++17 -Wall -Wextra -Wpedantic -I include -I third_party/monocypher -I third_party/zstd"
IMGUI_FLAGS=""

# Check for GUI mode
USE_IMGUI=0
USE_SDL2=0
case "$MODE" in
    gui)          MODE="release"; USE_IMGUI=1 ;;
    gui-debug)    MODE="debug";   USE_IMGUI=1 ;;
    gui-sdl)      MODE="release"; USE_IMGUI=1; USE_SDL2=1 ;;
    gui-sdl-debug) MODE="debug";  USE_IMGUI=1; USE_SDL2=1 ;;
esac

if [ "$MODE" = "debug" ]; then
    CXXFLAGS="$CXXFLAGS -g -O0 -DDEBUG"
else
    CXXFLAGS="$CXXFLAGS -O2 -DNDEBUG"
fi

# Platform detection
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "mingw"* || "$OSTYPE" == "cygwin" ]]; then
    BINARY="iris.exe"
    CXXFLAGS="$CXXFLAGS -D_WIN32_WINNT=0x0601"
    LDFLAGS="-static -lws2_32 -lole32 -lwinmm -lbcrypt"

    if [ "$USE_IMGUI" = "1" ]; then
        CXXFLAGS="$CXXFLAGS -DIRIS_HAS_IMGUI -I third_party/imgui -I third_party/imgui/backends"
        IMGUI_FLAGS="-lopengl32 -lgdi32 -ldwmapi"

        if [ "$USE_SDL2" = "1" ]; then
            CXXFLAGS="$CXXFLAGS -DIRIS_USE_SDL2 $(sdl2-config --cflags 2>/dev/null || echo -I/mingw64/include/SDL2)"
            IMGUI_FLAGS="$IMGUI_FLAGS $(sdl2-config --libs 2>/dev/null || echo -lSDL2)"
        fi
    fi
elif [[ "$OSTYPE" == "linux"* ]]; then
    LDFLAGS="-lpthread -lasound"

    if [ "$USE_IMGUI" = "1" ]; then
        CXXFLAGS="$CXXFLAGS -DIRIS_HAS_IMGUI -DIRIS_USE_SDL2 -I third_party/imgui -I third_party/imgui/backends"
        CXXFLAGS="$CXXFLAGS $(sdl2-config --cflags 2>/dev/null || pkg-config --cflags sdl2)"
        IMGUI_FLAGS="-lGL $(sdl2-config --libs 2>/dev/null || pkg-config --libs sdl2)"
        USE_SDL2=1
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    LDFLAGS="-lpthread"

    if [ "$USE_IMGUI" = "1" ]; then
        CXXFLAGS="$CXXFLAGS -DIRIS_HAS_IMGUI -DIRIS_USE_SDL2 -I third_party/imgui -I third_party/imgui/backends"
        CXXFLAGS="$CXXFLAGS $(sdl2-config --cflags 2>/dev/null || pkg-config --cflags sdl2)"
        IMGUI_FLAGS="-framework OpenGL $(sdl2-config --libs 2>/dev/null || pkg-config --libs sdl2)"
        USE_SDL2=1
    fi
else
    LDFLAGS="-lpthread"
fi

mkdir -p "$OUTDIR"

# Compile C libraries
CC="${CC:-gcc}"
MONOCYPHER_OBJ="$OUTDIR/monocypher.o"
$CC -c -O2 -I third_party/monocypher third_party/monocypher/monocypher.c -o "$MONOCYPHER_OBJ"

ZSTD_OBJ="$OUTDIR/zstd.o"
$CC -c -O2 -I third_party/zstd third_party/zstd/zstd.c -o "$ZSTD_OBJ"

SOURCES=$(find source -name '*.cc' -o -name '*.cpp' | sort)

if [ "$USE_IMGUI" = "1" ]; then
    # Add Dear ImGui core
    IMGUI_SOURCES="third_party/imgui/imgui.cpp third_party/imgui/imgui_draw.cpp third_party/imgui/imgui_tables.cpp third_party/imgui/imgui_widgets.cpp third_party/imgui/backends/imgui_impl_opengl3.cpp"

    if [ "$USE_SDL2" = "1" ]; then
        IMGUI_SOURCES="$IMGUI_SOURCES third_party/imgui/backends/imgui_impl_sdl2.cpp"
        echo "=== Building Iris ($MODE + Dear ImGui + SDL2) ==="
    else
        IMGUI_SOURCES="$IMGUI_SOURCES third_party/imgui/backends/imgui_impl_win32.cpp"
        echo "=== Building Iris ($MODE + Dear ImGui + Win32) ==="
    fi

    SOURCES="$SOURCES $IMGUI_SOURCES"
else
    echo "=== Building Iris ($MODE) ==="
fi

echo "  CXX: $CXX"
echo "  Sources: $(echo $SOURCES | wc -w) files"

$CXX $CXXFLAGS $SOURCES "$MONOCYPHER_OBJ" "$ZSTD_OBJ" -o "$OUTDIR/$BINARY" $LDFLAGS $IMGUI_FLAGS

echo "=== Build complete: $OUTDIR/$BINARY ==="
ls -la "$OUTDIR/$BINARY"
