#!/bin/bash
# Iris FM data modem build script
# Usage: bash build.sh [debug|release|o0|o1|o2|o3|asan|ubsan|gui|gui-debug|gui-sdl|gui-sdl-debug]

set -e

MODE="${1:-gui}"
CXX="${CXX:-g++}"
OUTDIR="build"
BINARY="iris"

CXXFLAGS="-std=c++17 -Wall -Wextra -Wpedantic -I include -I third_party/monocypher -I third_party/zstd -I third_party/ppmd -I third_party/lzhuf -I third_party/mlkem"
IMGUI_FLAGS=""
SANITIZER_FLAGS=""

# Check for GUI mode and sanitizer modes
USE_IMGUI=0
USE_SDL2=0
OPT_LEVEL=""
case "$MODE" in
    o0)           OPT_LEVEL="-O0"; MODE="release" ;;
    o1)           OPT_LEVEL="-O1"; MODE="release" ;;
    o2)           OPT_LEVEL="-O2"; MODE="release" ;;
    o3)           OPT_LEVEL="-O3 -march=native -ffast-math"; MODE="release"; USE_IMGUI=1 ;;
    gui)          OPT_LEVEL="-O3 -march=native -ffast-math"; MODE="release"; USE_IMGUI=1 ;;
    gui-debug)    MODE="debug";   USE_IMGUI=1 ;;
    gui-sdl)      MODE="release"; USE_IMGUI=1; USE_SDL2=1 ;;
    gui-sdl-debug) MODE="debug";  USE_IMGUI=1; USE_SDL2=1 ;;
    asan)         MODE="debug";   SANITIZER_FLAGS="-fsanitize=address -fno-omit-frame-pointer" ;;
    ubsan)        MODE="debug";   SANITIZER_FLAGS="-fsanitize=undefined -fno-omit-frame-pointer" ;;
esac

if [ "$MODE" = "debug" ]; then
    CXXFLAGS="$CXXFLAGS -g -O0 -DDEBUG"
elif [ -n "$OPT_LEVEL" ]; then
    CXXFLAGS="$CXXFLAGS $OPT_LEVEL -DNDEBUG"
else
    CXXFLAGS="$CXXFLAGS -O2 -DNDEBUG"
fi

if [ -n "$SANITIZER_FLAGS" ]; then
    CXXFLAGS="$CXXFLAGS $SANITIZER_FLAGS"
fi

# Platform detection
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "mingw"* || "$OSTYPE" == "cygwin" ]]; then
    BINARY="iris.exe"
    CXXFLAGS="$CXXFLAGS -D_WIN32_WINNT=0x0601"
    LDFLAGS="-static -lws2_32 -lole32 -lwinmm -lbcrypt -lsetupapi -lhid"

    if [ "$USE_IMGUI" = "1" ]; then
        CXXFLAGS="$CXXFLAGS -DIRIS_HAS_IMGUI -I third_party/imgui -I third_party/imgui/backends"
        IMGUI_FLAGS="-lopengl32 -lgdi32 -ldwmapi"

        if [ "$USE_SDL2" = "1" ]; then
            CXXFLAGS="$CXXFLAGS -DIRIS_USE_SDL2 $(sdl2-config --cflags 2>/dev/null || echo -I/mingw64/include/SDL2)"
            IMGUI_FLAGS="$IMGUI_FLAGS $(sdl2-config --libs 2>/dev/null || echo -lSDL2)"
        fi
    fi
elif [[ "$OSTYPE" == "linux"* ]]; then
    # Audio backend: pulse > alsa > oss
    if pkg-config --exists libpulse-simple 2>/dev/null; then
        CXXFLAGS="$CXXFLAGS -DIRIS_USE_PULSE"
        LDFLAGS="-lpthread $(pkg-config --libs libpulse-simple)"
    elif pkg-config --exists alsa 2>/dev/null || [ -f /usr/include/alsa/asoundlib.h ]; then
        LDFLAGS="-lpthread -lasound"
    else
        CXXFLAGS="$CXXFLAGS -DIRIS_USE_OSS"
        LDFLAGS="-lpthread"
    fi

    if [ "$USE_IMGUI" = "1" ]; then
        CXXFLAGS="$CXXFLAGS -DIRIS_HAS_IMGUI -DIRIS_USE_SDL2 -I third_party/imgui -I third_party/imgui/backends"
        CXXFLAGS="$CXXFLAGS $(sdl2-config --cflags 2>/dev/null || pkg-config --cflags sdl2)"
        IMGUI_FLAGS="-lGL $(sdl2-config --libs 2>/dev/null || pkg-config --libs sdl2)"
        USE_SDL2=1
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    LDFLAGS="-lpthread -framework AudioToolbox -framework CoreAudio -framework CoreFoundation"

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

# C library optimization level (matches CXX for o3, else O2)
C_OPT="${OPT_LEVEL:--O2}"

# Parallel job count
NPROC=$(nproc 2>/dev/null || echo 4)

# Compile C libraries (in parallel)
CC="${CC:-gcc}"
MONOCYPHER_OBJ="$OUTDIR/monocypher.o"
ZSTD_OBJ="$OUTDIR/zstd.o"
LZHUF_OBJ="$OUTDIR/lzhuf.o"
MLKEM_OBJ="$OUTDIR/mlkem_native.o"

$CC -c $C_OPT $SANITIZER_FLAGS -I third_party/monocypher third_party/monocypher/monocypher.c -o "$MONOCYPHER_OBJ" &
$CC -c $C_OPT $SANITIZER_FLAGS -I third_party/zstd third_party/zstd/zstd.c -o "$ZSTD_OBJ" &
$CC -c $C_OPT $SANITIZER_FLAGS -DLZHUF -DB2F -I third_party/lzhuf third_party/lzhuf/lzhuf.c -o "$LZHUF_OBJ" &
$CC -c $C_OPT $SANITIZER_FLAGS -I third_party/mlkem -I third_party/mlkem/src third_party/mlkem/mlkem_native.c -o "$MLKEM_OBJ" &

PPMD_OBJS=""
for f in third_party/ppmd/Ppmd8.c third_party/ppmd/Ppmd8Enc.c third_party/ppmd/Ppmd8Dec.c; do
    OBJ="$OUTDIR/$(basename $f .c).o"
    $CC -c $C_OPT $SANITIZER_FLAGS -I third_party/ppmd "$f" -o "$OBJ" &
    PPMD_OBJS="$PPMD_OBJS $OBJ"
done
wait

SOURCES=$(find source -name '*.cc' -o -name '*.cpp' | sort)

if [ "$USE_IMGUI" = "1" ]; then
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

echo "  CXX: $CXX  (${NPROC} parallel jobs)"
echo "  Sources: $(echo $SOURCES | wc -w) files"

# Compile C++ sources in parallel
CXX_OBJS=""
for src in $SOURCES; do
    OBJ="$OUTDIR/$(echo "$src" | sed 's|/|_|g; s|\.cc$|.o|; s|\.cpp$|.o|')"
    CXX_OBJS="$CXX_OBJS $OBJ"
done

# Use xargs for parallel compilation
echo "$SOURCES" | tr ' ' '\n' | while read src; do
    OBJ="$OUTDIR/$(echo "$src" | sed 's|/|_|g; s|\.cc$|.o|; s|\.cpp$|.o|')"
    echo "$CXX $CXXFLAGS -c $src -o $OBJ"
done | xargs -P "$NPROC" -I CMD sh -c 'CMD'

# Link
$CXX $CXX_OBJS "$MONOCYPHER_OBJ" "$ZSTD_OBJ" $PPMD_OBJS "$LZHUF_OBJ" "$MLKEM_OBJ" -o "$OUTDIR/$BINARY" $LDFLAGS $IMGUI_FLAGS $SANITIZER_FLAGS

echo "=== Build complete: $OUTDIR/$BINARY ==="
ls -la "$OUTDIR/$BINARY"

# Install reminder
echo ""
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "mingw"* || "$OSTYPE" == "cygwin" ]]; then
    INSTALL_DIR="/c/Program Files/Iris"
    mkdir -p "$INSTALL_DIR" 2>/dev/null && cp "$OUTDIR/$BINARY" "$INSTALL_DIR/" && \
        echo "*** Installed to $INSTALL_DIR/$BINARY" || \
        echo "*** REMINDER: Install to Program Files with:
    cp $OUTDIR/$BINARY \"$INSTALL_DIR/\""
elif [[ "$OSTYPE" == "linux"* || "$OSTYPE" == "darwin"* ]]; then
    echo "*** REMINDER: Install with:"
    echo "    sudo install -m 755 $OUTDIR/$BINARY /usr/local/bin/$BINARY"
fi
