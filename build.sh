#!/bin/bash
# Iris FM data modem build script
# Usage: bash build.sh [debug|release|o0|o1|o2|o3|asan|ubsan|gui|gui-debug|gui-sdl|gui-sdl-debug|rpi|arm] [clean]

set -e

MODE="${1:-gui}"
CXX="${CXX:-g++}"
BINARY="iris"
CLEAN=""

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        clean) CLEAN=1 ;;
    esac
done

# ---------------------------------------------------------------------------
# RPi / aarch64-linux-gnu cross-compile target
# ---------------------------------------------------------------------------
# Runs inside WSL Ubuntu where aarch64-linux-gnu-g++ is installed.
# The workspace lives on a network share (X:) that WSL cannot mount, so
# sources are staged to C:\Users\kamer\AppData\Local\Temp\iris-cross-build
# (visible in WSL as /mnt/c/Users/kamer/AppData/Local/Temp/iris-cross-build)
# using Git Bash's built-in cp -r (avoids robocopy flag mangling by MSYS2).
# ---------------------------------------------------------------------------
if [[ "$MODE" == "rpi" || "$MODE" == "arm" ]]; then
    # Resolve the iris root directory regardless of where the script is called from
    IRIS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    OUTDIR="$IRIS_ROOT/build"
    mkdir -p "$OUTDIR"
    OUTPUT_BINARY="$OUTDIR/iris-arm"

    echo "=== Building Iris (aarch64-linux-gnu cross-compile for RPi) ==="

    # WSL distro to use
    WSL_DISTRO="Ubuntu-22.04"

    # Check that wsl.exe is available
    if ! command -v wsl.exe &>/dev/null; then
        echo "ERROR: wsl.exe not found. WSL 2 is required for cross-compilation."
        echo "Install WSL: wsl --install -d Ubuntu-22.04"
        exit 1
    fi

    # Verify the cross-compiler is installed in WSL
    if ! MSYS_NO_PATHCONV=1 wsl.exe -d "$WSL_DISTRO" -e sh -c \
            "command -v aarch64-linux-gnu-g++ >/dev/null 2>&1"; then
        echo "ERROR: aarch64-linux-gnu-g++ not found in WSL distro '$WSL_DISTRO'."
        echo "Install it with:"
        echo "  wsl -d $WSL_DISTRO -- sudo apt-get install -y g++-aarch64-linux-gnu gcc-aarch64-linux-gnu"
        exit 1
    fi

    # Staging dir on C: (accessible by both Git Bash and WSL)
    STAGE_BASH="/c/Users/kamer/AppData/Local/Temp/iris-cross-build"
    STAGE_WSL="/mnt/c/Users/kamer/AppData/Local/Temp/iris-cross-build"

    echo "  Staging sources to $STAGE_BASH ..."
    # Use cp -r (Git Bash built-in) — robocopy flags are mangled by MSYS2 path conversion
    rm -rf "$STAGE_BASH/source" "$STAGE_BASH/include" "$STAGE_BASH/third_party"
    cp -r "$IRIS_ROOT/source"   "$STAGE_BASH/source"
    cp -r "$IRIS_ROOT/include"  "$STAGE_BASH/include"
    mkdir -p "$STAGE_BASH/third_party"
    for pkg in monocypher zstd ppmd lzhuf mlkem; do
        cp -r "$IRIS_ROOT/third_party/$pkg" "$STAGE_BASH/third_party/$pkg"
    done
    mkdir -p "$STAGE_BASH/build/obj_arm"

    echo "  Staging done. Cross-compiling in WSL ($WSL_DISTRO)..."

    # Build script runs entirely inside WSL. Use a heredoc written to a temp file
    # so we don't have to escape everything in a -c "..." string.
    WSL_SCRIPT="$STAGE_BASH/wsl_cross_build.sh"
    cat > "$WSL_SCRIPT" << 'WSLEOF'
#!/bin/bash
set -e

STAGE="$1"
CXX_CROSS="aarch64-linux-gnu-g++"
CC_CROSS="aarch64-linux-gnu-gcc"
STRIP_CROSS="aarch64-linux-gnu-strip"
NPROC=$(nproc)
OBJDIR="$STAGE/build/obj_arm"

CXXFLAGS="-std=c++17 -O2 -ffast-math -march=armv8-a -Wall -Wextra -Wpedantic -DNDEBUG"
CXXFLAGS="$CXXFLAGS -I $STAGE/include"
CXXFLAGS="$CXXFLAGS -I $STAGE/third_party/monocypher"
CXXFLAGS="$CXXFLAGS -I $STAGE/third_party/zstd"
CXXFLAGS="$CXXFLAGS -I $STAGE/third_party/ppmd"
CXXFLAGS="$CXXFLAGS -I $STAGE/third_party/lzhuf"
CXXFLAGS="$CXXFLAGS -I $STAGE/third_party/mlkem"
CXXFLAGS="$CXXFLAGS -I $STAGE/third_party/mlkem/src"

# ALSA is the audio backend on RPi.
# When cross-compiling, libasound2-dev:arm64 installs headers to /usr/include/alsa/
# and the cross library to /usr/lib/aarch64-linux-gnu/libasound.so
LDFLAGS="-lpthread"
if [ -f "/usr/include/alsa/asoundlib.h" ] || \
   [ -f "/usr/aarch64-linux-gnu/include/alsa/asoundlib.h" ]; then
    LDFLAGS="$LDFLAGS -lasound"
    echo "  Audio: ALSA"
else
    echo "  Audio: OSS (ALSA cross-headers not found — install with: sudo apt-get install libasound2-dev:arm64)"
    CXXFLAGS="$CXXFLAGS -DIRIS_USE_OSS"
fi

mkdir -p "$OBJDIR"

# Flatten source path -> object filename (mirrors build.sh obj_name)
obj_name() {
    echo "$OBJDIR/$(echo "$1" | sed 's|/|_|g; s|\.cc$|.o|; s|\.cpp$|.o|; s|\.c$|.o|')"
}

PIDS=()
FAIL=0

wait_all() {
    for pid in "${PIDS[@]}"; do
        wait "$pid" || FAIL=1
    done
    PIDS=()
}

# --- C libraries ---
C_SRCS="
$STAGE/third_party/monocypher/monocypher.c
$STAGE/third_party/zstd/zstd.c
$STAGE/third_party/lzhuf/lzhuf.c
$STAGE/third_party/mlkem/mlkem_native.c
$STAGE/third_party/ppmd/Ppmd8.c
$STAGE/third_party/ppmd/Ppmd8Enc.c
$STAGE/third_party/ppmd/Ppmd8Dec.c
"

C_OBJS=""
for src in $C_SRCS; do
    [ -f "$src" ] || continue
    obj=$(obj_name "$src")
    C_OBJS="$C_OBJS $obj"
    EXTRA=""
    case "$src" in
        *lzhuf.c)       EXTRA="-DLZHUF -DB2F -I $STAGE/third_party/lzhuf" ;;
        *monocypher.c)  EXTRA="-I $STAGE/third_party/monocypher" ;;
        *zstd.c)        EXTRA="-I $STAGE/third_party/zstd" ;;
        *mlkem_native.c) EXTRA="-I $STAGE/third_party/mlkem -I $STAGE/third_party/mlkem/src" ;;
        *Ppmd*)         EXTRA="-I $STAGE/third_party/ppmd" ;;
    esac
    echo "  CC  $(basename $src)"
    $CC_CROSS -c -O2 -march=armv8-a $EXTRA "$src" -o "$obj" &
    PIDS+=($!)
done

# --- C++ sources ---
SOURCES=$(find "$STAGE/source" -name '*.cc' | sort)

CXX_OBJS=""
for src in $SOURCES; do
    obj=$(obj_name "$src")
    CXX_OBJS="$CXX_OBJS $obj"
    echo "  CXX $(basename $src)"
    $CXX_CROSS $CXXFLAGS -c "$src" -o "$obj" &
    PIDS+=($!)
done

echo "  (compiling $NPROC jobs in parallel...)"
wait_all
if [ "$FAIL" = "1" ]; then
    echo "*** Compilation failed"
    exit 1
fi

echo "  Linking..."
$CXX_CROSS $CXX_OBJS $C_OBJS -o "$STAGE/build/iris-arm" $LDFLAGS

echo "  Stripping..."
$STRIP_CROSS "$STAGE/build/iris-arm"

echo "  Done."
ls -lh "$STAGE/build/iris-arm"
WSLEOF

    chmod +x "$WSL_SCRIPT"

    # Run the build script inside WSL, passing the WSL-side stage path as $1
    MSYS_NO_PATHCONV=1 wsl.exe -d "$WSL_DISTRO" -e bash "$STAGE_WSL/wsl_cross_build.sh" "$STAGE_WSL"
    WSL_EXIT=$?

    if [ "$WSL_EXIT" -ne 0 ]; then
        echo "*** Cross-compile failed (WSL exit $WSL_EXIT)"
        exit 1
    fi

    # Copy binary back from the staging dir to the workspace build/ directory
    cp "$STAGE_BASH/build/iris-arm" "$OUTPUT_BINARY"

    echo "=== Build complete: $OUTPUT_BINARY ==="
    ls -la "$OUTPUT_BINARY"
    echo ""
    echo "*** Upload to RPi via butler:"
    echo "    UPLOAD <lease_id> rpi1 $OUTPUT_BINARY /home/pi/iris-arm"
    exit 0
fi
# ---------------------------------------------------------------------------
# End RPi cross-compile target
# ---------------------------------------------------------------------------

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
    o3-nogui)     OPT_LEVEL="-O3 -march=native -ffast-math"; MODE="release"; USE_IMGUI=0 ;;
    gui)          OPT_LEVEL="-O3 -march=native -ffast-math"; MODE="release"; USE_IMGUI=1 ;;
    gui-debug)    MODE="debug";   USE_IMGUI=1 ;;
    gui-sdl)      MODE="release"; USE_IMGUI=1; USE_SDL2=1 ;;
    gui-sdl-debug) MODE="debug";  USE_IMGUI=1; USE_SDL2=1 ;;
    asan)         MODE="debug";   SANITIZER_FLAGS="-fsanitize=address -fno-omit-frame-pointer" ;;
    ubsan)        MODE="debug";   SANITIZER_FLAGS="-fsanitize=undefined -fno-omit-frame-pointer" ;;
esac

if [ "$MODE" = "debug" ]; then
    CXXFLAGS="$CXXFLAGS -g -O0 -DDEBUG -UNDEBUG"
elif [ -n "$OPT_LEVEL" ]; then
    CXXFLAGS="$CXXFLAGS $OPT_LEVEL -DNDEBUG"
else
    CXXFLAGS="$CXXFLAGS -O2 -DNDEBUG"
fi

if [ -n "$SANITIZER_FLAGS" ]; then
    CXXFLAGS="$CXXFLAGS $SANITIZER_FLAGS"
fi

# Build directory (separate per mode)
OUTDIR="build"
OBJDIR="build/obj_${MODE}"

# Clean function
do_clean() {
    echo "Cleaning..."
    rm -rf build/
    echo "Clean done."
}

if [ "$CLEAN" = "1" ]; then
    do_clean
    if [ "$MODE" = "clean" ] || [ "$1" = "clean" ]; then
        exit 0
    fi
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
    # Audio backend: ALSA preferred, PulseAudio available via --pulse flag
    # Both are compiled and linked when available; runtime selection in audio_alsa.cc
    ALSA_LIBS=""
    if pkg-config --exists alsa 2>/dev/null || [ -f /usr/include/alsa/asoundlib.h ]; then
        ALSA_LIBS="-lasound"
    fi

    if [ -n "$ALSA_LIBS" ]; then
        LDFLAGS="-lpthread $ALSA_LIBS"
        # Also link PulseAudio if available (for --pulse runtime flag)
        if pkg-config --exists libpulse-simple 2>/dev/null; then
            CXXFLAGS="$CXXFLAGS -DIRIS_USE_PULSE"
            LDFLAGS="$LDFLAGS $(pkg-config --libs libpulse-simple)"
        fi
    elif pkg-config --exists libpulse-simple 2>/dev/null; then
        CXXFLAGS="$CXXFLAGS -DIRIS_USE_PULSE"
        LDFLAGS="-lpthread $(pkg-config --libs libpulse-simple)"
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

mkdir -p "$OUTDIR" "$OBJDIR"

# C library optimization level (matches CXX for o3, else O2)
C_OPT="${OPT_LEVEL:--O2}"

# Parallel job count
NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# --- Dependency-tracked parallel compilation ---

PIDS=()
FAIL=0

wait_slot() {
    while [ ${#PIDS[@]} -ge "$NPROC" ]; do
        local new_pids=()
        for pid in "${PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                new_pids+=("$pid")
            else
                wait "$pid" || FAIL=1
            fi
        done
        if [ ${#new_pids[@]} -ge "$NPROC" ]; then
            wait -n 2>/dev/null || {
                for pid in "${PIDS[@]}"; do wait "$pid" || FAIL=1; done
                PIDS=()
                return
            }
            new_pids=()
            for pid in "${PIDS[@]}"; do
                if kill -0 "$pid" 2>/dev/null; then
                    new_pids+=("$pid")
                else
                    wait "$pid" || FAIL=1
                fi
            done
        fi
        PIDS=("${new_pids[@]}")
    done
}

wait_all() {
    for pid in "${PIDS[@]}"; do
        wait "$pid" || FAIL=1
    done
    PIDS=()
    if [ "$FAIL" = "1" ]; then
        echo "*** Compilation failed"
        exit 1
    fi
}

# needs_rebuild SRC OBJ — true if obj missing, src newer, or any dep header changed
needs_rebuild() {
    local src="$1" obj="$2"
    [ ! -f "$obj" ] && return 0
    [ "$src" -nt "$obj" ] && return 0
    local dep="${obj%.o}.d"
    if [ -f "$dep" ]; then
        while IFS= read -r line; do
            line="${line%\\}"
            line="${line#*: }"
            for f in $line; do
                [ -f "$f" ] && [ "$f" -nt "$obj" ] && return 0
            done
        done < "$dep"
    else
        return 0
    fi
    return 1
}

# obj_name: flatten source path to single filename in objdir
obj_name() {
    echo "$OBJDIR/$(echo "$1" | sed 's|/|_|g; s|\.cc$|.o|; s|\.cpp$|.o|; s|\.c$|.o|')"
}

# Collect sources
SOURCES=$(find source -name '*.cc' -o -name '*.cpp' | sort)

IMGUI_SOURCES=""
if [ "$USE_IMGUI" = "1" ]; then
    IMGUI_SOURCES="third_party/imgui/imgui.cpp third_party/imgui/imgui_draw.cpp third_party/imgui/imgui_tables.cpp third_party/imgui/imgui_widgets.cpp third_party/imgui/backends/imgui_impl_opengl3.cpp"

    if [ "$USE_SDL2" = "1" ]; then
        IMGUI_SOURCES="$IMGUI_SOURCES third_party/imgui/backends/imgui_impl_sdl2.cpp"
        echo "=== Building Iris ($MODE + Dear ImGui + SDL2) ==="
    else
        IMGUI_SOURCES="$IMGUI_SOURCES third_party/imgui/backends/imgui_impl_win32.cpp"
        echo "=== Building Iris ($MODE + Dear ImGui + Win32) ==="
    fi
else
    echo "=== Building Iris ($MODE) ==="
fi

echo "  CXX: $CXX  (${NPROC} parallel jobs)"

# C library sources
C_LIB_SOURCES="
third_party/monocypher/monocypher.c
third_party/zstd/zstd.c
third_party/lzhuf/lzhuf.c
third_party/mlkem/mlkem_native.c
third_party/ppmd/Ppmd8.c
third_party/ppmd/Ppmd8Enc.c
third_party/ppmd/Ppmd8Dec.c
"

# --- Compile C libraries ---
CC="${CC:-gcc}"
C_OBJS=""
C_COMPILED=0

for src in $C_LIB_SOURCES; do
    obj=$(obj_name "$src")
    C_OBJS="$C_OBJS $obj"
    if needs_rebuild "$src" "$obj"; then
        EXTRA_C=""
        case "$src" in
            *lzhuf.c) EXTRA_C="-DLZHUF -DB2F -I third_party/lzhuf" ;;
            *monocypher.c) EXTRA_C="-I third_party/monocypher" ;;
            *zstd.c) EXTRA_C="-I third_party/zstd" ;;
            *mlkem_native.c) EXTRA_C="-I third_party/mlkem -I third_party/mlkem/src" ;;
            *Ppmd*) EXTRA_C="-I third_party/ppmd" ;;
        esac
        echo "  $src"
        wait_slot
        dep="${obj%.o}.d"
        $CC -c $C_OPT $SANITIZER_FLAGS -MMD -MF "$dep" $EXTRA_C "$src" -o "$obj" &
        PIDS+=($!)
        ((C_COMPILED++)) || true
    fi
done

# --- Compile C++ sources ---
CXX_OBJS=""
CXX_COMPILED=0

for src in $SOURCES $IMGUI_SOURCES; do
    obj=$(obj_name "$src")
    CXX_OBJS="$CXX_OBJS $obj"
    if needs_rebuild "$src" "$obj"; then
        echo "  $src"
        wait_slot
        dep="${obj%.o}.d"
        $CXX $CXXFLAGS -MMD -MF "$dep" -c "$src" -o "$obj" &
        PIDS+=($!)
        ((CXX_COMPILED++)) || true
    fi
done

wait_all

TOTAL=$((C_COMPILED + CXX_COMPILED))
if [ "$TOTAL" -eq 0 ] && [ -f "$OUTDIR/$BINARY" ]; then
    echo "  (nothing changed)"
    echo "=== Build complete: $OUTDIR/$BINARY ==="
    ls -la "$OUTDIR/$BINARY"
    echo ""
    if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "mingw"* || "$OSTYPE" == "cygwin" ]]; then
        INSTALL_DIR="/c/Program Files/Iris"
        mkdir -p "$INSTALL_DIR" 2>/dev/null && cp "$OUTDIR/$BINARY" "$INSTALL_DIR/" && \
            echo "*** Installed to $INSTALL_DIR/$BINARY" || true
    fi
    exit 0
fi

echo "  $TOTAL files compiled"

# Link
echo "Linking $OUTDIR/$BINARY..."
$CXX $CXX_OBJS $C_OBJS -o "$OUTDIR/$BINARY" $LDFLAGS $IMGUI_FLAGS $SANITIZER_FLAGS

echo "=== Build complete: $OUTDIR/$BINARY ==="
ls -la "$OUTDIR/$BINARY"

# Install
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
