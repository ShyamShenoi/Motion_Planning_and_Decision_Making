#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STARTER_DIR="$SCRIPT_DIR/../project/starter_files"

echo "=== Building C++ planner ==="

if [ -d "/opt/carla-source" ]; then
    # Full CARLA build (Udacity workspace or after setup_build_env.sh)
    echo "=== Using CARLA at /opt/carla-source ==="
    cd "$STARTER_DIR"
    cmake .
    make -j$(nproc)
    echo "=== Build complete: $STARTER_DIR/planner ==="
else
    # Local stub build — no CARLA installation needed
    echo "=== Local stub build (CARLA stubs for type definitions) ==="

    # Build uWebSockets locally if not already there
    if [ ! -f "/tmp/uWebSockets/build/libuWS.so" ]; then
        echo "=== Building uWebSockets ==="
        cd /tmp
        [ -d "uWebSockets" ] || git clone https://github.com/uWebSockets/uWebSockets
        cd uWebSockets && git checkout e94b6e1 2>/dev/null || true
        mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
        make -j$(nproc)
        echo "uWS built at /tmp/uWebSockets/build/libuWS.so"
    fi

    # Use CMakeLists.local.txt: swap it in, build, restore
    cd "$STARTER_DIR"
    ORIG="CMakeLists.txt"
    LOCAL="CMakeLists.local.txt"
    BACKUP="CMakeLists.txt.carla_backup"

    # Back up original CMakeLists.txt
    cp "$ORIG" "$BACKUP"

    # Swap in local version
    cp "$LOCAL" "$ORIG"

    # Clean previous config
    rm -f CMakeCache.txt

    # Build
    cmake .
    make -j$(nproc)

    # Restore original
    cp "$BACKUP" "$ORIG"
    rm -f "$BACKUP"

    echo "=== Build complete: $STARTER_DIR/planner ==="
fi
