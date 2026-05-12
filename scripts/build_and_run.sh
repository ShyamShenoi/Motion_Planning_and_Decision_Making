#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR/../project"
STARTER_DIR="$PROJECT_DIR/starter_files"

echo "=== Installing project dependencies (uWebSockets) ==="
cd "$PROJECT_DIR"
bash install-ubuntu.sh

echo "=== Building C++ planner ==="
cd "$STARTER_DIR"
cmake .
make -j$(nproc)

echo "=== Build successful. Binary: $STARTER_DIR/planner ==="
echo "=== Starting planner against CARLA... ==="
cd "$PROJECT_DIR"
./run_main.sh
