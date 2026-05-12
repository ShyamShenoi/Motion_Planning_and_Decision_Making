#!/bin/bash
# Downloads and installs the CARLA 0.9.9.4 C++ client library needed to build the planner.
# This extracts headers and prebuilt libraries from the CARLA 0.9.9.4 release.
# Run once before build_only.sh.
set -e

CARLA_VERSION="0.9.9.4"
CARLA_DIR="/opt/carla-source"

echo "=== Setting up CARLA $CARLA_VERSION build environment ==="
echo "=== This will install to $CARLA_DIR (requires sudo) ==="

# Install build dependencies
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libuv1-dev \
    libz-dev \
    python3-dev \
    wget \
    clang-8 \
    libc++-8-dev \
    libc++abi-8-dev

# Clone CARLA source at the correct tag
if [ ! -d "/tmp/carla_src" ]; then
    echo "=== Cloning CARLA $CARLA_VERSION source ==="
    git clone --depth=1 --branch $CARLA_VERSION \
        https://github.com/carla-simulator/carla.git /tmp/carla_src
fi

cd /tmp/carla_src

echo "=== Installing CARLA Python dependencies ==="
pip3 install --user -r requirements.txt 2>/dev/null || true

echo "=== Building LibCarla client library (takes 10-30 minutes) ==="
make setup
make LibCarla.client.release

echo "=== Installing to $CARLA_DIR ==="
sudo mkdir -p "$CARLA_DIR"
sudo cp -r /tmp/carla_src/LibCarla "$CARLA_DIR/"
sudo cp -r /tmp/carla_src/Build "$CARLA_DIR/"

echo "=== Build environment setup complete ==="
echo "=== You can now run: bash scripts/build_only.sh ==="
