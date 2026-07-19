#!/bin/bash
# Build lib80211 for ARM (ADALM-Pluto Cortex-A9)
#
# Prerequisites:
#   Pluto connected (to extract libiio.so if not cached), or
#   docker/sysroot/usr/lib/libiio.so* already present.
#
# Usage:
#   ./scripts/hardware/build_arm.sh          # full build
#   ./scripts/hardware/build_arm.sh clean    # remove build-arm/ and rebuild
#   ./scripts/hardware/build_arm.sh rebuild  # force Docker image rebuild

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build-arm"
IMAGE="lib80211-pluto"
TOOLCHAIN="/src/cmake/toolchain-arm-linux-gnueabihf.cmake"

# Check for libiio sysroot
if [ ! -f "$PROJECT_DIR/docker/sysroot/usr/lib/libiio.so" ]; then
    echo "libiio.so not in sysroot, extracting from Pluto..."
    mkdir -p "$PROJECT_DIR/docker/sysroot/usr/lib"
    PLUTO_PASS="${PLUTO_PASS:-analog}"
    sshpass -p "$PLUTO_PASS" scp -O -o StrictHostKeyChecking=no \
        root@192.168.2.1:/usr/lib/libiio.so.0.25 \
        "$PROJECT_DIR/docker/sysroot/usr/lib/libiio.so.0.25"
    cd "$PROJECT_DIR/docker/sysroot/usr/lib"
    ln -sf libiio.so.0.25 libiio.so.0
    ln -sf libiio.so.0 libiio.so
    cd "$PROJECT_DIR"
fi

# Rebuild Docker image if requested or missing
if [ "${1:-}" = "rebuild" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Building Docker image..."
    docker build -t "$IMAGE" -f "$PROJECT_DIR/docker/Dockerfile.pluto" "$PROJECT_DIR/docker/"
    if [ "${1:-}" = "rebuild" ]; then
        rm -rf "$BUILD_DIR"
    fi
fi

# Clean if requested
if [ "${1:-}" = "clean" ]; then
    rm -rf "$BUILD_DIR"
fi

# Configure
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "Configuring..."
    docker run --rm -v "$PROJECT_DIR:/src" "$IMAGE" \
        cmake -B /src/build-arm -S /src \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
        -DCMAKE_BUILD_TYPE=Release
fi

# Build
echo "Building..."
docker run --rm -v "$PROJECT_DIR:/src" "$IMAGE" \
    cmake --build /src/build-arm --parallel

# Report
echo ""
echo "Build complete. Binaries:"
echo "  Library: build-arm/src/liblib80211.a"
file "$BUILD_DIR/src/liblib80211.a" 2>/dev/null || true
echo ""
echo "Test binaries:"
ls "$BUILD_DIR/test/test_"* 2>/dev/null | while read f; do
    echo "  $(basename $f)"
done
echo ""
echo "Tools (offline — static):"
for f in "$BUILD_DIR/tools/rx_file" "$BUILD_DIR/tools/tx_beacon" "$BUILD_DIR/tools/gen_test_cf32"; do
    [ -f "$f" ] && echo "  $(basename $f)"
done
echo ""
echo "Tools (OTA — dynamic, need libiio on device):"
for f in "$BUILD_DIR/tools/ota_rx" "$BUILD_DIR/tools/ota_tx_beacon"; do
    [ -f "$f" ] && echo "  $(basename $f)"
done
