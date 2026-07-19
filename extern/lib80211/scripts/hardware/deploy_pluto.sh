#!/bin/bash
# Deploy lib80211 ARM binaries to ADALM-Pluto and run tests.
#
# Prerequisites:
#   ./scripts/hardware/build_arm.sh   (builds ARM binaries)
#   Pluto connected and reachable at 192.168.2.1
#
# Usage:
#   ./scripts/hardware/deploy_pluto.sh              # deploy and test
#   ./scripts/hardware/deploy_pluto.sh --test-only  # skip deploy, just run tests

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build-arm"

PLUTO_HOST="${PLUTO_HOST:-root@192.168.2.1}"
PLUTO_PASS="${PLUTO_PASS:-analog}"
SSH="sshpass -p $PLUTO_PASS ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"
SCP="sshpass -p $PLUTO_PASS scp -O -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

# Check prerequisites
if ! command -v sshpass >/dev/null 2>&1; then
    echo "Error: sshpass required (brew install sshpass)" >&2
    exit 1
fi

if ! ping -c 1 -t 2 192.168.2.1 >/dev/null 2>&1; then
    echo "Error: Pluto not reachable at 192.168.2.1" >&2
    exit 1
fi

# Deploy binaries
if [ "${1:-}" != "--test-only" ]; then
    echo "Deploying to $PLUTO_HOST..."

    BINARIES=""
    for f in "$BUILD_DIR"/test/test_fft "$BUILD_DIR"/test/test_scrambler \
             "$BUILD_DIR"/test/test_mac "$BUILD_DIR"/tools/rx_file \
             "$BUILD_DIR"/tools/tx_beacon "$BUILD_DIR"/tools/gen_test_cf32 \
             "$BUILD_DIR"/tools/ota_rx "$BUILD_DIR"/tools/ota_tx_beacon; do
        if [ -f "$f" ]; then
            BINARIES="$BINARIES $f"
        fi
    done

    if [ -z "$BINARIES" ]; then
        echo "Error: no ARM binaries found. Run ./scripts/hardware/build_arm.sh first" >&2
        exit 1
    fi

    $SCP $BINARIES $PLUTO_HOST:/tmp/
    $SSH $PLUTO_HOST "chmod +x /tmp/test_* /tmp/rx_file /tmp/tx_beacon /tmp/gen_test_cf32 /tmp/ota_rx /tmp/ota_tx_beacon 2>/dev/null"
    echo "Deploy complete."
    echo ""
fi

# Run tests
echo "=== Running tests on Pluto ($(echo $PLUTO_HOST)) ==="
echo ""

PASS=0
FAIL=0

run_test() {
    local name="$1"
    local cmd="$2"
    echo "--- $name ---"
    if $SSH $PLUTO_HOST "$cmd" 2>&1; then
        PASS=$((PASS + 1))
    else
        FAIL=$((FAIL + 1))
    fi
    echo ""
}

run_test "test_mac" "/tmp/test_mac"
run_test "gen_test_cf32 + rx_file roundtrip" \
    "/tmp/gen_test_cf32 /tmp/test_beacon.cf32 && /tmp/rx_file -q /tmp/test_beacon.cf32 | grep -q '\"fcs_ok\": true' && echo PASS"
run_test "tx_beacon + rx_file roundtrip (3 beacons)" \
    "/tmp/tx_beacon -s PlutoARM -n 3 -o /tmp/b.cf32 && /tmp/rx_file -q /tmp/b.cf32 | grep -q '\"fcs_ok\": 3' && echo PASS"

echo "==============================="
echo "Results: $PASS passed, $FAIL failed"
echo "==============================="

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
