#!/bin/bash
# SPDX-License-Identifier: MIT
# scripts/validate.sh — Post-flash validation of PlutoSDR
#
# Checks:
#   1. Pluto is reachable via SSH
#   2. AD9361 initialized (IIO device present)
#   3. Build fingerprint matches expected
#
# Usage:
#   ./scripts/validate.sh                    # use build/fpga/fingerprint
#   ./scripts/validate.sh <expected_fp>      # explicit fingerprint

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PLUTO="root@192.168.2.1"
PLUTO_PASS="${PLUTO_PASS:-analog}"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

EXPECTED_FP="${1:-}"
if [ -z "$EXPECTED_FP" ] && [ -f "$PROJECT_DIR/build/fpga/fingerprint" ]; then
    EXPECTED_FP=$(cat "$PROJECT_DIR/build/fpga/fingerprint")
fi

ssh_cmd() {
    sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO "$@" 2>/dev/null
}

echo "=== Validating PlutoSDR ==="

# 1. Connectivity
echo -n "SSH connectivity... "
if ! ssh_cmd "echo ok" >/dev/null; then
    echo "FAIL (cannot reach 192.168.2.1)"
    echo "If Pluto shows as mass storage, USB mode may be RNDIS (broken on Mac)."
    exit 1
fi
echo "OK"

# 2. AD9361 / IIO (search all IIO devices — probe order varies with DTB patches)
echo -n "AD9361 (IIO)... "
AD9361_FOUND=$(ssh_cmd "grep -rl ad9361-phy /sys/bus/iio/devices/*/name 2>/dev/null | head -1" || echo "")
if [ -n "$AD9361_FOUND" ]; then
    echo "OK (ad9361-phy)"
else
    echo "FAIL (no ad9361-phy in any IIO device)"
    echo "AD9361 not initialized. Bitstream may be broken."
    exit 1
fi

# 3. Build fingerprint (read AXI build_id register at 0x43C00000)
if [ -n "$EXPECTED_FP" ]; then
    echo -n "Build fingerprint... "
    ACTUAL_FP=$(ssh_cmd "devmem 0x43C00000" || echo "")
    # Case-insensitive compare (devmem returns uppercase, fingerprint may be lower)
    ACTUAL_LOWER=$(echo "$ACTUAL_FP" | tr '[:upper:]' '[:lower:]')
    EXPECTED_LOWER=$(echo "$EXPECTED_FP" | tr '[:upper:]' '[:lower:]')
    if [ "$ACTUAL_LOWER" = "$EXPECTED_LOWER" ]; then
        echo "OK ($ACTUAL_FP)"
    else
        echo "MISMATCH (expected=$EXPECTED_FP actual=$ACTUAL_FP)"
        exit 1
    fi
fi

# 4. Kernel boot
echo -n "Kernel... "
KVER=$(ssh_cmd "uname -r")
echo "OK ($KVER)"

echo ""
echo "=== Validation PASSED ==="
