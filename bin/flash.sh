#!/bin/bash
# SPDX-License-Identifier: MIT
# scripts/flash.sh — Flash pluto.frm to PlutoSDR via mtd3
#
# Uploads firmware to Pluto over USB-network, erases flash, writes,
# and reboots. Does NOT use DFU — this is the safe mtd3 update path.
#
# EXPECTED WALL TIME: 10-12 minutes (flash_erase is the bottleneck)
#
# Usage:
#   ./scripts/flash.sh              # flash build/fpga/pluto.frm
#   ./scripts/flash.sh path/to.frm  # flash specific file
#
# WARNING: flash_erase takes ~10 minutes. Be patient.
# The script will print progress. Do not interrupt.
#
# Prerequisites:
#   - Pluto connected via USB (192.168.2.1 reachable)
#   - sshpass installed (brew install hudochenkov/sshpass/sshpass)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FRM="${1:-$PROJECT_DIR/build/fpga/pluto.frm}"
PLUTO="root@192.168.2.1"
PLUTO_PASS="${PLUTO_PASS:-analog}"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

if [ ! -f "$FRM" ]; then
    echo "ERROR: $FRM not found"
    echo "Run 'make package' first, or specify path to .frm"
    exit 1
fi

echo "=== Firmware: $FRM ($(du -h "$FRM" | cut -f1)) ==="

# ---- Verify .frm checksum (last 33 bytes = MD5 of ITB payload) ----
TOTAL=$(wc -c < "$FRM" | tr -d ' ')
ITB_SIZE=$((TOTAL - 33))
MD5_EXPECTED=$(tail -c 33 "$FRM" | tr -d '\n')
MD5_ACTUAL=$(dd if="$FRM" bs=1 count=$ITB_SIZE 2>/dev/null | md5 -q)

if [ "$MD5_EXPECTED" != "$MD5_ACTUAL" ]; then
    echo "CHECKSUM MISMATCH — aborting"
    echo "  Expected: $MD5_EXPECTED"
    echo "  Actual:   $MD5_ACTUAL"
    exit 1
fi
echo "Checksum verified OK (ITB=$ITB_SIZE bytes)"

# ---- Check Pluto connectivity ----
echo ""
echo "=== Checking Pluto at 192.168.2.1 ==="
if ! sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO "echo ok" 2>/dev/null; then
    echo "ERROR: Cannot reach Pluto at 192.168.2.1"
    echo "Check USB connection. If Pluto appears as mass storage,"
    echo "it may be in RNDIS mode (broken on Mac). Fix config.txt."
    exit 1
fi
echo "Pluto reachable."

# ---- Upload ----
echo ""
echo "=== Uploading firmware ($(du -h "$FRM" | cut -f1)) ==="
sshpass -p "$PLUTO_PASS" scp -O $SSH_OPTS "$FRM" "$PLUTO:/tmp/pluto.frm"
echo "Upload complete."

# ---- Flash ----
echo ""
echo "=== Flashing (flash_erase takes ~10 min) ==="
sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO '
    set -e
    TOTAL=$(wc -c < /tmp/pluto.frm)
    ITB_SIZE=$((TOTAL - 33))
    echo "Stripping MD5 trailer -> ITB ($ITB_SIZE bytes)"
    dd if=/tmp/pluto.frm of=/tmp/fw.itb bs=1 count=$ITB_SIZE 2>/dev/null
    echo "Erasing mtd3 (patience — this is slow)..."
    flash_erase /dev/mtd3 0 0
    echo "Writing ITB to mtdblock3..."
    dd if=/tmp/fw.itb of=/dev/mtdblock3 bs=64k
    echo "Setting fit_size=$(printf "%X" $ITB_SIZE)"
    fw_setenv fit_size $(printf "%X" $ITB_SIZE)
    sync
    rm -f /tmp/pluto.frm /tmp/fw.itb
    echo "FLASH COMPLETE"
'

# ---- Reboot ----
echo ""
echo "=== Rebooting Pluto ==="
sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS $PLUTO "device_reboot reset" 2>/dev/null || true

echo ""
echo "Pluto is rebooting (~20 seconds)."
echo "Validate with: make validate"
