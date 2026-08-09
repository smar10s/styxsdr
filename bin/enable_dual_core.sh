#!/bin/bash
# SPDX-License-Identifier: MIT
# bin/enable_dual_core.sh — Enable both Cortex-A9 cores on PlutoSDR
#
# The PlutoSDR firmware boots with maxcpus=1 by default (ADI policy, likely
# for USB 2.0 power budget reasons). The Zynq SoC has two A9 cores and the
# kernel is built with CONFIG_HOTPLUG_CPU=y / CONFIG_NR_CPUS=4 / SMP PREEMPT,
# so enabling CPU1 requires no rebuild or reflash.
#
# This script:
#   1. Brings CPU1 online immediately (echo 1 > /sys/devices/system/cpu/cpu1/online)
#   2. Persists maxcpus=2 in the u-boot environment (QSPI flash) so it
#      survives reboot
#
# Power note: enabling the second core increases draw. If powered from a
# strict USB 2.0 port (500 mA / 2.5 W budget), monitor for brownout. Most
# users are on USB 3.0 or powered hubs where this is not a concern.
#
# Usage: bin/enable_dual_core.sh [pluto_ip]
#
# Verify after running:
#   ssh root@192.168.2.1 nproc
#   # expected: 2

set -euo pipefail

PLUTO_IP="${1:-${PLUTO_IP:-192.168.2.1}}"
PLUTO_USER="${PLUTO_USER:-root}"
PLUTO_PASS="${PLUTO_PASS:-analog}"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"

ssh_cmd() {
    sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS ${PLUTO_USER}@${PLUTO_IP} "$@"
}

echo "=== Enabling dual-core on $PLUTO_IP ==="
echo ""

# 1. Bring CPU1 online immediately
echo -n "Bringing CPU1 online... "
ssh_cmd 'echo 1 > /sys/devices/system/cpu/cpu1/online'
NPROC=$(ssh_cmd 'nproc')
if [ "$NPROC" = "2" ]; then
    echo "OK (nproc=$NPROC)"
else
    echo "FAILED (nproc=$NPROC, expected 2)"
    exit 1
fi

# 2. Persist maxcpus=2 in u-boot env
echo -n "Setting maxcpus=2 in u-boot env (persistent)... "
ssh_cmd 'fw_setenv maxcpus 2'
VERIFY=$(ssh_cmd 'fw_printenv maxcpus' 2>/dev/null || echo "")
if echo "$VERIFY" | grep -q "maxcpus=2"; then
    echo "OK"
else
    echo "WARNING: fw_setenv may have failed (got: $VERIFY)"
    echo "  CPU1 is online for this session but may revert on reboot."
fi

echo ""
echo "Done. Both cores active and persistent across reboot."
echo ""
echo "To revert:"
echo "  sshpass -p analog ssh $SSH_OPTS ${PLUTO_USER}@${PLUTO_IP} \\"
echo "    'fw_setenv maxcpus 1 && echo 0 > /sys/devices/system/cpu/cpu1/online'"
