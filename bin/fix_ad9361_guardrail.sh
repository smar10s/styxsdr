#!/bin/bash
# SPDX-License-Identifier: MIT
# bin/fix_ad9361_guardrail.sh — Remove the ad9361→ad9363a boot guardrail
#
# The v0.39 plutosdr-fw u-boot environment contains a script that forcibly
# resets attr_val from "ad9361" to "ad9363a" on every boot for non-Rev.C
# boards. This prevents using the full AD9361 tuning range (70 MHz–6 GHz).
#
# This script overwrites the adi_loadvals_pluto env variable with a patched
# version that removes only that one guardrail. All other checks (refclk,
# mode validation, ad9364 DDS compat, Rev.C clock mux) remain intact.
#
# Usage: bin/fix_ad9361_guardrail.sh [pluto_ip]
#
# After running, set your config and reboot:
#   ssh root@192.168.2.1 'fw_setenv attr_name compatible && fw_setenv attr_val ad9361 && reboot'
#
# Verify after reboot:
#   ssh root@192.168.2.1 fw_printenv attr_val
#   # expected: attr_val=ad9361

set -euo pipefail

PLUTO_IP="${1:-${PLUTO_IP:-192.168.2.1}}"
PLUTO_USER="${PLUTO_USER:-root}"
PLUTO_PASS="${PLUTO_PASS:-analog}"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PATCH_SCRIPT="$SCRIPT_DIR/../packaging/patch_adi_loadvals.sh"

echo "=== Patching adi_loadvals_pluto on $PLUTO_IP ==="
echo "    Removes: ad9361→ad9363a forced reset on non-Rev.C boards"
echo ""

# Upload and execute the patch script on the Pluto
sshpass -p "$PLUTO_PASS" scp -O $SSH_OPTS "$PATCH_SCRIPT" ${PLUTO_USER}@${PLUTO_IP}:/tmp/patch_adi_loadvals.sh
sshpass -p "$PLUTO_PASS" ssh $SSH_OPTS ${PLUTO_USER}@${PLUTO_IP} 'chmod +x /tmp/patch_adi_loadvals.sh && /tmp/patch_adi_loadvals.sh && rm /tmp/patch_adi_loadvals.sh'

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to patch adi_loadvals_pluto"
    exit 1
fi

echo ""
echo "Patched successfully."
echo ""
echo "Now apply your AD9361 config:"
echo "  sshpass -p analog ssh $SSH_OPTS ${PLUTO_USER}@${PLUTO_IP} \\"
echo "    'fw_setenv attr_name compatible && fw_setenv attr_val ad9361 && reboot'"
echo ""
echo "Verify after reboot (~20s):"
echo "  sshpass -p analog ssh $SSH_OPTS ${PLUTO_USER}@${PLUTO_IP} fw_printenv attr_val"
