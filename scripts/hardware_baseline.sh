#!/bin/bash
# scripts/hardware_baseline.sh — Verify hardware state before starting work
#
# Checks:
#   1. Pluto reachable + firmware deployed
#   2. Build fingerprint match
#   3. ARM decode loopback at 5180 MHz (primary — 5 GHz, clean band)
#   4. ARM decode loopback at 915 MHz  (secondary — ISM band, same baseline)
#
# Results appended to logs/hardware.jsonl (persistent, append-only).
#
# Usage:
#   ./scripts/hardware_baseline.sh          # full check (~3 min)
#   ./scripts/hardware_baseline.sh --quick  # fingerprint only (~30 sec)

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PLUTO_IP="${PLUTO_IP:-192.168.2.1}"
PLUTO_PASS="${PLUTO_PASS:-analog}"
SSH="sshpass -p $PLUTO_PASS ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@$PLUTO_IP"

QUICK=false
[[ "${1:-}" == "--quick" ]] && QUICK=true

LOG_FILE="$PROJECT_DIR/logs/hardware.jsonl"
mkdir -p "$PROJECT_DIR/logs"

TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
BRANCH=$(git -C "$PROJECT_DIR" branch --show-current 2>/dev/null || echo "unknown")
GIT_SHA=$(git -C "$PROJECT_DIR" rev-parse --short HEAD 2>/dev/null || echo "unknown")

PASS=0
FAIL=0

result() {
    local status="$1" msg="$2"
    if [ "$status" = "PASS" ]; then
        echo "  [PASS] $msg"; PASS=$((PASS + 1))
    elif [ "$status" = "FAIL" ]; then
        echo "  [FAIL] $msg"; FAIL=$((FAIL + 1))
    else
        echo "  [WARN] $msg"
    fi
}

echo "=== Session Start: Hardware State Check ==="
echo ""

# ---- Step 1: Connectivity ----
echo "1. Connectivity & firmware deploy"
if ! $SSH "true" 2>/dev/null; then
    result "FAIL" "Cannot reach Pluto at $PLUTO_IP"
    exit 2
fi
result "PASS" "Pluto reachable at $PLUTO_IP"

# Deploy firmware tools
echo "  Deploying firmware..."
make -C "$PROJECT_DIR" deploy >/dev/null 2>&1
if $SSH "which pluto_loopback" >/dev/null 2>&1; then
    result "PASS" "Firmware deployed"
else
    result "FAIL" "Firmware deploy failed — run 'make firmware' first"
fi

echo ""

# ---- Step 2: Fingerprint ----
echo "2. Build fingerprint"
BUILD_ID_ADDR="0x43C00000"
RAW=$($SSH "devmem $BUILD_ID_ADDR 32" 2>/dev/null || echo "FAIL")
FINGERPRINT="unknown"

if [ "$RAW" = "FAIL" ]; then
    result "FAIL" "Cannot read fingerprint register"
else
    REG_HEX=$(echo "$RAW" | tr -d '[:space:]' | sed 's/0[xX]//' | tr '[:upper:]' '[:lower:]')
    FINGERPRINT="$REG_HEX"
    echo "  Hardware fingerprint: 0x$REG_HEX"
    if [ -f "$PROJECT_DIR/build/fpga/fingerprint" ]; then
        EXPECTED=$(cat "$PROJECT_DIR/build/fpga/fingerprint" | tr '[:upper:]' '[:lower:]')
        EXPECTED=$(echo "$EXPECTED" | sed 's/^0x//')
        if [ "$REG_HEX" = "$EXPECTED" ]; then
            result "PASS" "Fingerprint matches build/"
        else
            result "WARN" "Fingerprint MISMATCH — flashed != built (expected 0x$EXPECTED)"
        fi
    else
        result "WARN" "No build/fpga/fingerprint file"
    fi
fi

if [ "$QUICK" = true ]; then
    LOG_ENTRY="{\"timestamp\":\"${TIMESTAMP}\",\"event\":\"hardware_baseline\",\"branch\":\"${BRANCH}\",\"commit\":\"${GIT_SHA}\",\"fingerprint\":\"${FINGERPRINT}\",\"quick\":true}"
    echo "$LOG_ENTRY" >> "$LOG_FILE"
    echo ""
    echo "Quick check complete: PASS=$PASS FAIL=$FAIL"
    exit 0
fi

echo ""

# ---- Step 3: ARM decode at 5180 MHz (primary) ----
echo "3. ARM decode at 5180 MHz (5 GHz, 10 trials/rate)"
echo "   This proves the analog chain is clean."
ARM_JSON=$($SSH "pluto_loopback -f 5180 -a 3 -g 15 -n 10" 2>/dev/null || echo '{}')
ARM5180_OK=0
ARM5180_TOTAL=0
ARM5180_LOG=""
for rate in 6 9 12 18 24 36 48 54; do
    RATE_PASS=$(echo "$ARM_JSON" | grep -o "\"rate\":${rate}[^}]*\"pass\":[0-9]*" | grep -o '"pass":[0-9]*' | grep -o '[0-9]*')
    RATE_PASS=${RATE_PASS:-0}
    ARM5180_TOTAL=$((ARM5180_TOTAL + 1))
    if [ "$RATE_PASS" -ge 8 ]; then
        ARM5180_OK=$((ARM5180_OK + 1))
    else
        echo "    Rate ${rate}M: ${RATE_PASS}/10 (threshold: 8)"
    fi
    ARM5180_LOG="${ARM5180_LOG}\"${rate}\":${RATE_PASS},"
done
ARM5180_LOG="${ARM5180_LOG%,}"
ARM5180_PASS=$(echo "$ARM_JSON" | grep -o '"rates_passed":[0-9]*' | grep -o '[0-9]*')
ARM5180_PASS=${ARM5180_PASS:-0}
echo "  Rates passed: $ARM5180_PASS/8 (>=8/10 per-rate)"
if [ "$ARM5180_OK" -ge 6 ]; then
    result "PASS" "5180 MHz ARM decode: ${ARM5180_OK}/${ARM5180_TOTAL} rates >= 8/10"
else
    result "FAIL" "5180 MHz ARM decode: ${ARM5180_OK}/${ARM5180_TOTAL} rates >= 8/10"
fi

echo ""

# ---- Step 4: ARM decode at 915 MHz (secondary) ----
echo "4. ARM decode at 915 MHz (ISM band, 10 trials/rate)"
ARM_JSON2=$($SSH "pluto_loopback -f 915 -a 3 -g 15 -n 10" 2>/dev/null || echo '{}')
ARM915_OK=0
ARM915_TOTAL=0
ARM915_LOG=""
for rate in 6 9 12 18 24 36 48 54; do
    RATE_PASS=$(echo "$ARM_JSON2" | grep -o "\"rate\":${rate}[^}]*\"pass\":[0-9]*" | grep -o '"pass":[0-9]*' | grep -o '[0-9]*')
    RATE_PASS=${RATE_PASS:-0}
    ARM915_TOTAL=$((ARM915_TOTAL + 1))
    if [ "$RATE_PASS" -ge 8 ]; then
        ARM915_OK=$((ARM915_OK + 1))
    else
        echo "    Rate ${rate}M: ${RATE_PASS}/10 (threshold: 8)"
    fi
    ARM915_LOG="${ARM915_LOG}\"${rate}\":${RATE_PASS},"
done
ARM915_LOG="${ARM915_LOG%,}"
ARM915_PASS=$(echo "$ARM_JSON2" | grep -o '"rates_passed":[0-9]*' | grep -o '[0-9]*')
ARM915_PASS=${ARM915_PASS:-0}
echo "  Rates passed: $ARM915_PASS/8 (>=8/10 per-rate)"
if [ "$ARM915_OK" -ge 6 ]; then
    result "PASS" "915 MHz ARM decode: ${ARM915_OK}/${ARM915_TOTAL} rates >= 8/10"
else
    result "FAIL" "915 MHz ARM decode: ${ARM915_OK}/${ARM915_TOTAL} rates >= 8/10"
fi

echo ""

# ---- Log entry ----
LOG_ENTRY="{\"timestamp\":\"${TIMESTAMP}\",\"event\":\"hardware_baseline\",\"branch\":\"${BRANCH}\",\"commit\":\"${GIT_SHA}\",\"fingerprint\":\"${FINGERPRINT}\",\"arm_5180\":{${ARM5180_LOG}},\"arm_915\":{${ARM915_LOG}},\"trials\":10}"
echo "$LOG_ENTRY" >> "$LOG_FILE"

echo "========================================"
echo "SESSION START SUMMARY"
echo "  PASS: $PASS"
echo "  FAIL: $FAIL"
echo "  Logged to: logs/hardware.jsonl"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi
exit 0
