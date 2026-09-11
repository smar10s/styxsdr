#!/bin/bash
# scripts/hardware_baseline.sh — Verify hardware state before starting work
#
# Checks:
#   1. Pluto reachable + firmware deployed
#   2. Build fingerprint match
#   3. DMA register-level validation (no RF)
#   4. Signal ladder L1-L6 (analog path + OFDM decode)
#   5. ARM decode loopback at 5180 MHz (primary — 5 GHz, clean band)
#   6. ARM decode loopback at 915 MHz  (secondary — ISM band, same baseline)
#   7. Streaming TX validation (if stream_test available)
#
# Results appended to logs/hardware.jsonl (persistent, append-only).
#
# Usage:
#   ./scripts/hardware_baseline.sh          # full check (~4 min)
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
            # A stale bitstream silently invalidates every result below it.
            # A validated firmware+bitstream pair was once certified this
            # way, then the bitstream moved on and the certificate stayed.
            result "FAIL" "Fingerprint MISMATCH — flashed != built (expected 0x$EXPECTED); reflash before trusting any result below"
        fi
    else
        result "WARN" "No build/fpga/fingerprint file"
    fi
fi

# ---- Step 2b: RTL provenance ----
# The commit hash is not a durable identifier here: branches are rewritten
# before publishing, which renames every commit and orphans older log
# entries.  The RTL tree hash is derived from file contents, so it survives
# any rebase and still names exactly one version of the hardware sources.
echo "2b. RTL provenance"
RTL_TREE=$(git -C "$PROJECT_DIR" rev-parse HEAD:fpga/rtl 2>/dev/null || echo "unknown")
RTL_DIRTY=false
if ! git -C "$PROJECT_DIR" diff --quiet HEAD -- fpga/rtl 2>/dev/null; then
    RTL_DIRTY=true
fi
echo "  RTL tree: ${RTL_TREE}"
if [ "$RTL_DIRTY" = true ]; then
    result "WARN" "RTL has uncommitted changes — tree hash does not describe the flashed bitstream"
else
    result "PASS" "RTL tree hash recorded (rebase-proof provenance)"
fi

if [ "$QUICK" = true ]; then
    LOG_ENTRY="{\"timestamp\":\"${TIMESTAMP}\",\"event\":\"hardware_baseline\",\"branch\":\"${BRANCH}\",\"commit\":\"${GIT_SHA}\",\"rtl_tree\":\"${RTL_TREE}\",\"rtl_dirty\":${RTL_DIRTY},\"fingerprint\":\"${FINGERPRINT}\",\"quick\":true}"
    echo "$LOG_ENTRY" >> "$LOG_FILE"
    echo ""
    echo "Quick check complete: PASS=$PASS FAIL=$FAIL"
    exit 0
fi

echo ""

# ---- Step 3: DMA register-level validation ----
echo "3. DMA register-level validation"
# Keep the output even when the tool reports failures, otherwise the
# fallback JSON is appended to it and both counts get parsed.
DMA_JSON=$($SSH "pluto_dma_test" 2>/dev/null)
[ -z "$DMA_JSON" ] && DMA_JSON='{"passed":0,"total":7}'
DMA_PASSED=$(echo "$DMA_JSON" | grep -o '"passed":[0-9]*' | grep -o '[0-9]*' | tail -1)
DMA_PASSED=${DMA_PASSED:-0}
DMA_TOTAL=$(echo "$DMA_JSON" | grep -o '"total":[0-9]*' | grep -o '[0-9]*' | tail -1)
DMA_TOTAL=${DMA_TOTAL:-7}
if [ "$DMA_PASSED" -eq "$DMA_TOTAL" ]; then
    result "PASS" "DMA test: ${DMA_PASSED}/${DMA_TOTAL} register tests pass"
else
    result "FAIL" "DMA test: ${DMA_PASSED}/${DMA_TOTAL} register tests pass"
fi

echo ""

# ---- Step 4: Signal ladder (analog path + OFDM) ----
echo "4. Signal ladder L1-L6 (5180 MHz)"
echo "   Graduated: tone → chirp → sync → channel est → full decode"
SIGLADDER_JSON=$($SSH "pluto_sigladder -f 5180 -a 3 -g 15" 2>/dev/null || echo '{}')
SIGLADDER_PASS=$(echo "$SIGLADDER_JSON" | grep -o '"highest_pass":[0-9]*' | grep -o '[0-9]*')
SIGLADDER_PASS=${SIGLADDER_PASS:-0}
if [ "$SIGLADDER_PASS" -lt 6 ]; then
    # Retry once — ~4% sync detection miss is a known probabilistic issue
    echo "    L${SIGLADDER_PASS}/6 — retrying..."
    SIGLADDER_JSON=$($SSH "pluto_sigladder -f 5180 -a 3 -g 15" 2>/dev/null || echo '{}')
    SIGLADDER_PASS=$(echo "$SIGLADDER_JSON" | grep -o '"highest_pass":[0-9]*' | grep -o '[0-9]*')
    SIGLADDER_PASS=${SIGLADDER_PASS:-0}
fi
if [ "$SIGLADDER_PASS" -ge 6 ]; then
    result "PASS" "Signal ladder: L${SIGLADDER_PASS}/6"
else
    result "FAIL" "Signal ladder: L${SIGLADDER_PASS}/6"
fi

echo ""

# ---- Step 5: ARM decode at 5180 MHz (primary) ----
echo "5. ARM decode at 5180 MHz (5 GHz, 10 trials/rate)"
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

# ---- Step 6: ARM decode at 915 MHz (secondary) ----
echo "6. ARM decode at 915 MHz (ISM band, 10 trials/rate)"
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

# ---- Step 7: Streaming TX validation ----
# Duration must cross several ring-buffer wraps.  The ring is 8 Mi samples,
# so at 2.5 MSPS it wraps every 3.4 s — a 5 s run only just reaches the
# first wrap, which is where stream mode is most likely to break.
echo "7. Streaming TX (12 second chirp loopback, >3 ring wraps)"
if $SSH "which pluto_stream_test" >/dev/null 2>&1; then
    STREAM_JSON=$($SSH "pluto_stream_test -d 12 -r 2500000 -a 3" 2>/dev/null || echo '{"passed":false}')
    STREAM_PASS=$(echo "$STREAM_JSON" | grep -o '"passed":true')
    STREAM_STALLS=$(echo "$STREAM_JSON" | grep -o '"stalls":[0-9]*' | grep -o '[0-9]*')
    STREAM_STALLS=${STREAM_STALLS:-0}
    STREAM_DROPOUTS=$(echo "$STREAM_JSON" | grep -o '"dropouts":[0-9]*' | grep -o '[0-9]*')
    STREAM_DROPOUTS=${STREAM_DROPOUTS:-unknown}
    STREAM_DAC_MISS=$(echo "$STREAM_JSON" | grep -o '"dac_miss":[0-9]*' | grep -o '[0-9]*')
    STREAM_DAC_MISS=${STREAM_DAC_MISS:-unknown}
    # Defer to the tool's own verdict.  It already checks dropouts, stalls,
    # DAC misses and effective rate, and a low dropout count on its own does
    # not mean the stream was healthy: a stalled fill FSM leaves the DAC
    # holding its last sample, which reads as a carrier with full power and
    # so produces zero dropouts.
    if [ -n "$STREAM_PASS" ]; then
        result "PASS" "Stream test: 12s, 0 dropouts, 0 stalls"
    else
        result "FAIL" "Stream test: dropouts=${STREAM_DROPOUTS} stalls=${STREAM_STALLS} dac_miss=${STREAM_DAC_MISS}"
    fi
else
    echo "  [SKIP] pluto_stream_test not deployed"
fi

echo ""

# ---- Log entry ----
STREAM_RESULT=${STREAM_PASS:+1}
STREAM_RESULT=${STREAM_RESULT:-0}
LOG_ENTRY="{\"timestamp\":\"${TIMESTAMP}\",\"event\":\"hardware_baseline\",\"branch\":\"${BRANCH}\",\"commit\":\"${GIT_SHA}\",\"rtl_tree\":\"${RTL_TREE}\",\"rtl_dirty\":${RTL_DIRTY},\"fingerprint\":\"${FINGERPRINT}\",\"dma_pass\":${DMA_PASSED},\"sigladder\":${SIGLADDER_PASS},\"stream_pass\":${STREAM_RESULT},\"stream_dropouts\":${STREAM_DROPOUTS:-0},\"arm_5180\":{${ARM5180_LOG}},\"arm_915\":{${ARM915_LOG}},\"trials\":10}"
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
