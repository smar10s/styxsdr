#!/bin/bash
# SPDX-License-Identifier: MIT
# packaging/package.sh — Package bitstream + kernel + rootfs into pluto.frm
#
# Flow:
#   1. Stage files into plutosdr-fw/build/
#   2. Patch DTBs via device tree overlay (disable ADI DMA, reserve DDR)
#   3. Build FIT image with upstream scripts/pluto.its
#   4. Append MD5 trailer → pluto.frm
#
# Usage: packaging/package.sh <output_dir>
#
# Requires:
#   - Bitstream at <output_dir>/system_top.bit
#   - Built plutosdr-fw at extern/plutosdr-fw/
#   - dtc and fdtoverlay on PATH (or in plutosdr-fw buildroot host tools)
#
# Produces:
#   - <output_dir>/pluto.frm

set -euo pipefail

OUTPUT_DIR="${1:?Usage: $0 <output_dir>}"
case "$OUTPUT_DIR" in
    /*) ;;
    *) OUTPUT_DIR="$(pwd)/$OUTPUT_DIR" ;;
esac
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FW_DIR="$PROJECT_DIR/extern/plutosdr-fw"
OVERLAY_SRC="$SCRIPT_DIR/styx-pluto.dtso"

# ---- Validate inputs ----
if [ ! -f "$OUTPUT_DIR/system_top.bit" ]; then
    echo "ERROR: No bitstream at $OUTPUT_DIR/system_top.bit"
    exit 1
fi

if [ ! -f "$FW_DIR/buildroot/output/images/rootfs.cpio.gz" ]; then
    echo "ERROR: plutosdr-fw not built. Run 'make setup' first."
    exit 1
fi

if [ ! -f "$FW_DIR/linux/arch/arm/boot/zImage" ]; then
    echo "ERROR: No zImage at $FW_DIR/linux/arch/arm/boot/zImage"
    exit 1
fi

if [ ! -f "$FW_DIR/scripts/pluto.its" ]; then
    echo "ERROR: Upstream ITS not found at $FW_DIR/scripts/pluto.its"
    exit 1
fi

if [ ! -f "$OVERLAY_SRC" ]; then
    echo "ERROR: Device tree overlay not found at $OVERLAY_SRC"
    exit 1
fi

# ---- Locate tools ----
# mkimage: prefer u-boot-xlnx (matches Pluto's u-boot 2021.07), fall back to buildroot's
MKIMAGE="$FW_DIR/u-boot-xlnx/tools/mkimage"
if [ ! -x "$MKIMAGE" ]; then
    MKIMAGE="$FW_DIR/buildroot/output/host/bin/mkimage"
fi
if [ ! -x "$MKIMAGE" ]; then
    echo "ERROR: mkimage not found (need u-boot 2021.07 version, not system)"
    echo "  Checked: $FW_DIR/u-boot-xlnx/tools/mkimage"
    echo "  Checked: $FW_DIR/buildroot/output/host/bin/mkimage"
    exit 1
fi

# dtc + fdtoverlay: prefer buildroot's, fall back to system
DTC="$FW_DIR/buildroot/output/host/bin/dtc"
if [ ! -x "$DTC" ]; then
    DTC=$(command -v dtc) || { echo "ERROR: dtc not found"; exit 1; }
fi

FDTOVERLAY_BIN="$FW_DIR/buildroot/output/host/bin/fdtoverlay"
if [ ! -x "$FDTOVERLAY_BIN" ]; then
    FDTOVERLAY_BIN=$(command -v fdtoverlay) || { echo "ERROR: fdtoverlay not found"; exit 1; }
fi

# Ensure tools are on PATH for mkimage's internal use
export PATH="$FW_DIR/buildroot/output/host/bin:$PATH"

echo "=== Packaging pluto.frm ==="
echo "  mkimage:    $MKIMAGE"
echo "  dtc:        $DTC"
echo "  fdtoverlay: $FDTOVERLAY_BIN"
echo "  overlay:    $OVERLAY_SRC"

# ---- Stage files into plutosdr-fw/build/ ----
# scripts/pluto.its references ../build/* (relative to scripts/)
BUILD_DIR="$FW_DIR/build"
mkdir -p "$BUILD_DIR"

echo ""
echo "Staging files into $BUILD_DIR/"

cp "$OUTPUT_DIR/system_top.bit" "$BUILD_DIR/system_top.bit"
echo "  system_top.bit"

cp "$FW_DIR/linux/arch/arm/boot/zImage" "$BUILD_DIR/zImage"
echo "  zImage"

cp "$FW_DIR/buildroot/output/images/rootfs.cpio.gz" "$BUILD_DIR/rootfs.cpio.gz"
echo "  rootfs.cpio.gz"

# ---- Compile device tree overlay ----
echo ""
echo "Compiling device tree overlay..."
"$DTC" -q -@ -I dts -O dtb -o "$BUILD_DIR/styx-pluto.dtbo" "$OVERLAY_SRC"
echo "  styx-pluto.dtbo"

# ---- Apply overlay to stock DTBs ----
echo ""
echo "Patching device trees..."

DTB_SRC_DIR="$FW_DIR/linux/arch/arm/boot/dts"

for dtb_name in zynq-pluto-sdr.dtb zynq-pluto-sdr-revb.dtb zynq-pluto-sdr-revc.dtb; do
    src="$DTB_SRC_DIR/$dtb_name"
    if [ ! -f "$src" ]; then
        echo "  WARNING: $dtb_name not found (skipping)"
        continue
    fi

    "$FDTOVERLAY_BIN" -i "$src" -o "$BUILD_DIR/$dtb_name" "$BUILD_DIR/styx-pluto.dtbo"
    echo "  $dtb_name patched"
done

# ---- Build FIT image using upstream ITS ----
echo ""
echo "Building FIT image..."

# mkimage must run from plutosdr-fw root so scripts/pluto.its resolves ../build/*
cd "$FW_DIR"
"$MKIMAGE" -f scripts/pluto.its build/pluto.itb

# ---- Create pluto.frm (ITB + MD5 trailer) ----
# Format: [ITB bytes][32 hex chars of MD5][newline] = ITB + 33 bytes
md5sum build/pluto.itb | cut -d ' ' -f 1 > build/pluto.frm.md5
cat build/pluto.itb build/pluto.frm.md5 > build/pluto.frm

# ---- Copy to output ----
cp build/pluto.frm "$OUTPUT_DIR/pluto.frm"

SIZE=$(du -h "$OUTPUT_DIR/pluto.frm" | cut -f1)
echo ""
echo "=== Packaged: $OUTPUT_DIR/pluto.frm ($SIZE) ==="
