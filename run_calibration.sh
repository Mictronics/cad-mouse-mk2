#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

declare -A INSTRUCTIONS
INSTRUCTIONS[TX]="Slide the puck LEFT and RIGHT (translate only, no tilt or twist)"
INSTRUCTIONS[TY]="Slide the puck FORWARD and BACK (translate only, no tilt or twist)"
INSTRUCTIONS[TZ]="Press the puck straight DOWN then release (no tilt or sideways)"
INSTRUCTIONS[RX]="TILT the puck front-to-back (nod: front edge dips, then rises)"
INSTRUCTIONS[RY]="TILT the puck left-to-right (rock: left edge dips, then rises)"
INSTRUCTIONS[RZ]="TWIST the puck clockwise then counterclockwise around its centre"

DOFS=(TX TY TZ RX RY RZ)

echo ""
echo "========================================"
echo "  CAD Mouse Calibration Data Collection"
echo "========================================"
echo ""
echo "Plug in the CAD Mouse and wait for the LED to turn green (idle)."
echo "No reflashing required — calibration runs over USB HID."
echo ""

echo "You will collect one movement log per axis (6 total)."
echo "For each one:"
echo "  1. Read the instruction."
echo "  2. Press Enter to launch the monitor."
echo "  3. Make the movement slowly (stay well under the limit)."
echo "  4. Press Q to stop recording."
echo ""
echo "Press Ctrl+C at any time to abort."
echo ""

for DOF in "${DOFS[@]}"; do
    echo "----------------------------------------"
    echo "  Step: $DOF"
    echo "  ${INSTRUCTIONS[$DOF]}"
    echo "----------------------------------------"
    read -rp "  Press Enter when ready..."

    python3 monitor.py

    if [ ! -f mouse_monitor.log ]; then
        echo "  ERROR: mouse_monitor.log not found. Did monitor.py run correctly?"
        exit 1
    fi

    cp mouse_monitor.log "cal_${DOF}.log"
    LINES=$(wc -l < "cal_${DOF}.log")
    echo "  Saved cal_${DOF}.log  ($((LINES - 1)) samples)"
    echo ""
done

echo "========================================"
echo "  All 6 logs collected. Fitting matrix..."
echo "========================================"
echo ""

python3 calibrate.py \
    TX:cal_TX.log \
    TY:cal_TY.log \
    TZ:cal_TZ.log \
    RX:cal_RX.log \
    RY:cal_RY.log \
    RZ:cal_RZ.log

echo ""
echo "========================================"
echo "  Done."
echo "  The device is running the new calibration matrix."
echo "  firmware/include/Calibration.h was also updated as a fallback."
echo "========================================"
