#!/usr/bin/env python3
"""
Fit a 6x9 decoupling matrix from per-DOF calibration log files and write it
directly into firmware/include/Config.h.

Usage (normally invoked by run_calibration.sh):

    python3 calibrate.py TX:cal_TX.log TY:cal_TY.log TZ:cal_TZ.log \\
                         RX:cal_RX.log RY:cal_RY.log RZ:cal_RZ.log

Tips for calibration movements:
  TX – slide the puck left/right only, slowly
  TY – slide the puck forward/back only, slowly
  TZ – press/lift the puck straight up/down
  RX – tilt the puck front/back (rock it, no sideways drift)
  RY – tilt the puck left/right (rock it, no forward drift)
  RZ – twist the puck around its vertical axis (no translation)

Keep movements slow to avoid hitting the ±350 output clamp.
"""

import os
import struct
import sys
import numpy as np

DOF_NAMES = ['TX', 'TY', 'TZ', 'RX', 'RY', 'RZ']

VID = 0x2886
PID = 0x0058
OUTPUT_REPORT_ID = 5

SENSOR_COLS = slice(1, 10)   # s0x .. s2z
MOTION_COLS = slice(10, 16)  # X .. Rz

SENSOR_ACTIVITY_THRESHOLD = 0.5

CALIBRATION_H = os.path.join(os.path.dirname(__file__), 'firmware', 'include', 'Calibration.h')


def load_log(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('sample'):
                continue
            parts = line.split('\t')
            if len(parts) < 16:
                continue
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue

    if not rows:
        raise ValueError(f"No valid data rows in {path}")

    data    = np.array(rows)
    sensors = data[:, SENSOR_COLS]
    motion  = data[:, MOTION_COLS]
    active  = np.max(np.abs(sensors), axis=1) > SENSOR_ACTIVITY_THRESHOLD

    if not active.any():
        raise ValueError(f"No active samples in {path}")

    return sensors[active], motion[active]


def fit_matrix(labeled_logs):
    all_sensors = np.vstack([s for _, s, _ in labeled_logs])
    M = np.zeros((6, 9))
    for j, dof in enumerate(DOF_NAMES):
        b = np.zeros(all_sensors.shape[0])
        offset = 0
        for dof_name, sensors, motion in labeled_logs:
            n = sensors.shape[0]
            if dof_name == dof:
                b[offset:offset + n] = motion[:, j]
            offset += n
        w, _, _, _ = np.linalg.lstsq(all_sensors, b, rcond=None)
        M[j] = w
    return M


def print_crosstalk_table(labeled_logs, M):
    print("\nCross-talk after correction (RMS predicted output per calibration log):")
    print(f"\n  {'Log':<6}  " + "  ".join(f"{d:>8}" for d in DOF_NAMES))
    print(f"  {'------':<6}  " + "  ".join("--------" for _ in DOF_NAMES))
    warnings = []
    for dof_name, sensors, _ in labeled_logs:
        pred = sensors @ M.T
        rms  = np.sqrt(np.mean(pred ** 2, axis=0))
        diag_idx = DOF_NAMES.index(dof_name)
        diag = rms[diag_idx]
        row  = "  ".join(
            f"{'>>':>2}{v:6.1f}" if i == diag_idx else f"{v:8.1f}"
            for i, v in enumerate(rms)
        )
        print(f"  {dof_name:<6}  {row}")
        for i, v in enumerate(rms):
            if i != diag_idx and diag > 0 and v / diag > 0.5:
                warnings.append(
                    f"  WARNING: {DOF_NAMES[i]} bleeds {v/diag*100:.0f}% into {dof_name} "
                    f"— consider recollecting cal_{dof_name}.log"
                )
    if warnings:
        print()
        for w in warnings:
            print(w)
    print()


def find_hidraw(vid, pid):
    base = "/sys/bus/hid/devices"
    try:
        entries = os.listdir(base)
    except FileNotFoundError:
        return None
    for entry in entries:
        parts = entry.split(":")
        if len(parts) < 3:
            continue
        try:
            e_vid = int(parts[1], 16)
            e_pid = int(parts[2].split(".")[0], 16)
        except ValueError:
            continue
        if e_vid != vid or e_pid != pid:
            continue
        hidraw_dir = os.path.join(base, entry, "hidraw")
        try:
            nodes = os.listdir(hidraw_dir)
        except FileNotFoundError:
            continue
        for node in nodes:
            return f"/dev/{node}"
    return None


def send_matrix_to_device(M):
    hidraw_path = find_hidraw(VID, PID)
    if not hidraw_path:
        print("  WARNING: Device not found — matrix not sent to device.")
        print("  Calibration.h was updated; reflash to apply it.")
        return
    flat = M.flatten().tolist()
    data = struct.pack("<B54f", OUTPUT_REPORT_ID, *flat)
    try:
        fd = os.open(hidraw_path, os.O_RDWR)
        os.write(fd, data)
        os.close(fd)
        print(f"  Calibration matrix sent to device ({hidraw_path}).")
        print("  The device will use it immediately and has saved it to flash.")
    except PermissionError:
        print(f"  WARNING: Cannot open {hidraw_path} — check udev rules.")
    except OSError as e:
        print(f"  WARNING: Failed to send matrix: {e}")


def write_calibration_h(M):
    rows = []
    for i, (row, dof) in enumerate(zip(M, DOF_NAMES)):
        vals  = ", ".join(f"{v:11.6f}f" for v in row)
        comma = "," if i < 5 else " "
        rows.append(f"    {{ {vals} }}{comma}  // {dof}")
    matrix_str = "\n".join(rows)

    content = f"""\
#pragma once

// Auto-generated by calibrate.py — do not edit by hand.
// Re-run ./run_calibration.sh to regenerate.
//
// Decoupling matrix: maps 9 baseline-subtracted sensor deltas -> 6 DOF outputs.
// Rows: TX, TY, TZ, RX, RY, RZ  |  Cols: mag1x,y,z  mag2x,y,z  mag3x,y,z

namespace Config {{

const float DECOUPLING_M[6][9] = {{
{matrix_str}
}};

}}  // namespace Config
"""
    with open(CALIBRATION_H, 'w') as f:
        f.write(content)
    print(f"Updated {CALIBRATION_H}")


def main():
    if len(sys.argv) < 7:
        print(__doc__)
        print("Usage: calibrate.py TX:file TY:file TZ:file RX:file RY:file RZ:file")
        sys.exit(1)

    labeled_logs = []
    for arg in sys.argv[1:]:
        if ':' not in arg:
            print(f"Bad argument '{arg}': expected DOF:filename (e.g. TX:cal_TX.log)")
            sys.exit(1)
        dof, path = arg.split(':', 1)
        dof = dof.upper()
        if dof not in DOF_NAMES:
            print(f"Unknown DOF '{dof}'. Must be one of: {DOF_NAMES}")
            sys.exit(1)
        sensors, motion = load_log(path)
        print(f"  Loaded {dof}: {path}  ({sensors.shape[0]} active samples)")
        labeled_logs.append((dof, sensors, motion))

    provided = {dof for dof, _, _ in labeled_logs}
    missing  = [d for d in DOF_NAMES if d not in provided]
    if missing:
        print(f"Missing calibration logs for: {missing}")
        sys.exit(1)

    order = {d: i for i, d in enumerate(DOF_NAMES)}
    labeled_logs.sort(key=lambda x: order[x[0]])

    M = fit_matrix(labeled_logs)
    print_crosstalk_table(labeled_logs, M)
    write_calibration_h(M)
    send_matrix_to_device(M)
    print("\nCalibration complete. The device is using the new matrix immediately.")


if __name__ == '__main__':
    main()
