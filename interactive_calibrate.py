#!/usr/bin/env python3
"""
Interactive, self-paced calibration tool. Run this directly in your own
terminal (not through an AI assistant) — it drives itself, no external
timing cues needed.

Usage:
  python3 interactive_calibrate.py

Shows a live, constantly-updating view of:
  - raw sensor deltas
  - computed output per axis (using the currently-compiled matrix)
  - which axis is currently active/dominant, so you can see in real time
    what the firmware would interpret your motion as

Menu (single keypress, no Enter needed):
  1  Recalibrate TX  (slide left/right)
  2  Recalibrate TY  (slide forward/back)
  3  Recalibrate TZ  (press down/pull up)
  4  Recalibrate RX  (nod front/back)
  5  Recalibrate RY  (tilt/rock left/right)
  6  Recalibrate RZ  (twist cw/ccw)
  c  Run calibrate.py on whichever cal_*.log files exist, refit + push live
  g  Fit the geometric model's own decoupling matrix (geo_matrix.json)
     from the same cal_*.log files — separates push/slide from tilt
     crosstalk in THE MOUSE THINKS line; a fixed threshold can't (the
     raw ranges genuinely overlap). Run after all 6 axes are captured.
  +/-  Adjust the 3D-point scale factor (see below) live
  q  Quit

3D point (experimental, separate from the calibration matrix above):
  Treats each of the 3 magnetic sensors' baseline-subtracted reading as a
  vector, offset by that sensor's real physical mounting position (from
  the PCB design: pcbs/src/sensor_board.brd — all 3 are exactly 16.51mm
  from center, 120 deg apart), averaged into one (x,y,z) point in cm,
  with (0,0,TARGET_HEIGHT_CM) as the rest reference. The scale factor
  (raw units -> cm) is a rough starting guess, not physically measured —
  tune it live with +/- until the displayed motion feels proportionate.
"""

import fcntl
import json
import math
import os
import re
import select
import struct
import subprocess
import sys
import termios
import time
import tty

import numpy as np

VID = 0x2886
PID = 0x0058

# Real sensor mounting positions, from pcbs/src/sensor_board.brd (Eagle PCB
# design file — exact component placement, not measured/guessed). All 3 are
# exactly 16.51mm from board center, 120 deg apart. mm -> cm for the 3D point.
SENSOR_POS_CM = [
    (0.0 / 10, -16.51 / 10),      # MAG1 bottom
    (-14.2981 / 10, 8.255 / 10),  # MAG2 top-left
    (14.2981 / 10, 8.255 / 10),   # MAG3 top-right
]
TARGET_HEIGHT_CM = 15.0
point_scale = 0.05  # raw delta units -> cm; tune live with +/-

# Display rounding for the geometric model — also doubles as the unit for
# candidate-selection in interpret_geometric() (see there): dividing each
# reading by its own step converts cm and degrees into a comparable
# "how many noise-floors over rest" number. cm and degrees are different
# units, so comparing raw px_r against raw rx_r (e.g. 2.0cm vs 2.0deg)
# silently favored whichever unit happens to produce bigger numbers for a
# given motion — degrees usually, which is why slides and pushes kept
# losing to tilt crosstalk. Hard-gating Z on a single other axis (tried
# twice before this) didn't work either: real Z motion has genuine
# multi-degree Rx/Ry crosstalk (same crosstalk documented for the linear
# calibration matrix in Config.h), so any fixed rotation threshold either
# blocks real Z or lets real tilt through.
POS_ROUND_CM = 0.2
ROT_ROUND_DEG = 1.0

AXES = ["TX", "TY", "TZ", "RX", "RY", "RZ"]
DOF_LABELS = {
    "TX": "slide left/right",
    "TY": "slide forward/back",
    "TZ": "press down / pull up",
    "RX": "nod front/back",
    "RY": "tilt/rock left/right",
    "RZ": "twist cw/ccw",
}
SENSOR_KEYS = ["s0x", "s0y", "s0z", "s1x", "s1y", "s1z", "s2x", "s2y", "s2z"]
GEO_KEYS = ["gpx", "gpy", "gdz", "grx", "gry", "grz"]

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_H = os.path.join(REPO_ROOT, "firmware", "include", "Calibration.h")
CONFIG_H = os.path.join(REPO_ROOT, "firmware", "include", "Config.h")

# 6x6 decoupling matrix for the geometric model's own outputs (px,py,dz,
# rx,ry,rz), fit the same way calibrate.py fits the sensor matrix: least
# squares against "self during own-axis capture, 0 during others", using
# the gpx..grz columns captured alongside the usual cal_<dof>.log files
# (menu 1-6), then 'g' to fit. Defaults to identity (no correction) until
# fit at least once — see fit_geo_matrix()/save_geo_matrix().
GEO_MATRIX_PATH = os.path.join(REPO_ROOT, "geo_matrix.json")


def load_geo_matrix():
    if os.path.exists(GEO_MATRIX_PATH):
        with open(GEO_MATRIX_PATH) as f:
            return np.array(json.load(f), dtype=float)
    return np.eye(6)


def save_geo_matrix(M):
    with open(GEO_MATRIX_PATH, "w") as f:
        json.dump(M.tolist(), f, indent=2)


GEO_M = load_geo_matrix()


FEATURE_REPORT_ID = 4
FEATURE_BUF_SIZE = 1 + 36
CAPTURE_SECONDS = 5.0


def HIDIOCGFEATURE(length):
    return (3 << 30) | (ord("H") << 8) | 0x07 | (length << 16)


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


def parse_matrix(path):
    """Extract the 6x9 DECOUPLING_M matrix straight from Calibration.h."""
    with open(path) as f:
        text = f.read()
    body = re.search(r"DECOUPLING_M\[6\]\[9\]\s*=\s*\{(.*?)\};", text, re.S)
    rows = re.findall(r"\{([^{}]*)\}", body.group(1))
    matrix = []
    for row in rows:
        nums = [float(x.rstrip("f")) for x in re.findall(r"-?\d+\.\d+f?", row)]
        matrix.append(nums[:9])
    return matrix


def parse_array(path, name):
    """Extract a `const float NAME[6] = {...};` array from Config.h."""
    with open(path) as f:
        text = f.read()
    m = re.search(name + r"\[6\]\s*=\s*\{([^}]*)\}", text)
    return [float(x) for x in re.findall(r"-?\d+\.?\d*", m.group(1))]


def read_matrix_and_config():
    matrix = parse_matrix(CALIBRATION_H)
    gain = parse_array(CONFIG_H, "AXIS_GAIN")
    dead = parse_array(CONFIG_H, "DEAD_AXIS")
    return matrix, gain, dead


def poll_sensor_delta(fd):
    buf = bytearray(FEATURE_BUF_SIZE)
    buf[0] = FEATURE_REPORT_ID
    fcntl.ioctl(fd, HIDIOCGFEATURE(FEATURE_BUF_SIZE), buf, True)
    return struct.unpack_from("<9f", buf, 1)


def round_to(v, step):
    return round(v / step) * step


def compute_axes(deltas, matrix, gain):
    y = []
    for i in range(6):
        v = sum(matrix[i][j] * deltas[j] for j in range(9)) * gain[i]
        y.append(v)
    return y


def compute_3d_point(deltas):
    """Each sensor's baseline-subtracted (dx,dy,dz) becomes a vector, offset
    by that sensor's real mounting position, scaled to cm. Averaging the 3
    sensors' implied points gives one (x,y,z) estimate of where the magnet
    is relative to rest (0,0,TARGET_HEIGHT_CM). Also returns each sensor's
    individual point and raw magnitude, for the per-sensor diagnostic."""
    points = []
    mags = []
    for i in range(3):
        dx, dy, dz = deltas[i * 3], deltas[i * 3 + 1], deltas[i * 3 + 2]
        sx, sy = SENSOR_POS_CM[i]
        points.append((
            sx + point_scale * dx,
            sy + point_scale * dy,
            TARGET_HEIGHT_CM + point_scale * dz,
        ))
        mags.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    avg = (
        sum(p[0] for p in points) / 3,
        sum(p[1] for p in points) / 3,
        sum(p[2] for p in points) / 3,
    )
    return avg, points, mags


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(v):
    m = math.sqrt(sum(c * c for c in v))
    if m == 0:
        return (0.0, 0.0, 1.0)
    return (v[0] / m, v[1] / m, v[2] / m)


REST_ANGLES_RAD = [math.atan2(sy, sx) for sx, sy in SENSOR_POS_CM]


def compute_orientation(sensor_pts, avg_pt):
    """Genuine 6DOF from the same 3 points compute_3d_point already gives
    us, no extra sensors needed: the plane through the 3 current points has
    a normal vector whose tilt away from straight-up gives Rx (from the
    normal's Y-component) and Ry (from its X-component). Each point's
    angular position around the centroid, compared to its known rest angle
    (exact, from the real PCB geometry), gives twist Rz. All in degrees;
    signs are whatever the raw geometry gives — verify against the live
    LED/motion the same way we tuned the calibration matrix's signs."""
    p1, p2, p3 = sensor_pts
    normal = _normalize(_cross(_sub(p2, p1), _sub(p3, p1)))
    if normal[2] < 0:  # keep normal pointing "up" regardless of winding
        normal = (-normal[0], -normal[1], -normal[2])
    rx_deg = math.degrees(math.atan2(normal[1], normal[2]))
    ry_deg = math.degrees(math.atan2(-normal[0], normal[2]))

    cx, cy, _cz = avg_pt
    diffs = []
    for i, (px, py, _pz) in enumerate(sensor_pts):
        cur_angle = math.atan2(py - cy, px - cx)
        diff = cur_angle - REST_ANGLES_RAD[i]
        diff = (diff + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi,pi]
        diffs.append(diff)
    rz_deg = math.degrees(sum(diffs) / 3)
    return rx_deg, ry_deg, rz_deg


def interpret(y, dead):
    """Plain-English interpretation from the OLD linear-matrix output —
    kept only so the Axis/Live/Dead/Bar table (still matrix-based) has a
    matching label for comparison. This is the one that had all the
    cross-talk problems; THE MOUSE THINKS line uses interpret_geometric()
    instead, which is far cleaner."""
    tx, ty, tz, rx, ry, rz = y
    dtx, dty, dtz, drx, dry, _drz = dead

    if abs(tz) >= dtz:
        return "PULL UP" if tz > 0 else "PUSH DOWN"

    mags = [abs(tx), abs(ty), abs(rx), abs(ry)]
    deads = [dtx, dty, drx, dry]
    active = [mags[i] >= deads[i] for i in range(4)]
    if not any(active):
        return "(no motion detected)"
    best = max((i for i in range(4) if active[i]), key=lambda i: mags[i])
    if best == 0:
        return "SLIDE LEFT" if tx > 0 else "SLIDE RIGHT"
    if best == 1:
        return "SLIDE BACK" if ty > 0 else "SLIDE FORWARD"
    if best == 2:
        return "TILT BACK" if rx > 0 else "TILT FRONT"
    return "TILT RIGHT" if ry > 0 else "TILT LEFT"


def interpret_geometric(px_r, py_r, dz_r, rx_r, ry_r, rz_r):
    """Plain-English interpretation from the geometric model (position +
    orientation), using the same rounded values shown on screen. All signs
    confirmed from live testing: RY+ = tilt left, RX+ = tilt back, RZ+ =
    twist cw, PX+ = slide left, PY+ = slide forward (TY unconfirmed, TX
    flipped from the original guess).

    All six candidates compete on equal footing, normalized by their own
    rounding step (POS_ROUND_CM / ROT_ROUND_DEG) so cm and degree readings
    are comparable — no axis gets a hard veto over another. See the
    POS_ROUND_CM comment for why: Z has real multi-degree crosstalk into
    Rx/Ry, so a hard suppression rule always ends up wrong in one
    direction or the other."""
    candidates = []
    if px_r != 0:
        candidates.append((abs(px_r) / POS_ROUND_CM, "SLIDE LEFT" if px_r > 0 else "SLIDE RIGHT"))
    if py_r != 0:
        candidates.append((abs(py_r) / POS_ROUND_CM, "SLIDE FORWARD" if py_r > 0 else "SLIDE BACK"))
    if dz_r != 0:
        candidates.append((abs(dz_r) / POS_ROUND_CM, "PULL UP" if dz_r > 0 else "PUSH DOWN"))
    if rx_r != 0:
        candidates.append((abs(rx_r) / ROT_ROUND_DEG, "TILT BACK" if rx_r > 0 else "TILT FRONT"))
    if ry_r != 0:
        candidates.append((abs(ry_r) / ROT_ROUND_DEG, "TILT LEFT" if ry_r > 0 else "TILT RIGHT"))
    if rz_r != 0:
        candidates.append((abs(rz_r) / ROT_ROUND_DEG, "TWIST CW" if rz_r > 0 else "TWIST CCW"))
    if not candidates:
        return "(no motion detected)"
    return max(candidates, key=lambda c: c[0])[1]


# Session-lifetime min/max per raw sensor channel — a channel stuck near
# zero range while others move freely is the signature of a dead/stuck
# sensor axis or a wiring fault, not a calibration problem.
SENSOR_LABELS = [
    "MAG1(bottom) x", "MAG1(bottom) y", "MAG1(bottom) z",
    "MAG2(topL)   x", "MAG2(topL)   y", "MAG2(topL)   z",
    "MAG3(topR)   x", "MAG3(topR)   y", "MAG3(topR)   z",
]
sensor_range = [[None, None] for _ in range(9)]


def update_sensor_range(deltas):
    for i, v in enumerate(deltas):
        lo, hi = sensor_range[i]
        sensor_range[i][0] = v if lo is None else min(lo, v)
        sensor_range[i][1] = v if hi is None else max(hi, v)


BAR_HALF = 20
BAR_COLS = BAR_HALF * 2 + 3


def make_bar(val, limit=350):
    frac = max(-1.0, min(1.0, val / limit))
    filled = int(abs(frac) * BAR_HALF)
    if frac >= 0:
        left = " " * BAR_HALF
        right = "#" * filled + "." * (BAR_HALF - filled)
    else:
        left = "." * (BAR_HALF - filled) + "#" * filled
        right = " " * BAR_HALF
    return f"[{left}|{right}]"


def render(y, dead, status_line, deltas=None):
    mags = [abs(y[i]) for i in range(6)]
    active = [mags[i] >= dead[i] for i in range(6)]
    dominant = None
    if any(active):
        dominant = max(range(6), key=lambda i: mags[i] if active[i] else -1)

    # Geometric model (position + orientation from the 3 sensor points) —
    # computed up front so both THE MOUSE THINKS line and the detail
    # section below use the same rounded/suppressed values.
    geo = None
    if deltas is not None:
        avg_pt, sensor_pts, sensor_mags = compute_3d_point(deltas)
        orient = compute_orientation(sensor_pts, avg_pt)
        # Decouple via GEO_M (identity/no-op until 'g' has been run at
        # least once) before rounding — this is what actually separates
        # push/slide from tilt crosstalk; see fit_geo_matrix().
        g_raw = np.array([
            avg_pt[0], avg_pt[1], avg_pt[2] - TARGET_HEIGHT_CM,
            orient[0], orient[1], orient[2],
        ])
        g_corr = (GEO_M @ g_raw) * GEO_AXIS_GAIN
        # Round to nearest 2mm / 1 deg — matches the observed noise floor
        # (xyz noise ~sub-mm, rxryrz noise ~0.6 deg) so it settles to a
        # clean zero at rest instead of flickering.
        px_r = round_to(g_corr[0], POS_ROUND_CM)
        py_r = round_to(g_corr[1], POS_ROUND_CM)
        dz_r = round_to(g_corr[2], POS_ROUND_CM)
        rx_r = round_to(g_corr[3], ROT_ROUND_DEG)
        ry_r = round_to(g_corr[4], ROT_ROUND_DEG)
        rz_r = round_to(g_corr[5], ROT_ROUND_DEG)
        pz_r = TARGET_HEIGHT_CM + dz_r
        geo = (px_r, py_r, pz_r, dz_r, rx_r, ry_r, rz_r, sensor_pts, sensor_mags)

    thinks = interpret_geometric(px_r, py_r, dz_r, rx_r, ry_r, rz_r) if geo else interpret(y, dead)

    lines = [
        "\033[H\033[2J",
        "  CAD Mouse Interactive Calibration",
        "",
        "  1) TX slide left/right    2) TY slide forward/back   3) TZ press/pull",
        "  4) RX nod front/back      5) RY tilt/rock left/right 6) RZ twist cw/ccw",
        "  c) run calibrate.py (refit + push live)   g) fit geo matrix   q) quit",
        "",
        f"  >>> THE MOUSE THINKS: {thinks}",
        "",
        f"  {'Axis':<4}  {'Live':>8}  {'Dead':>6}  {'Bar':{BAR_COLS}}  Active",
        f"  {'-'*4}  {'-'*8}  {'-'*6}  {'-'*BAR_COLS}  ------",
    ]
    for i, ax in enumerate(AXES):
        mark = "<<< DOMINANT" if i == dominant else ("active" if active[i] else "")
        lines.append(
            f"  {ax:<4}  {y[i]:>8.1f}  {dead[i]:>6.1f}  {make_bar(y[i])}  {mark}"
        )

    if geo:
        px_r, py_r, pz_r, dz_r, rx_r, ry_r, rz_r, sensor_pts, sensor_mags = geo
        lines.append("")
        lines.append(
            f"  3D POINT (scale={point_scale:.4f} cm/unit, +/- to adjust):  "
            f"x={px_r:+5.1f}cm  y={py_r:+5.1f}cm  z={pz_r:+5.1f}cm  "
            f"(rest = 0, 0, {TARGET_HEIGHT_CM:.0f})"
        )
        lines.append(
            f"  ORIENTATION (from plane through the 3 points, rest = 0,0,0):     "
            f"Rx={rx_r:+4.0f}deg  Ry={ry_r:+4.0f}deg  Rz={rz_r:+4.0f}deg"
        )
        lines.append(f"  {'Per-sensor':<10}  {'x':>7}  {'y':>7}  {'z':>7}  {'|delta|':>8}")
        for i, label in enumerate(["MAG1(bottom)", "MAG2(topL)  ", "MAG3(topR)  "]):
            px, py, pz = (round(v, 1) for v in sensor_pts[i])
            lines.append(f"  {label:<10}  {px:>7.1f}  {py:>7.1f}  {pz:>7.1f}  {sensor_mags[i]:>8.3f}")

    if deltas is not None:
        update_sensor_range(deltas)
        lines.append("")
        lines.append(f"  {'Raw sensor channel':<16}  {'Live':>8}  {'Min':>8}  {'Max':>8}  {'Range':>8}")
        lines.append(f"  {'-'*16}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}")
        for i, label in enumerate(SENSOR_LABELS):
            lo, hi = sensor_range[i]
            lo = 0.0 if lo is None else lo
            hi = 0.0 if hi is None else hi
            rng = hi - lo
            flag = "  <-- barely moving, check this sensor" if rng < 0.3 else ""
            lines.append(f"  {label:<16}  {deltas[i]:>8.3f}  {lo:>8.3f}  {hi:>8.3f}  {rng:>8.3f}{flag}")

    lines.append("")
    lines.append(f"  {status_line}")
    lines.append("")
    return "\r\n".join(lines)


def capture_and_save(fd, dof, stdin_fd):
    """5-second timed capture for one DOF, live-updating the display throughout."""
    matrix, gain, dead = read_matrix_and_config()
    label = DOF_LABELS[dof]

    log_path = os.path.join(REPO_ROOT, f"cal_{dof}.log")
    rows = []
    t0 = time.monotonic()
    last_poll = 0
    y = [0.0] * 6

    while time.monotonic() - t0 < CAPTURE_SECONDS:
        # allow quitting mid-capture with Ctrl-C / q, but otherwise just capture
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch in ("q", "Q", "\x03"):
                return False

        now = time.monotonic()
        if now - last_poll >= 0.02:
            last_poll = now
            try:
                deltas = poll_sensor_delta(fd)
            except OSError:
                deltas = None

            axes = [0.0] * 6
            r, _, _ = select.select([fd], [], [], 0)
            if r:
                try:
                    data = os.read(fd, 64)
                    if data and data[0] == 1 and len(data) >= 13:
                        vals = struct.unpack_from("<6h", data, 1)
                        axes = [float(v) for v in vals]
                except BlockingIOError:
                    pass

            if deltas is not None:
                y = compute_axes(deltas, matrix, gain)
                avg_pt, sensor_pts, _ = compute_3d_point(deltas)
                orient = compute_orientation(sensor_pts, avg_pt)
                geo_raw = (
                    avg_pt[0], avg_pt[1], avg_pt[2] - TARGET_HEIGHT_CM,
                    orient[0], orient[1], orient[2],
                )
                rows.append((deltas, axes, geo_raw))
                remaining = CAPTURE_SECONDS - (now - t0)
                status = f"CAPTURING {dof} ({label}) — {remaining:0.1f}s left. Move now!"
                print(render(y, dead, status, deltas), end="", flush=True)
        else:
            time.sleep(0.002)

    with open(log_path, "w") as f:
        f.write("sample\t" + "\t".join(SENSOR_KEYS + ["X", "Y", "Z", "Rx", "Ry", "Rz"] + GEO_KEYS) + "\n")
        for i, (deltas, axes, geo_raw) in enumerate(rows, 1):
            vals = list(deltas) + list(axes) + list(geo_raw)
            f.write(f"{i}\t" + "\t".join(f"{v:.4f}" for v in vals) + "\n")

    maxabs = max((abs(v) for deltas, _, _ in rows for v in deltas), default=0.0)
    sensor_ok = maxabs > 0.5

    # The real thing calibrate.py needs is a live target that actually left
    # zero during capture — real sensor motion with a flat-zero live output
    # (the OLD matrix's own dead zone never cleared for this axis) produces
    # a doomed log that will only surface as a zeroed row much later, after
    # 'c' is pressed. Catch it here instead.
    axis_idx = AXES.index(dof)
    target_vals = [axes[axis_idx] for _, axes, _ in rows]
    target_ok = any(v != 0.0 for v in target_vals)

    if not sensor_ok:
        verdict = "TOO WEAK (sensors barely moved) — redo with more motion."
    elif not target_ok:
        verdict = (
            f"BAD CAPTURE: sensors moved fine, but the live {dof} output stayed at "
            f"exactly 0 the whole time — the CURRENT matrix's dead zone for {dof} "
            f"never cleared, so this log has nothing for calibrate.py to learn from. "
            f"Fitting with this file will zero out {dof} again. Redo this one, more "
            f"firmly, or with 'c' run first on a working matrix."
        )
    else:
        verdict = "OK"

    print(
        render(
            y,
            dead,
            f"Saved cal_{dof}.log ({len(rows)} samples, max|delta|={maxabs:.2f}) "
            f"{verdict} Press any key to continue.",
            deltas,
        ),
        end="",
        flush=True,
    )
    # wait for a keypress (blocking) before returning to the live idle view
    tty.setraw(stdin_fd)
    sys.stdin.read(1)
    return True


SENSOR_ACTIVITY_THRESHOLD = 0.5  # matches calibrate.py's own threshold


def load_geo_log(path):
    """gpx..grz columns from a cal_<dof>.log (see capture_and_save),
    filtered to samples where the raw sensor deltas actually moved — the
    5s capture window includes idle/transition time at rest, and including
    those rows (near-zero true signal, noise-dominated) both pollutes the
    fit and made an earlier evaluation of it look far worse than motion
    samples alone actually are."""
    rows = []
    with open(path) as f:
        next(f, None)
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 22:
                continue
            try:
                vals = [float(x) for x in parts]
            except ValueError:
                continue
            if max(abs(v) for v in vals[1:10]) > SENSOR_ACTIVITY_THRESHOLD:
                rows.append(vals[16:22])
    return np.array(rows) if rows else np.zeros((0, 6))


# Regularization strength, in normalized (steps)^2 units — see fit_geo_matrix.
# ~239 samples/axis makes the data dominate a weak prior; empirically swept
# 4..1e5 against the actual cal_*.log captures — 5000 is where TX/TY/TZ
# settle to near-identity (0.95-0.99 own-axis weight) while RX/RY/RZ still
# keep the genuine (non-noise) part of their z-crosstalk correction.
GEO_RIDGE = 5000.0

# Per-output gain applied after GEO_M, same role as firmware's AXIS_GAIN in
# Config.h and for the same reason: TX/TY/TZ are structurally quiet next to
# the rotation crosstalk they induce (the 3 sensors' short baseline
# amplifies any Z-asymmetry into a large angle — see fit_geo_matrix's
# docstring), so a fair "biggest signal wins" comparison in
# interpret_geometric needs them boosted first. Values are the minimum
# found (via crosstalk-table sweep against real captures) for each
# translation axis's own diagonal to beat its worst rotation crosstalk,
# plus a small margin. RX/RY/RZ don't need it — already dominant.
GEO_AXIS_GAIN = [4.0, 2.0, 3.0, 1.0, 1.0, 1.0]


def fit_geo_matrix(logs, ridge=GEO_RIDGE):
    """Same self/zero decoupling calibrate.py uses for the sensor matrix
    (see fit_matrix there): during dof's own capture the target is that
    log's own column (preserve real scale), during every other capture the
    target is 0 (suppress crosstalk). Applied to the geometric model's own
    px,py,dz,rx,ry,rz outputs instead of the 9 sensor channels.

    Unlike the sensor matrix, plain least-squares here badly overfits: 6
    highly-correlated inputs feeding 6 outputs is a poorly-conditioned
    system, and a first attempt produced coefficients like RY's weight on
    raw x/y position being 8-10x its weight on its own rotation input —
    fits the captured samples almost exactly, but massively amplifies any
    live position signal (a push, a slide) into a fake rotation reading.
    Ridge-regularized toward the identity (each output trusts its own
    input at weight 1 by default, only deviating where the data strongly
    supports it) fixes that. Inputs are normalized by POS_ROUND_CM /
    ROT_ROUND_DEG first so the penalty applies fairly — cm and degree
    columns have very different natural scales, and an unnormalized ridge
    would penalize whichever unit is smaller (position) far more."""
    offsets = {}
    offset = 0
    for dof in AXES:
        offsets[dof] = offset
        offset += logs[dof].shape[0]
    all_geo = np.vstack([logs[d] for d in AXES])

    scale = np.array([POS_ROUND_CM, POS_ROUND_CM, POS_ROUND_CM,
                       ROT_ROUND_DEG, ROT_ROUND_DEG, ROT_ROUND_DEG])
    X = all_geo / scale

    M = np.eye(6)
    for j, dof in enumerate(AXES):
        b = np.zeros(all_geo.shape[0])
        o, n = offsets[dof], logs[dof].shape[0]
        b[o:o + n] = logs[dof][:, j]
        # Prior is raw identity (M[j] == e_j, i.e. output_j = input_j
        # untouched) — since M[j] = w/scale, that prior is w0 = scale[j]*e_j
        # in normalized-w space, not plain e_j (using e_j here was the bug:
        # at high ridge it converges to M_diag = 1/scale instead of 1).
        Xa = np.vstack([X, math.sqrt(ridge) * np.eye(6)])
        ba = np.concatenate([b, math.sqrt(ridge) * scale[j] * np.eye(6)[j]])
        w, _, _, _ = np.linalg.lstsq(Xa, ba, rcond=None)
        M[j] = w / scale
    return M


def fit_and_save_geo_matrix(stdin_fd):
    global GEO_M
    print("\033[H\033[2J", end="")
    missing = [d for d in AXES if not os.path.exists(os.path.join(REPO_ROOT, f"cal_{d}.log"))]
    if missing:
        print(f"Missing logs for: {missing} — capture those first (menu 1-6).\n")
        print("Press any key to continue.")
        sys.stdin.read(1)
        return

    logs = {}
    for dof in AXES:
        g = load_geo_log(os.path.join(REPO_ROOT, f"cal_{dof}.log"))
        if g.shape[0] == 0:
            print(f"cal_{dof}.log has no geo columns — recapture it (menu 1-6) to refresh it.\n")
            print("Press any key to continue.")
            sys.stdin.read(1)
            return
        logs[dof] = g

    M = fit_geo_matrix(logs)
    save_geo_matrix(M)
    GEO_M = M
    print(f"Fit and saved {GEO_MATRIX_PATH}:\n")
    print(np.array2string(M, precision=3, suppress_small=True))
    print("\nPress any key to continue.")
    sys.stdin.read(1)


def check_log_target(dof):
    """True if cal_<dof>.log's own axis column ever left zero — i.e. the
    live output actually cleared its dead zone at some point during that
    capture, giving calibrate.py something real to fit. False means this
    log will produce a zeroed/degenerate row no matter how many times you
    refit with it."""
    path = os.path.join(REPO_ROOT, f"cal_{dof}.log")
    col = 10 + AXES.index(dof)  # sample, 9 sensor cols, then X,Y,Z,Rx,Ry,Rz
    try:
        with open(path) as f:
            next(f, None)
            for line in f:
                parts = line.rstrip("\n").split("\t")
                if len(parts) <= col:
                    continue
                if float(parts[col]) != 0.0:
                    return True
    except FileNotFoundError:
        return False
    return False


def run_calibrate_py(stdin_fd):
    have_logs = [d for d in AXES if os.path.exists(os.path.join(REPO_ROOT, f"cal_{d}.log"))]
    missing = [d for d in AXES if d not in have_logs]
    print("\033[H\033[2J", end="")
    if missing:
        print(f"Missing logs for: {missing} — capture those first (menu 1-6).\n")
        print("Press any key to continue.")
        sys.stdin.read(1)
        return

    dead_logs = [d for d in AXES if d not in missing and not check_log_target(d)]
    if dead_logs:
        print(f"WARNING: these logs have a flat-zero live target (will zero out that row): {dead_logs}")
        print("Recapture them first for a real fit, or press 'y' to fit anyway, any other key to cancel.")
        ch = sys.stdin.read(1)
        if ch not in ("y", "Y"):
            print("\nCancelled. Press any key to continue.")
            sys.stdin.read(1)
            return
        print()

    args = [f"{d}:cal_{d}.log" for d in AXES]
    termios.tcsetattr(stdin_fd, termios.TCSADRAIN, termios.tcgetattr(stdin_fd))
    result = subprocess.run(
        [sys.executable, os.path.join(REPO_ROOT, "calibrate.py")] + args,
        cwd=REPO_ROOT,
    )
    print(f"\ncalibrate.py exited with code {result.returncode}. Press any key to continue.")
    tty.setraw(stdin_fd)
    sys.stdin.read(1)


def main():
    global point_scale
    hidraw_path = find_hidraw(VID, PID)
    if not hidraw_path:
        print(f"ERROR: CAD Mouse ({VID:04x}:{PID:04x}) not found. Is it plugged in?")
        sys.exit(1)

    try:
        fd = os.open(hidraw_path, os.O_RDWR | os.O_NONBLOCK)
    except PermissionError:
        print(f"ERROR: Cannot open {hidraw_path}. Check udev rules or run with sudo.")
        sys.exit(1)

    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)
    tty.setraw(stdin_fd)

    try:
        matrix, gain, dead = read_matrix_and_config()
        last_poll = time.monotonic()
        y = [0.0] * 6
        deltas = [0.0] * 9
        status = "Idle — pick an axis (1-6) to recalibrate, or 'c'/'g' to refit."

        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch in ("q", "Q", "\x03"):
                    break
                if ch in "123456":
                    dof = AXES[int(ch) - 1]
                    termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
                    tty.setraw(stdin_fd)
                    print(
                        render(y, dead, f"Get ready: {dof} ({DOF_LABELS[dof]})... starting in 1.5s", deltas),
                        end="",
                        flush=True,
                    )
                    time.sleep(1.5)
                    capture_and_save(fd, dof, stdin_fd)
                    matrix, gain, dead = read_matrix_and_config()
                    status = "Idle — pick an axis (1-6) to recalibrate, or 'c'/'g' to refit."
                elif ch in ("c", "C"):
                    run_calibrate_py(stdin_fd)
                    matrix, gain, dead = read_matrix_and_config()
                    status = "Idle — pick an axis (1-6) to recalibrate, or 'c'/'g' to refit."
                elif ch in ("g", "G"):
                    fit_and_save_geo_matrix(stdin_fd)
                    status = "Idle — pick an axis (1-6) to recalibrate, or 'c'/'g' to refit."
                elif ch == "+":
                    point_scale *= 1.25
                elif ch == "-":
                    point_scale /= 1.25
                tty.setraw(stdin_fd)

            now = time.monotonic()
            if now - last_poll >= 0.02:
                last_poll = now
                try:
                    deltas = poll_sensor_delta(fd)
                    y = compute_axes(deltas, matrix, gain)
                except OSError:
                    pass
                print(render(y, dead, status, deltas), end="", flush=True)
            else:
                time.sleep(0.002)

    finally:
        os.close(fd)
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
        print("\033[H\033[2J\033[?25h", end="")


if __name__ == "__main__":
    main()
