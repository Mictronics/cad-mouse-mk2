#!/usr/bin/env python3
"""
Monitor CAD Mouse sensor deltas and motion output via HID feature report.
No serial port or special firmware build required.

Usage:
  python3 monitor.py [HIDRAW_PATH]

If HIDRAW_PATH is omitted, the device is discovered automatically.

Keys:
  r  - reset aggregate accumulator and log file
  q  - quit
"""

import fcntl
import os
import select
import struct
import sys
import termios
import time
import tty

VID = 0x2886
PID = 0x0058

AXES = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
SENSOR_KEYS = ["s0x", "s0y", "s0z", "s1x", "s1y", "s1z", "s2x", "s2y", "s2z"]
LOG_KEYS = SENSOR_KEYS + AXES
LOG_HEADER = "sample\t" + "\t".join(LOG_KEYS) + "\n"
LOG_FILE = "mouse_monitor.log"

# HIDIOCGFEATURE(len) = _IOWR('H', 0x07, len)
def HIDIOCGFEATURE(length):
    return (3 << 30) | (ord('H') << 8) | 0x07 | (length << 16)

FEATURE_REPORT_ID = 4
FEATURE_BUF_SIZE  = 1 + 36  # report_id byte + 9 floats


def find_hidraw(vid, pid):
    """Walk sysfs to find /dev/hidrawN for the given VID:PID."""
    base = "/sys/bus/hid/devices"
    try:
        entries = os.listdir(base)
    except FileNotFoundError:
        return None
    for entry in entries:
        # Entry format: 0003:VID:PID.NNNN
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


def poll_sensor_delta(fd):
    """Read feature report ID 4 and return 9 floats (sensor deltas)."""
    buf = bytearray(FEATURE_BUF_SIZE)
    buf[0] = FEATURE_REPORT_ID
    fcntl.ioctl(fd, HIDIOCGFEATURE(FEATURE_BUF_SIZE), buf, True)
    return struct.unpack_from("<9f", buf, 1)


def read_axes_nonblocking(fd, current_axes):
    """Read any pending input reports; return updated motion axes or current."""
    r, _, _ = select.select([fd], [], [], 0)
    if not r:
        return current_axes
    try:
        data = os.read(fd, 64)
    except BlockingIOError:
        return current_axes
    if data and data[0] == 1 and len(data) >= 13:
        vals = struct.unpack_from("<6h", data, 1)
        return [float(v) for v in vals]
    return current_axes


BAR_HALF = 20
BAR_COLS = BAR_HALF * 2 + 3


def make_bar(val, limit=350):
    frac = max(-1.0, min(1.0, val / limit))
    filled = int(abs(frac) * BAR_HALF)
    if frac >= 0:
        left  = " " * BAR_HALF
        right = "#" * filled + "." * (BAR_HALF - filled)
    else:
        left  = "." * (BAR_HALF - filled) + "#" * filled
        right = " " * BAR_HALF
    return f"[{left}|{right}]"


def render(live, aggregate, sample_count):
    lines = [
        "\033[H\033[2J",
        f"  CAD Mouse Monitor  (samples: {sample_count})  [r=reset  q=quit]",
        "",
        f"  {'Axis':<4}  {'Live':>8}  {'Bar (live)':{BAR_COLS}}  {'Aggregate':>12}",
        f"  {'-'*4}  {'-'*8}  {'-'*BAR_COLS}  {'-'*12}",
    ]
    for ax in AXES:
        lv = live.get(ax, 0.0)
        ag = aggregate.get(ax, 0.0)
        lines.append(f"  {ax:<4}  {lv:>8.2f}  {make_bar(lv)}  {ag:>12.1f}")
    lines.append("")
    return "\r\n".join(lines)


def open_log():
    f = open(LOG_FILE, "w", buffering=1)
    f.write(LOG_HEADER)
    return f


def reset_log():
    try:
        os.remove(LOG_FILE)
    except FileNotFoundError:
        pass
    return open_log()


def main():
    if len(sys.argv) > 1:
        hidraw_path = sys.argv[1]
    else:
        hidraw_path = find_hidraw(VID, PID)
        if not hidraw_path:
            print(f"ERROR: CAD Mouse ({VID:04x}:{PID:04x}) not found. Is it plugged in?")
            sys.exit(1)

    print(f"Using {hidraw_path}")

    try:
        fd = os.open(hidraw_path, os.O_RDWR | os.O_NONBLOCK)
    except PermissionError:
        print(f"ERROR: Cannot open {hidraw_path}. Check udev rules or run with sudo.")
        sys.exit(1)

    live = {k: 0.0 for k in LOG_KEYS}
    aggregate = {ax: 0.0 for ax in AXES}
    sample_count = 0
    current_axes = [0.0] * 6
    log = open_log()

    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)
    tty.setraw(stdin_fd)

    try:
        print(render(live, aggregate, sample_count), end="", flush=True)
        last_poll = time.monotonic()

        while True:
            # Keyboard
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch in ("q", "Q", "\x03"):
                    break
                if ch in ("r", "R"):
                    aggregate = {ax: 0.0 for ax in AXES}
                    sample_count = 0
                    log.close()
                    log = reset_log()

            # Poll feature report at ~50 Hz
            now = time.monotonic()
            if now - last_poll >= 0.02:
                last_poll = now

                try:
                    deltas = poll_sensor_delta(fd)
                except OSError:
                    deltas = None

                current_axes = read_axes_nonblocking(fd, current_axes)

                if deltas is not None:
                    for i, k in enumerate(SENSOR_KEYS):
                        live[k] = deltas[i]
                    for i, ax in enumerate(AXES):
                        live[ax] = current_axes[i]
                        aggregate[ax] += current_axes[i]

                    sample_count += 1
                    log.write(
                        f"{sample_count}\t"
                        + "\t".join(f"{live[k]:.4f}" for k in LOG_KEYS)
                        + "\n"
                    )
                    print(render(live, aggregate, sample_count), end="", flush=True)
            else:
                time.sleep(0.002)

    finally:
        log.close()
        os.close(fd)
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
        print("\033[H\033[2J\033[?25h", end="")


if __name__ == "__main__":
    main()
