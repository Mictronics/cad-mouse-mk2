#!/usr/bin/env python3
"""
CAD Mouse MK2 — button-to-key mapper.

Reads button HID reports directly from /dev/hidrawN (non-exclusive, works
alongside spacenavd which grabs the evdev node). Fires xdotool key sequences
based on the active window title.

Usage:
    python3 spacemouse-buttons.py [--config PATH] [--list-events]

    --config PATH    Path to button-map.conf (default: ~/.config/spacemouse/button-map.conf)
    --list-events    Print raw HID reports on button press (for debugging).
"""

import argparse
import glob
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

DEVICE_VID = 0x2886
DEVICE_PID = 0x0058
REPORT_ID_BUTTONS = 0x03   # from HID descriptor
NUM_BUTTONS = 2
DEFAULT_CONFIG = Path.home() / ".config/spacemouse/button-map.conf"
PID_FILE = "/tmp/spacemouse-buttons.pid"
KEY_DELAY_S = 0.05  # seconds between keys in a sequence


# ── Config ─────────────────────────────────────────────────────────────────────

def load_config(path):
    """Parse button-map.conf → list of (button_int, pattern_str, [key_str])."""
    rules = []
    try:
        with open(path) as f:
            for lineno, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) < 3 or parts[0] != 'button':
                    print(f"[button-mapper] {path}:{lineno}: skipping: {line!r}", flush=True)
                    continue
                try:
                    btn = int(parts[1])
                except ValueError:
                    print(f"[button-mapper] {path}:{lineno}: bad button number {parts[1]!r}", flush=True)
                    continue
                pattern = parts[2]
                keys = parts[3:]
                rules.append((btn, pattern, keys))
    except FileNotFoundError:
        print(f"[button-mapper] config not found: {path}", flush=True)
    return rules


def match_keys(rules, button, window_title):
    """Return key list for first matching rule, or [] for no-op."""
    title_lower = window_title.lower()
    for btn, pattern, keys in rules:
        if btn != button:
            continue
        if pattern == '*' or pattern.lower() in title_lower:
            return keys
    return []


# ── Window detection ────────────────────────────────────────────────────────────

def active_window_title():
    try:
        wid = subprocess.check_output(
            ['xdotool', 'getactivewindow'], stderr=subprocess.DEVNULL
        ).decode().strip()
        return subprocess.check_output(
            ['xdotool', 'getwindowname', wid], stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        return ''


# ── Key sending ────────────────────────────────────────────────────────────────

def send_keys(keys):
    for key in keys:
        subprocess.run(['xdotool', 'key', '--clearmodifiers', key], check=False)
        time.sleep(KEY_DELAY_S)


# ── hidraw device discovery ─────────────────────────────────────────────────────

def find_hidraw():
    """Walk sysfs to find /dev/hidrawN for our VID:PID."""
    for hidraw_sys in glob.glob('/sys/class/hidraw/hidraw*'):
        try:
            path = os.path.realpath(hidraw_sys + '/device')
            # Walk up the sysfs tree looking for idVendor / idProduct
            while path != '/':
                vendor_f = os.path.join(path, 'idVendor')
                product_f = os.path.join(path, 'idProduct')
                if os.path.exists(vendor_f) and os.path.exists(product_f):
                    vid = int(open(vendor_f).read().strip(), 16)
                    pid = int(open(product_f).read().strip(), 16)
                    if vid == DEVICE_VID and pid == DEVICE_PID:
                        return '/dev/' + os.path.basename(hidraw_sys)
                    break  # found USB device but VID/PID didn't match
                path = os.path.dirname(path)
        except Exception:
            continue
    return None


# ── HID report parsing ──────────────────────────────────────────────────────────

def parse_button_report(data):
    """
    Report ID 3 layout (from HID descriptor):
      byte 0 : report ID (0x03)
      byte 1 : bits 0-1 = button 1, button 2; bits 2-7 = padding
      byte 2 : padding
    Returns list of 1-based button numbers that are currently pressed.
    """
    if len(data) < 2 or data[0] != REPORT_ID_BUTTONS:
        return None  # not a button report
    pressed = []
    for i in range(NUM_BUTTONS):
        if data[1] & (1 << i):
            pressed.append(i + 1)
    return pressed


# ── Modes ──────────────────────────────────────────────────────────────────────

def list_events(hidraw_path):
    print(f"Reading from {hidraw_path}. Press buttons then Ctrl+C.\n", flush=True)
    with open(hidraw_path, 'rb', buffering=0) as f:
        while True:
            data = f.read(64)
            if data:
                print(f"report id=0x{data[0]:02x}  raw={data.hex()}  "
                      f"{'BUTTON REPORT' if data[0] == REPORT_ID_BUTTONS else ''}", flush=True)


def run(config_path, hidraw_path):
    rules = load_config(config_path)

    def reload_config(sig, frame):
        nonlocal rules
        print("[button-mapper] reloading config...", flush=True)
        rules = load_config(config_path)

    signal.signal(signal.SIGHUP, reload_config)

    print(f"[button-mapper] device : {hidraw_path}", flush=True)
    print(f"[button-mapper] config : {config_path} ({len(rules)} rules)", flush=True)

    Path(PID_FILE).write_text(str(os.getpid()))

    prev_pressed = set()

    with open(hidraw_path, 'rb', buffering=0) as f:
        while True:
            data = f.read(64)
            if not data:
                continue

            pressed = parse_button_report(data)
            if pressed is None:
                continue  # not a button report

            pressed_set = set(pressed)
            newly_pressed = pressed_set - prev_pressed

            for button_num in sorted(newly_pressed):
                title = active_window_title()
                keys = match_keys(rules, button_num, title)
                print(f"[button-mapper] btn{button_num}  window={title!r}  keys={keys}", flush=True)
                send_keys(keys)

            prev_pressed = pressed_set


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='CAD Mouse MK2 button mapper')
    parser.add_argument('--config', default=str(DEFAULT_CONFIG),
                        help=f'button-map.conf path (default: {DEFAULT_CONFIG})')
    parser.add_argument('--list-events', action='store_true',
                        help='print raw HID reports (for debugging button codes)')
    args = parser.parse_args()

    hidraw_path = find_hidraw()
    if hidraw_path is None:
        sys.exit(f"[button-mapper] hidraw device {DEVICE_VID:04x}:{DEVICE_PID:04x} not found — is it plugged in?")

    if args.list_events:
        list_events(hidraw_path)
    else:
        run(args.config, hidraw_path)


if __name__ == '__main__':
    main()
