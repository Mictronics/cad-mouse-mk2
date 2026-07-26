# CAD Mouse MK2

Watch the build video ↓

[<img src="./images/CAD_Mouse_MK2_Thumbnail.jpg">](https://youtu.be/62xlzGs8LXA)

This is the second iteration of my DIY CAD Mouse, rebuilt to behave like a real 6DoF controller. It uses a custom PCB with three magnetic sensors, a 3D printed spring, and a redesigned enclosure that is smaller and easier to build.

  Build instructions → [Instructables](https://www.instructables.com/CAD-Mouse-MK2-a-6DoF-Space-Mouse-Using-Magnets)

<sub>⚠️ There have been several comments raising concerns about the longevity of the PETG spring. If it does not last as expected, a revision of the knob design will be needed.</sub>

## Linux Setup

After flashing the firmware, run the one-time setup script to configure spacenavd (the 6DoF motion driver) and the button mapper:

```bash
./linux-setup.sh
```

This script:
- Installs `spacenavd` and `xdotool`
- Writes `/etc/spnavrc` so spacenavd recognises the device by USB VID:PID
- Installs udev rules so the device is accessible without root
- Installs `spacemouse-buttons.py` as a systemd user service that maps button presses to keystrokes

After running it, plug in the mouse and wait a couple of seconds for the LED to turn green — the device is running its zero-calibration on startup.

### Button gestures

| Gesture | Effect |
|---------|--------|
| Hold both buttons 3s | Enter calibration mode |
| Hold both buttons 10s | Reboot into the UF2 bootloader (drag-and-drop reflash, no case-opening needed) |
| Hold the right button only, 3s | Cycle the idle LED color (persists across power cycles) |

### Button mapping

Button behaviour is configured in `~/.config/spacemouse/button-map.conf`. The file shipped with the repo maps:

| Button | App | Key(s) |
|--------|-----|---------|
| 1 | FreeCAD | `2` (top view) |
| 1 | * (all others) | `f` (fit/zoom all) |
| 2 | * | `5` (isometric view) |

Edit the file to match your preferred CAD application. Reload without restarting:

```bash
kill -HUP $(cat /tmp/spacemouse-buttons.pid)
```

Live log:

```bash
journalctl --user -fu spacemouse-buttons
```

Debug button events:

```bash
python3 ~/.local/bin/spacemouse-buttons.py --list-events
```

## Calibration

Because the three magnetic sensors are physically close together, each sensor responds to motion in every direction — not just the one you're moving. Without correction, pushing the puck sideways also produces spurious tilt and twist outputs. Calibration solves this by fitting a **6×9 decoupling matrix** that maps the raw 9-channel sensor readings (3 axes × 3 sensors) to clean 6-DoF outputs (TX, TY, TZ, RX, RY, RZ).

The fitted matrix is sent to the device over USB and saved to flash — **no reflashing required**. The device loads it automatically on every boot. `firmware/include/Calibration.h` is also updated as a compiled-in fallback for fresh firmware flashes.

You should recalibrate whenever you rebuild the mouse, swap a sensor, or notice persistent drift or cross-axis bleed-through.

### Requirements

- Python 3 with `numpy` (`pip install numpy`)
- The mouse flashed and plugged in, LED solid green (idle)

### Steps

1. From the project root, run:
   ```bash
   ./run_calibration.sh
   ```
2. The script walks you through **6 movements**, one per axis:

   | Step | Movement |
   |------|----------|
   | TX | Slide the puck **left and right** — translate only, no tilt or twist |
   | TY | Slide the puck **forward and back** — translate only, no tilt or twist |
   | TZ | Press the puck **straight down** then release — no tilt or sideways |
   | RX | **Tilt front-to-back** — front edge dips then rises |
   | RY | **Tilt left-to-right** — left edge dips then rises |
   | RZ | **Twist clockwise then counterclockwise** around the puck's centre |

3. For each step: press **Enter** to open the live monitor, make the movement slowly (stay well under the ±350 output limit), then press **Q** to stop recording.
4. After all 6 logs are collected, `calibrate.py` fits the matrix and prints a cross-talk table. Any axis bleeding more than 50% into another is flagged — if you see warnings, recollect that axis's log.
5. The new matrix is sent to the device immediately and takes effect without restarting. It is also written to `firmware/include/Calibration.h`.

### How it works

**Data collection** — `monitor.py` polls HID feature report ID 4 via the Linux `hidraw` interface (`HIDIOCGFEATURE` ioctl) at ~50 Hz. The firmware fills this report with the 9 baseline-subtracted sensor delta values each motion loop cycle. Motion axis values (TX–RZ) are read from the standard HID input report. No serial port or special firmware build is required.

**Matrix fitting** — `calibrate.py` uses least-squares regression (`numpy.linalg.lstsq`) across all six labeled movement logs simultaneously to find the 6×9 matrix that best predicts the intended single-axis output from the 9 sensor deltas. Fitting all axes together lets the solver distinguish shared sensor patterns that belong to one DoF from those that belong to another.

**Sending the result** — after fitting, `calibrate.py` writes the matrix to the device via HID output report ID 5 (`write()` on the hidraw device). The firmware applies the matrix immediately and saves it to LittleFS flash. On every subsequent boot, the saved matrix is loaded automatically. If no saved matrix exists (e.g. after first flash), the firmware falls back to the matrix compiled into `Calibration.h`.

## Motion model

Live motion output no longer comes straight from the 6×9 matrix above — that matrix is still fit and kept as a compiled-in fallback, but the firmware's active pipeline (`MotionController::compute`) instead computes a **3D point and orientation** from the sensor geometry directly. Each sensor's baseline-subtracted reading becomes a vector, offset by that sensor's real mounting position on the PCB (`pcbs/src/sensor_board.brd`); averaging the three implied points gives an (x, y, z) position, and the plane through them gives tilt (Rx, Ry) and twist (Rz). This is more physically direct than the linear matrix, but the three sensors' short baseline amplifies any Z-asymmetry into a large apparent tilt angle — so a second, ridge-regularized 6×6 matrix (`GEO_M`, in `firmware/include/GeoCalibration.h`) decouples the resulting cross-axis crosstalk between those six geometric outputs, plus a per-axis gain (`GEO_AXIS_GAIN` in `Config.h`) so translation axes can compete fairly against the rotation crosstalk they induce.

### `interactive_calibrate.py`

A standalone, self-paced host tool (run directly in your own terminal, not through an AI assistant) for tuning and verifying the geometric model live:

```bash
python3 interactive_calibrate.py
```

- **1–6**: capture 5s of motion for one axis (same six movements as [Calibration](#calibration) above)
- **g**: fit `GEO_M` from the captured logs and save it to `geo_matrix.json`
- **c**: run `calibrate.py` (the linear-matrix fallback described above)
- **+ / -**: adjust the 3D-point scale factor live

The live view shows raw per-sensor deltas, the fitted 3D point and orientation, and a plain-English "THE MOUSE THINKS: ..." line so you can watch the model's interpretation update in real time as you move the puck — useful both for tuning and for telling a real hardware fault from a calibration problem.

`geo_matrix.json` is committed to the repo as a starting point — it's fit from one physical unit, but the three sensors' mounting geometry is fixed by the PCB design, so the fit mostly captures per-sensor gain/crosstalk that carries over reasonably well across similarly-built units. Re-running the capture + `g` on your own device will fine-tune it.

**Porting to firmware**: unlike `Calibration.h`, `GeoCalibration.h`'s `GEO_M` is not yet auto-written by the host tool — after refitting, copy the 6 printed rows into that file by hand and reflash.

## LED indicator

The ring shows what the mouse currently thinks it's detecting (see `lightpattern.md`):

| Motion | Pattern |
|---|---|
| Idle | Solid cyan (cycle color with the button gesture above) |
| Push down | Solid red, blinking |
| Pull up | Solid green, blinking |
| Tilt (front/back/left/right) | Cyan ring, green on the 2 LEDs toward the tilt direction |
| Slide left | Green 6–12 o'clock, red 1–5 o'clock |
| Slide right | Green 12–6 o'clock, red 7–11 o'clock |
| Slide forward | Green 3–9 o'clock (through 12), red on the rest |
| Slide backward | Green 9–3 o'clock (through 6), red on the rest |
| Twist (cw/ccw) | Cyan ring, single blue LED circling in the twist direction |

Blink rate (and, for twist, circling speed) scales with how hard the motion is — faster near full deflection, slower near the dead zone.

[![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg
