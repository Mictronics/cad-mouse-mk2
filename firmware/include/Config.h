#pragma once

#include <Arduino.h>

namespace Config {

const bool ENABLE_TELEMETRY = false;

// Hardware pins (XIAO RP2350)
const int PIN_RIGHT_BTN = D0;
const int PIN_LEFT_BTN = D2;
const int PIN_LED_DATA = D3;
const int PIN_LED_LS = D1;
const int PIN_MAG1_LS = D10;
const int PIN_MAG2_LS = D9;
const int PIN_MAG3_LS = D8;

// Samples for calibration offset
const int ZERO_SAMPLES = 200;

// Per-axis output gain, applied right after the calibration matrix multiply
// (before dead-zone/filter/clamp). Kept separate from the matrix itself so
// it survives recalibration. TZ's fitted signal peaks around 10-14 even at
// full deflection — heavy TX/TY crosstalk in cal_TZ.log limits how strong a
// linear fit can be (see calibrate.py's own bleed warnings) — versus TX/TY's
// 100-300. Real 3D input devices expect axes to use a comparable fraction of
// their range, so a raw TZ value that's "correct" but 10x smaller than TX/TY
// reads to spacenavd/FreeCAD as barely-there motion. 10x gain brings it to a
// comparable scale without another recalibration attempt. TZ's sign is
// negated here (-10.0) — live testing showed pull-up/push-down producing
// the opposite of the expected view motion.
const float AXIS_GAIN[6] = {1.0, 1.0, -10.0, 1.0, 1.0, 1.0};

// Per-axis dead zones (TX, TY, TZ, RX, RY, RZ), in post-gain units.
// Temporarily lowered for TZ/RX/RY/RZ (measured noise floor with EMA
// smoothing: 0.35, 0.11, 0.25, 1.4 respectively — still 2-14x margin at
// these values) for a light-touch recalibration pass; TX/TY stay at 16.0
// since their noise floor (9.3, 4.3) is already close to that, and light
// touch isn't the right approach for them anyway. Re-measure after
// recalibrating (see run_calibration.sh).
const float DEAD_AXIS[6] = {16.0, 16.0, 5.0, 1.5, 1.0, 3.0};

// Smoothing
const float SMOOTH_TAU_S = 0.08;

// Final axis output range
const float AXIS_LIMIT = 350.0;

// ── Geometric model (position + orientation from the 3 sensor points) ──────
// Ported from interactive_calibrate.py, where this was developed and
// verified live (see GeoCalibration.h). Replaces the direct 9-sensor ->
// 6-axis DECOUPLING_M path below as the live motion pipeline
// (MotionController::compute) — DECOUPLING_M/AXIS_GAIN/DEAD_AXIS above are
// no longer read by compute() but left in place (still receive live
// pushes/persist via CalibrationStore) since nothing else depends on
// removing them.

// Real sensor mounting positions, from pcbs/src/sensor_board.brd — all 3
// exactly 16.51mm from board center, 120 deg apart. mm -> cm.
const float GEO_SENSOR_POS_CM[3][2] = {
    {  0.0f,     -1.651f },  // MAG1 bottom
    { -1.42981f,  0.8255f },  // MAG2 top-left
    {  1.42981f,  0.8255f },  // MAG3 top-right
};
const float GEO_TARGET_HEIGHT_CM = 15.0f;
// Raw delta units -> cm. Host tool default (interactive_calibrate.py);
// wasn't changed live during verification, so ported as-is.
const float GEO_POINT_SCALE = 0.05f;

// Per-axis gain applied after GEO_M, before the dead zone — same role as
// AXIS_GAIN above and for the same reason: TX/TY/TZ are structurally quiet
// next to the rotation crosstalk they induce (the sensors' short baseline
// amplifies any Z-asymmetry into a large angle), so a fair comparison needs
// them boosted first. Magnitudes are what interactive_calibrate.py
// converged on via live testing (see GEO_AXIS_GAIN there). TX's sign is
// negated here (-4.0, host tool used +4.0) — live testing on-device showed
// slide-left/right producing the opposite of the expected CAD view motion,
// same reasoning as TZ's negative gain above. IdleState.cpp compensates so
// the LED direction indicator (independently verified against
// lightpattern.md) still matches the physical motion.
const float GEO_AXIS_GAIN[6] = {-4.0f, 2.0f, 3.0f, 1.0f, 1.0f, 1.0f};

// Dead zone in post-gain, pre-GEO_OUTPUT_SCALE units (cm/deg range,
// matching interactive_calibrate.py's POS_ROUND_CM/ROT_ROUND_DEG * gain —
// chosen there with margin above the measured noise floor: sub-mm
// position, ~0.6deg orientation).
const float GEO_DEAD[6] = {0.8f, 0.4f, 0.6f, 1.0f, 1.0f, 1.0f};

// UNVERIFIED ON HARDWARE — the host tool never needed absolute HID-range
// magnitude (only relative comparison, for picking a text label), so there
// was nothing to port for this. Rough starting estimate only: sized so a
// solid ~0.5cm position motion / ~20deg tilt (typical "confident" values
// seen live) lands around 2/3 of AXIS_LIMIT, leaving clamp headroom. Tune
// live after flashing, the same way AXIS_GAIN above was originally tuned.
const float GEO_OUTPUT_SCALE[6] = {125.0f, 250.0f, 167.0f, 12.5f, 12.5f, 12.5f};

// RGB LEDs
const int LED_COUNT = 8;
const int LED_BRIGHTNESS = 40;
// Idle color palette. Cycle at runtime by holding ONLY the right button
// for 3s while idle; the selection persists in flash across power cycles.
// First entry is the factory default — cyan, not green, so idle doesn't
// look identical to the TZ pull-up indicator flash.
const unsigned long LED_IDLE_PALETTE[] = {
    0x00FFFF,  // cyan (default)
    0x9400D3,  // purple
    0x0080FF,  // sky blue
    0x00FF00,  // green
    0xFF00FF,  // magenta
    0xFF4000,  // orange
    0xFFFFFF,  // white
};
const int LED_IDLE_PALETTE_COUNT =
    sizeof(LED_IDLE_PALETTE) / sizeof(LED_IDLE_PALETTE[0]);
const unsigned long LED_CALIBRATING_COLOR = 0x0000FF;

// FSM timing
const long IDLE_SLEEP_TIMEOUT_MS = 15 * 60 * 1000;

}  // namespace Config

#include "Calibration.h"
#include "GeoCalibration.h"
