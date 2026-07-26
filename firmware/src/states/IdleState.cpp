#include "states/IdleState.h"

#include <Arduino.h>
#include <math.h>

#include "CalibrationStore.h"
#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

namespace {
// Ring layout measured empirically, one pixel at a time (see calibration
// session notes): index 0=11:00, 1=10:00, 2=8:45, 3=6:45, 4=5:45, 5=3:45,
// 6=2:45, 7=1:00 — an even 360 deg loop going counterclockwise. Tilt
// (RX/RY) shows as a 2-LED pair straddling the compass point; slide
// (TX/TY) shows as a 4-LED half-ring. See lightpattern.md for the display
// grammar this implements.
const int kTiltFront[]  = {7, 0};
const int kTiltBack[]   = {3, 4};
const int kTiltLeft[]   = {1, 2};
const int kTiltRight[]  = {5, 6};
const int kSlideLeftGrp[]  = {0, 1, 2, 3};  // 6 to 12 o'clock
const int kSlideRightGrp[] = {4, 5, 6, 7};  // 12 to 6 o'clock
const int kSlideFwdGrp[]   = {0, 1, 6, 7};  // 3 to 9 o'clock
const int kSlideBackGrp[]  = {2, 3, 4, 5};  // 9 to 3 o'clock

const unsigned long kCyan = 0x00FFFF;
const unsigned long kGreen = 0x00FF00;
const unsigned long kRed = 0xFF0000;
const unsigned long kBlue = 0x0000FF;
const unsigned long kOff = 0x000000;

// Blink/rotation rate scales with magnitude (lightpattern.md: "the active
// color will blink at a rate proportional to the angle or twist") — linear
// between a slow rate near the dead zone and a fast rate at full deflection.
const unsigned long kBlinkPeriodSlowMs = 500;
const unsigned long kBlinkPeriodFastMs = 80;
const float kRotMaxLedsPerSec = 48.0f;  // ~6 full revolutions/sec at full deflection (8 LEDs/rev)

unsigned long blinkPeriodMs(float magnitude) {
  const float frac = fminf(1.0f, fabsf(magnitude) / Config::AXIS_LIMIT);
  return kBlinkPeriodSlowMs -
         static_cast<unsigned long>(frac * (kBlinkPeriodSlowMs - kBlinkPeriodFastMs));
}

enum DisplayMode {
  kNone = -1,
  kModeSlideLeft,
  kModeSlideRight,
  kModeSlideFwd,
  kModeSlideBack,
  kModeTiltFront,
  kModeTiltBack,
  kModeTiltLeft,
  kModeTiltRight,
  kModeTzPull,
  kModeTzPush,
  kModeTwist,
};
}  // namespace

void IdleState::enter() {
  lastUpdateMs_ = 0;
  lastActivityMs_ = millis();
  lastDirectionMode_ = kNone;
  blinkOn_ = false;
  lastBlinkMs_ = 0;
  rotPhase_ = 0.0f;
  lastRotUpdateMs_ = 0;
  ledController.setSolid(ledController.idleColor());
}

bool IdleState::handleCalibrationRequest() {
  if (inputController.takeCalibrationRequest()) {
    stateMachine.changeState(&StateMachine::calibratingState);
    return true;
  }
  return false;
}

void IdleState::runMotionPipeline(float dt, unsigned long now) {
  float raw[9] = {};
  if (!sensorController.readRaw(raw)) {
    return;
  }

  const float* baseline = sensorController.baseline();

  // Expose baseline-subtracted deltas via HID feature report for calibration host.
  float delta[9];
  for (int i = 0; i < 9; i++) delta[i] = raw[i] - baseline[i];
  hidController.setSensorDelta(delta);

  float motion[6] = {};
  motionController.compute(raw, baseline, dt, motion);

  if (motionController.hasMotionActivity()) {
    lastActivityMs_ = now;
  }

  // Pick the single dominant axis to display — GEO_M already decouples the
  // 6 outputs (unlike the old linear-matrix model, no per-axis suppression
  // is needed here anymore: motion[] is already dead-zone gated, so any
  // nonzero value already cleared its own axis's threshold).
  const float tx = motion[0];
  const float ty = motion[1];
  const float tz = motion[2];
  const float rx = motion[3];
  const float ry = motion[4];
  const float rz = motion[5];
  const float mags[6] = {fabsf(tx), fabsf(ty), fabsf(tz), fabsf(rx), fabsf(ry), fabsf(rz)};
  int best = 0;
  for (int k = 1; k < 6; k++) {
    if (mags[k] > mags[best]) best = k;
  }

  int mode = kNone;
  float activeMag = 0.0f;
  if (mags[best] > 0.0f) {
    activeMag = mags[best];
    switch (best) {
      // GEO_AXIS_GAIN[TX] is negative (fixes inverted CAD view motion, see
      // Config.h), so tx's sign is physically backwards here — flip to
      // keep the LED direction matching the real motion.
      case 0: mode = (tx > 0) ? kModeSlideRight : kModeSlideLeft; break;
      case 1: mode = (ty > 0) ? kModeSlideFwd : kModeSlideBack; break;
      case 2: mode = (tz > 0) ? kModeTzPull : kModeTzPush; break;
      case 3: mode = (rx > 0) ? kModeTiltBack : kModeTiltFront; break;
      case 4: mode = (ry > 0) ? kModeTiltLeft : kModeTiltRight; break;
      case 5: mode = kModeTwist; break;
    }
  }
  updateDirectionDisplay(mode, activeMag, rz, now);

  const uint16_t buttonBits = inputController.buttonBits();
  const bool hidReportSent = hidController.sendReports(motion, buttonBits);
  if (telemetryController.enabled()) {
    telemetryController.publish(raw, baseline, motion, buttonBits, hidReportSent);
  }
}

void IdleState::updateDirectionDisplay(int mode, float magnitude, float rz, unsigned long now) {
  if (mode == kModeTwist) {
    // Continuous circling dot, speed/direction proportional to RZ. Ring
    // index order is counterclockwise (see layout comment), so a
    // clockwise-looking spin (RZ+ = twist cw, confirmed live) needs the
    // index moving in decreasing order.
    const float frac = fminf(1.0f, fabsf(magnitude) / Config::AXIS_LIMIT);
    const float speed = frac * kRotMaxLedsPerSec;  // LEDs/sec
    const float elapsedS = (lastRotUpdateMs_ == 0) ? 0.0f : (now - lastRotUpdateMs_) / 1000.0f;
    lastRotUpdateMs_ = now;
    const float dir = (rz > 0) ? -1.0f : 1.0f;
    const int ledCount = ledController.pixelCount();
    rotPhase_ += dir * speed * elapsedS;
    while (rotPhase_ < 0.0f) rotPhase_ += ledCount;
    while (rotPhase_ >= ledCount) rotPhase_ -= ledCount;
    const int idx = static_cast<int>(rotPhase_) % ledCount;
    lastDirectionMode_ = mode;
    ledController.setPixelGroupOnBackground(&idx, 1, kBlue, kCyan);
    return;
  }
  lastRotUpdateMs_ = 0;  // so twist doesn't jump on re-entry after another mode

  if (mode == kNone) {
    if (lastDirectionMode_ != kNone) {
      lastDirectionMode_ = kNone;
      ledController.setSolid(ledController.idleColor());
    }
    return;
  }

  bool repaint = false;
  if (mode != lastDirectionMode_) {
    lastDirectionMode_ = mode;
    blinkOn_ = true;
    lastBlinkMs_ = now;
    repaint = true;
  } else if (now - lastBlinkMs_ >= blinkPeriodMs(magnitude)) {
    lastBlinkMs_ = now;
    blinkOn_ = !blinkOn_;
    repaint = true;
  }
  if (!repaint) {
    return;
  }

  switch (mode) {
    case kModeTzPull:
      ledController.setSolid(blinkOn_ ? kGreen : kOff); break;
    case kModeTzPush:
      ledController.setSolid(blinkOn_ ? kRed : kOff); break;
    case kModeSlideLeft:
      ledController.setTwoGroups(kSlideLeftGrp, 4, blinkOn_ ? kGreen : kOff,
                                  kSlideRightGrp, 4, kRed); break;
    case kModeSlideRight:
      ledController.setTwoGroups(kSlideRightGrp, 4, blinkOn_ ? kGreen : kOff,
                                  kSlideLeftGrp, 4, kRed); break;
    case kModeSlideFwd:
      ledController.setTwoGroups(kSlideFwdGrp, 4, blinkOn_ ? kGreen : kOff,
                                  kSlideBackGrp, 4, kRed); break;
    case kModeSlideBack:
      ledController.setTwoGroups(kSlideBackGrp, 4, blinkOn_ ? kGreen : kOff,
                                  kSlideFwdGrp, 4, kRed); break;
    case kModeTiltFront:
      ledController.setPixelGroupOnBackground(kTiltFront, 2, blinkOn_ ? kGreen : kCyan, kCyan); break;
    case kModeTiltBack:
      ledController.setPixelGroupOnBackground(kTiltBack, 2, blinkOn_ ? kGreen : kCyan, kCyan); break;
    case kModeTiltLeft:
      ledController.setPixelGroupOnBackground(kTiltLeft, 2, blinkOn_ ? kGreen : kCyan, kCyan); break;
    case kModeTiltRight:
      ledController.setPixelGroupOnBackground(kTiltRight, 2, blinkOn_ ? kGreen : kCyan, kCyan); break;
  }
}

void IdleState::handleSleepTransition(unsigned long now) {
  const unsigned long inactiveMs = now - lastActivityMs_;
  if (inactiveMs >= Config::IDLE_SLEEP_TIMEOUT_MS) {
    stateMachine.changeState(&StateMachine::sleepState);
  }
}

void IdleState::update() {
  inputController.update();

  if (handleCalibrationRequest()) {
    return;
  }

  if (inputController.takeColorCycleRequest()) {
    ledController.cycleIdleColor();
  }

  const unsigned long now = millis();
  if (inputController.takeActivity()) {
    lastActivityMs_ = now;
  }

  const float dt = (lastUpdateMs_ == 0) ? 0.01
                                        : ((now - lastUpdateMs_) / 1000.0);
  lastUpdateMs_ = now;
  runMotionPipeline(dt, now);
  handleSleepTransition(now);

  // Apply any calibration matrix received via HID output report.
  float newMatrix[6][9];
  if (hidController.takeCalibrationMatrix(newMatrix)) {
    CalibrationStore::save(newMatrix);
    motionController.setDecouplingMatrix(newMatrix);
  }

  // Apply any diagnostic LED color received via HID output report.
  unsigned long ledColor;
  if (hidController.takeLedColor(ledColor)) {
    ledController.setSolid(ledColor);
  }

  // Apply any diagnostic single-pixel command received via HID output report.
  int pixelIndex;
  unsigned long pixelColor;
  if (hidController.takeLedPixel(pixelIndex, pixelColor)) {
    ledController.setPixel(pixelIndex, pixelColor);
  }
}

void IdleState::exit() {}
