#include "controllers/LEDController.h"

#include <LittleFS.h>

#include "Config.h"

namespace {
const char* kColorPath = "/led_color.bin";
}  // namespace

LEDController::LEDController()
    : ring_(Config::LED_COUNT, Config::PIN_LED_DATA, NEO_GRB + NEO_KHZ800)
#if defined(PIN_NEOPIXEL)
    , onboardLed_(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800)
#endif
    {}

void LEDController::fillAll(unsigned long color) {
  for (int i = 0; i < ring_.numPixels(); i++) {
    ring_.setPixelColor(i, color);
  }
}

unsigned long LEDController::toNeoColor(unsigned long color) {
  int r = (color >> 16) & 0xFF;
  int g = (color >> 8) & 0xFF;
  int b = color & 0xFF;
  return ring_.Color(r, g, b);
}

void LEDController::begin() {
#if defined(PIN_NEOPIXEL)
  // The XIAO RP2350 replaced discrete R/G/B on-board LEDs with a single
  // WS2812. Framework GPIO init can send garbage data to it; silence it first.
  onboardLed_.begin();
  onboardLed_.setPixelColor(0, 0);
  onboardLed_.show();
#endif

  pinMode(Config::PIN_LED_LS, OUTPUT);
  digitalWrite(Config::PIN_LED_LS, LOW);

  File f = LittleFS.open(kColorPath, "r");
  if (f) {
    const int idx = f.read();
    f.close();
    if (idx >= 0 && idx < Config::LED_IDLE_PALETTE_COUNT) {
      idleColorIndex_ = idx;
    }
  }

  ring_.begin();
  ring_.setBrightness(Config::LED_BRIGHTNESS);
  ring_.show();
}

unsigned long LEDController::idleColor() const {
  return Config::LED_IDLE_PALETTE[idleColorIndex_];
}

void LEDController::cycleIdleColor() {
  idleColorIndex_ = (idleColorIndex_ + 1) % Config::LED_IDLE_PALETTE_COUNT;
  File f = LittleFS.open(kColorPath, "w");
  if (f) {
    f.write(static_cast<uint8_t>(idleColorIndex_));
    f.close();
  }
  setSolid(idleColor());
}

void LEDController::setPower(bool enabled) {
  if (enabled == isPowered_) {
    return;
  }

  isPowered_ = enabled;
  digitalWrite(Config::PIN_LED_LS, enabled ? HIGH : LOW);
  delay(10);
  
}

void LEDController::setSolid(unsigned long color) {
  mode_ = Mode::Solid;
  color_ = toNeoColor(color);
  setPower(true);
  fillAll(color_);
  ring_.show();
}

void LEDController::startSpinner(unsigned long color) {
  mode_ = Mode::Spinner;
  color_ = toNeoColor(color);
  spinnerIndex_ = 0;
  spinnerStarted_ = false;
  lastSpinnerStepMs_ = 0;
  setPower(true);
}

void LEDController::updateSpinner() {
  if (mode_ != Mode::Spinner) {
    return;
  }

  const unsigned long now = millis();
  if (spinnerStarted_ && (now - lastSpinnerStepMs_) < 60) {
    return;
  }
  spinnerStarted_ = true;
  lastSpinnerStepMs_ = now;

  fillAll(0);
  int pixelCount = ring_.numPixels();
  ring_.setPixelColor(spinnerIndex_, color_);
  ring_.show();

  spinnerIndex_++;
  if (spinnerIndex_ >= pixelCount) {
    spinnerIndex_ = 0;
  }
}

void LEDController::setPixel(int index, unsigned long color) {
  mode_ = Mode::Solid;
  setPower(true);
  fillAll(0);
  if (index >= 0 && index < ring_.numPixels()) {
    ring_.setPixelColor(index, toNeoColor(color));
  }
  ring_.show();
}

void LEDController::setPixelGroup(const int* indices, int count, unsigned long color) {
  mode_ = Mode::Solid;
  setPower(true);
  fillAll(0);
  const unsigned long neo = toNeoColor(color);
  for (int k = 0; k < count; k++) {
    const int idx = indices[k];
    if (idx >= 0 && idx < ring_.numPixels()) {
      ring_.setPixelColor(idx, neo);
    }
  }
  ring_.show();
}

void LEDController::setTwoGroups(const int* indicesA, int countA, unsigned long colorA,
                                  const int* indicesB, int countB, unsigned long colorB) {
  mode_ = Mode::Solid;
  setPower(true);
  fillAll(0);
  const unsigned long neoA = toNeoColor(colorA);
  const unsigned long neoB = toNeoColor(colorB);
  for (int k = 0; k < countA; k++) {
    const int idx = indicesA[k];
    if (idx >= 0 && idx < ring_.numPixels()) {
      ring_.setPixelColor(idx, neoA);
    }
  }
  for (int k = 0; k < countB; k++) {
    const int idx = indicesB[k];
    if (idx >= 0 && idx < ring_.numPixels()) {
      ring_.setPixelColor(idx, neoB);
    }
  }
  ring_.show();
}

void LEDController::setPixelGroupOnBackground(const int* indices, int count,
                                               unsigned long fgColor,
                                               unsigned long bgColor) {
  mode_ = Mode::Solid;
  setPower(true);
  fillAll(toNeoColor(bgColor));
  const unsigned long neo = toNeoColor(fgColor);
  for (int k = 0; k < count; k++) {
    const int idx = indices[k];
    if (idx >= 0 && idx < ring_.numPixels()) {
      ring_.setPixelColor(idx, neo);
    }
  }
  ring_.show();
}

void LEDController::off() {
  mode_ = Mode::Off;
  fillAll(0);
  ring_.show();
  setPower(false);
}
