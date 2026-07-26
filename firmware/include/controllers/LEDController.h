#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

class LEDController {
 public:
  LEDController();
  void begin();
  void setSolid(unsigned long color);
  void startSpinner(unsigned long color);
  void updateSpinner();
  void off();

  // Lights exactly one ring LED (all others off) — used for diagnostics
  // and the live direction indicator.
  void setPixel(int index, unsigned long color);
  int pixelCount() const { return ring_.numPixels(); }

  // Lights a set of ring LEDs (all others off) — used to show a compass
  // point that falls between two physical LEDs, or a half-ring arc.
  void setPixelGroup(const int* indices, int count, unsigned long color);

  // Lights two ring LED groups in two different colors simultaneously —
  // used to show slide-left/right as a two-color split ring.
  void setTwoGroups(const int* indicesA, int countA, unsigned long colorA,
                     const int* indicesB, int countB, unsigned long colorB);

  // Lights `indices` in fgColor, every other ring LED in bgColor — used for
  // tilt (background cyan, direction LEDs green) and the twist "circling
  // dot" (background cyan, one rotating LED green).
  void setPixelGroupOnBackground(const int* indices, int count,
                                  unsigned long fgColor, unsigned long bgColor);

  // Persistent idle color selection (cycled via button gesture).
  unsigned long idleColor() const;
  void cycleIdleColor();

 private:
  enum class Mode {
    Off,
    Solid,
    Spinner,
  };

  void setPower(bool enabled);
  void fillAll(unsigned long color);
  unsigned long toNeoColor(unsigned long color);

  bool isPowered_ = false;
  int idleColorIndex_ = 0;
  Mode mode_ = Mode::Off;
  unsigned long color_ = 0;
  int spinnerIndex_ = 0;
  bool spinnerStarted_ = false;
  unsigned long lastSpinnerStepMs_ = 0;
  Adafruit_NeoPixel ring_;
#if defined(PIN_NEOPIXEL)
  Adafruit_NeoPixel onboardLed_;
#endif
};
