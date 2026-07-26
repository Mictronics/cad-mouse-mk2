#pragma once

#include "State.h"

class IdleState : public State {
 public:
  void enter() override;
  void update() override;
  void exit() override;

 private:
  bool handleCalibrationRequest();
  void runMotionPipeline(float dt, unsigned long now);
  void updateDirectionDisplay(int mode, float magnitude, float rz, unsigned long now);
  void handleSleepTransition(unsigned long now);

  unsigned long lastUpdateMs_ = 0;
  unsigned long lastActivityMs_ = 0;
  // -1 = showing idle color, otherwise one of the DisplayMode values in
  // IdleState.cpp (live direction indicator). Tracked so the ring is only
  // re-driven when the displayed direction changes.
  int lastDirectionMode_ = -1;
  // Active-color blink, rate scales with motion magnitude (see
  // lightpattern.md) — used by push/pull, tilt, and slide.
  bool blinkOn_ = false;
  unsigned long lastBlinkMs_ = 0;
  // Twist: a single green LED circles the (cyan) ring at a rate/direction
  // matching RZ. Position tracked as a float so slow twists still animate
  // smoothly instead of jumping a whole LED at a time.
  float rotPhase_ = 0.0f;
  unsigned long lastRotUpdateMs_ = 0;
};
