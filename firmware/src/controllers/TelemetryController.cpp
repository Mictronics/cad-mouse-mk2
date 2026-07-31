#include "controllers/TelemetryController.h"
#include <Arduino.h>
#include "Config.h"

namespace {
const int kPrintEvery = 5;
}

void TelemetryController::begin() { tick_ = 0; }

bool TelemetryController::enabled() const { return Config::ENABLE_TELEMETRY; }

void TelemetryController::publish(const float raw[9], const float baseline[9],
                                  const float motion[6], int buttonBits,
                                  bool hidReportSent) {
  float delta[9];
  for (int i = 0; i < 9; i++) {
    delta[i] = raw[i] - baseline[i];
  }
  if (!enabled()) {
    return;
  }

  tick_++;
  if ((tick_ % kPrintEvery) != 0) {
    return;
  }

  // Baseline-subtracted sensor deltas (inputs to the motion pipeline)
  Serial.print(">s0x:"); Serial.println(delta[0]);
  Serial.print(">s0y:"); Serial.println(delta[1]);
  Serial.print(">s0z:"); Serial.println(delta[2]);
  Serial.print(">s1x:"); Serial.println(delta[3]);
  Serial.print(">s1y:"); Serial.println(delta[4]);
  Serial.print(">s1z:"); Serial.println(delta[5]);
  Serial.print(">s2x:"); Serial.println(delta[6]);
  Serial.print(">s2y:"); Serial.println(delta[7]);
  Serial.print(">s2z:"); Serial.println(delta[8]);

  Serial.print(">X:");
  Serial.println(motion[0]);
  Serial.print(">Y:");
  Serial.println(motion[1]);
  Serial.print(">Z:");
  Serial.println(motion[2]);
  Serial.print(">Rx:");
  Serial.println(motion[3]);
  Serial.print(">Ry:");
  Serial.println(motion[4]);
  Serial.print(">Rz:");
  Serial.println(motion[5]);
  Serial.print(">btn:");
  Serial.println(buttonBits & 0x0003);
  Serial.print(">hid:");
  Serial.println(hidReportSent ? 1 : 0);
}
