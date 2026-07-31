#include <Arduino.h>
#include <LittleFS.h>

#include "CalibrationStore.h"
#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

InputController inputController;
LEDController ledController;
SensorController sensorController;
MotionController motionController;
HIDController hidController;
TelemetryController telemetryController;

void setup() {
  hidController.begin();

  if (Config::ENABLE_TELEMETRY) {
    Serial.begin(115200);
    delay(200);
  }

  LittleFS.begin();

  inputController.begin();
  ledController.begin();
  sensorController.begin();
  motionController.reset();
  telemetryController.begin();

  // Load persisted calibration matrix if one has been saved.
  float matrix[6][9];
  if (CalibrationStore::load(matrix)) {
    motionController.setDecouplingMatrix(matrix);
  }

  stateMachine.changeState(&StateMachine::calibratingState);
}

void loop() {
  hidController.task();
  stateMachine.update();
}
