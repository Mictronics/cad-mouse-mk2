#include "states/SleepState.h"

#include <Arduino.h>

#include "CalibrationStore.h"
#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void SleepState::enter() {
  ledController.off();
}

void SleepState::update() {
  inputController.update();

  if (inputController.takeActivity()) {
    stateMachine.changeState(&StateMachine::idleState);
    return;
  }

  // A diagnostic LED command (HID output report 6/7) is host-initiated test
  // activity — wake up so the tester gets visible feedback instead of the
  // command being silently dropped (only IdleState normally applies it).
  unsigned long ledColor;
  if (hidController.takeLedColor(ledColor)) {
    stateMachine.changeState(&StateMachine::idleState);
    ledController.setSolid(ledColor);
    return;
  }

  int pixelIndex;
  unsigned long pixelColor;
  if (hidController.takeLedPixel(pixelIndex, pixelColor)) {
    stateMachine.changeState(&StateMachine::idleState);
    ledController.setPixel(pixelIndex, pixelColor);
    return;
  }

  // A calibration matrix (HID output report 5) is host-initiated too —
  // apply and save it immediately rather than silently dropping it (only
  // IdleState normally consumes it).
  float newMatrix[6][9];
  if (hidController.takeCalibrationMatrix(newMatrix)) {
    CalibrationStore::save(newMatrix);
    motionController.setDecouplingMatrix(newMatrix);
    stateMachine.changeState(&StateMachine::idleState);
    return;
  }
}

void SleepState::exit() {}
