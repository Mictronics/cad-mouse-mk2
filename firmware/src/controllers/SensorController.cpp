#include "controllers/SensorController.h"

#include <string.h>

#include "Config.h"

using namespace ifx::tlx493d;

SensorController::SensorController()
    : mag1Sensor_(Wire, TLx493D_IIC_ADDR_A0_e),
      mag2Sensor_(Wire, TLx493D_IIC_ADDR_A0_e),
      mag3Sensor_(Wire, TLx493D_IIC_ADDR_A0_e) {}

void SensorController::powerOff(int pin) { digitalWrite(pin, LOW); }

void SensorController::powerOn(int pin) {
  digitalWrite(pin, HIGH);
  delay(5);
}

void SensorController::begin() {

  pinMode(Config::PIN_MAG1_LS, OUTPUT);
  pinMode(Config::PIN_MAG2_LS, OUTPUT);
  pinMode(Config::PIN_MAG3_LS, OUTPUT);

  // All three rails are pulled high in hardware, so force them all off first
  // before bringing sensors up one-by-one for address assignment.

  powerOff(Config::PIN_MAG1_LS);
  powerOff(Config::PIN_MAG2_LS);
  powerOff(Config::PIN_MAG3_LS);
  delay(5);

  Wire.begin();
  Wire.setClock(400000);

  powerOn(Config::PIN_MAG1_LS);
  mag1Sensor_.begin(true, false, false, true);
  mag1Sensor_.setIICAddress(TLx493D_IIC_ADDR_A2_e);
  mag1Sensor_.setSensitivity(TLx493D_EXTRA_SHORT_RANGE_e);
  delay(10);

  powerOn(Config::PIN_MAG2_LS);
  mag2Sensor_.begin(true, false, false, true);
  mag2Sensor_.setIICAddress(TLx493D_IIC_ADDR_A1_e);
  mag2Sensor_.setSensitivity(TLx493D_EXTRA_SHORT_RANGE_e);
  delay(10);

  powerOn(Config::PIN_MAG3_LS);
  mag3Sensor_.begin(true, false, false, true);
  mag3Sensor_.setSensitivity(TLx493D_EXTRA_SHORT_RANGE_e);
  delay(10);
}

bool SensorController::readRaw(float out[9]) {
  double mag1x = 0, mag1y = 0, mag1z = 0, temp1 = 0;
  double mag2x = 0, mag2y = 0, mag2z = 0, temp2 = 0;
  double mag3x = 0, mag3y = 0, mag3z = 0, temp3 = 0;

  const bool ok1 = mag1Sensor_.getMagneticFieldAndTemperature(&mag1x, &mag1y, &mag1z, &temp1);
  const bool ok2 = mag2Sensor_.getMagneticFieldAndTemperature(&mag2x, &mag2y, &mag2z, &temp2);
  const bool ok3 = mag3Sensor_.getMagneticFieldAndTemperature(&mag3x, &mag3y, &mag3z, &temp3);

  if (!ok1 || !ok2 || !ok3) {
    return false;
  }

  // MAG1 = bottom, MAG2 = top left, MAG3 = top right.
  const float raw[9] = {
    static_cast<float>(mag1x), static_cast<float>(mag1y), static_cast<float>(mag1z),
    static_cast<float>(mag2x), static_cast<float>(mag2y), static_cast<float>(mag2z),
    static_cast<float>(mag3x), static_cast<float>(mag3y), static_cast<float>(mag3z),
  };

  // Running average (exponential) of the raw per-channel readings — every
  // caller (real-time motion, baseline calibration, and the HID feature
  // report calibrate.py reads) was previously seeing an unsmoothed single
  // I2C sample, which fed noise straight into the dead-zone comparison and
  // into calibration data collection.
  if (!smoothedInit_) {
    memcpy(smoothed_, raw, sizeof(smoothed_));
    smoothedInit_ = true;
  } else {
    const float alpha = 0.3;
    for (int i = 0; i < 9; i++) {
      smoothed_[i] += alpha * (raw[i] - smoothed_[i]);
    }
  }
  memcpy(out, smoothed_, sizeof(smoothed_));
  return true;
}

void SensorController::beginCalibration() {
  calibrationActive_ = true;
  calibrationDone_ = false;
  calibrationSamples_ = 0;
  calibrationSampleStarted_ = false;
  lastCalibrationSampleMs_ = 0;
  for (int i = 0; i < 9; i++) {
    calibrationSum_[i] = 0.0;
  }
}

void SensorController::updateCalibration() {
  if (!calibrationActive_) {
    return;
  }

  const unsigned long now = millis();
  if (calibrationSampleStarted_ && (now - lastCalibrationSampleMs_) < 10) {
    return;
  }
  calibrationSampleStarted_ = true;
  lastCalibrationSampleMs_ = now;

  float raw[9] = {};
  if (!readRaw(raw)) {
    return;
  }

  for (int i = 0; i < 9; i++) {
    calibrationSum_[i] += raw[i];
  }

  calibrationSamples_++;
  if (calibrationSamples_ < Config::ZERO_SAMPLES) {
    return;
  }

  for (int i = 0; i < 9; i++) {
    baseline_[i] = calibrationSum_[i] / Config::ZERO_SAMPLES;
  }

  calibrationActive_ = false;
  calibrationDone_ = true;
}

bool SensorController::calibrationDone() const { return calibrationDone_; }

const float* SensorController::baseline() const { return baseline_; }
