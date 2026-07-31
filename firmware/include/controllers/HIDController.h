#pragma once

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

class HIDController {
 public:
  void begin();
  void task();
  bool sendReports(const float motion[6], uint16_t buttonBits);

  // Called from IdleState each loop: updates the feature report buffer
  void setSensorDelta(const float delta[9]);

  // Returns true if a new calibration matrix arrived via output report.
  // Clears the flag; caller should save + apply the matrix.
  bool takeCalibrationMatrix(float matrix[6][9]);

  // Returns true if a host-driven LED color arrived via output report ID 6
  // (diagnostic use: lets a host script flash the LED ring to cue the
  // tester without needing physical button gestures). Clears the flag.
  bool takeLedColor(unsigned long& color);

  // Returns true if a host-driven single-pixel command arrived via output
  // report ID 7 (index + RGB). Clears the flag.
  bool takeLedPixel(int& index, unsigned long& color);

 private:
  struct __attribute__((packed)) ReportAxes {
    int16_t x, y, z, rx, ry, rz;
  };

  struct __attribute__((packed)) ReportButtons {
    uint16_t bits;
  };

  static ReportAxes makeAxesReport(const float motion[6]);
  bool axesReportChanged(const ReportAxes& axes) const;

  // TinyUSB callback stubs — route to singleton instance
  static uint16_t onGetReport(uint8_t report_id, hid_report_type_t report_type,
                               uint8_t* buffer, uint16_t reqlen);
  static void onSetReport(uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const* buffer, uint16_t bufsize);

  uint16_t handleGetReport(uint8_t report_id, uint8_t* buffer, uint16_t reqlen);
  void handleSetReport(uint8_t report_id, uint8_t const* buffer, uint16_t bufsize);

  Adafruit_USBD_HID usbHid_;
  uint16_t buttonBitsSent_ = 0;
  ReportAxes lastSentAxes_{};

  // Feature report ID 4: 9 sensor-delta floats (36 bytes)
  float sensorDelta_[9] = {};

  // Output report ID 5: pending calibration matrix (54 floats = 216 bytes)
  float pendingMatrix_[6][9] = {};
  bool calibrationMatrixReady_ = false;

  // Output report ID 6: pending diagnostic LED color (3 bytes RGB)
  unsigned long pendingLedColor_ = 0;
  bool ledColorReady_ = false;

  // Output report ID 7: pending single-pixel command (index + RGB)
  int pendingLedPixelIndex_ = 0;
  unsigned long pendingLedPixelColor_ = 0;
  bool ledPixelReady_ = false;
};
