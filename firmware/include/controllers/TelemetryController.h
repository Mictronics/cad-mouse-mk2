#pragma once

class TelemetryController {
 public:
  void begin();
  void publish(const float raw[9], const float baseline[9], const float motion[6], int buttonBits, bool hidReportSent);
  bool enabled() const;

 private:
  int tick_ = 0;
};
