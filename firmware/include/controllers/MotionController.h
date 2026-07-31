#pragma once

class MotionController {
 public:
  void reset();
  void compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;
  void setDecouplingMatrix(const float matrix[6][9]);

 private:
  static float clampf(float v, float lo, float hi);
  static float lowpass(float prev, float x, float dt, float tau);
  // Raw (pre-GEO_M) position+orientation from the 3 sensor points — see
  // MotionController.cpp for the derivation (ported from
  // interactive_calibrate.py's compute_3d_point/compute_orientation).
  static void computeGeometric(const float delta[9], float g[6]);
  float filt_[6] = {};
  bool motionActive_ = false;
  float customMatrix_[6][9] = {};
  bool hasCustomMatrix_ = false;
};
