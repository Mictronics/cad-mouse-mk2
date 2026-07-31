#include "controllers/MotionController.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "Config.h"

namespace {
enum RawIndex {
  RAW_MAG1_X = 0,
  RAW_MAG1_Y,
  RAW_MAG1_Z,
  RAW_MAG2_X,
  RAW_MAG2_Y,
  RAW_MAG2_Z,
  RAW_MAG3_X,
  RAW_MAG3_Y,
  RAW_MAG3_Z
};

enum AxisIndex {
  AXIS_TX = 0,
  AXIS_TY,
  AXIS_TZ,
  AXIS_RX,
  AXIS_RY,
  AXIS_RZ
};

const float kPi = 3.14159265358979323846f;
const float kRadToDeg = 180.0f / kPi;

void crossProduct(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}
}  // namespace

void MotionController::reset() {
  for (int i = 0; i < 6; i++) {
    filt_[i] = 0.0;
  }
  motionActive_ = false;
}

void MotionController::setDecouplingMatrix(const float matrix[6][9]) {
  memcpy(customMatrix_, matrix, sizeof(customMatrix_));
  hasCustomMatrix_ = true;
}

float MotionController::clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float MotionController::lowpass(float prev, float x, float dt, float tau) {
  if (tau <= 0.0) return x;
  const float a = dt / (tau + dt);
  return prev + a * (x - prev);
}

// Raw (pre-GEO_M) position+orientation from the 3 sensor points. Ported
// from interactive_calibrate.py's compute_3d_point()/compute_orientation():
// each sensor's baseline-subtracted (dx,dy,dz) becomes a vector, offset by
// that sensor's real mounting position and scaled to cm; averaging the 3
// implied points gives (x,y,z) relative to rest. The plane through the 3
// current points gives a normal vector whose tilt away from straight-up is
// Rx/Ry; each point's angular position around the centroid vs. its known
// rest angle (from the real PCB geometry) gives twist Rz.
void MotionController::computeGeometric(const float delta[9], float g[6]) {
  float pts[3][3];
  for (int i = 0; i < 3; i++) {
    const float dx = delta[i * 3 + 0];
    const float dy = delta[i * 3 + 1];
    const float dz = delta[i * 3 + 2];
    pts[i][0] = Config::GEO_SENSOR_POS_CM[i][0] + Config::GEO_POINT_SCALE * dx;
    pts[i][1] = Config::GEO_SENSOR_POS_CM[i][1] + Config::GEO_POINT_SCALE * dy;
    pts[i][2] = Config::GEO_TARGET_HEIGHT_CM + Config::GEO_POINT_SCALE * dz;
  }
  const float avgX = (pts[0][0] + pts[1][0] + pts[2][0]) / 3.0f;
  const float avgY = (pts[0][1] + pts[1][1] + pts[2][1]) / 3.0f;
  const float avgZ = (pts[0][2] + pts[1][2] + pts[2][2]) / 3.0f;

  const float v1[3] = {pts[1][0] - pts[0][0], pts[1][1] - pts[0][1], pts[1][2] - pts[0][2]};
  const float v2[3] = {pts[2][0] - pts[0][0], pts[2][1] - pts[0][1], pts[2][2] - pts[0][2]};
  float normal[3];
  crossProduct(v1, v2, normal);
  const float nmag = sqrtf(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
  if (nmag < 1e-9f) {
    normal[0] = 0.0f;
    normal[1] = 0.0f;
    normal[2] = 1.0f;
  } else {
    normal[0] /= nmag;
    normal[1] /= nmag;
    normal[2] /= nmag;
  }
  if (normal[2] < 0.0f) {  // keep normal pointing "up" regardless of winding
    normal[0] = -normal[0];
    normal[1] = -normal[1];
    normal[2] = -normal[2];
  }
  const float rxDeg = atan2f(normal[1], normal[2]) * kRadToDeg;
  const float ryDeg = atan2f(-normal[0], normal[2]) * kRadToDeg;

  float diffSum = 0.0f;
  for (int i = 0; i < 3; i++) {
    const float restAngle = atan2f(Config::GEO_SENSOR_POS_CM[i][1], Config::GEO_SENSOR_POS_CM[i][0]);
    const float curAngle = atan2f(pts[i][1] - avgY, pts[i][0] - avgX);
    float diff = curAngle - restAngle;
    while (diff > kPi) diff -= 2.0f * kPi;
    while (diff < -kPi) diff += 2.0f * kPi;
    diffSum += diff;
  }
  const float rzDeg = (diffSum / 3.0f) * kRadToDeg;

  g[AXIS_TX] = avgX;
  g[AXIS_TY] = avgY;
  g[AXIS_TZ] = avgZ - Config::GEO_TARGET_HEIGHT_CM;
  g[AXIS_RX] = rxDeg;
  g[AXIS_RY] = ryDeg;
  g[AXIS_RZ] = rzDeg;
}

void MotionController::compute(const float raw[9], const float* baseline, float dt,
                               float out[6]) {
  // Baseline subtraction converts magnetic deltas around the calibrated rest pose.
  const float mag1x = raw[RAW_MAG1_X] - baseline[RAW_MAG1_X];
  const float mag1y = raw[RAW_MAG1_Y] - baseline[RAW_MAG1_Y];
  const float mag1z = raw[RAW_MAG1_Z] - baseline[RAW_MAG1_Z];
  const float mag2x = raw[RAW_MAG2_X] - baseline[RAW_MAG2_X];
  const float mag2y = raw[RAW_MAG2_Y] - baseline[RAW_MAG2_Y];
  const float mag2z = raw[RAW_MAG2_Z] - baseline[RAW_MAG2_Z];
  const float mag3x = raw[RAW_MAG3_X] - baseline[RAW_MAG3_X];
  const float mag3y = raw[RAW_MAG3_Y] - baseline[RAW_MAG3_Y];
  const float mag3z = raw[RAW_MAG3_Z] - baseline[RAW_MAG3_Z];

  const float delta[9] = {
    mag1x, mag1y, mag1z,
    mag2x, mag2y, mag2z,
    mag3x, mag3y, mag3z,
  };

  float g[6];
  computeGeometric(delta, g);

  float y[6] = {};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      y[i] += Config::GEO_M[i][j] * g[j];
    }
    y[i] *= Config::GEO_AXIS_GAIN[i];
  }

  // Filter, clamp to range and dead zones.
  motionActive_ = false;
  for (int i = 0; i < 6; i++) {
    const float dead = Config::GEO_DEAD[i];

    if (fabs(y[i]) < dead) {
      filt_[i] = 0.0;
    } else {
      filt_[i] = lowpass(filt_[i], y[i], dt, Config::SMOOTH_TAU_S);
    }

    const float scaled = filt_[i] * Config::GEO_OUTPUT_SCALE[i];
    out[i] = clampf(scaled, -Config::AXIS_LIMIT, Config::AXIS_LIMIT);
    if (fabs(filt_[i]) >= dead) {
      motionActive_ = true;
    }
  }
}

bool MotionController::hasMotionActivity() const { return motionActive_; }
