#include "CalibrationStore.h"

#include <LittleFS.h>
#include <string.h>

namespace {
const char* kPath = "/calibration.bin";
const uint32_t kMagic = 0xCADB0001;
const size_t kMatrixBytes = sizeof(float) * 6 * 9;  // 216
}  // namespace

namespace CalibrationStore {

bool load(float matrix[6][9]) {
  File f = LittleFS.open(kPath, "r");
  if (!f) return false;

  uint32_t magic = 0;
  if (f.read(reinterpret_cast<uint8_t*>(&magic), 4) != 4 || magic != kMagic) {
    f.close();
    return false;
  }

  const bool ok = (f.read(reinterpret_cast<uint8_t*>(matrix), kMatrixBytes)
                   == kMatrixBytes);
  f.close();
  return ok;
}

void save(const float matrix[6][9]) {
  File f = LittleFS.open(kPath, "w");
  if (!f) return;

  f.write(reinterpret_cast<const uint8_t*>(&kMagic), 4);
  f.write(reinterpret_cast<const uint8_t*>(matrix), kMatrixBytes);
  f.close();
}

}  // namespace CalibrationStore
