#pragma once

// Persists the 6x9 decoupling matrix to LittleFS flash so calibration
// survives power cycles without reflashing the firmware.
namespace CalibrationStore {

// Reads the stored matrix into `matrix`. Returns true on success.
// Call after LittleFS.begin() (done in main.cpp setup).
bool load(float matrix[6][9]);

// Writes `matrix` to flash. Overwrites any previous value.
void save(const float matrix[6][9]);

}  // namespace CalibrationStore
