#include "atum8/sensors/imus.hpp"
#include "main.h"


namespace atum8 {
void Imus::resetImuSensors() {
  imuSensorAlpha.tare();
  imuSensorBeta.tare();
  imuSensorCharlie.tare();
}

double Imus::getImuSensorAverages() {
  return ((imuSensorAlpha.get_rotation() + imuSensorBeta.get_rotation() + imuSensorCharlie.get_rotation()) / 3);
}
} // namespace atum8