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

void Imus::calibrateImuSensors() {
  atum8::imuSensorAlpha.reset();
  atum8::imuSensorBeta.reset();
  atum8::imuSensorCharlie.reset();
   while (atum8::imuSensorAlpha.is_calibrating() ||
          atum8::imuSensorBeta.is_calibrating() ||
          atum8::imuSensorCharlie.is_calibrating()) {
     pros::lcd::set_text(1, "IMUs ARE CALIBRATING DON'T TOUCH!!!");
     pros::delay(100);
   }
}
} // namespace atum8