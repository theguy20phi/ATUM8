#include "atum8/sensors/imus.hpp"
#include "main.h"


namespace atum8 {
void Imus::resetImuSensors() {
  imuSensorAlpha.tare();
  imuSensorBeta.tare();
  imuSensorCharlie.tare();
}

double Imus::getImuSensorAverages() {
  const double rawAvg{((imuSensorAlpha.get_rotation() + imuSensorBeta.get_rotation() + 0)/2)};//imuSensorCharlie.get_rotation()) / 3.0)};
  return rawAvg;
}

void Imus::calibrateImuSensors() {
  atum8::imuSensorAlpha.reset();
  atum8::imuSensorBeta.reset();
  atum8::imuSensorCharlie.reset(true);
   while (atum8::imuSensorAlpha.is_calibrating() ||
          atum8::imuSensorBeta.is_calibrating() ||
          atum8::imuSensorCharlie.is_calibrating()) {
     pros::lcd::set_text(1, "IMUs ARE CALIBRATING DON'T TOUCH!!!");
     pros::delay(100);
   }
}

double Imus::getDeltaHeadingImu() {
  const double heading{getImuSensorAverages()};
  const double deltaHeadingOutput{heading - prevHeading};
  prevHeading = heading;
  return deltaHeadingOutput * (M_PI / 180.0);
}
} // namespace atum8