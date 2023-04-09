#include "atum8/misc/utility.hpp"
namespace atum8{
namespace utility {
double rpmToPower(double rpm) { return (rpm * 12000) / 200; }

double reduce_0_to_360(double angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

double constrain180(double angle) {
  while(angle < -180 || angle >= 180) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

double constrain90(double angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { angle += 180; }
    if(angle >= 90) { angle -= 180; }
  }
  return(angle);
}
double convertRadianToDegree(double angleRadian) {
  return angleRadian * (180.0 / M_PI);
}
double convertDegreeToRadian(double angleDegree) {
  return angleDegree / (180.0 / M_PI);
}
double clamp(double input, double min, double max) {
  if (input > max)
    return max;
  if (input < min)
    return min;
  return input;
}
} // namespace utility
}