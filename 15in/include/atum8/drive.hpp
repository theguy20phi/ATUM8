#pragma once
#include "catapult.hpp"
#include "driveHelpers.hpp"
#include "globals.hpp"
#include "main.h"


namespace atum8 {

class Drive : DriveHelpers, Catapult {
public:
  void controller();
  void move(double inches, double rpm, double acceleration, bool dift,
            double secThreshold);
  void turn(double angle, double rpm, double acceleration, double secThreshold);

private:
  void resetDrive();
  void linearPID();
  void turnPID();

  // Yes
  // double sampleTime {}
  float speedResistor{1};
  double power;
  double maxPower;
  bool autoCorrecting;
  double driftCorrection;
  double msCounter;

  // Linear PD Variables
  double linearkP;
  double linearkI;
  double linearkD;
  double driftP;

  double linearPower;
  double linearPosition;
  double linearDesired;
  double linearError;
  double linearIntegral;
  double newLinearIntegral;
  double linearDerivative;
  double linearPrevError;

  // Turn PD Variables
  double turnkP;
  double turnkI;
  double turnkD;

  double turnPower;
  double turnPosition;
  double turnDesired;
  double turnError;
  double turnIntegral;
  double newTurnIntegral;
  double turnDerivative;
  double turnPrevError;
};
} // namespace atum8