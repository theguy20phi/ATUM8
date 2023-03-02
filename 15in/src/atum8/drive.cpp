#include "atum8/drive.hpp"
#include "main.h"

namespace atum8 {

// User Control Drive Code
void Drive::controller() {
  if (Chris.get_digital(DIGITAL_A)) {
    if (speedResistor == 0.5) {
      speedResistor = 1;
      //Chris.clear();
      //Chris.print(0, 0, "Drive: Max Speed");
      pros::delay(100);
    }

    else if (speedResistor == 1) {
      speedResistor = 0.5;
      //Chris.clear();
      //Chris.print(0, 0, "Drive: Half Speed");
      pros::delay(100);
    }
  }

  setDriveBrakeMode("BRAKE");

  setMoveRight(Chris.get_analog(ANALOG_RIGHT_Y) * speedResistor);
  setMoveLeft(Chris.get_analog(ANALOG_LEFT_Y) * speedResistor);
};

void Drive::resetDrive() {
  power = 0;
  maxPower = 12000;
  driftCorrection = 0;
  autoCorrecting = false;
  msCounter = 0;

  // Reset Linear PID
  linearPower = 0;
  linearPosition = 0;
  linearDesired = 0;
  linearError = 0;
  linearIntegral = 0;
  newLinearIntegral = 0;
  linearDerivative = 0;
  linearPrevError = 0;

  // reset Turn PID
  turnPower = 0;
  turnPosition = 0;
  turnDesired = 0;
  turnError = 0;
  turnIntegral = 0;
  newTurnIntegral = 0;
  turnDerivative = 0;
  turnPrevError = 0;

  resetEncoders();
  resetImuSensors();
  setDriveBrakeMode("COAST");
  setRightPower(0);
  setLeftPower(0);
};

void Drive::move(double inches, double rpm, double acceleration, bool drift,
                 double secThreshold) {
  resetDrive();
  power = rpmToPower(rpm);
  linearkP = 800; //850
  linearkI = 1;
  linearkD = 8.8; //-.022
  driftP = 1000;

  linearDesired = inches;
  maxPower = power;
  drift = autoCorrecting;

  while (true) {
    linearPosition =
        (2 * M_PI * encoderWheelRadius * getEncoderAverages()) / 360;
     std::cout << "linear Position:" << linearPosition << std::endl;

    // Proportional Calculations
    linearError = linearDesired - linearPosition;
    std::cout << "linearError: " << linearError << std:: endl;
            
    // Integral Calculations
    newLinearIntegral = linearIntegral + linearError;

    // Derivative Calculations
    linearDerivative = linearError - linearPrevError;
    linearPrevError = linearError;
    //std:: cout << "linearDerivative:" << linearDerivative << std::endl;

    // Power Caculations
    linearPower = linearError * linearkP + linearIntegral * linearkI +
                  linearDerivative * linearkD;

    // Output Clamping and Integral Wind-up
    if (linearPower > maxPower)
      linearPower = maxPower;
    else if (linearPower < -maxPower)
      linearPower = -maxPower;
    else
      linearIntegral = newLinearIntegral;

    // Drift correction that I will never use
    if (autoCorrecting == true)
      driftCorrection =
          driftP * (getRightEncoderValues() - getLeftEncoderValues());
    else
      driftCorrection = 0;

    // Send power to the drive
    rightAcceleration(linearPower - driftCorrection, acceleration);
    leftAcceleration(linearPower + driftCorrection, acceleration);

    
    // Break if within error threshold
    if (fabs(linearError) < .5 && fabs(linearDerivative) <= .05)
      break;
    
    // Timeout and break if the robot takes too long
    msCounter += 10;
    if (msCounter / 1000 > secThreshold)
      break;

    // Auto lower catapult to save time
    if(atum8::catapultStop.get_value() != 1)
      down();
    else
      stop();

    pros::delay(10);  
  }

  resetDrive();
  //std::cout << "drive has been reset" << std::endl;
};

void Drive::turn(double angle, double rpm, double acceleration,
                 double secThreshold) {
  resetDrive();
  power = rpmToPower(rpm);

  if(fabs(angle) > 50) {
    turnkP = 278;
    turnkI = 0.8;//.036
    turnkD = 0.0;
  }
  else if(fabs(angle) > 30) {
    turnkP = 362;
    turnkI = 0.08;
    turnkD = 0;
  }
  else {
    turnkP = 300;
    turnkI = 0.8;
    turnkD = 0;
  }
  

  turnPrevError = 0;

  turnDesired = angle;
  maxPower = power;

  while (true) {

    turnPosition = getImuSensorAverages();
    std::cout << "Turn Position: " << turnPosition << std::endl;

    // Proportional Calculations
    turnError = (turnDesired - turnPosition);
    std::cout << "Turn Error: " << turnError << std::endl;

    // Integral Calculations
    newTurnIntegral = turnIntegral + turnError;

    // Derivative Calculations
    turnDerivative = turnError - turnPrevError;
    turnPrevError = turnError;
    // std::cout << "Turn D: " << turnDerivative << std::endl;

    // Power Calculations
    turnPower =
        turnError * turnkP + +turnIntegral * turnkI + turnDerivative * turnkD;

    // Output Clamping and Integral Wind-up
    if (turnPower > maxPower)
      turnPower = maxPower;
    else if (turnPower < -maxPower)
      turnPower = -maxPower;
    else
      turnIntegral = newTurnIntegral;

    rightAcceleration(-turnPower, acceleration);
    leftAcceleration(turnPower, acceleration);

   // Break if within error threshold
    if (fabs(turnError) < 1.5 && fabs(turnDerivative) <= .05)
      break;

    // Timeout and break if the robot takes too long
    msCounter += 10;
    if (msCounter / 1000 > secThreshold)
      break;
      
    // Auto lower catapult to save time
    if(atum8::catapultStop.get_value() != 1)
      down();
    else
      stop();

    pros::delay(10);
  }
  
  resetDrive();
};
} // namespace atum8