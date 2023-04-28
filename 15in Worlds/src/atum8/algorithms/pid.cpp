#include "atum8/algorithms/pid.hpp"
#include "main.h"


namespace atum8 {

  Pid::Pid(float Kp, float Ki, float Kd, double errorThreshold, double derivativeThreshold){
    kP = Kp;
    kI = Ki;
    kD = Kd;

    eT = errorThreshold;
    dT = derivativeThreshold;
}

double Pid::getOutput(double current, double desired) {

  // Calculate Proportional Value
  error = desired - current;

  // Calculuate Integral Value
  newIntegral = integral + error;

  // Calculate Derivative Value and Update Error
  derivative = error - prevError;
  prevError = error;

  // Calculuate Ouput
  output = error * kP + integral * kI + derivative * kD;

  // Output Clamping and Anti-Integral Wind-up
  if (output > maxOutput)
    output = maxOutput;
  else if (output < -maxOutput)
    output = -maxOutput;
  else
    integral = newIntegral;

  return output;
}

double Pid::getOutput(double error_) {
  //update error
  error = error_;
  // Calculuate Integral Value
  newIntegral = integral + error;

  // Calculate Derivative Value and Update Error
  derivative = error - prevError;
  prevError = error;

  if(!(output > maxOutput) and !(output < -maxOutput))
    integral = newIntegral;

  // Calculuate Ouput
  output = error * kP + integral * kI + derivative * kD;

  return output;
}

void Pid::setMaxOutput(double output) { maxOutput = output; }

bool Pid::isSettled(){
    if(fabs(error) < eT && fabs(derivative) <= dT)
        return true;
    else
        return false;
}

void Pid::reset() {
  error = 0;
  integral = 0;
  newIntegral = 0;
  derivative = 0;
  prevError = 0;
  maxOutput = 12000;
}
} // namespace atum8