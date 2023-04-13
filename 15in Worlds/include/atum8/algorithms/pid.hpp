/**
 * @file pid.hpp
 * @author Thomas Tran Dang (thomasdang92@gmail.com.com)
 * @brief This file provides a class for a PID control loop. Since the 15in is only a catapult we only use this for the drive. 
 *        (proportional, integral, deriviate)
 * @version 0.3
 * @date 2023-04-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include "main.h"

namespace atum8 {
class Pid {
public:
  Pid() {
    kP = 0;
    kI = 0;
    kD = 0;
    eT = 0;
    dT = 0;
  }

  Pid(float Kp, float Ki, float Kd, double errorThreshold, double derivativeThreshold);

  double getOutput(double current, double desired);
  double getOutput(double error_);
  void setMaxOutput(double output);
  void reset();
  bool isSettled();

private:
  float kP;
  float kI;
  float kD;

  double error;
  double integral;
  double newIntegral;
  double derivative;
  double prevError;
  double output;
  double maxOutput;

  float eT;
  float dT;
};
}; // namespace atum8