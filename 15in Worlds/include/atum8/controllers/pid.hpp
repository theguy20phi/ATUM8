#pragma once

namespace atum8 {
class Pid {
public:
  Pid(float, float, float, double, double);

  double getOutput(double state, double reference);
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