#pragma once
#include "atum8/controllers/pid.hpp"
#include "atum8/imus.hpp"
#include "atum8/slewRate.hpp"

namespace atum8 {
    class Drive : public Pid, Imus, SlewRate{
        public:
          void controller();
          void move(double inches, double rpm, double acceleration, bool dift, double secThreshold);
          void turn(double angle, double rpm, double acceleration, double secThreshold);


        private:
        void setRightPower(double power);
        void setLeftPower(double power);
        double getRightPower();
        double getLeftPower();
        void setDriveBrakeMode(const std::string brakeMode);
        double getRightEncoderValues();
        double getLeftEncoderValues();
        double getEncoderAverages();
        void resetEncoders();
        void reset();
        double msCounter;
        double power;
    };
}