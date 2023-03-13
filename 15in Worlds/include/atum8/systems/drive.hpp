#pragma once
#include "atum8/controllers/pid.hpp"
#include "atum8/imus.hpp"

namespace atum8 {
    class Drive : public Pid, Imus{
        public:
          void controller();
          void move(double inches, double rpm, double acceleration, bool dift, double secThreshold);
          void turn(double angle, double rpm, double acceleration, double secThreshold);


        private:
        float speedResistor;
        void setRightPower(double power);
        void setLeftPower(double power);
        double getRightEncoderValues();
        double getLeftEncoderValues();
        double getEncoderAverages();
        double power;




    };
}