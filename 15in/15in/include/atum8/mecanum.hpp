#pragma once

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "constants.hpp"
#include "controllers/controller.hpp"

namespace atum8
{

    class Mecanum
    {
    public:
        Mecanum(UPMotor iRFMotor,
                UPMotor iLFMotor,
                UPMotor iLBMotor,
                UPMotor iRBMotor,
                UPController iLateralController,
                UPController iTurnController,
                UPImu iImu = nullptr);

        void forward(okapi::QLength distance, int maxForward = 127);

        void turn(okapi::QAngle angle, int maxTurn = 127);

        void move(int forward, int strafe, int turn);

    private:
        UPMotor rFMotor;
        UPMotor lFMotor;
        UPMotor lBMotor;
        UPMotor rBMotor;
        UPController lateralController;
        UPController turnController;
        UPImu imu;
    };

    using UPMecanum = std::unique_ptr<Mecanum>;
    using SPMecanum = std::shared_ptr<Mecanum>;
}