#include "mecanum.hpp"

namespace atum8
{
    Mecanum::Mecanum(UPMotor iRFMotor,
                     UPMotor iLFMotor,
                     UPMotor iLBMotor,
                     UPMotor iRBMotor,
                     UPController iLateralController,
                     UPController iTurnController,
                     UPImu iImu) : rFMotor{std::move(iRFMotor)},
                                   lFMotor{std::move(iLFMotor)},
                                   lBMotor{std::move(iLBMotor)},
                                   rBMotor{std::move(iRBMotor)},
                                   lateralController{std::move(iLateralController)},
                                   turnController{std::move(iTurnController)},
                                   imu{std::move(iImu)}
    {
    }

    void Mecanum::move(int forward, int strafe, int turn)
    {
        rFMotor->move(forward - strafe - turn);
        lFMotor->move(forward + strafe + turn);
        lBMotor->move(forward - strafe + turn);
        rBMotor->move(forward + strafe - turn);
    }
}