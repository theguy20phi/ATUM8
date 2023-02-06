#pragma once

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "constants.hpp"
#include "controllers/controller.hpp"
#include "settledChecker.hpp"
#include "slewRate.hpp"

using namespace okapi::literals;

namespace atum8
{

    class Mecanum
    {
    public:
        struct Dimensions
        {
            okapi::QLength baseWidth;
            okapi::QLength wheelCircum;
        };

        Mecanum(UPMotor iRFMotor,
                UPMotor iLFMotor,
                UPMotor iLBMotor,
                UPMotor iRBMotor,
                const Dimensions &iDimensions,
                UPController iforwardController,
                UPController iTurnController,
                UPSettledChecker<okapi::QLength, okapi::QSpeed> iforwardSettledChecker,
                UPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> iTurnSettledChecker,
                UPSlewRate iforwardSlewRate,
                UPSlewRate iTurnSlewRate,
                UPImu iImu = nullptr,
                double iImuTrust = 0.5);

        void forward(const okapi::QLength &distance, const okapi::QTime &maxTime = 0_s, int maxForward = 127);

        void turn(const okapi::QAngle &angle, const okapi::QTime &maxTime = 0_s, int maxTurn = 127);

        void move(int forward = 0, int strafe = 0, int turn = 0);

        okapi::QLength getDistance() const;

        okapi::QAngle getAngle() const;

        bool isSettled(const okapi::QLength &distanceError, const okapi::QAngle &angleError);

        void reset();

        void tare();

    private:
        bool isTimeExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime);
        int driveForwardController(const okapi::QLength &distanceError, int maxForward = 127);
        int driveTurnController(const okapi::QAngle &angleError, int maxTurn = 127);
        UPMotor rFMotor;
        UPMotor lFMotor;
        UPMotor lBMotor;
        UPMotor rBMotor;
        Dimensions dimensions;
        UPController forwardController;
        UPController turnController;
        UPSettledChecker<okapi::QLength, okapi::QSpeed> forwardSettledChecker;
        UPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> turnSettledChecker;
        UPSlewRate forwardSlewRate;
        UPSlewRate turnSlewRate;
        UPImu imu;
        double imuTrust{0.5};
    };

    using UPMecanum = std::unique_ptr<Mecanum>;
    using SPMecanum = std::shared_ptr<Mecanum>;
}