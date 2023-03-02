#pragma once

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "atum8/controllers/controller.hpp"
#include "atum8/settledChecker.hpp"
#include "atum8/slewRate.hpp"
#include "atum8/imus.hpp"

#include <iostream>

using namespace okapi::literals;

namespace atum8
{

    class Mecanum
    {
    public:
        struct DriverSettings
        {
            int deadZone{0};
            SPSlewRate forwardSlewRate{nullptr};
            SPSlewRate strafeSlewRate{nullptr};
            SPSlewRate turnSlewRate{nullptr};
            std::function<int(int)> stickFunction = {[](int input)
                                                     { return input; }};
            pros::motor_brake_mode_e brakeMode{pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST};
            double maxPower{1.0};
        };

        using UPDriverSettings = std::unique_ptr<DriverSettings>;
        using SPDriverSettings = std::shared_ptr<DriverSettings>;

        struct Dimensions
        {
            okapi::QLength baseWidth;
            okapi::QLength wheelCircum;
        };

        using UPDimensions = std::unique_ptr<Dimensions>;
        using SPDimensions = std::shared_ptr<Dimensions>;

        Mecanum(UPMotor iRFMotor,
                UPMotor iLFMotor,
                UPMotor iLBMotor,
                UPMotor iRBMotor,
                SPDimensions iDimensions,
                SPDriverSettings iDriverSettings,
                SPController iForwardController,
                SPController iTurnController,
                SPSettledChecker<okapi::QLength, okapi::QSpeed> iforwardSettledChecker,
                SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> iTurnSettledChecker,
                SPSlewRate iForwardSlewRate,
                SPSlewRate iTurnSlewRate,
                UPImus iImus = nullptr,
                double iImuTrust = 0.5);

        void driver(int forward = 0, int strafe = 0, int turn = 0);

        void forward(const okapi::QLength &distance, const okapi::QTime &maxTime = 0_s, int maxForward = 127);

        void turn(const okapi::QAngle &angle, const okapi::QTime &maxTime = 0_s, int maxTurn = 127);

        void move(int forward = 0, int strafe = 0, int turn = 0);

        okapi::QLength getDistance() const;

        okapi::QAngle getAngle() const;

        bool isSettled(const okapi::QLength &distanceError, const okapi::QAngle &angleError);

        void reset();

        void tare();

        void setBrakeMode(const pros::motor_brake_mode_e &brakeMode);

        SPDriverSettings getDriverSettings() const;

    private:
        bool isTimeNotExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime);
        void toReference(const std::function<okapi::QLength()> &distanceError,
                         const std::function<okapi::QAngle()> &angleError,
                         const okapi::QTime &maxTime,
                         int maxForward = 127,
                         int maxTurn = 127);
        int useForwardController(const okapi::QLength &distanceError, int maxForward = 127);
        int useTurnController(const okapi::QAngle &angleError, int maxTurn = 127);
        UPMotor rFMotor;
        UPMotor lFMotor;
        UPMotor lBMotor;
        UPMotor rBMotor;
        SPDimensions dimensions;
        SPDriverSettings driverSettings;
        SPController forwardController;
        SPController turnController;
        SPSettledChecker<okapi::QLength, okapi::QSpeed> forwardSettledChecker;
        SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> turnSettledChecker;
        SPSlewRate forwardSlewRate;
        SPSlewRate turnSlewRate;
        UPImus imus;
        double imuTrust{0.5};
    };

    using UPMecanum = std::unique_ptr<Mecanum>;
    using SPMecanum = std::shared_ptr<Mecanum>;

    class SPMecanumBuilder
    {
    public:
        SPMecanum build() const;

        SPMecanumBuilder withRFMotor(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);
        SPMecanumBuilder withLFMotor(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);
        SPMecanumBuilder withLBMotor(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);
        SPMecanumBuilder withRBMotor(int port,
                                     const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE);
        SPMecanumBuilder withBaseWidth(const okapi::QLength &baseWidth);
        SPMecanumBuilder withWheelCircum(const okapi::QLength &wheelCircum);
        SPMecanumBuilder withStickDeadZone(int deadZone);
        SPMecanumBuilder witStickSlew(double slew);
        SPMecanumBuilder withStickFunction(const std::function<int(int)> &stickFunction);
        SPMecanumBuilder withStickMax(int maxPower);
        SPMecanumBuilder withForwardController(SPController iForwardController);
        SPMecanumBuilder withTurnController(SPController iTurnController);
        SPMecanumBuilder withForwardSettledChecker(const okapi::QLength &distance,
                                                   const okapi::QSpeed &speed = 0_inps,
                                                   const okapi::QTime &time = 0_s);
        SPMecanumBuilder withTurnSettledChecker(const okapi::QAngle &angle,
                                                const okapi::QAngularSpeed &angularSpeed,
                                                const okapi::QTime &time);
        SPMecanumBuilder withForwardSlew(double slewRate);
        SPMecanumBuilder withTurnSlew(double slewRate);
        SPMecanumBuilder withImus(const std::vector<int> &ports, double trust = 0.5);
        SPMecanumBuilder withBrakeMode(const pros::motor_brake_mode_e &brakeMode);

    private:
        int rFPort;
        pros::motor_gearset_e_t rFGearset;
        int lFPort;
        pros::motor_gearset_e_t lFGearset;
        int lBPort;
        pros::motor_gearset_e_t lBGearset;
        int rBPort;
        pros::motor_gearset_e_t rBGearset;
        Mecanum::SPDimensions dimensions{std::make_shared<Mecanum::Dimensions>()};
        Mecanum::SPDriverSettings driverSettings{std::make_shared<Mecanum::DriverSettings>()};
        SPController forwardController;
        SPController turnController;
        SPSettledChecker<okapi::QLength, okapi::QSpeed> forwardSettledChecker;
        SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> turnSettledChecker;
        SPSlewRate forwardSlewRate;
        SPSlewRate turnSlewRate;
        std::vector<int> imuPorts;
        double imuTrust{0.5};
    };
}