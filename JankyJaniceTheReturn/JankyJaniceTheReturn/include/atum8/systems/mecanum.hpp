#pragma once

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "atum8/controllers/controller.hpp"
#include "atum8/settledChecker.hpp"
#include "atum8/slewRate.hpp"

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
            int maxPower{127};
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

        Mecanum(UPMotorGroup iRFMotor,
                UPMotorGroup iLFMotor,
                UPMotorGroup iLBMotor,
                UPMotorGroup iRBMotor,
                SPDimensions iDimensions,
                SPDriverSettings iDriverSettings,
                SPController iForwardController,
                SPController iTurnController,
                SPSettledChecker<okapi::QLength, okapi::QSpeed> iforwardSettledChecker,
                SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> iTurnSettledChecker,
                SPSlewRate iForwardSlewRate,
                SPSlewRate iTurnSlewRate,
                const pros::motor_brake_mode_e &brakeMode = pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST,
                UPImu iImu = nullptr,
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

        SPDimensions getDimensions() const;

        SPDriverSettings getDriverSettings() const;

        SPController getForwardController() const;

        SPController getTurnController() const;

        SPSettledChecker<okapi::QLength, okapi::QSpeed> getForwardSettledChecker() const;

        SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> getTurnSettledChecker() const;

        SPSlewRate getForwardSlewRate() const;
        
        SPSlewRate getTurnSlewRate() const;

    private:
        void applyBrakes();
        bool isTimeExpired(const okapi::QTime &startTime, const okapi::QTime &maxTime);
        int useForwardController(const okapi::QLength &distanceError, int maxForward = 127);
        int useTurnController(const okapi::QAngle &angleError, int maxTurn = 127);
        UPMotorGroup rFMotor;
        UPMotorGroup lFMotor;
        UPMotorGroup lBMotor;
        UPMotorGroup rBMotor;
        SPDimensions dimensions;
        SPDriverSettings driverSettings;
        SPController forwardController;
        SPController turnController;
        SPSettledChecker<okapi::QLength, okapi::QSpeed> forwardSettledChecker;
        SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> turnSettledChecker;
        SPSlewRate forwardSlewRate;
        SPSlewRate turnSlewRate;
        UPImu imu;
        double imuTrust{0.5};
    };

    using UPMecanum = std::unique_ptr<Mecanum>;
    using SPMecanum = std::shared_ptr<Mecanum>;

    class SPMecanumBuilder
    {
    public:
        SPMecanum build() const;

        SPMecanumBuilder withRFMotor(const std::vector<std::int8_t> &iPorts);
        SPMecanumBuilder withLFMotor(const std::vector<std::int8_t> &iPorts);
        SPMecanumBuilder withLBMotor(const std::vector<std::int8_t> &iPorts);
        SPMecanumBuilder withRBMotor(const std::vector<std::int8_t> &iPorts);
        SPMecanumBuilder withBaseWidth(const okapi::QLength &baseWidth);
        SPMecanumBuilder withWheelCircum(const okapi::QLength &wheelCircum);
        SPMecanumBuilder withStickDeadZone(int deadZone);
        SPMecanumBuilder witStickSlew(double slew);
        SPMecanumBuilder withStickFunction(const std::function<int(int)> &stickFunction);
        SPMecanumBuilder withStickMax(int maxPower);
        SPMecanumBuilder withForwardController(SPController iForwardController);
        SPMecanumBuilder withTurnController(SPController iTurnController);
        SPMecanumBuilder withForwardSettledChecker(const okapi::QLength &distance,
                                                   const okapi::QSpeed &speed,
                                                   const okapi::QTime &time);
        SPMecanumBuilder withTurnSettledChecker(const okapi::QAngle &angle,
                                                const okapi::QAngularSpeed &angularSpeed,
                                                const okapi::QTime &time);
        SPMecanumBuilder withForwardSlew(double slewRate);
        SPMecanumBuilder withTurnSlew(double slewRate);
        SPMecanumBuilder withImu(int port, double trust = 0.5);
        SPMecanumBuilder withBrakeMode(const pros::motor_brake_mode_e &brakeMode);

    private:
        std::vector<std::int8_t> rFPort;
        std::vector<std::int8_t> lFPort;
        std::vector<std::int8_t> lBPort;
        std::vector<std::int8_t> rBPort;
        Mecanum::SPDimensions dimensions{std::make_shared<Mecanum::Dimensions>()};
        Mecanum::SPDriverSettings driverSettings{std::make_shared<Mecanum::DriverSettings>()};
        SPController forwardController;
        SPController turnController;
        SPSettledChecker<okapi::QLength, okapi::QSpeed> forwardSettledChecker;
        SPSettledChecker<okapi::QAngle, okapi::QAngularSpeed> turnSettledChecker;
        SPSlewRate forwardSlewRate;
        SPSlewRate turnSlewRate;
        int imuPort;
        double imuTrust{0.5};
        pros::motor_brake_mode_e brakeMode{pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST};
    };
}