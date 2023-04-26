#pragma once

#include "atum8/misc/constants.hpp"
#include "atum8/controllers/controller.hpp"
#include "atum8/misc/settledChecker.hpp"
#include "atum8/misc/task.hpp"
#include "atum8/filters/filter.hpp"
#include "atum8/misc/slewRate.hpp"
#include "atum8/devices/potentiometer.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QAngularAcceleration.hpp"
#include "pros/misc.hpp"

namespace atum8
{
    class Shooter : public Task
    {
    public:
        enum class ShooterState
        {
            Idle,
            Single,
            Multi
        };

        enum class ControlState
        {
            Multi,
            Single,
            Loader
        };

        Shooter(UPMotor iIndexer,
                UPMotorGroup iFlywheel,
                UPMotor iIntake,
                SPADIDigitalOut iLoader,
                SPADIDigitalOut iAngleAdjuster,
                SPADIDigitalOut iIntakeAdjuster,
                SPController iVelocityController,
                SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                SPPotentiometer iPotentiometer,
                SPFilter iFilter = std::make_shared<Filter>(),
                const okapi::QAngularSpeed &iMultiShotAdjustment = 50_rpm,
                SPSlewRate iSlewRate = nullptr,
                double iGearing = 7.0);

        TaskFn velocityControlTask();

        TaskFn shootingControlTask();

        void control(pros::Controller master);

        void singleShot(int iNumOfTimes = 1, const okapi::QTime &iShotTimeout = 0_s);

        void multiShot(int iNumOfTimes = 1, const okapi::QTime &iShotTimeout = 0_s);

        void singleShotPrepare(const okapi::QAngularSpeed speed);

        void multiShotPrepare(const okapi::QAngularSpeed speed);

        void raiseAngleAdjuster();

        void lowerAngleAdjuster();

        void runIntake(int input = 127);

        void stopIntake();

        void raiseIntake();

        void lowerIntake();

        void toggleIntake();

        void raiseLoader();

        void lowerLoader();

        void toggleLoader();

        int getDisks();

        bool isShooting() const;

        void setReferenceSpeed(const okapi::QAngularSpeed speed);

        okapi::QAngularSpeed getReferenceSpeed() const;

        okapi::QAngularSpeed getSpeed() const;

        bool readyToFire(okapi::QAngularSpeed speed);

        bool readyToFire() const;

        void reset();

        SPController getVelocityController() const;

        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> getVelocitySettledChecker() const;

    private:
        void intakeControls(pros::Controller master);
        void flywheelControls(pros::Controller master);
        void setCommand(const ShooterState &iShooterState,
                        int iNumOfShots,
                        const okapi::QTime &iShotTimeout);
        void perfSingleShot();
        void perfMultiShot();
        void index();
        int numOfShots{0};
        int numOfDisks{0};
        ShooterState shooterState{ShooterState::Idle};
        ControlState controlState{ControlState::Multi};
        okapi::QTime shotTimeout;
        okapi::QTime prevTime;
        UPMotor indexer;
        UPMotorGroup flywheel;
        UPMotor intake;
        SPADIDigitalOut loader;
        SPADIDigitalOut angleAdjuster;
        SPADIDigitalOut intakeAdjuster;
        SPPotentiometer potentiometer;
        double gearing{7.0};
        SPController velocityController;
        SPSlewRate slewRate;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
        SPFilter filter;
        okapi::QAngularSpeed referenceSpeed{0_degps};
        okapi::QAngularSpeed multiShotAdjustment;
        bool intakeRaised{false};
        bool loaderRaised{false};
    };

    using UPShooter = std::unique_ptr<Shooter>;
    using SPShooter = std::shared_ptr<Shooter>;

    class SPShooterBuilder
    {
    public:
        SPShooter build() const;

        SPShooterBuilder withIndexerMotor(int8_t port,
                                          const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

        SPShooterBuilder withFlywheelMotors(const std::vector<int8_t> &ports);

        SPShooterBuilder withIntakeMotor(int8_t port,
                                         const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

        SPShooterBuilder withLoader(uint8_t smartPort, uint8_t adiPort);

        SPShooterBuilder withLoader(uint8_t port);

        SPShooterBuilder withAngleAdjuster(uint8_t smartPort, uint8_t adiPort);

        SPShooterBuilder withAngleAdjuster(uint8_t port);

        SPShooterBuilder withIntakeAdjuster(uint8_t smartPort, uint8_t adiPort);

        SPShooterBuilder withIntakeAdjuster(uint8_t port);

        SPShooterBuilder withPotentiometer(uint8_t smartPort,
                                           uint8_t adiPort,
                                           const std::vector<double> &iPositionMap,
                                           int medFilterSize = 10);

        SPShooterBuilder withPotentiometer(uint8_t port,
                                           const std::vector<double> &iPositionMap,
                                           int medFilterSize = 10);

        SPShooterBuilder withMultiShotAdjustment(const okapi::QAngularSpeed &iMultiShotAdjustment);

        SPShooterBuilder withGearing(double iGearing);

        SPShooterBuilder withController(SPController iVelocityController);

        SPShooterBuilder withSettledChecker(const okapi::QAngularSpeed &maxSpeedError,
                                            const okapi::QAngularAcceleration &maxAccelError = 0_rpmps,
                                            const okapi::QTime &minTime = 0_s);

        SPShooterBuilder withFilter(SPFilter iFilter);

        SPShooterBuilder withSlew(double maxNegChange, double maxPosChange);

    private:
        int8_t indexerPort;
        pros::motor_gearset_e_t indexerGearset;
        std::vector<int8_t> flywheelPorts;
        int8_t intakePort;
        pros::motor_gearset_e_t intakeGearset;
        SPADIDigitalOut loader;
        SPADIDigitalOut angleAdjuster;
        SPADIDigitalOut intakeAdjuster;
        SPPotentiometer potentiometer;
        double gearing{15.0};
        SPController velocityController;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
        SPFilter filter{std::make_shared<Filter>()};
        SPSlewRate slewRate;
        okapi::QAngularSpeed multiShotAdjustment{50_rpm};
    };
}