/**
 * @file Shooter.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief This provides the implementation for the shooter.
 * @version 0.3
 * @date 2023-03-02
 *
 * @copyright Copyright (c) 2023
 *
 */

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

namespace atum8
{
    /**
     * @brief This provides the implementation for the shooter.
     *
     */
    class Shooter : public Task
    {
    public:
        /**
         * @brief The various states of the shooter.
         *
         */
        enum class ShooterState
        {
            Idle,
            Single,
            Multi
        };

        /**
         * @brief Constructs a new Shooter object.
         *
         * @param iIndexerMotor
         * @param iFlywheelMotor
         * @param iIntake
         * @param iLoader
         * @param iAngleAdjuster
         * @param iIntakeAdjuster
         * @param iVelocityController
         * @param iVelocitySettledChecker
         * @param iPotentiometer
         * @param iRollingAverage
         * @param iSlewRate
         * @param iGearing
         */
        Shooter(UPMotor iIndexer,
                UPMotorGroup iFlywheel,
                UPMotor iIntake,
                SPADIDigitalOut iLoader,
                SPADIDigitalOut iAngleAdjuster,
                SPADIDigitalOut iIntakeAdjuster,
                SPController iVelocityController,
                SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                SPPotentiometer iPotentiometer = nullptr,
                double iIndexDistance = 100.0,
                SPFilter iFilter = std::make_shared<Filter>(),
                const okapi::QAngularSpeed &iMultiShotAdjustment = 50_rpm,
                SPSlewRate iSlewRate = nullptr,
                double iGearing = 7.0);

        /**
         * @brief This will be ran in parallel and will be responsible for keeping the
         * flywheel's velocity at the reference velocity.
         *
         * @return TaskFn
         */
        TaskFn velocityControl();

        /**
         * @brief This will be ran in parallel and will be responsible for any actions
         * involving shooting.
         *
         * @return TaskFn
         */
        TaskFn shootingControl();

        /**
         * @brief Instructs the shooter to shoot a specified number of times with a timeout
         * each shot (0_s for no timeout).
         *
         * @param iNumOfTimes
         * @param iShotTimeout
         */
        void singleShot(int iNumOfTimes = 1, const okapi::QTime &iShotTimeout = 0_s);

        /**
         * @brief Instructs the shooter to fire all disks current in the indexer in rapid
         * succession.
         *
         * @param iNumOfTimes
         * @param iShotTimeout
         */
        void multiShot(int iNumOfTimes = 1, const okapi::QTime &iShotTimeout = 0_s);

        /**
         * @brief Raises the angle adjuster.
         *
         */
        void raiseAngleAdjuster();

        /**
         * @brief Lowers the angle adjuster.
         *
         */
        void lowerAngleAdjuster();

        /**
         * @brief Runs the intake at the given voltage command.
         *
         * @param input
         */
        void runIntake(int input = 127);

        /**
         * @brief Stops the intake.
         *
         */
        void stopIntake();

        /**
         * @brief Raises the intake.
         *
         */
        void raiseIntake();

        /**
         * @brief Lowers the intake.
         *
         */
        void lowerIntake();

        /**
         * @brief Raises the loader.
         *
         */
        void raiseLoader();

        /**
         * @brief Lowers the loader.
         *
         */
        void lowerLoader();

        /**
         * @brief Gets the number of disks in the indexer.
         *
         * @return int
         */
        int getDisks();

        /**
         * @brief Returns if the shooter is currently shooting.
         *
         * @return true
         * @return false
         */
        bool isShooting() const;

        /**
         * @brief Sets the reference speed.
         *
         * @param speed
         */
        void setReferenceSpeed(const okapi::QAngularSpeed speed);

        /**
         * @brief Gets the reference speed.
         *
         * @return okapi::QAngularSpeed
         */
        okapi::QAngularSpeed getReferenceSpeed() const;

        /**
         * @brief Gets the current speed.
         *
         * @return okapi::QAngularSpeed
         */
        okapi::QAngularSpeed getSpeed() const;

        /**
         * @brief Determines if the Shooter is ready to fire.
         *
         * @param speed
         * @return true
         * @return false
         */
        bool readyToFire(okapi::QAngularSpeed speed);

        /**
         * @brief Determines if the Shooter is ready to fire.
         *
         * @return true
         * @return false
         */
        bool readyToFire() const;

        /**
         * @brief Resets the slew rate, velocity controller, and turns off
         * the motors.
         *
         */
        void reset();

        /**
         * @brief Gets the velocity controller.
         *
         * @return SPController
         */
        SPController getVelocityController() const;

        /**
         * @brief Gets the velocity settled-checker.
         *
         * @return SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration>
         */
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> getVelocitySettledChecker() const;

    private:
        void setCommand(const ShooterState &iShooterState,
                        int iNumOfShots,
                        const okapi::QTime &iShotTimeout);
        void perfSingleShot();
        void perfMultiShot();
        void index();
        int numOfShots = 0;
        ShooterState shooterState;
        okapi::QTime shotTimeout;
        okapi::QTime prevTime;
        UPMotor indexer;
        UPMotorGroup flywheel;
        UPMotor intake;
        SPADIDigitalOut loader;
        SPADIDigitalOut angleAdjuster;
        SPADIDigitalOut intakeAdjuster;
        SPPotentiometer potentiometer;
        double indexDistance;
        double gearing{7.0};
        SPController velocityController;
        SPSlewRate slewRate;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
        SPFilter filter;
        okapi::QAngularSpeed referenceSpeed{0_degps};
        okapi::QAngularSpeed multiShotAdjustment;
    };

    using UPShooter = std::unique_ptr<Shooter>;
    using SPShooter = std::shared_ptr<Shooter>;

    /**
     * @brief Provides a builder for the Shooter.
     *
     */
    class SPShooterBuilder
    {
    public:
        /**
         * @brief Constructs the Shooter object.
         *
         * @return SPShooter
         */
        SPShooter build() const;

        /**
         * @brief Shooter configured with this indexer motor.
         *
         * @param iIndexerPort
         * @param iIndexerGearset
         * @return SPShooterBuilder
         */
        SPShooterBuilder withIndexerMotor(int8_t port,
                                          const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

        /**
         * @brief Shooter configured with these flywheel motor ports.
         *
         * @param iPorts
         * @return SPShooterBuilder
         */
        SPShooterBuilder withFlywheelMotors(const std::vector<int8_t> &ports);

        /**
         * @brief Shooter configured with this intake motor.
         *
         * @param iIndexerPort
         * @param iIndexerGearset
         * @return SPShooterBuilder
         */
        SPShooterBuilder withIntakeMotor(int8_t port,
                                         const pros::motor_gearset_e_t &gearset = pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

        /**
         * @brief Shooter configured with this loader.
         *
         * @param smartPort
         * @param adiPort
         * @return SPShooterBuilder
         */
        SPShooterBuilder withLoader(uint8_t smartPort, uint8_t adiPort);

        /**
         * @brief Shooter configured with this loader.
         *
         * @param port
         * @return SPShooterBuilder
         */
        SPShooterBuilder withLoader(uint8_t port);

        /**
         * @brief Shooter configured with this angle adjuster.
         *
         * @param smartPort
         * @param adiPort
         * @return SPShooterBuilder
         */
        SPShooterBuilder withAngleAdjuster(uint8_t smartPort, uint8_t adiPort);

        /**
         * @brief Shooter configured with this angle adjuster.
         *
         * @param port
         * @return SPShooterBuilder
         */
        SPShooterBuilder withAngleAdjuster(uint8_t port);

        /**
         * @brief Shooter configured with this intake adjuster.
         *
         * @param smartPort
         * @param adiPort
         * @return SPShooterBuilder
         */
        SPShooterBuilder withIntakeAdjuster(uint8_t smartPort, uint8_t adiPort);

        /**
         * @brief Shooter configured with this intake adjuster.
         *
         * @param port
         * @return SPShooterBuilder
         */
        SPShooterBuilder withIntakeAdjuster(uint8_t port);

        /**
         * @brief Shooter configured with this potentiometer.
         *
         * @param smartPort
         * @param adiPort
         * @param iPositionMap
         * @param medFilterSize
         * @return SPShooterBuilder
         */
        SPShooterBuilder withPotentiometer(uint8_t smartPort,
                                           uint8_t adiPort,
                                           const std::vector<double> &iPositionMap,
                                           int medFilterSize = 10);

        /**
         * @brief Shooter configured with this potentiometer.
         *
         * @param port
         * @param iPositionMap
         * @param medFilterSize
         * @return SPShooterBuilder
         */
        SPShooterBuilder withPotentiometer(uint8_t port,
                                           const std::vector<double> &iPositionMap,
                                           int medFilterSize = 10);

        /**
         * @brief Shooter configured with this multi-shot adjustment.
         *
         * @param iMultiShotAdjustment
         * @return SPShooterBuilder
         */
        SPShooterBuilder withMultiShotAdjustment(const okapi::QAngularSpeed &iMultiShotAdjustment);

        /**
         * @brief Shooter configured with this index distance.
         *
         * @param iIndexDistance
         * @return SPShooterBuilder
         */
        SPShooterBuilder withIndexDistance(double iIndexDistance);

        /**
         * @brief Shooter configured with this gearing,  which is the gear ratio
         * of the Shooter (consider the motor as a green motor and a blue cartridge as a 1:3
         * ratio).
         *
         * @param iGearing
         * @return SPShooterBuilder
         */
        SPShooterBuilder withGearing(double iGearing);

        /**
         * @brief Shooter configured with this velocity controller.
         *
         * @param iVelocityController
         * @return SPShooterBuilder
         */
        SPShooterBuilder withController(SPController iVelocityController);

        /**
         * @brief Shooter configured with these parameters for the settled-checker.
         *
         * @param maxSpeedError
         * @param maxAccelError
         * @param minTime
         * @return SPShooterBuilder
         */
        SPShooterBuilder withSettledChecker(const okapi::QAngularSpeed &maxSpeedError,
                                            const okapi::QAngularAcceleration &maxAccelError = 0_rpmps,
                                            const okapi::QTime &minTime = 0_s);

        /**
         * @brief Shooter configured with this filter.
         *
         * @param iFilter
         * @return SPShooterBuilder
         */
        SPShooterBuilder withFilter(SPFilter iFilter);

        /**
         * @brief Shooter configured with these parameters for a slew rate.
         *
         * @param maxNegChange
         * @param maxPosChange
         * @return SPShooterBuilder
         */
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
        double indexDistance{100.0};
        double gearing{15.0};
        SPController velocityController;
        SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> velocitySettledChecker;
        SPFilter filter;
        SPSlewRate slewRate;
        okapi::QAngularSpeed multiShotAdjustment{50_rpm};
    };
}