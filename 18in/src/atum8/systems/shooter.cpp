#include "shooter.hpp"

namespace atum8
{
    Shooter::Shooter(UPMotor iIndexer,
                     UPMotorGroup iFlywheel,
                     UPMotor iIntake,
                     SPADIDigitalOut iLoader,
                     SPADIDigitalOut iAngleAdjuster,
                     SPADIDigitalOut iIntakeAdjuster,
                     SPController iVelocityController,
                     SPSettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration> iVelocitySettledChecker,
                     SPPotentiometer iPotentiometer,
                     double iIndexDistance,
                     SPFilter iFilter,
                     const okapi::QAngularSpeed &iMultiShotAdjustment,
                     SPSlewRate iSlewRate,
                     double iGearing) : indexer{std::move(iIndexer)},
                                        flywheel{std::move(iFlywheel)},
                                        intake{std::move(iIntake)},
                                        loader{iLoader},
                                        angleAdjuster{iAngleAdjuster},
                                        intakeAdjuster{iIntakeAdjuster},
                                        velocityController{iVelocityController},
                                        velocitySettledChecker{iVelocitySettledChecker},
                                        potentiometer{iPotentiometer},
                                        indexDistance{iIndexDistance},
                                        filter{iFilter},
                                        multiShotAdjustment{iMultiShotAdjustment},
                                        slewRate{iSlewRate},
                                        gearing{iGearing}
    {
        flywheel->set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        addTaskFns({velocityControl(), shootingControl()});
    }

    TaskFn Shooter::velocityControl()
    {
        return [=]()
        {
            while (true)
            {
                const okapi::QAngularSpeed speed{getSpeed()};
                velocitySettledChecker->isSettled(speed, referenceSpeed);
                double output{velocityController->getOutput(speed.convert(okapi::rpm), referenceSpeed.convert(okapi::rpm))};
                if (referenceSpeed == 0_rpm || output <= 0)
                    output = 0;
                if (slewRate)
                    output = slewRate->slew(output);
                flywheel->move_voltage(output);
                pros::delay(stdDelay);
            }
        };
    }

    TaskFn Shooter::shootingControl()
    {
        return [=]()
        {
            while (true)
            {
                switch (shooterState)
                {
                case ShooterState::Single:
                    perfSingleShot();
                    break;
                case ShooterState::Multi:
                    perfMultiShot();
                    break;
                default:
                    break;
                };
                shooterState = ShooterState::Idle;
                pros::delay(stdDelay);
            }
        };
    }

    void Shooter::singleShot(int iNumOfShots, const okapi::QTime &iShotTimeout)
    {
        setCommand(ShooterState::Single, iNumOfShots, iShotTimeout);
    }

    void Shooter::multiShot(int iNumOfShots, const okapi::QTime &iShotTimeout)
    {
        setCommand(ShooterState::Multi, iNumOfShots, iShotTimeout);
    }

    void Shooter::raiseAngleAdjuster()
    {
        angleAdjuster->set_value(1);
    }

    void Shooter::lowerAngleAdjuster()
    {
        angleAdjuster->set_value(0);
    }

    void Shooter::runIntake(int input)
    {
        indexer->move(-input);
        intake->move(input);
    }

    void Shooter::stopIntake()
    {
        indexer->move(0);
        intake->move(0);
    }

    void Shooter::raiseIntake()
    {
        intakeAdjuster->set_value(1);
    }

    void Shooter::lowerIntake()
    {
        intakeAdjuster->set_value(0);
    }

    void Shooter::raiseLoader()
    {
        loader->set_value(1);
    }

    void Shooter::lowerLoader()
    {
        loader->set_value(0);
    }

    int Shooter::getDisks()
    {
        return potentiometer->getMappedPosition();
    }

    bool Shooter::isShooting() const
    {
        return numOfShots;
    }

    void Shooter::setReferenceSpeed(const okapi::QAngularSpeed speed)
    {
        referenceSpeed = speed;
    }

    okapi::QAngularSpeed Shooter::getReferenceSpeed() const
    {
        return referenceSpeed;
    }

    okapi::QAngularSpeed Shooter::getSpeed() const
    {
        double avgRawVelocity{0.0};
        for (double velocity : flywheel->get_actual_velocities())
            avgRawVelocity += velocity;
        avgRawVelocity /= flywheel->size();
        // speedMultiplier adjusts for gearing
        avgRawVelocity *= gearing;
        return filter->get(avgRawVelocity) * okapi::rpm;
        return avgRawVelocity * okapi::rpm;
    }

    bool Shooter::readyToFire(okapi::QAngularSpeed speedError)
    {
        return velocitySettledChecker->isSettled(speedError);
    }

    bool Shooter::readyToFire() const
    {
        return velocitySettledChecker->isSettled();
    }

    void Shooter::reset()
    {
        velocityController->reset();
        slewRate->reset();
        flywheel->move_voltage(0);
        indexer->move_voltage(0);
        intake->move_voltage(0);
        lowerAngleAdjuster();
        lowerIntake();
        lowerLoader();
    }

    void Shooter::setCommand(const ShooterState &iShooterState,
                             int iNumOfShots,
                             const okapi::QTime &iShotTimeout)
    {
        shooterState = iShooterState;
        numOfShots = iNumOfShots;
        shotTimeout = iShotTimeout;
        prevTime = pros::millis() * okapi::millisecond;
    }

    void Shooter::perfSingleShot()
    {
        for (int i{0}; i < numOfShots; i++)
        {
            waitFor([=]()
                    { return readyToFire(); },
                    shotTimeout);
            index();
        }
    }

    void Shooter::perfMultiShot()
    {
        waitFor([=]()
                { return readyToFire(); });
        const okapi::QAngularSpeed prevReferenceSpeed{referenceSpeed};
        for (int i{0}; i < numOfShots; i++)
        {
            index();
            referenceSpeed += multiShotAdjustment;
        }
        referenceSpeed = prevReferenceSpeed;
    }

    void Shooter::index()
    {
        if (!potentiometer)
        {
            intake->move(-127);
            indexer->move_relative(indexDistance, 200);
            waitFor([=]()
                    { return indexer->is_stopped(); },
                    2_s);
        }
        else
        {
            intake->move(-127);
            indexer->move(127);
            const int goalNumOfDisks{getDisks() - 1};
            // Arbitrary timeout given, since it should never timeout
            waitFor([=]()
                    { return getDisks() == goalNumOfDisks || getDisks() == 0; },
                    2_s);
        }
        stopIntake();
    }

    /* -------------------------------------------------------------------------- */
    /*                              Shooter Builder                              */
    /* -------------------------------------------------------------------------- */

    SPShooter SPShooterBuilder::build() const
    {
        return std::make_shared<Shooter>(std::make_unique<pros::Motor>(indexerPort, indexerGearset),
                                         std::make_unique<pros::MotorGroup>(flywheelPorts),
                                         std::make_unique<pros::Motor>(intakePort, intakeGearset),
                                         loader,
                                         angleAdjuster,
                                         intakeAdjuster,
                                         velocityController,
                                         velocitySettledChecker,
                                         potentiometer,
                                         indexDistance,
                                         filter,
                                         multiShotAdjustment,
                                         slewRate,
                                         gearing);
    }

    SPShooterBuilder SPShooterBuilder::withIndexerMotor(int8_t port, const pros::motor_gearset_e_t &gearset)
    {
        indexerPort = port;
        indexerGearset = gearset;
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withFlywheelMotors(const std::vector<int8_t> &ports)
    {
        flywheelPorts = ports;
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withIntakeMotor(int8_t port, const pros::motor_gearset_e_t &gearset)
    {
        intakePort = port;
        intakeGearset = gearset;
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withLoader(uint8_t smartPort, uint8_t adiPort)
    {
        loader = std::make_shared<pros::ADIDigitalOut>(pros::ext_adi_port_pair_t{smartPort, adiPort});
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withLoader(uint8_t port)
    {
        loader = std::make_shared<pros::ADIDigitalOut>(port);
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withAngleAdjuster(uint8_t smartPort, uint8_t adiPort)
    {
        angleAdjuster = std::make_shared<pros::ADIDigitalOut>(pros::ext_adi_port_pair_t{smartPort, adiPort});
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withAngleAdjuster(uint8_t port)
    {
        angleAdjuster = std::make_shared<pros::ADIDigitalOut>(port);
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withIntakeAdjuster(uint8_t smartPort, uint8_t adiPort)
    {
        intakeAdjuster = std::make_shared<pros::ADIDigitalOut>(pros::ext_adi_port_pair_t{smartPort, adiPort});
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withIntakeAdjuster(uint8_t port)
    {
        intakeAdjuster = std::make_shared<pros::ADIDigitalOut>(port);
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withPotentiometer(uint8_t smartPort,
                                                         uint8_t adiPort,
                                                         const std::vector<double> &iPositionMap,
                                                         int medFilterSize)
    {
        potentiometer = std::make_shared<Potentiometer>(smartPort, adiPort, iPositionMap, medFilterSize);
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withPotentiometer(uint8_t port,
                                                         const std::vector<double> &iPositionMap,
                                                         int medFilterSize)
    {
        potentiometer = std::make_shared<Potentiometer>(port, iPositionMap, medFilterSize);
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withMultiShotAdjustment(const okapi::QAngularSpeed &iMultiShotAdjustment)
    {
        multiShotAdjustment = iMultiShotAdjustment;
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withIndexDistance(double iIndexDistance)
    {
        indexDistance = iIndexDistance;
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withGearing(double iGearing)
    {
        gearing = iGearing;
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withController(SPController iVelocityController)
    {
        velocityController = iVelocityController;
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withSettledChecker(const okapi::QAngularSpeed &maxSpeedError,
                                                          const okapi::QAngularAcceleration &maxAccelError,
                                                          const okapi::QTime &minTime)
    {
        velocitySettledChecker = std::make_shared<SettledChecker<okapi::QAngularSpeed, okapi::QAngularAcceleration>>(maxSpeedError,
                                                                                                                     maxAccelError,
                                                                                                                     minTime);
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withFilter(SPFilter iFilter)
    {
        filter = iFilter;
        return *this;
    }

    SPShooterBuilder SPShooterBuilder::withSlew(double maxNegChange, double maxPosChange)
    {
        slewRate = std::make_shared<SlewRate>(maxNegChange, maxPosChange);
        return *this;
    }
}