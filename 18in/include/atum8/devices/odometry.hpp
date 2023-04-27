#pragma once

#include "okapi/api/odometry/OdomMath.hpp"
#include "atum8/misc/constants.hpp"
#include "atum8/misc/task.hpp"
#include "atum8/devices/imus.hpp"
#include "odometer.hpp"
#include "poseEstimator.hpp"

using namespace okapi::literals;

namespace atum8
{
    class Odometry : public PoseEstimator
    {
    public:
        Odometry(UPOdometer iLeft,
                 UPOdometer iRight,
                 UPOdometer iSide,
                 UPImus iImus = nullptr,
                 double iImuTrust = 0.99);

    private:
        TaskFn track();
        UPOdometer left;
        UPOdometer right;
        UPOdometer side;
        UPImus imus;
        double imuTrust;
    };

    using UPOdometry = std::unique_ptr<Odometry>;
    using SPOdometry = std::shared_ptr<Odometry>;

    class SPOdometryBuilder
    {
    public:
        SPOdometry build() const;

        SPOdometryBuilder withLeft(char iLeftA, char iLeftB, bool reversed = false);

        SPOdometryBuilder withRight(char iRightA, char iRightB, bool reversed = false);

        SPOdometryBuilder withSide(char iSideA,
                                   char iSideB,
                                   const okapi::QLength &iSideDistanceToCenter,
                                   bool reversed = false);

        SPOdometryBuilder withEncoderMultiplier(double iEncoderMultiplier);

        SPOdometryBuilder withWheelCircum(const okapi::QLength &iCircum);

        SPOdometryBuilder withWidth(const okapi::QLength &iWidth);

        SPOdometryBuilder withImus(const std::vector<int> &iImuPorts, double iImuTrust = 0.99);

    private:
        char leftA, leftB;
        bool leftReversed{false};
        char rightA, rightB;
        bool rightReversed{false};
        char sideA, sideB;
        bool sideReversed{false};
        okapi::QLength sideDistanceToCenter;
        double encoderMultiplier;
        okapi::QLength circum;
        okapi::QLength width;
        std::vector<int> imuPorts;
        double imuTrust;
    };
}