#pragma once

#include "atum8/misc/constants.hpp"
#include "atum8/misc/task.hpp"
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
                 const Position &startingPosition = {0_in, 0_in, 0_deg});

        void taskFn();

    private:
        UPOdometer left;
        UPOdometer right;
        UPOdometer side;
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

        SPOdometryBuilder withStartingPosition(const Position &iStartingPosition);

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
        Position startingPosition{0_in, 0_in, 0_deg};
    };
}