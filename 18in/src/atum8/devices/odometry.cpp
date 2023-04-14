#include "odometry.hpp"

namespace atum8
{
    Odometry::Odometry(UPOdometer iLeft,
                       UPOdometer iRight,
                       UPOdometer iSide,
                       const Position &startingPosition) : left{std::move(iLeft)},
                                                           right{std::move(iRight)},
                                                           side{std::move(iSide)},
                                                           PoseEstimator(startingPosition)
    {
        addTaskFns({track()});
    }

    TaskFn Odometry::track()
    {
        return [=]()
        {
            const okapi::QLength baseWidth = left->getDistanceToCenter() + right->getDistanceToCenter();
            while (true)
            {
                const okapi::QLength dL = left->getDistance();
                const okapi::QLength dR = right->getDistance();
                const okapi::QAngle dh{(dL - dR) / baseWidth * okapi::radian};
                okapi::QLength dx{side->getDistance()};
                okapi::QLength dy{dR};
                if (dh != 0_deg)
                {
                    dx = okapi::inch * 2 * okapi::sin(dh / 2) * (dx.convert(okapi::inch) / dh.convert(okapi::radian) + side->getDistanceToCenter().convert(okapi::inch));
                    dy = okapi::inch * 2 * okapi::sin(dh / 2) * (dy.convert(okapi::inch) / dh.convert(okapi::radian) + right->getDistanceToCenter().convert(okapi::inch));
                }
                const okapi::QAngle hAvg = position.h + dh / 2;
                position.x += dx * okapi::cos(hAvg) + dy * okapi::sin(hAvg);
                position.y += dy * okapi::cos(hAvg) - dx * okapi::sin(hAvg);
                position.h += dh;
                pros::delay(stdDelay);
            }
        };
    }

    SPOdometry SPOdometryBuilder::build() const
    {
        return std::make_unique<Odometry>(std::make_unique<Odometer>(leftA, leftB, (leftReversed ? -1 : 1) * encoderMultiplier, Odometer::Dimensions{circum, width / 2}),
                                          std::make_unique<Odometer>(rightA, rightB, (rightReversed ? -1 : 1) * encoderMultiplier, Odometer::Dimensions{circum, width / 2}),
                                          std::make_unique<Odometer>(sideA, sideB, (sideReversed ? -1 : 1) * encoderMultiplier, Odometer::Dimensions{circum, sideDistanceToCenter}),
                                          startingPosition);
    }

    SPOdometryBuilder SPOdometryBuilder::withLeft(char iLeftA, char iLeftB, bool reversed)
    {
        leftA = iLeftA;
        leftB = iLeftB;
        leftReversed = reversed;
        return *this;
    }

    SPOdometryBuilder SPOdometryBuilder::withRight(char iRightA, char iRightB, bool reversed)
    {
        rightA = iRightA;
        rightB = iRightB;
        rightReversed = reversed;
        return *this;
    }

    SPOdometryBuilder SPOdometryBuilder::withSide(char iSideA,
                                                  char iSideB,
                                                  const okapi::QLength &iSideDistanceToCenter,
                                                  bool reversed)
    {
        sideA = iSideA;
        sideB = iSideB;
        sideDistanceToCenter = iSideDistanceToCenter;
        sideReversed = reversed;
        return *this;
    }

    SPOdometryBuilder SPOdometryBuilder::withEncoderMultiplier(double iEncoderMultiplier)
    {
        encoderMultiplier = iEncoderMultiplier;
        return *this;
    }

    SPOdometryBuilder SPOdometryBuilder::withWheelCircum(const okapi::QLength &iCircum)
    {
        circum = iCircum;
        return *this;
    }

    SPOdometryBuilder SPOdometryBuilder::withWidth(const okapi::QLength &iWidth)
    {
        width = iWidth;
        return *this;
    }

    SPOdometryBuilder SPOdometryBuilder::withStartingPosition(const Position &iStartingPosition)
    {
        startingPosition = iStartingPosition;
        return *this;
    }
}