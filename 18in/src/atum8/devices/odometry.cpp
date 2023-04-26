#include "odometry.hpp"

namespace atum8
{
    Odometry::Odometry(UPOdometer iLeft,
                       UPOdometer iRight,
                       UPOdometer iSide,
                       UPImus iImus,
                       double iImuTrust) : left{std::move(iLeft)},
                                           right{std::move(iRight)},
                                           side{std::move(iSide)},
                                           imus{std::move(iImus)},
                                           imuTrust{iImuTrust}
    {
        if(imus)
            imus->reset();
        addTaskFns({track()});
    }

    TaskFn Odometry::track()
    {
        return [=]()
        {
            const okapi::QLength baseWidth = left->getDistanceToCenter() + right->getDistanceToCenter();
            while (true)
            {
                Position currentPosition = getPosition();
                const okapi::QLength dL = left->getDistance();
                const okapi::QLength dR = right->getDistance();
                okapi::QAngle dh{(dL - dR) / baseWidth * okapi::radian};
                if (imus)
                    dh = imuTrust * imus->get_delta() + (1 - imuTrust) * dh;
                okapi::QLength dx{side->getDistance()};
                okapi::QLength dy{dR};
                if (dh != 0_deg)
                {
                    dx = 2_in * okapi::sin(dh / 2) * (dx.convert(okapi::inch) / dh.convert(okapi::radian) + side->getDistanceToCenter().convert(okapi::inch));
                    dy = 2_in * okapi::sin(dh / 2) * (dy.convert(okapi::inch) / dh.convert(okapi::radian) + right->getDistanceToCenter().convert(okapi::inch));
                }
                const okapi::QAngle hAvg = currentPosition.h + dh / 2;
                currentPosition.x += dx * okapi::cos(hAvg) + dy * okapi::sin(hAvg);
                currentPosition.y += dy * okapi::cos(hAvg) - dx * okapi::sin(hAvg);
                currentPosition.h += dh;
                currentPosition.h = okapi::OdomMath::constrainAngle360(currentPosition.h);
                setPosition(currentPosition);
                pros::delay(stdDelay);
            }
        };
    }

    SPOdometry SPOdometryBuilder::build() const
    {
        return std::make_unique<Odometry>(std::make_unique<Odometer>(leftA, leftB, encoderMultiplier, Odometer::Dimensions{circum, width / 2}, leftReversed),
                                          std::make_unique<Odometer>(rightA, rightB, encoderMultiplier, Odometer::Dimensions{circum, width / 2}, rightReversed),
                                          std::make_unique<Odometer>(sideA, sideB, encoderMultiplier, Odometer::Dimensions{circum, sideDistanceToCenter}, sideReversed),
                                          std::make_unique<Imus>(imuPorts),
                                          imuTrust);
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

    SPOdometryBuilder SPOdometryBuilder::withImus(const std::vector<int> &iImuPorts, double iImuTrust)
    {
        imuPorts = iImuPorts;
        imuTrust = iImuTrust;
        return *this;
    }
}