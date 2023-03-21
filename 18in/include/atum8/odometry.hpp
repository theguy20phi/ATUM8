#pragma once

#include "misc/constants.hpp"
#include "misc/task.hpp"
#include "odometer.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"

using namespace okapi::literals;

namespace atum8
{

    struct Position
    {
        okapi::QLength x;
        okapi::QLength y;
        okapi::QAngle h;
    };

    class Odometry : public Task
    {
    public:
        Odometry(UPOdometer iLeft,
                 UPOdometer iRight,
                 UPOdometer iSide,
                 const Position &startingPosition = {0_tile, 0_tile, 0_deg});

        void taskFn();

        void setPosition(const Position &iPosition);

        Position getPosition() const;

        void tare();

    private:
        UPOdometer left;
        UPOdometer right;
        UPOdometer side;
        Position position;
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
        char leftA,
            leftB;
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