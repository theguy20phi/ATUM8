#pragma once

#include "atum8/misc/task.hpp"
#include "position.hpp"
#include <memory>

namespace atum8
{
    class PoseEstimator : public Task
    {
    public:
        PoseEstimator(const Position &startingPosition = {0_in, 0_in, 0_deg});

        virtual void setPosition(const Position &iPosition);

        virtual Position getPosition();

        virtual void tare();

    protected:
        Position position;
        pros::Mutex positionMutex;
    };

    using UPPoseEstimator = std::unique_ptr<PoseEstimator>;
    using SPPoseEstimator = std::shared_ptr<PoseEstimator>;
}