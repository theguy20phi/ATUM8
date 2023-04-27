#include "poseEstimator.hpp"

namespace atum8
{
    PoseEstimator::PoseEstimator(const Position &startingPosition) : position{startingPosition},
                                                                     Task(9)
    {
    }

    void PoseEstimator::setPosition(const Position &iPosition)
    {
        positionMutex.take();
        position = iPosition;
        positionMutex.give();
    }

    Position PoseEstimator::getPosition()
    {
        positionMutex.take();
        const Position currentPosition{position};
        positionMutex.give();
        return currentPosition;
    }

    void PoseEstimator::tare()
    {
        setPosition({0_in, 0_in, 0_deg});
    }
}