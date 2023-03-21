#include "poseEstimator.hpp"

namespace atum8
{
    PoseEstimator::PoseEstimator(const Position &startingPosition) : position{startingPosition}
    {
    }

    void PoseEstimator::setPosition(const Position &iPosition)
    {
        position = iPosition;
    }

    Position PoseEstimator::getPosition() const
    {
        return position;
    }

    void PoseEstimator::tare()
    {
        position = {0_in, 0_in, 0_deg};
    }
}