/**
 * @file poseEstimator.hpp
 * @author Braden Pierce (bradenwepierce@gmail.com)
 * @brief Provides a common interface for all classes that are used
 * to estimate the pose of the robot (its position + heading).
 * @version 0.1
 * @date 2023-03-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "atum8/misc/task.hpp"
#include "position.hpp"
#include <memory>

namespace atum8
{
    /**
     * @brief Provides a common interface for all classes that are used
     * to estimate the pose of the robot (its position + heading).
     *
     */
    class PoseEstimator : public Task
    {
    public:
        PoseEstimator(const Position &startingPosition = {0_in, 0_in, 0_deg});

        virtual void setPosition(const Position &iPosition);

        virtual Position getPosition() const;

        virtual void tare();

    protected:
        Position position;
    };

    using UPPoseEstimator = std::unique_ptr<PoseEstimator>;
    using SPPoseEstimator = std::shared_ptr<PoseEstimator>;
}