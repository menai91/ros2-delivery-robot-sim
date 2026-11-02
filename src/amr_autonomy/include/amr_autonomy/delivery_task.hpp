#pragma once

#include "amr_navigation/pose2d.hpp"

namespace amr_autonomy
{
    struct DeliveryTask
    {
        unsigned int task_id = 0;
        amr_navigation::Pose2D pickup_pose;
        amr_navigation::Pose2D dropoff_pose;
    };
}