#pragma once

namespace amr_navigation
{
    // A pose in the map frame, in the same units Navigator::start() takes (metres, radians)
    struct Pose2D
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

} // namespace amr_navigation
