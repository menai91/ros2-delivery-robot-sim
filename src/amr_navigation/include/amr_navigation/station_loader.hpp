#pragma once

#include <map>
#include <string>

namespace amr_navigation
{
    // A pose in the map frame, in the same units Navigator::start() takes (metres, radians)
    struct Pose2D
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    using StationMap = std::map<std::string, Pose2D>;

    /* Loads named station poses from a YAML file of the form:

        stations:
          room1_station: {x: -3.5, y: -3.49, yaw: 3.14}
          room2_station: {x: 5.4,  y: -1.48, yaw: 3.14}

    Throws std::runtime_error (message includes the file and station name) if the file
    is missing, the YAML is malformed, a field is missing, or a value is not a number. */
    StationMap load_stations(const std::string &path);

} // namespace amr_navigation
