#pragma once

#include <map>
#include <string>

#include "amr_navigation/pose2d.hpp"

namespace amr_navigation
{
    using StationMap = std::map<std::string, Pose2D>;

    /* Loads the named poses under one top-level section (e.g. "stations" or "chargers")
    of a YAML file of the form:

        stations:
          room1_station: {x: -3.5, y: -3.49, yaw: 3.14}
          room2_station: {x: 5.4,  y: -1.48, yaw: 3.14}
        chargers:
          charger1: {x: 5.5, y: 3.5, yaw: 0.0}

    Throws std::runtime_error (message includes the file and station name) if the file
    is missing, the YAML is malformed, the section is missing or empty, a field is missing,
    or a value is not a number. */
    StationMap load_stations(const std::string &path, const std::string &section = "stations");

} // namespace amr_navigation
