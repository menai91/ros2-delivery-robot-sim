#include "amr_navigation/station_loader.hpp"

#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace amr_navigation
{
    StationMap load_stations(const std::string &path)
    {
        YAML::Node root;
        try
        {
            root = YAML::LoadFile(path);
        }
        catch (const YAML::Exception &e)
        {
            // covers both "file not found" and malformed YAML
            throw std::runtime_error("Failed to load station file '" + path + "': " + e.what());
        }

        const YAML::Node stations = root["stations"];
        if (!stations || !stations.IsMap())
        {
            throw std::runtime_error("Station file '" + path + "' has no 'stations:' map at the top level");
        }

        StationMap result;
        for (const auto &entry : stations)
        {
            // each entry is a key/value pair: the station name and its {x, y, yaw} map
            const std::string name = entry.first.as<std::string>();
            const YAML::Node &fields = entry.second;

            try
            {
                Pose2D pose;
                pose.x = fields["x"].as<double>();
                pose.y = fields["y"].as<double>();
                pose.yaw = fields["yaw"].as<double>();
                result[name] = pose;
            }
            catch (const YAML::Exception &e)
            {
                // a missing key or a value like "3.14oops" both end up here
                throw std::runtime_error("Station '" + name + "' in '" + path + "' needs numeric x, y and yaw: " + e.what());
            }
        }

        if (result.empty())
        {
            throw std::runtime_error("Station file '" + path + "' defines no stations");
        }

        return result;
    }

} // namespace amr_navigation
