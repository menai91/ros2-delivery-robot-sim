#include <fstream>
#include <sstream>
#include <stdexcept>

#include "amr_navigation/waypoint_loader.hpp"

std::vector<Waypoint> load_waypoints(const std::string &path)
{
    std::ifstream file(path);
    if (!file)
    {
        throw std::runtime_error("Cannot open waypoint file: " + path);
    }

    std::vector<Waypoint> waypoints;
    std::string line;
    int line_number = 0;

    while (std::getline(file, line))
    {
        ++line_number;

        const auto hash = line.find('#');
        if (hash != std::string::npos)
        {
            line.erase(hash);
        }
        if (line.find_first_not_of(" \t\r") == std::string::npos)
        {
            continue; // blank or comment-only line
        }

        std::istringstream stream(line);
        Waypoint wp;
        std::string extra;
        if (!(stream >> wp.x >> wp.y >> wp.yaw) || (stream >> extra))
        {
            throw std::runtime_error(
                "Bad waypoint on line " + std::to_string(line_number) + ": expected 'x y yaw'");
        }

        waypoints.push_back(wp);
    }

    return waypoints;
}