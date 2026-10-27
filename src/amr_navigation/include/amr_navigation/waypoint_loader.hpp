#pragma once

#include <string>
#include <vector>

struct Waypoint
{
  double x;
  double y;
  double yaw;
};

/* Just a simple function that reads the waypoints saved in a txt file*/
std::vector<Waypoint> load_waypoints(const std::string &path);