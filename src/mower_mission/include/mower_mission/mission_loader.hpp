#ifndef MOWER_MISSION_MISSION_LOADER_HPP
#define MOWER_MISSION_MISSION_LOADER_HPP

#include <string>
#include <vector>

namespace mower_mission
{

/** Load mission waypoints from .waypoints (x y meters) or .wgs84 (lat lon -> ENU). */
std::vector<double> load_mission(
  const std::string& path,
  double farm_origin_lat,
  double farm_origin_lon,
  double farm_origin_alt);

bool is_wgs84_mission(const std::string& path);

}  // namespace mower_mission

#endif
