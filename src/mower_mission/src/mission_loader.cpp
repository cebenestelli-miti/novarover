#include "mower_mission/mission_loader.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>

namespace mower_mission
{

namespace
{
constexpr double DEG2RAD = M_PI / 180.0;
// Approximate meters per degree at equator; at latitude lat use cos(lat) for longitude.
constexpr double METERS_PER_DEG_LAT = 111320.0;

bool isWgs84Extension(const std::string& path)
{
  const std::string ext = ".wgs84";
  if (path.size() < ext.size()) return false;
  return path.compare(path.size() - ext.size(), ext.size(), ext) == 0;
}

// Convert WGS84 lat,lon (degrees) to ENU (east, north) meters relative to origin (degrees).
void wgs84ToEnu(double lat_deg, double lon_deg, double lat0_deg, double lon0_deg,
                double& east_m, double& north_m)
{
  double lat0 = lat0_deg * DEG2RAD;
  north_m = (lat_deg - lat0_deg) * METERS_PER_DEG_LAT;
  east_m = (lon_deg - lon0_deg) * METERS_PER_DEG_LAT * std::cos(lat0);
}

// Parse optional "# origin: lat lon" from line. Returns true if line was an origin line.
bool tryParseOriginLine(const std::string& line, double& out_lat, double& out_lon)
{
  std::string s = line;
  const size_t hash = s.find('#');
  if (hash != std::string::npos) s = s.substr(hash + 1u);  // skip '#'
  size_t start = s.find_first_not_of(" \t");
  if (start == std::string::npos) return false;
  s = s.substr(start);
  if (s.size() < 8) return false;
  std::string prefix = s.substr(0, 8);
  std::transform(prefix.begin(), prefix.end(), prefix.begin(), ::tolower);
  if (prefix != "origin: ") return false;
  std::istringstream iss(s.substr(8));
  return static_cast<bool>(iss >> out_lat >> out_lon);
}

std::vector<double> loadWaypointsMeters(const std::string& path)
{
  std::vector<double> out;
  std::ifstream f(path);
  if (!f.is_open()) return out;
  std::string line;
  while (std::getline(f, line)) {
    size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    std::istringstream iss(line);
    double x = 0.0, y = 0.0;
    if (iss >> x >> y) {
      out.push_back(x);
      out.push_back(y);
    }
  }
  return out;
}

std::vector<double> loadWgs84AndConvert(const std::string& path,
                                       double farm_origin_lat,
                                       double farm_origin_lon,
                                       double /* farm_origin_alt */)
{
  std::vector<double> out;
  std::ifstream f(path);
  if (!f.is_open()) return out;

  double origin_lat = farm_origin_lat;
  double origin_lon = farm_origin_lon;
  bool origin_from_file = false;
  std::string line;

  while (std::getline(f, line)) {
    double olat, olon;
    if (tryParseOriginLine(line, olat, olon)) {
      origin_lat = olat;
      origin_lon = olon;
      origin_from_file = true;
      continue;
    }
    size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    std::istringstream iss(line);
    double lat_deg = 0.0, lon_deg = 0.0;
    if (iss >> lat_deg >> lon_deg) {
      double east_m, north_m;
      wgs84ToEnu(lat_deg, lon_deg, origin_lat, origin_lon, east_m, north_m);
      out.push_back(east_m);
      out.push_back(north_m);
    }
  }
  (void)origin_from_file;
  return out;
}
}  // namespace

bool is_wgs84_mission(const std::string& path)
{
  return isWgs84Extension(path);
}

std::vector<double> load_mission(const std::string& path,
                                 double farm_origin_lat,
                                 double farm_origin_lon,
                                 double farm_origin_alt)
{
  if (path.empty()) return {};
  if (isWgs84Extension(path))
    return loadWgs84AndConvert(path, farm_origin_lat, farm_origin_lon, farm_origin_alt);
  return loadWaypointsMeters(path);
}

}  // namespace mower_mission
