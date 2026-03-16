/*
 * QGC WPL 110 mission converter.
 *
 * Reads a Mission Planner / QGroundControl .waypoints file in QGC WPL 110 text
 * format and writes a repo-native mission file:
 *
 *   - Output format: .wgs84 file with "lat lon" pairs (degrees), one per line.
 *
 * The existing mission_loader already supports .wgs84 files and performs
 * WGS84 -> ENU conversion at load time, so this tool only needs to extract
 * the latitude/longitude values.
 *
 * Behavior:
 *   - Ignores the QGC header line ("QGC WPL 110", case-sensitive).
 *   - Handles tab or space separated fields via std::istringstream.
 *   - Skips empty / whitespace-only lines.
 *   - Skips lines starting with '#' (treated as comments).
 *   - Skips non-navigation rows (only NAV_WAYPOINT, MAV_CMD_NAV_WAYPOINT = 16).
 *   - Logs input file, number of parsed waypoints, and output file.
 *
 * Parameters:
 *   - input_qgc_file   (string, required): path to QGC .waypoints file
 *   - output_wgs84_file(string, required): path to write .wgs84 lat/lon file
 */

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

// MAVLink command numeric value for NAV_WAYPOINT.
// See MAV_CMD_NAV_WAYPOINT = 16.
constexpr int kNavWaypointCommand = 16;

struct QgcRow
{
  int index = 0;
  int current = 0;
  int frame = 0;
  int command = 0;
  double param1 = 0.0;
  double param2 = 0.0;
  double param3 = 0.0;
  double param4 = 0.0;
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  int autocontinue = 0;
};

bool parseQgcLine(const std::string & line, QgcRow & out_row)
{
  std::string trimmed = line;
  // Treat lines starting with '#' as comments.
  auto first_non_ws = trimmed.find_first_not_of(" \t\r\n");
  if (first_non_ws == std::string::npos) {
    return false;
  }
  if (trimmed[first_non_ws] == '#') {
    return false;
  }

  std::istringstream iss(trimmed);
  QgcRow row;
  if (!(iss >> row.index
           >> row.current
           >> row.frame
           >> row.command
           >> row.param1
           >> row.param2
           >> row.param3
           >> row.param4
           >> row.lat
           >> row.lon
           >> row.alt
           >> row.autocontinue)) {
    return false;
  }
  out_row = row;
  return true;
}

}  // namespace

class QgcMissionConverterNode : public rclcpp::Node
{
public:
  QgcMissionConverterNode()
  : rclcpp::Node("qgc_mission_converter_node")
  {
    declare_parameter<std::string>("input_qgc_file", "");
    declare_parameter<std::string>("output_wgs84_file", "");

    const std::string input_path = get_parameter("input_qgc_file").as_string();
    const std::string output_path = get_parameter("output_wgs84_file").as_string();

    if (input_path.empty() || output_path.empty()) {
      RCLCPP_ERROR(get_logger(),
        "Both 'input_qgc_file' and 'output_wgs84_file' parameters must be set. "
        "Got input_qgc_file='%s', output_wgs84_file='%s'.",
        input_path.c_str(), output_path.c_str());
      throw std::runtime_error("Missing required parameters");
    }

    if (!convert(input_path, output_path)) {
      throw std::runtime_error("QGC mission conversion failed");
    }
  }

private:
  bool convert(const std::string & input_path, const std::string & output_path)
  {
    RCLCPP_INFO(get_logger(),
      "QGC mission converter: input_qgc_file='%s', output_wgs84_file='%s'",
      input_path.c_str(), output_path.c_str());

    std::ifstream in(input_path);
    if (!in.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open input_qgc_file '%s'", input_path.c_str());
      return false;
    }

    std::ofstream out(output_path);
    if (!out.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open output_wgs84_file '%s' for writing", output_path.c_str());
      return false;
    }

    std::string line;
    bool header_seen = false;
    std::size_t waypoint_count = 0;

    while (std::getline(in, line)) {
      // QGC WPL header: "QGC WPL 110"
      if (!header_seen) {
        std::string header = line;
        // Trim leading spaces/tabs for header detection.
        const auto first_non_ws = header.find_first_not_of(" \t\r\n");
        if (first_non_ws != std::string::npos) {
          header = header.substr(first_non_ws);
        }
        if (header.rfind("QGC WPL", 0) == 0) {
          header_seen = true;
          continue;
        }
        // If first non-empty, non-comment line is not a header, keep going but
        // still attempt to parse (some tools may omit the header).
      }

      QgcRow row;
      if (!parseQgcLine(line, row)) {
        continue;
      }

      // Only keep navigation waypoints.
      if (row.command != kNavWaypointCommand) {
        continue;
      }

      // Preserve high precision: write lat/lon with fixed 8 decimal places.
      out.setf(std::ios::fixed, std::ios::floatfield);
      out.precision(8);
      out << row.lat << " " << row.lon << "\n";
      waypoint_count++;
    }

    RCLCPP_INFO(get_logger(),
      "QGC mission converter: wrote %zu NAV_WAYPOINT(s) to '%s'",
      waypoint_count, output_path.c_str());

    if (waypoint_count == 0) {
      RCLCPP_WARN(get_logger(),
        "QGC mission converter: no NAV_WAYPOINT rows were found in '%s'. "
        "Output file '%s' will be empty or contain no usable waypoints.",
        input_path.c_str(), output_path.c_str());
    }

    return true;
  }
};

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QgcMissionConverterNode>();
    // One-shot node: perform conversion in constructor, then shut down.
    rclcpp::shutdown();
    (void)node;
    return 0;
  } catch (const std::exception & e) {
    // rclcpp::init may not have been called successfully; safe to call shutdown.
    rclcpp::shutdown();
    fprintf(stderr, "qgc_mission_converter_node: exception: %s\n", e.what());
    return 1;
  }
}

