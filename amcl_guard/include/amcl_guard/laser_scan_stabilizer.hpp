#ifndef AMCL_GUARD__LASER_SCAN_STABILIZER_HPP_
#define AMCL_GUARD__LASER_SCAN_STABILIZER_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cstddef>
#include <optional>

namespace amcl_guard
{

struct RelativeMotion
{
  double translation_m = 0.0;
  double yaw_rad = 0.0;
  double roll_pitch_rad = 0.0;
  bool finite = false;
};

struct StabilizedScan
{
  sensor_msgs::msg::LaserScan scan;
  std::size_t valid_beam_count = 0;
};

RelativeMotion relativeMotion(
  const geometry_msgs::msg::TransformStamped & transform);

std::optional<StabilizedScan> warpLaserScan(
  const sensor_msgs::msg::LaserScan & source,
  const sensor_msgs::msg::LaserScan & current_template,
  const geometry_msgs::msg::TransformStamped & current_from_source,
  std::size_t min_valid_beams);

}  // namespace amcl_guard

#endif  // AMCL_GUARD__LASER_SCAN_STABILIZER_HPP_
