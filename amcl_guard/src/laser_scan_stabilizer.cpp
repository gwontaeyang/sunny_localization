#include "amcl_guard/laser_scan_stabilizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace amcl_guard
{
namespace
{

constexpr double kEpsilon = 1.0e-12;

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return
    std::isfinite(q.x) && std::isfinite(q.y) &&
    std::isfinite(q.z) && std::isfinite(q.w);
}

bool validScanGeometry(const sensor_msgs::msg::LaserScan & scan)
{
  return
    !scan.ranges.empty() &&
    std::isfinite(scan.angle_min) &&
    std::isfinite(scan.angle_increment) &&
    std::fabs(scan.angle_increment) > kEpsilon &&
    std::isfinite(scan.range_min) &&
    std::isfinite(scan.range_max) &&
    scan.range_max > scan.range_min;
}

}  // namespace

RelativeMotion relativeMotion(
  const geometry_msgs::msg::TransformStamped & transform)
{
  RelativeMotion motion;
  const auto & translation = transform.transform.translation;
  const auto & q_in = transform.transform.rotation;
  if (
    !std::isfinite(translation.x) || !std::isfinite(translation.y) ||
    !std::isfinite(translation.z) || !finiteQuaternion(q_in))
  {
    return motion;
  }

  const double norm = std::sqrt(
    q_in.x * q_in.x + q_in.y * q_in.y +
    q_in.z * q_in.z + q_in.w * q_in.w);
  if (!std::isfinite(norm) || norm <= kEpsilon) {
    return motion;
  }

  const double x = q_in.x / norm;
  const double y = q_in.y / norm;
  const double z = q_in.z / norm;
  const double w = q_in.w / norm;
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
  const double pitch = std::asin(sinp);
  const double yaw = std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));

  motion.translation_m = std::hypot(translation.x, translation.y);
  motion.yaw_rad = yaw;
  motion.roll_pitch_rad = std::hypot(roll, pitch);
  motion.finite =
    std::isfinite(motion.translation_m) &&
    std::isfinite(motion.yaw_rad) &&
    std::isfinite(motion.roll_pitch_rad);
  return motion;
}

std::optional<StabilizedScan> warpLaserScan(
  const sensor_msgs::msg::LaserScan & source,
  const sensor_msgs::msg::LaserScan & current_template,
  const geometry_msgs::msg::TransformStamped & current_from_source,
  const std::size_t min_valid_beams)
{
  if (!validScanGeometry(source) || !validScanGeometry(current_template)) {
    return std::nullopt;
  }

  const RelativeMotion motion = relativeMotion(current_from_source);
  if (!motion.finite) {
    return std::nullopt;
  }

  const auto & translation = current_from_source.transform.translation;
  StabilizedScan result;
  result.scan = current_template;
  result.scan.ranges.assign(
    current_template.ranges.size(),
    std::numeric_limits<float>::infinity());
  result.scan.intensities.clear();
  result.scan.time_increment = 0.0F;
  result.scan.scan_time = 0.0F;

  const double cos_yaw = std::cos(motion.yaw_rad);
  const double sin_yaw = std::sin(motion.yaw_rad);
  std::vector<bool> filled(result.scan.ranges.size(), false);
  for (std::size_t index = 0; index < source.ranges.size(); ++index) {
    const double range = source.ranges[index];
    if (
      !std::isfinite(range) ||
      range < source.range_min ||
      range > source.range_max)
    {
      continue;
    }

    const double angle =
      source.angle_min + static_cast<double>(index) * source.angle_increment;
    const double source_x = range * std::cos(angle);
    const double source_y = range * std::sin(angle);
    const double current_x =
      cos_yaw * source_x - sin_yaw * source_y + translation.x;
    const double current_y =
      sin_yaw * source_x + cos_yaw * source_y + translation.y;
    const double current_range = std::hypot(current_x, current_y);
    if (
      !std::isfinite(current_range) ||
      current_range < current_template.range_min ||
      current_range > current_template.range_max)
    {
      continue;
    }

    const double current_angle = std::atan2(current_y, current_x);
    const double floating_bin =
      (current_angle - current_template.angle_min) /
      current_template.angle_increment;
    if (!std::isfinite(floating_bin)) {
      continue;
    }
    const auto output_index = static_cast<long long>(std::llround(floating_bin));
    if (
      output_index < 0 ||
      output_index >= static_cast<long long>(result.scan.ranges.size()))
    {
      continue;
    }

    const std::size_t bin = static_cast<std::size_t>(output_index);
    const double bin_angle =
      current_template.angle_min +
      static_cast<double>(bin) * current_template.angle_increment;
    if (
      std::fabs(current_angle - bin_angle) >
      0.5 * std::fabs(current_template.angle_increment) + 1.0e-9)
    {
      continue;
    }

    result.scan.ranges[bin] = std::min(
      result.scan.ranges[bin],
      static_cast<float>(current_range));
    if (!filled[bin]) {
      filled[bin] = true;
      ++result.valid_beam_count;
    }
  }

  if (result.valid_beam_count < min_valid_beams) {
    return std::nullopt;
  }
  return result;
}

}  // namespace amcl_guard
