// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__MAP_ODOM_TRANSFORM_CACHE_HPP_
#define SLAM_3D_LOCALIZATION__MAP_ODOM_TRANSFORM_CACHE_HPP_

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>

namespace gole::slam_3d_localization
{

class MapOdomTransformCache
{
public:
  bool reanchor(
    const Eigen::Isometry3d & map_T_base,
    const Eigen::Isometry3d & odom_T_base);
  void invalidate();
  std::optional<Eigen::Isometry3d> value() const;
  std::optional<Eigen::Isometry3d> takeForOdomStamp(std::int64_t stamp_ns);

private:
  std::optional<Eigen::Isometry3d> map_T_odom_;
  std::optional<std::int64_t> last_emitted_odom_stamp_ns_;
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__MAP_ODOM_TRANSFORM_CACHE_HPP_
