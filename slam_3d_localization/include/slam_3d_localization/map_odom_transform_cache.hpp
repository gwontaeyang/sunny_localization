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
  // This pure state helper is intentionally not internally synchronized.
  // Concurrent callers must serialize every method with one external mutex.
  bool reanchor(
    const Eigen::Isometry3d & map_T_base,
    const Eigen::Isometry3d & odom_T_base,
    std::int64_t anchor_stamp_ns);
  void retireIdentityBootstrap();
  void clearCorrectionPreserveWatermark();
  void resetOdomEpoch();
  std::optional<Eigen::Isometry3d> value() const;
  std::optional<Eigen::Isometry3d> takeForOdomStamp(
    std::int64_t stamp_ns, bool allow_identity_before_first_reanchor = false);

private:
  std::optional<Eigen::Isometry3d> map_T_odom_;
  std::optional<std::int64_t> correction_anchor_stamp_ns_;
  std::optional<std::int64_t> last_emitted_odom_stamp_ns_;
  bool identity_bootstrap_retired_{false};
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__MAP_ODOM_TRANSFORM_CACHE_HPP_
