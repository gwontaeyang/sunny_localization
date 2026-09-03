// Copyright 2026 Gole Robotics
#include "slam_3d_localization/map_odom_transform_cache.hpp"

namespace gole::slam_3d_localization
{

bool MapOdomTransformCache::reanchor(
  const Eigen::Isometry3d & map_T_base,
  const Eigen::Isometry3d & odom_T_base)
{
  if (!map_T_base.matrix().allFinite() || !odom_T_base.matrix().allFinite()) {
    invalidate();
    return false;
  }

  const Eigen::Isometry3d candidate = map_T_base * odom_T_base.inverse();
  if (!candidate.matrix().allFinite()) {
    invalidate();
    return false;
  }
  map_T_odom_ = candidate;
  return true;
}

void MapOdomTransformCache::invalidate()
{
  map_T_odom_.reset();
  last_emitted_odom_stamp_ns_.reset();
}

std::optional<Eigen::Isometry3d> MapOdomTransformCache::value() const
{
  return map_T_odom_;
}

std::optional<Eigen::Isometry3d> MapOdomTransformCache::takeForOdomStamp(
  std::int64_t stamp_ns)
{
  if (!map_T_odom_.has_value() ||
    (last_emitted_odom_stamp_ns_.has_value() &&
    stamp_ns <= *last_emitted_odom_stamp_ns_))
  {
    return std::nullopt;
  }
  last_emitted_odom_stamp_ns_ = stamp_ns;
  return map_T_odom_;
}

}  // namespace gole::slam_3d_localization
