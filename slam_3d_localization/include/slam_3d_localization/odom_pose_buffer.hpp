// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__ODOM_POSE_BUFFER_HPP_
#define SLAM_3D_LOCALIZATION__ODOM_POSE_BUFFER_HPP_

#include <Eigen/Geometry>

#include <cstdint>
#include <deque>
#include <string>

namespace gole::slam_3d_localization
{

enum class OdomLookupMode
{
  kUnavailable,
  kExact,
  kInterpolated,
  kNearestPast,
  kNearestFuture,
};

struct OdomLookupResult
{
  bool valid{false};
  Eigen::Isometry3d odom_T_base{Eigen::Isometry3d::Identity()};
  OdomLookupMode mode{OdomLookupMode::kUnavailable};
  double nearest_age_s{0.0};
  double bracket_gap_s{0.0};
};

class OdomPoseBuffer
{
public:
  OdomPoseBuffer(
    double buffer_duration_s, double max_interpolation_gap_s,
    double max_nearest_age_s, double timestamp_reset_threshold_s);

  bool insert(std::int64_t stamp_ns, const Eigen::Isometry3d & odom_T_base);
  OdomLookupResult lookup(std::int64_t stamp_ns) const;
  void clear();
  std::size_t size() const;

  static const char * modeName(OdomLookupMode mode);

private:
  struct Sample
  {
    std::int64_t stamp_ns;
    Eigen::Isometry3d odom_T_base;
  };

  OdomLookupResult nearestResult(
    const Sample & sample, std::int64_t stamp_ns, OdomLookupMode mode,
    double bracket_gap_s) const;

  std::int64_t buffer_duration_ns_;
  std::int64_t max_interpolation_gap_ns_;
  std::int64_t max_nearest_age_ns_;
  std::int64_t timestamp_reset_threshold_ns_;
  std::deque<Sample> samples_;
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__ODOM_POSE_BUFFER_HPP_
