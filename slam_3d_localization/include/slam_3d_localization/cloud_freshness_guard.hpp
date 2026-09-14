// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__CLOUD_FRESHNESS_GUARD_HPP_
#define SLAM_3D_LOCALIZATION__CLOUD_FRESHNESS_GUARD_HPP_

#include <cstdint>

namespace gole::slam_3d_localization
{

struct CloudFreshnessResult
{
  bool enabled{false};
  bool ros_time_started{false};
  bool stale{false};
  double age_s{0.0};
};

class CloudFreshnessGuard
{
public:
  explicit CloudFreshnessGuard(double max_cloud_age_s);

  CloudFreshnessResult evaluate(
    std::int64_t ros_now_ns, std::int64_t cloud_stamp_ns) const;

private:
  double max_cloud_age_s_;
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__CLOUD_FRESHNESS_GUARD_HPP_
