// Copyright 2026 Gole Robotics
#include "slam_3d_localization/cloud_freshness_guard.hpp"

#include <cmath>
#include <stdexcept>

namespace gole::slam_3d_localization
{

CloudFreshnessGuard::CloudFreshnessGuard(double max_cloud_age_s)
: max_cloud_age_s_(max_cloud_age_s)
{
  if (!std::isfinite(max_cloud_age_s_) || max_cloud_age_s_ < 0.0) {
    throw std::invalid_argument("max_cloud_age_s must be finite and nonnegative");
  }
}

CloudFreshnessResult CloudFreshnessGuard::evaluate(
  std::int64_t ros_now_ns, std::int64_t cloud_stamp_ns) const
{
  CloudFreshnessResult result;
  result.enabled = max_cloud_age_s_ > 0.0;
  result.ros_time_started = ros_now_ns > 0;
  if (!result.ros_time_started) {
    return result;
  }
  result.age_s = static_cast<double>(ros_now_ns - cloud_stamp_ns) * 1.0e-9;
  result.stale = result.enabled && result.age_s > max_cloud_age_s_;
  return result;
}

}  // namespace gole::slam_3d_localization
