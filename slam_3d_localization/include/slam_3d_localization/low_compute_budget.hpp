// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__LOW_COMPUTE_BUDGET_HPP_
#define SLAM_3D_LOCALIZATION__LOW_COMPUTE_BUDGET_HPP_

#include <cstddef>
#include <cstdint>

namespace gole::slam_3d_localization
{

// Fixed CPU-first policy, intentionally not another set of ROS parameters.
// Keep every IMU measurement, but bound scan work and outgoing odometry.
struct LowComputeBudget
{
  static constexpr std::int64_t kScanPeriodNs = 2000000000LL;
  static constexpr std::int64_t kOdometryPeriodNs = 500000000LL;
  static constexpr std::size_t kMaxDeskewPoints = 1024U;
  static constexpr int kPendingRetryMs = 50;
  static constexpr int kImuBatchMs = 50;

  static std::size_t pointStride(const std::size_t count)
  {
    return count == 0U ? 1U : 1U + (count - 1U) / kMaxDeskewPoints;
  }

  static bool odometryDue(const std::int64_t stamp, const std::int64_t previous)
  {
    return previous < 0 || stamp < previous || stamp - previous >= kOdometryPeriodNs;
  }

  static bool admitScan(
    const std::int64_t stamp, std::int64_t & previous,
    const std::int64_t reset_threshold)
  {
    if (stamp < 0) {
      return false;
    }
    if (previous >= 0 && stamp >= previous - reset_threshold &&
      stamp - previous < kScanPeriodNs)
    {
      return false;
    }
    previous = stamp;
    return true;
  }
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__LOW_COMPUTE_BUDGET_HPP_
