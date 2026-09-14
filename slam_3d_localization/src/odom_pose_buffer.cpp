// Copyright 2026 Gole Robotics
#include "slam_3d_localization/odom_pose_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{

std::int64_t secondsToNanoseconds(double seconds)
{
  return static_cast<std::int64_t>(std::llround(seconds * 1.0e9));
}

}  // namespace

namespace gole::slam_3d_localization
{

OdomPoseBuffer::OdomPoseBuffer(
  double buffer_duration_s, double max_interpolation_gap_s,
  double max_nearest_age_s, double timestamp_reset_threshold_s)
: buffer_duration_ns_(0),
  max_interpolation_gap_ns_(0),
  max_nearest_age_ns_(0),
  timestamp_reset_threshold_ns_(0)
{
  if (!std::isfinite(buffer_duration_s) ||
    !std::isfinite(max_interpolation_gap_s) ||
    !std::isfinite(max_nearest_age_s) ||
    !std::isfinite(timestamp_reset_threshold_s) ||
    buffer_duration_s <= 0.0 || max_interpolation_gap_s <= 0.0 ||
    max_nearest_age_s < 0.0 || max_interpolation_gap_s > buffer_duration_s ||
    max_nearest_age_s > buffer_duration_s || timestamp_reset_threshold_s <= 0.0)
  {
    throw std::invalid_argument("invalid odometry buffer timing parameters");
  }
  buffer_duration_ns_ = secondsToNanoseconds(buffer_duration_s);
  max_interpolation_gap_ns_ = secondsToNanoseconds(max_interpolation_gap_s);
  max_nearest_age_ns_ = secondsToNanoseconds(max_nearest_age_s);
  timestamp_reset_threshold_ns_ =
    secondsToNanoseconds(timestamp_reset_threshold_s);
}

bool OdomPoseBuffer::insert(
  std::int64_t stamp_ns, const Eigen::Isometry3d & odom_T_base)
{
  if (!odom_T_base.matrix().allFinite()) {
    return false;
  }

  bool timestamp_reset = false;
  if (highest_observed_stamp_ns_.has_value() &&
    stamp_ns < *highest_observed_stamp_ns_ - timestamp_reset_threshold_ns_)
  {
    samples_.clear();
    timestamp_reset = true;
    highest_observed_stamp_ns_ = stamp_ns;
  } else if (!highest_observed_stamp_ns_.has_value() ||
    stamp_ns > *highest_observed_stamp_ns_)
  {
    highest_observed_stamp_ns_ = stamp_ns;
  }

  const auto position = std::lower_bound(
    samples_.begin(), samples_.end(), stamp_ns,
    [](const Sample & sample, std::int64_t stamp) {
      return sample.stamp_ns < stamp;
    });
  if (position != samples_.end() && position->stamp_ns == stamp_ns) {
    position->odom_T_base = odom_T_base;
  } else {
    samples_.insert(position, Sample{stamp_ns, odom_T_base});
  }

  const std::int64_t newest_stamp_ns = samples_.back().stamp_ns;
  const std::int64_t oldest_allowed_ns = newest_stamp_ns - buffer_duration_ns_;
  while (!samples_.empty() && samples_.front().stamp_ns < oldest_allowed_ns) {
    samples_.pop_front();
  }
  return timestamp_reset;
}

OdomLookupResult OdomPoseBuffer::lookup(std::int64_t stamp_ns) const
{
  OdomLookupResult result;
  if (samples_.empty()) {
    return result;
  }

  const auto after = std::lower_bound(
    samples_.begin(), samples_.end(), stamp_ns,
    [](const Sample & sample, std::int64_t stamp) {
      return sample.stamp_ns < stamp;
    });

  if (after != samples_.end() && after->stamp_ns == stamp_ns) {
    result.valid = true;
    result.odom_T_base = after->odom_T_base;
    result.mode = OdomLookupMode::kExact;
    return result;
  }

  if (after == samples_.begin()) {
    OdomLookupResult future = nearestResult(
      *after, stamp_ns, OdomLookupMode::kNearestFuture, 0.0);
    future.valid = false;
    return future;
  }
  if (after == samples_.end()) {
    return nearestResult(
      samples_.back(), stamp_ns, OdomLookupMode::kNearestPast, 0.0);
  }

  const Sample & before_sample = *std::prev(after);
  const Sample & after_sample = *after;
  const std::int64_t gap_ns = after_sample.stamp_ns - before_sample.stamp_ns;
  const double gap_s = static_cast<double>(gap_ns) * 1.0e-9;
  if (gap_ns <= max_interpolation_gap_ns_) {
    const double alpha = static_cast<double>(stamp_ns - before_sample.stamp_ns) /
      static_cast<double>(gap_ns);
    const Eigen::Quaterniond before_rotation(before_sample.odom_T_base.linear());
    const Eigen::Quaterniond after_rotation(after_sample.odom_T_base.linear());
    result.valid = true;
    result.mode = OdomLookupMode::kInterpolated;
    result.bracket_gap_s = gap_s;
    result.nearest_age_s = static_cast<double>(
      std::min(
        stamp_ns - before_sample.stamp_ns,
        after_sample.stamp_ns - stamp_ns)) * 1.0e-9;
    result.odom_T_base.translation() =
      (1.0 - alpha) * before_sample.odom_T_base.translation() +
      alpha * after_sample.odom_T_base.translation();
    result.odom_T_base.linear() =
      before_rotation.normalized().slerp(alpha, after_rotation.normalized())
      .normalized().toRotationMatrix();
    return result;
  }

  return nearestResult(
    before_sample, stamp_ns, OdomLookupMode::kNearestPast, gap_s);
}

void OdomPoseBuffer::clear()
{
  samples_.clear();
}

std::size_t OdomPoseBuffer::size() const
{
  return samples_.size();
}

const char * OdomPoseBuffer::modeName(OdomLookupMode mode)
{
  switch (mode) {
    case OdomLookupMode::kExact:
      return "exact";
    case OdomLookupMode::kInterpolated:
      return "interpolated";
    case OdomLookupMode::kNearestPast:
      return "nearest_past";
    case OdomLookupMode::kNearestFuture:
      return "nearest_future";
    case OdomLookupMode::kUnavailable:
    default:
      return "unavailable";
  }
}

OdomLookupResult OdomPoseBuffer::nearestResult(
  const Sample & sample, std::int64_t stamp_ns, OdomLookupMode mode,
  double bracket_gap_s) const
{
  OdomLookupResult result;
  const std::int64_t age_ns = std::abs(stamp_ns - sample.stamp_ns);
  result.mode = mode;
  result.nearest_age_s = static_cast<double>(age_ns) * 1.0e-9;
  result.bracket_gap_s = bracket_gap_s;
  if (age_ns > max_nearest_age_ns_) {
    return result;
  }
  result.valid = true;
  result.odom_T_base = sample.odom_T_base;
  return result;
}

}  // namespace gole::slam_3d_localization
