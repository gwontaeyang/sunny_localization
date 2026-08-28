// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__CONSTANT_VELOCITY_PREDICTOR_HPP_
#define SLAM_3D_LOCALIZATION__CONSTANT_VELOCITY_PREDICTOR_HPP_

#include <Eigen/Geometry>

#include <cstdint>

namespace gole::slam_3d_localization
{

enum class ConstantVelocityPredictionStatus
{
  kNotEvaluated,
  kNotReady,
  kApplied,
  kNonmonotonicStamp,
  kHistoryGapTooLarge,
  kPredictionGapTooLarge,
  kInvalidPose,
};

struct ConstantVelocityPredictionResult
{
  Eigen::Isometry3d map_T_base{Eigen::Isometry3d::Identity()};
  ConstantVelocityPredictionStatus status{
    ConstantVelocityPredictionStatus::kNotEvaluated};
  bool applied{false};
  bool history_ready{false};
  bool translation_clamped{false};
  bool rotation_clamped{false};
  double history_gap_s{0.0};
  double prediction_gap_s{0.0};
  double raw_translation_m{0.0};
  double raw_rotation_deg{0.0};
  double applied_translation_m{0.0};
  double applied_rotation_deg{0.0};
};

class ConstantVelocityPredictor
{
public:
  ConstantVelocityPredictor(
    double translation_limit_m, double rotation_limit_deg,
    double max_gap_s);

  void reset();
  bool rebaseAccepted(
    const Eigen::Isometry3d & map_T_base, std::int64_t stamp_ns);
  bool recordAccepted(
    const Eigen::Isometry3d & map_T_base, std::int64_t stamp_ns);
  ConstantVelocityPredictionResult predict(
    const Eigen::Isometry3d & fallback_map_T_base,
    std::int64_t stamp_ns);
  bool ready() const;

  static const char * statusName(ConstantVelocityPredictionStatus status);

private:
  void rebaseToLast();

  double translation_limit_m_;
  double rotation_limit_rad_;
  std::int64_t max_gap_ns_;
  bool have_previous_{false};
  bool have_last_{false};
  Eigen::Isometry3d previous_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_pose_{Eigen::Isometry3d::Identity()};
  std::int64_t previous_stamp_ns_{0};
  std::int64_t last_stamp_ns_{0};
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__CONSTANT_VELOCITY_PREDICTOR_HPP_
