// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__REGISTRATION_BACKEND_HPP_
#define SLAM_3D_LOCALIZATION__REGISTRATION_BACKEND_HPP_

#include "slam_3d_localization/voxel_surfel_registration.hpp"

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace gole::slam_3d_localization
{

struct RegistrationBackendConfig
{
  std::string method{"gicp"};
  int max_iterations{40};
  double max_correspondence_m{2.0};
  double transformation_epsilon{1.0e-3};
  double fitness_epsilon{1.0e-3};
  double ndt_resolution_m{3.0};
  double ndt_step_size_m{0.1};
  double ndt_outlier_ratio{0.55};
  int fast_gicp_num_threads{4};
  int fast_gicp_correspondence_randomness{20};
  std::string fast_gicp_regularization_method{"PLANE"};
  double fast_gicp_rotation_epsilon_rad{2.0e-3};
  double fast_vgicp_voxel_resolution_m{1.0};
  std::string fast_vgicp_neighbor_search_method{"DIRECT1"};
  double fast_vgicp_cuda_voxel_resolution_m{1.0};
  std::string fast_vgicp_cuda_neighbor_search_method{"DIRECT1"};
  double fast_vgicp_cuda_neighbor_search_radius{1.5};
  std::string fast_vgicp_cuda_nearest_neighbor_method{"CPU_PARALLEL_KDTREE"};
  double fast_vgicp_cuda_kernel_width{0.5};
  double fast_vgicp_cuda_kernel_max_distance{3.0};
  VoxelSurfelRegistration::Config surfel;
};

struct RegistrationBackendResult
{
  Eigen::Isometry3d target_T_source{Eigen::Isometry3d::Identity()};
  bool converged{false};
  // Native solvers may accept the final iterate without declaring convergence.
  bool accepted{false};
  std::string reason{"not_run"};
  double fitness{0.0};
  double rmse_m{0.0};
  VoxelSurfelRegistration::Result surfel;
  int iterations{0};
  bool first_call{false};
  bool target_reused{false};
  std::uint64_t target_revision{0};
  std::uint64_t target_upload_count{0};
  double target_update_runtime_ms{0.0};
  double align_runtime_ms{0.0};
  double total_runtime_ms{0.0};
};

struct RegistrationBackendBuildInfo
{
  bool fast_gicp_compiled{false};
  bool fast_vgicp_cuda_compiled{false};
  std::string fast_gicp_revision{"not_built"};
  std::string cuda_architectures{"not_built"};
};

class RegistrationBackend
{
public:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  virtual ~RegistrationBackend() = default;

  virtual const char * name() const noexcept = 0;
  virtual bool isCuda() const noexcept = 0;
  virtual double initializationRuntimeMs() const noexcept = 0;
  virtual std::string runtimeDescription() const = 0;
  // Stateful solvers are serialized by the owning node, not internally.
  // A changed target requires both a new immutable pointer and a new revision.
  virtual RegistrationBackendResult align(
    const CloudT::ConstPtr & source, const CloudT::ConstPtr & target,
    std::uint64_t target_revision, const Eigen::Isometry3d & initial_guess) = 0;
};

struct RegistrationOverlap
{
  int inlier_count{0};
  double inlier_ratio{0.0};
  double inlier_rmse_m{0.0};
  double transform_determinant{1.0};
  double transform_orthogonality_error{0.0};
};

struct RegistrationQualityLimits
{
  double fitness_threshold{1.0};
  double max_correction_translation_m{2.0};
  double max_correction_rotation_deg{30.0};
  double min_inlier_ratio{0.15};
  double max_inlier_rmse_m{1.5};
};

struct RegistrationAcceptance
{
  bool accepted{false};
  std::string reason{"not_run"};
};

RegistrationOverlap evaluateRegistrationOverlap(
  const RegistrationBackend::CloudT::ConstPtr & source,
  const RegistrationBackend::CloudT::ConstPtr & target,
  const Eigen::Isometry3d & target_T_source, double max_correspondence_m);
RegistrationAcceptance evaluateLioRegistration(
  const RegistrationBackendResult & result, const Eigen::Isometry3d & prediction,
  const RegistrationQualityLimits & limits);
RegistrationAcceptance evaluateMapRegistration(
  const RegistrationBackendResult & result, const RegistrationOverlap & overlap,
  bool use_surfel_gates, const Eigen::Isometry3d & prediction,
  const RegistrationQualityLimits & limits);

bool isKnownRegistrationBackend(const std::string & method) noexcept;
std::vector<std::string> availableRegistrationBackends();
RegistrationBackendBuildInfo registrationBackendBuildInfo();
std::unique_ptr<RegistrationBackend> createRegistrationBackend(
  const RegistrationBackendConfig & config);

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__REGISTRATION_BACKEND_HPP_
