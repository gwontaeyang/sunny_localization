// Copyright 2026 Gole Robotics
#include "slam_3d_localization/registration_backend.hpp"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_GICP
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>
#endif

#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_VGICP_CUDA
#include <cuda_runtime_api.h>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifndef SLAM_3D_LOCALIZATION_FAST_GICP_REVISION
#define SLAM_3D_LOCALIZATION_FAST_GICP_REVISION "unknown"
#endif

#ifndef SLAM_3D_LOCALIZATION_CUDA_ARCHITECTURES
#define SLAM_3D_LOCALIZATION_CUDA_ARCHITECTURES "unknown"
#endif

namespace gole::slam_3d_localization
{
namespace
{

using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

bool positiveFinite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

const RegistrationBackendConfig & validateAdditionalBackendConfig(
  const RegistrationBackendConfig & config)
{
  if (config.max_iterations <= 0 || !positiveFinite(config.max_correspondence_m) ||
    !positiveFinite(config.transformation_epsilon) || !positiveFinite(config.fitness_epsilon))
  {
    throw std::invalid_argument(config.method + ": invalid common registration configuration");
  }
  if (config.method == "ndt" &&
    (!positiveFinite(config.ndt_resolution_m) ||
    !positiveFinite(static_cast<float>(config.ndt_resolution_m)) ||
    !positiveFinite(config.ndt_step_size_m) ||
    !std::isfinite(config.ndt_outlier_ratio) ||
    config.ndt_outlier_ratio <= 0.0 || config.ndt_outlier_ratio >= 1.0))
  {
    throw std::invalid_argument(
            "ndt requires finite positive resolution/step size and 0 < outlier ratio < 1");
  }
  const bool uses_fast_gicp = config.method == "fast_gicp" ||
    config.method == "fast_vgicp" || config.method == "fast_vgicp_cuda";
  if (uses_fast_gicp) {
    if (config.fast_gicp_num_threads < 0 || config.fast_gicp_correspondence_randomness < 3 ||
      !positiveFinite(config.fast_gicp_rotation_epsilon_rad))
    {
      throw std::invalid_argument(
              config.method + ": requires threads >= 0, covariance k >= 3 and finite positive rotation epsilon");
    }
    const auto & regularization = config.fast_gicp_regularization_method;
    if (regularization != "NONE" && regularization != "MIN_EIG" &&
      regularization != "NORMALIZED_MIN_EIG" && regularization != "PLANE" &&
      regularization != "FROBENIUS")
    {
      throw std::invalid_argument("unknown fast_gicp_regularization_method: " + regularization);
    }
  }
  if (config.method == "fast_vgicp") {
    if (!positiveFinite(config.fast_vgicp_voxel_resolution_m)) {
      throw std::invalid_argument("fast_vgicp requires finite positive voxel resolution");
    }
    const auto & method = config.fast_vgicp_neighbor_search_method;
    if (method != "DIRECT1" && method != "DIRECT7" && method != "DIRECT27") {
      throw std::invalid_argument(
              "fast_vgicp_neighbor_search_method must be DIRECT1, DIRECT7 or DIRECT27");
    }
  }
  if (config.method == "fast_vgicp_cuda") {
    if (!positiveFinite(config.fast_vgicp_cuda_voxel_resolution_m) ||
      !positiveFinite(config.fast_vgicp_cuda_neighbor_search_radius) ||
      !positiveFinite(config.fast_vgicp_cuda_kernel_width) ||
      !positiveFinite(config.fast_vgicp_cuda_kernel_max_distance))
    {
      throw std::invalid_argument(
              "fast_vgicp_cuda requires finite positive resolution/radius/kernel sizes");
    }
    const auto & voxel_search = config.fast_vgicp_cuda_neighbor_search_method;
    if (voxel_search != "DIRECT1" && voxel_search != "DIRECT7" &&
      voxel_search != "DIRECT27" && voxel_search != "DIRECT_RADIUS")
    {
      throw std::invalid_argument("unknown fast_vgicp_cuda_neighbor_search_method: " + voxel_search);
    }
    const auto & covariance_search = config.fast_vgicp_cuda_nearest_neighbor_method;
    if (covariance_search != "CPU_PARALLEL_KDTREE" &&
      covariance_search != "GPU_BRUTEFORCE" && covariance_search != "GPU_RBF_KERNEL")
    {
      throw std::invalid_argument("unknown fast_vgicp_cuda_nearest_neighbor_method: " + covariance_search);
    }
  }
  return config;
}

bool validateTarget(
  const RegistrationBackend::CloudT::ConstPtr & source,
  const RegistrationBackend::CloudT::ConstPtr & target,
  const Eigen::Isometry3d & initial_guess,
  const RegistrationBackend::CloudT::ConstPtr & cached_target,
  const std::uint64_t revision, const std::uint64_t cached_revision)
{
  if (!source || source->empty() || !target || target->empty()) {
    throw std::invalid_argument("registration source and target must be nonempty");
  }
  if (!initial_guess.matrix().allFinite()) {
    throw std::invalid_argument("registration initial guess must be finite");
  }
  if (cached_target && target == cached_target && revision != cached_revision) {
    throw std::logic_error("registration target revision changed without a target pointer change");
  }
  if (cached_target && revision == cached_revision && target != cached_target) {
    throw std::logic_error("registration target pointer changed without a target revision change");
  }
  return cached_target && target == cached_target;
}

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  return std::acos(std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0));
}

template<typename RegistrationT>
class InstrumentedRegistration : public RegistrationT
{
public:
  int iterations() const noexcept
  {
    return this->nr_iterations_;
  }
};

template<typename RegistrationT>
class PclStyleBackend : public RegistrationBackend
{
public:
  explicit PclStyleBackend(const RegistrationBackendConfig & config)
  : config_(config)
  {
    const auto start = Clock::now();
    registration_ = std::make_unique<InstrumentedRegistration<RegistrationT>>();
    registration_->setMaximumIterations(config_.max_iterations);
    registration_->setMaxCorrespondenceDistance(config_.max_correspondence_m);
    registration_->setTransformationEpsilon(config_.transformation_epsilon);
    registration_->setEuclideanFitnessEpsilon(config_.fitness_epsilon);
    initialization_runtime_ms_ = elapsedMs(start);
  }

  double initializationRuntimeMs() const noexcept override
  {
    return initialization_runtime_ms_;
  }

  RegistrationBackendResult align(
    const CloudT::ConstPtr & source, const CloudT::ConstPtr & target,
    const std::uint64_t target_revision,
    const Eigen::Isometry3d & initial_guess) override
  {
    RegistrationBackendResult result;
    result.target_reused = validateTarget(
      source, target, initial_guess, target_, target_revision, target_revision_);
    result.first_call = call_count_ == 0;
    result.target_revision = target_revision;
    const auto total_start = Clock::now();

    if (!result.target_reused) {
      const auto target_start = Clock::now();
      beforeTargetUpdate();
      registration_->setInputTarget(target);
      afterTargetUpdate();
      result.target_update_runtime_ms = elapsedMs(target_start);
      target_ = target;
      target_revision_ = target_revision;
      ++target_upload_count_;
    }
    result.target_upload_count = target_upload_count_;

    const auto align_start = Clock::now();
    beforeAlign();
    registration_->setInputSource(source);
    CloudT aligned;
    registration_->align(aligned, initial_guess.matrix().cast<float>());
    afterAlign();
    result.align_runtime_ms = elapsedMs(align_start);
    result.converged = registration_->hasConverged();
    result.accepted = result.converged;
    result.reason = result.converged ? "converged" : "not_converged";
    result.fitness = result.converged ?
      registration_->getFitnessScore(config_.max_correspondence_m) :
      std::numeric_limits<double>::infinity();
    result.rmse_m = std::sqrt(std::max(0.0, result.fitness));
    result.target_T_source.matrix() =
      registration_->getFinalTransformation().template cast<double>();
    result.iterations = registration_->iterations();
    result.total_runtime_ms = elapsedMs(total_start);
    ++call_count_;
    return result;
  }

protected:
  virtual void beforeTargetUpdate()
  {
  }

  virtual void afterTargetUpdate()
  {
  }

  virtual void beforeAlign()
  {
  }

  virtual void afterAlign()
  {
  }

  RegistrationBackendConfig config_;
  std::unique_ptr<InstrumentedRegistration<RegistrationT>> registration_;
  CloudT::ConstPtr target_;
  std::uint64_t target_revision_{0};
  std::uint64_t target_upload_count_{0};
  std::uint64_t call_count_{0};
  double initialization_runtime_ms_{0.0};
};

class VoxelSurfelBackend final : public RegistrationBackend
{
public:
  explicit VoxelSurfelBackend(const RegistrationBackendConfig & config)
  : registration_(config.surfel)
  {
    if (!VoxelSurfelRegistration::isValidConfig(config.surfel)) {
      throw std::invalid_argument("invalid voxel_surfel registration configuration");
    }
  }

  const char * name() const noexcept override {return "voxel_surfel";}
  bool isCuda() const noexcept override {return false;}
  double initializationRuntimeMs() const noexcept override {return 0.0;}
  std::string runtimeDescription() const override
  {
    return "Native voxel-surfel Gauss-Newton (CPU, immutable target surfel cache)";
  }

  RegistrationBackendResult align(
    const CloudT::ConstPtr & source, const CloudT::ConstPtr & target,
    const std::uint64_t target_revision, const Eigen::Isometry3d & initial_guess) override
  {
    RegistrationBackendResult result;
    result.target_reused = validateTarget(
      source, target, initial_guess, target_, target_revision, target_revision_);
    const auto start = Clock::now();
    result.first_call = call_count_ == 0U;
    result.target_revision = target_revision;
    if (!result.target_reused) {
      const auto target_start = Clock::now();
      auto prepared = registration_.prepareTarget(*target);
      if (!prepared) {
        throw std::logic_error("native registration target preparation failed");
      }
      prepared_target_ = std::move(prepared);
      target_ = target;
      target_revision_ = target_revision;
      ++target_upload_count_;
      result.target_update_runtime_ms = elapsedMs(target_start);
    }
    result.target_upload_count = target_upload_count_;
    const auto align_start = Clock::now();
    result.surfel = registration_.alignPrepared(*source, prepared_target_, initial_guess);
    result.align_runtime_ms = elapsedMs(align_start);
    result.target_T_source = result.surfel.target_T_source;
    result.accepted = result.surfel.accepted;
    result.converged = result.surfel.converged;
    result.reason = result.surfel.reason;
    result.rmse_m = result.surfel.rmse;
    result.fitness = std::isfinite(result.rmse_m) ? result.rmse_m * result.rmse_m :
      std::numeric_limits<double>::infinity();
    result.iterations = result.surfel.iterations;
    result.total_runtime_ms = elapsedMs(start);
    ++call_count_;
    return result;
  }

private:
  VoxelSurfelRegistration registration_;
  VoxelSurfelRegistration::PreparedTargetPtr prepared_target_;
  CloudT::ConstPtr target_;
  std::uint64_t target_revision_{0};
  std::uint64_t target_upload_count_{0};
  std::uint64_t call_count_{0};
};

class PclIcpBackend final :
  public PclStyleBackend<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>
{
public:
  using Base = PclStyleBackend<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>;
  explicit PclIcpBackend(const RegistrationBackendConfig & config)
  : Base(validateAdditionalBackendConfig(config)) {}
  const char * name() const noexcept override {return "icp";}
  bool isCuda() const noexcept override {return false;}
  std::string runtimeDescription() const override
  {
    return "PCL IterativeClosestPoint (CPU, point-to-point, cached target KD-tree)";
  }
};

class PclNdtBackend final :
  public PclStyleBackend<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>
{
public:
  using Base = PclStyleBackend<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>;
  explicit PclNdtBackend(const RegistrationBackendConfig & config)
  : Base(validateAdditionalBackendConfig(config))
  {
    const auto start = Clock::now();
    registration_->setResolution(static_cast<float>(config_.ndt_resolution_m));
    registration_->setStepSize(config_.ndt_step_size_m);
    // PCL 1.12 deliberately spells this public API "Oulier".
    registration_->setOulierRatio(config_.ndt_outlier_ratio);
    initialization_runtime_ms_ += elapsedMs(start);
  }
  const char * name() const noexcept override {return "ndt";}
  bool isCuda() const noexcept override {return false;}
  std::string runtimeDescription() const override
  {
    std::ostringstream description;
    description << "PCL NormalDistributionsTransform (CPU, voxel="
                << config_.ndt_resolution_m << " m, step_size=" << config_.ndt_step_size_m
                << ", outlier_ratio=" << config_.ndt_outlier_ratio
                << ", cached target voxel grid, Euclidean fitness)";
    return description.str();
  }
};

class PclGicpBackend final :
  public PclStyleBackend<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>
{
public:
  using PclStyleBackend::PclStyleBackend;

  const char * name() const noexcept override
  {
    return "gicp";
  }

  bool isCuda() const noexcept override
  {
    return false;
  }

  std::string runtimeDescription() const override
  {
    return "PCL GeneralizedIterativeClosestPoint (CPU)";
  }
};

#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_GICP
fast_gicp::RegularizationMethod regularizationMethod(const std::string & name)
{
  if (name == "NONE") {
    return fast_gicp::RegularizationMethod::NONE;
  }
  if (name == "MIN_EIG") {
    return fast_gicp::RegularizationMethod::MIN_EIG;
  }
  if (name == "NORMALIZED_MIN_EIG") {
    return fast_gicp::RegularizationMethod::NORMALIZED_MIN_EIG;
  }
  if (name == "PLANE") {
    return fast_gicp::RegularizationMethod::PLANE;
  }
  if (name == "FROBENIUS") {
    return fast_gicp::RegularizationMethod::FROBENIUS;
  }
  throw std::invalid_argument("unknown fast_gicp_regularization_method: " + name);
}

class FastGicpBackend final :
  public PclStyleBackend<fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>>
{
public:
  using Base = PclStyleBackend<fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>>;

  explicit FastGicpBackend(const RegistrationBackendConfig & config)
  : Base(validateAdditionalBackendConfig(config))
  {
    const auto start = Clock::now();
    registration_->setNumThreads(config_.fast_gicp_num_threads);
    registration_->setCorrespondenceRandomness(config_.fast_gicp_correspondence_randomness);
    registration_->setRegularizationMethod(
      regularizationMethod(config_.fast_gicp_regularization_method));
    registration_->setRotationEpsilon(config_.fast_gicp_rotation_epsilon_rad);
    initialization_runtime_ms_ += elapsedMs(start);
  }

  const char * name() const noexcept override
  {
    return "fast_gicp";
  }

  bool isCuda() const noexcept override
  {
    return false;
  }

  std::string runtimeDescription() const override
  {
    std::ostringstream description;
    description << "fast_gicp FastGICP (OpenMP CPU, threads="
                << config_.fast_gicp_num_threads << ", k="
                << config_.fast_gicp_correspondence_randomness << ", regularization="
                << config_.fast_gicp_regularization_method << ")";
    return description.str();
  }

};

fast_gicp::NeighborSearchMethod cpuVoxelNeighborMethod(const std::string & name)
{
  if (name == "DIRECT1") {return fast_gicp::NeighborSearchMethod::DIRECT1;}
  if (name == "DIRECT7") {return fast_gicp::NeighborSearchMethod::DIRECT7;}
  if (name == "DIRECT27") {return fast_gicp::NeighborSearchMethod::DIRECT27;}
  throw std::invalid_argument("unsupported fast_vgicp_neighbor_search_method: " + name);
}

class FastVgicpBackend final :
  public PclStyleBackend<fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>>
{
public:
  using Base = PclStyleBackend<fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>>;
  explicit FastVgicpBackend(const RegistrationBackendConfig & config)
  : Base(validateAdditionalBackendConfig(config))
  {
    const auto start = Clock::now();
    registration_->setNumThreads(config_.fast_gicp_num_threads);
    registration_->setCorrespondenceRandomness(config_.fast_gicp_correspondence_randomness);
    registration_->setRegularizationMethod(
      regularizationMethod(config_.fast_gicp_regularization_method));
    registration_->setRotationEpsilon(config_.fast_gicp_rotation_epsilon_rad);
    registration_->setResolution(config_.fast_vgicp_voxel_resolution_m);
    registration_->setNeighborSearchMethod(
      cpuVoxelNeighborMethod(config_.fast_vgicp_neighbor_search_method));
    initialization_runtime_ms_ += elapsedMs(start);
  }
  const char * name() const noexcept override {return "fast_vgicp";}
  bool isCuda() const noexcept override {return false;}
  std::string runtimeDescription() const override
  {
    std::ostringstream description;
    description << "fast_gicp FastVGICP (OpenMP CPU, threads="
                << config_.fast_gicp_num_threads << ", k="
                << config_.fast_gicp_correspondence_randomness << ", regularization="
                << config_.fast_gicp_regularization_method << ", voxel="
                << config_.fast_vgicp_voxel_resolution_m << " m, voxel_neighbors="
                << config_.fast_vgicp_neighbor_search_method
                << ", cached target covariance; upstream voxel grid rebuilt per align)";
    return description.str();
  }
};
#endif

#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_VGICP_CUDA
void requireCudaSuccess(const cudaError_t error, const char * operation)
{
  if (error == cudaSuccess) {
    return;
  }
  throw std::runtime_error(
          std::string(operation) + " failed: " + cudaGetErrorString(error));
}

fast_gicp::NeighborSearchMethod cudaVoxelNeighborMethod(const std::string & name)
{
  if (name == "DIRECT1") {
    return fast_gicp::NeighborSearchMethod::DIRECT1;
  }
  if (name == "DIRECT7") {
    return fast_gicp::NeighborSearchMethod::DIRECT7;
  }
  if (name == "DIRECT27") {
    return fast_gicp::NeighborSearchMethod::DIRECT27;
  }
  if (name == "DIRECT_RADIUS") {
    return fast_gicp::NeighborSearchMethod::DIRECT_RADIUS;
  }
  throw std::invalid_argument(
          "unknown fast_vgicp_cuda_neighbor_search_method: " + name);
}

fast_gicp::NearestNeighborMethod cudaCovarianceNeighborMethod(const std::string & name)
{
  if (name == "CPU_PARALLEL_KDTREE") {
    return fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE;
  }
  if (name == "GPU_BRUTEFORCE") {
    return fast_gicp::NearestNeighborMethod::GPU_BRUTEFORCE;
  }
  if (name == "GPU_RBF_KERNEL") {
    return fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL;
  }
  throw std::invalid_argument(
          "unknown fast_vgicp_cuda_nearest_neighbor_method: " + name);
}

class FastVgicpCudaBackend final :
  public PclStyleBackend<fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ>>
{
public:
  using RegistrationT = fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ>;
  using Base = PclStyleBackend<RegistrationT>;

  explicit FastVgicpCudaBackend(const RegistrationBackendConfig & config)
  : Base(validateAndReturn(config))
  {
    const auto start = Clock::now();
    registration_->setResolution(config_.fast_vgicp_cuda_voxel_resolution_m);
    registration_->setRegularizationMethod(
      regularizationMethod(config_.fast_gicp_regularization_method));
    registration_->setNeighborSearchMethod(
      cudaVoxelNeighborMethod(config_.fast_vgicp_cuda_neighbor_search_method),
      config_.fast_vgicp_cuda_neighbor_search_radius);
    registration_->setNearestNeighborSearchMethod(
      cudaCovarianceNeighborMethod(config_.fast_vgicp_cuda_nearest_neighbor_method));
    registration_->setKernelWidth(
      config_.fast_vgicp_cuda_kernel_width,
      config_.fast_vgicp_cuda_kernel_max_distance);
    registration_->setRotationEpsilon(config_.fast_gicp_rotation_epsilon_rad);
    requireCudaSuccess(cudaDeviceSynchronize(), "CUDA backend initialization");
    int device = 0;
    requireCudaSuccess(cudaGetDevice(&device), "cudaGetDevice");
    cudaDeviceProp properties{};
    requireCudaSuccess(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
    std::ostringstream description;
    description << "fast_gicp FastVGICPCuda (device=" << device << ": "
                << properties.name << ", cc=" << properties.major << '.'
                << properties.minor << ", voxel="
                << config_.fast_vgicp_cuda_voxel_resolution_m << " m, voxel_neighbors="
                << config_.fast_vgicp_cuda_neighbor_search_method << ", covariance_neighbors="
                << config_.fast_vgicp_cuda_nearest_neighbor_method << ")";
    runtime_description_ = description.str();
    initialization_runtime_ms_ += elapsedMs(start);
  }

  const char * name() const noexcept override
  {
    return "fast_vgicp_cuda";
  }

  bool isCuda() const noexcept override
  {
    return true;
  }

  std::string runtimeDescription() const override
  {
    return runtime_description_;
  }

protected:
  void beforeTargetUpdate() override
  {
    requireCudaSuccess(cudaGetLastError(), "CUDA pre-target status");
  }

  void afterTargetUpdate() override
  {
    requireCudaSuccess(cudaDeviceSynchronize(), "CUDA target upload/covariance");
    requireCudaSuccess(cudaGetLastError(), "CUDA target upload/covariance status");
  }

  void beforeAlign() override
  {
    requireCudaSuccess(cudaGetLastError(), "CUDA pre-align status");
  }

  void afterAlign() override
  {
    requireCudaSuccess(cudaDeviceSynchronize(), "CUDA registration");
    requireCudaSuccess(cudaGetLastError(), "CUDA registration status");
  }

private:
  static RegistrationBackendConfig validateAndReturn(
    const RegistrationBackendConfig & config)
  {
    validateAdditionalBackendConfig(config);
    int device_count = 0;
    requireCudaSuccess(cudaGetDeviceCount(&device_count), "cudaGetDeviceCount");
    if (device_count <= 0) {
      throw std::runtime_error("fast_vgicp_cuda selected but no CUDA device is available");
    }
    return config;
  }

  std::string runtime_description_;
};
#endif

[[maybe_unused]] std::string availableBackendList()
{
  const auto names = availableRegistrationBackends();
  std::ostringstream stream;
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index != 0U) {stream << ", ";}
    stream << names[index];
  }
  return stream.str();
}

}  // namespace

bool isKnownRegistrationBackend(const std::string & method) noexcept
{
  return method == "voxel_surfel" || method == "icp" || method == "ndt" ||
         method == "gicp" || method == "fast_gicp" || method == "fast_vgicp" ||
         method == "fast_vgicp_cuda";
}

std::vector<std::string> availableRegistrationBackends()
{
  std::vector<std::string> methods{"gicp", "voxel_surfel", "icp", "ndt"};
#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_GICP
  methods.emplace_back("fast_gicp");
  methods.emplace_back("fast_vgicp");
#endif
#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_VGICP_CUDA
  methods.emplace_back("fast_vgicp_cuda");
#endif
  return methods;
}

RegistrationBackendBuildInfo registrationBackendBuildInfo()
{
  RegistrationBackendBuildInfo info;
#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_GICP
  info.fast_gicp_compiled = true;
  info.fast_gicp_revision = SLAM_3D_LOCALIZATION_FAST_GICP_REVISION;
#endif
#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_VGICP_CUDA
  info.fast_vgicp_cuda_compiled = true;
  info.cuda_architectures = SLAM_3D_LOCALIZATION_CUDA_ARCHITECTURES;
#endif
  return info;
}

RegistrationOverlap evaluateRegistrationOverlap(
  const RegistrationBackend::CloudT::ConstPtr & source,
  const RegistrationBackend::CloudT::ConstPtr & target,
  const Eigen::Isometry3d & target_T_source, const double max_correspondence_m)
{
  using CloudT = RegistrationBackend::CloudT;
  using PointT = RegistrationBackend::PointT;
  RegistrationOverlap result;
  const Eigen::Matrix3d linear = target_T_source.linear();
  result.transform_determinant = linear.determinant();
  result.transform_orthogonality_error =
    (linear.transpose() * linear - Eigen::Matrix3d::Identity()).norm();
  result.inlier_rmse_m = std::numeric_limits<double>::infinity();
  if (!source || !target || target->empty() || !target_T_source.matrix().allFinite() ||
    max_correspondence_m <= 0.0)
  {
    return result;
  }
  CloudT aligned;
  pcl::transformPointCloud(*source, aligned, target_T_source.matrix().cast<float>());
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);
  const double max_distance_sq = max_correspondence_m * max_correspondence_m;
  int valid_points = 0;
  double squared_error_sum = 0.0;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (const auto & point : aligned.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    ++valid_points;
    if (tree.nearestKSearch(point, 1, indices, squared_distances) <= 0 ||
      squared_distances.front() > max_distance_sq)
    {
      continue;
    }
    ++result.inlier_count;
    squared_error_sum += squared_distances.front();
  }
  if (valid_points > 0) {
    result.inlier_ratio = static_cast<double>(result.inlier_count) /
      static_cast<double>(valid_points);
  }
  if (result.inlier_count > 0) {
    result.inlier_rmse_m = std::sqrt(
      squared_error_sum / static_cast<double>(result.inlier_count));
  }
  return result;
}

RegistrationAcceptance evaluateLioRegistration(
  const RegistrationBackendResult & result, const Eigen::Isometry3d & prediction,
  const RegistrationQualityLimits & limits)
{
  if (!result.accepted) {return {false, result.reason};}
  if (!result.target_T_source.matrix().allFinite() || !std::isfinite(result.fitness)) {
    return {false, "invalid_result"};
  }
  if (result.fitness > limits.fitness_threshold) {return {false, "fitness"};}
  const Eigen::Isometry3d correction = prediction.inverse() * result.target_T_source;
  const double translation = correction.translation().norm();
  const double rotation_deg = rotationAngle(correction.linear()) * 57.295779513082320876;
  if (!std::isfinite(translation) || !std::isfinite(rotation_deg) ||
    translation > limits.max_correction_translation_m)
  {
    return {false, "translation_correction"};
  }
  if (rotation_deg > limits.max_correction_rotation_deg) {
    return {false, "rotation_correction"};
  }
  return {true, result.reason};
}

RegistrationAcceptance evaluateMapRegistration(
  const RegistrationBackendResult & result, const RegistrationOverlap & overlap,
  const bool use_surfel_gates, const Eigen::Isometry3d & prediction,
  const RegistrationQualityLimits & limits)
{
  if (!result.accepted) {
    return {false, use_surfel_gates ? "surfel_" + result.reason : result.reason};
  }
  if (!result.target_T_source.matrix().allFinite() ||
    (use_surfel_gates && (std::abs(overlap.transform_determinant - 1.0) > 0.05 ||
    overlap.transform_orthogonality_error > 0.05)))
  {
    return {false, "invalid_transform"};
  }
  if (!std::isfinite(result.fitness) || result.fitness > limits.fitness_threshold) {
    return {false, "fitness"};
  }
  if (use_surfel_gates &&
    (overlap.inlier_count <= 0 || overlap.inlier_ratio < limits.min_inlier_ratio))
  {
    return {false, "inlier_ratio"};
  }
  if (use_surfel_gates && (!std::isfinite(overlap.inlier_rmse_m) ||
    overlap.inlier_rmse_m > limits.max_inlier_rmse_m))
  {
    return {false, "inlier_rmse"};
  }
  const Eigen::Isometry3d correction = prediction.inverse() * result.target_T_source;
  const double translation = correction.translation().norm();
  const double rotation_deg = rotationAngle(correction.linear()) * 180.0 / M_PI;
  if (!std::isfinite(translation) || !std::isfinite(rotation_deg)) {
    return {false, "invalid_correction"};
  }
  if (translation > limits.max_correction_translation_m) {
    return {false, "translation_correction"};
  }
  if (rotation_deg > limits.max_correction_rotation_deg) {
    return {false, "rotation_correction"};
  }
  return {true, "accepted"};
}

std::unique_ptr<RegistrationBackend> createRegistrationBackend(
  const RegistrationBackendConfig & config)
{
  // Validate optional-backend parameters before either dependency errors or
  // CUDA device access, including use from the LIO node rather than map-only
  // parameter validation. Valid unavailable backends still fail explicitly.
  if (config.method == "fast_gicp" || config.method == "fast_vgicp" ||
    config.method == "fast_vgicp_cuda")
  {
    validateAdditionalBackendConfig(config);
  }
  if (config.method == "voxel_surfel") {
    return std::make_unique<VoxelSurfelBackend>(config);
  }
  if (config.method == "icp") {
    return std::make_unique<PclIcpBackend>(config);
  }
  if (config.method == "ndt") {
    return std::make_unique<PclNdtBackend>(config);
  }
  if (config.method == "gicp") {
    return std::make_unique<PclGicpBackend>(config);
  }
  if (config.method == "fast_gicp") {
#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_GICP
    return std::make_unique<FastGicpBackend>(config);
#else
    throw std::runtime_error(
            "registration_method=fast_gicp was requested, but slam_3d_localization "
            "was built without fast_gicp; available backends: " + availableBackendList());
#endif
  }
  if (config.method == "fast_vgicp_cuda") {
#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_VGICP_CUDA
    return std::make_unique<FastVgicpCudaBackend>(config);
#else
    throw std::runtime_error(
            "registration_method=fast_vgicp_cuda was requested, but the CUDA backend "
            "was not compiled and link-verified; available backends: " +
            availableBackendList());
#endif
  }
  if (config.method == "fast_vgicp") {
#ifdef SLAM_3D_LOCALIZATION_HAS_FAST_GICP
    return std::make_unique<FastVgicpBackend>(config);
#else
    throw std::runtime_error(
            "registration_method=fast_vgicp was requested, but slam_3d_localization "
            "was built without fast_gicp; available backends: " + availableBackendList());
#endif
  }
  throw std::invalid_argument("unknown registration backend: " + config.method);
}

}  // namespace gole::slam_3d_localization
