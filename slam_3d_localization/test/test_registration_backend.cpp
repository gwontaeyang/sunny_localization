// Copyright 2026 Gole Robotics
#include "slam_3d_localization/registration_backend.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace gole::slam_3d_localization
{
namespace
{

RegistrationBackend::CloudT::Ptr gridCloud()
{
  RegistrationBackend::CloudT::Ptr cloud(new RegistrationBackend::CloudT);
  for (int x = 0; x < 6; ++x) {
    for (int y = 0; y < 6; ++y) {
      for (int z = 0; z < 3; ++z) {
        cloud->push_back(
          pcl::PointXYZ(
            static_cast<float>(x) * 0.2F,
            static_cast<float>(y) * 0.2F,
            static_cast<float>(z) * 0.2F));
      }
    }
  }
  return cloud;
}

RegistrationBackend::CloudT::Ptr asymmetricCloud(const std::size_t point_count)
{
  RegistrationBackend::CloudT::Ptr cloud(new RegistrationBackend::CloudT);
  cloud->reserve(point_count);
  std::mt19937 generator(43U);
  std::uniform_real_distribution<float> xy(-8.0F, 8.0F);
  std::normal_distribution<float> noise(0.0F, 0.01F);
  for (std::size_t index = 0; index < point_count; ++index) {
    const float x = xy(generator);
    const float y = xy(generator);
    const float z = 0.12F * x + 0.07F * y +
      0.35F * std::sin(0.7F * x) + 0.2F * std::cos(0.5F * y) + noise(generator);
    cloud->push_back(pcl::PointXYZ{x, y, z});
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1U;
  cloud->is_dense = true;
  return cloud;
}

RegistrationBackend::CloudT::Ptr transformedCloud(
  const RegistrationBackend::CloudT & source,
  const Eigen::Isometry3d & target_T_source)
{
  RegistrationBackend::CloudT::Ptr target(new RegistrationBackend::CloudT);
  target->reserve(source.size());
  for (const auto & point : source) {
    const Eigen::Vector3d transformed =
      target_T_source * point.getVector3fMap().cast<double>();
    target->push_back(
      pcl::PointXYZ{
        static_cast<float>(transformed.x()),
        static_cast<float>(transformed.y()),
        static_cast<float>(transformed.z())});
  }
  target->width = static_cast<std::uint32_t>(target->size());
  target->height = 1U;
  target->is_dense = true;
  return target;
}

double rotationErrorRadians(
  const Eigen::Isometry3d & estimate, const Eigen::Isometry3d & expected)
{
  const Eigen::Matrix3d delta = estimate.linear().transpose() * expected.linear();
  const double cosine = std::clamp((delta.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

RegistrationBackend::CloudT::Ptr floorCloud()
{
  RegistrationBackend::CloudT::Ptr cloud(new RegistrationBackend::CloudT);
  for (int x = -20; x <= 20; ++x) {
    for (int y = -20; y <= 20; ++y) {
      cloud->push_back(pcl::PointXYZ(0.1F * x, 0.1F * y, 0.0F));
    }
  }
  return cloud;
}

TEST(RegistrationBackend, NativeSurfelPreservesDirectSolverAndReusesPreparedTarget)
{
  RegistrationBackendConfig config;
  config.method = "voxel_surfel";
  config.surfel.max_iterations = 1;
  auto backend = createRegistrationBackend(config);
  const auto target = floorCloud();
  auto shift = Eigen::Isometry3d::Identity();
  shift.translation().z() = 0.08;
  const auto source = transformedCloud(*target, shift);
  const VoxelSurfelRegistration native(config.surfel);
  const auto direct = native.align(*source, *target, Eigen::Isometry3d::Identity());
  const auto first = backend->align(source, target, 1U, Eigen::Isometry3d::Identity());
  const auto second = backend->align(source, target, 1U, Eigen::Isometry3d::Identity());
  EXPECT_STREQ(backend->name(), "voxel_surfel");
  EXPECT_FALSE(backend->isCuda());
  ASSERT_TRUE(direct.accepted) << direct.reason;
  EXPECT_FALSE(direct.converged);
  EXPECT_TRUE(first.accepted);
  EXPECT_FALSE(first.converged);
  EXPECT_EQ(first.reason, direct.reason);
  EXPECT_TRUE(first.target_T_source.matrix().isApprox(direct.target_T_source.matrix(), 0.0));
  EXPECT_TRUE(second.target_T_source.matrix().isApprox(direct.target_T_source.matrix(), 0.0));
  EXPECT_DOUBLE_EQ(first.rmse_m, direct.rmse);
  EXPECT_DOUBLE_EQ(first.fitness, direct.rmse * direct.rmse);
  EXPECT_TRUE(first.first_call);
  EXPECT_FALSE(first.target_reused);
  EXPECT_TRUE(second.target_reused);
  EXPECT_FALSE(second.first_call);
  EXPECT_EQ(second.target_upload_count, 1U);
  EXPECT_EQ(second.target_revision, 1U);
  EXPECT_DOUBLE_EQ(second.target_update_runtime_ms, 0.0);
  EXPECT_TRUE(second.surfel.physical_hessian.isApprox(direct.physical_hessian, 0.0));
}

TEST(RegistrationBackend, NativeSurfelRejectsStaleIdentityAndRebuildsChangedTarget)
{
  RegistrationBackendConfig config;
  config.method = "voxel_surfel";
  auto backend = createRegistrationBackend(config);
  const auto source = floorCloud();
  const auto first_target = floorCloud();
  auto shift = Eigen::Isometry3d::Identity();
  shift.translation().z() = 0.12;
  const auto second_target = transformedCloud(*first_target, shift);
  backend->align(source, first_target, 7U, Eigen::Isometry3d::Identity());
  EXPECT_THROW(backend->align(source, second_target, 7U, Eigen::Isometry3d::Identity()), std::logic_error);
  EXPECT_THROW(backend->align(source, first_target, 8U, Eigen::Isometry3d::Identity()), std::logic_error);
  const auto rebuilt = backend->align(source, second_target, 8U, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(rebuilt.accepted) << rebuilt.reason;
  EXPECT_NEAR(rebuilt.target_T_source.translation().z(), 0.12, 1.0e-6);
  EXPECT_EQ(rebuilt.target_upload_count, 2U);
  EXPECT_FALSE(rebuilt.target_reused);
  EXPECT_TRUE(backend->align(source, second_target, 8U, Eigen::Isometry3d::Identity()).target_reused);
}

TEST(RegistrationBackend, CommonInputValidationDoesNotPoisonTheNativeCache)
{
  RegistrationBackendConfig config;
  config.method = "voxel_surfel";
  auto backend = createRegistrationBackend(config);
  const auto cloud = floorCloud();
  auto invalid = Eigen::Isometry3d::Identity();
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(backend->align(nullptr, cloud, 1U, Eigen::Isometry3d::Identity()), std::invalid_argument);
  EXPECT_THROW(backend->align(cloud, cloud, 1U, invalid), std::invalid_argument);
  const auto first = backend->align(cloud, cloud, 1U, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(first.first_call);
  EXPECT_EQ(first.target_upload_count, 1U);
}

TEST(RegistrationQuality, EuclideanOverlapKeepsFiniteSourceDenominator)
{
  RegistrationBackend::CloudT::Ptr source(new RegistrationBackend::CloudT);
  RegistrationBackend::CloudT::Ptr target(new RegistrationBackend::CloudT);
  target->push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  source->push_back(pcl::PointXYZ(0.1F, 0.0F, 0.0F));
  source->push_back(pcl::PointXYZ(3.0F, 0.0F, 0.0F));
  source->push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));
  source->is_dense = false;
  const auto overlap = evaluateRegistrationOverlap(source, target, Eigen::Isometry3d::Identity(), 0.5);
  EXPECT_EQ(overlap.inlier_count, 1);
  EXPECT_DOUBLE_EQ(overlap.inlier_ratio, 0.5);
  EXPECT_NEAR(overlap.inlier_rmse_m, 0.1, 1.0e-7);
  EXPECT_DOUBLE_EQ(overlap.transform_determinant, 1.0);
  EXPECT_DOUBLE_EQ(overlap.transform_orthogonality_error, 0.0);
}

TEST(RegistrationQuality, LioPreservesAcceptedIterateAndRejectionReasons)
{
  RegistrationBackendResult result;
  result.accepted = true;
  result.converged = false;
  result.reason = "accepted_max_iterations";
  RegistrationQualityLimits limits;
  EXPECT_TRUE(evaluateLioRegistration(result, Eigen::Isometry3d::Identity(), limits).accepted);
  result.fitness = 2.0;
  EXPECT_EQ(evaluateLioRegistration(result, Eigen::Isometry3d::Identity(), limits).reason, "fitness");
  result.fitness = std::numeric_limits<double>::infinity();
  EXPECT_EQ(evaluateLioRegistration(result, Eigen::Isometry3d::Identity(), limits).reason, "invalid_result");
  result.fitness = 0.0;
  result.target_T_source.translation().x() = 3.0;
  EXPECT_EQ(evaluateLioRegistration(result, Eigen::Isometry3d::Identity(), limits).reason, "translation_correction");
  result.target_T_source = Eigen::Isometry3d::Identity();
  result.target_T_source.linear() = Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_EQ(evaluateLioRegistration(result, Eigen::Isometry3d::Identity(), limits).reason, "rotation_correction");
  result.accepted = false;
  result.reason = "no_planar_surfels";
  EXPECT_EQ(evaluateLioRegistration(result, Eigen::Isometry3d::Identity(), limits).reason, "no_planar_surfels");
}

TEST(RegistrationQuality, MapOverlapGatesRemainNativeOnly)
{
  RegistrationBackendResult result;
  result.accepted = true;
  result.converged = false;
  RegistrationOverlap overlap;
  RegistrationQualityLimits limits;
  EXPECT_TRUE(evaluateMapRegistration(result, overlap, false, Eigen::Isometry3d::Identity(), limits).accepted);
  EXPECT_EQ(evaluateMapRegistration(result, overlap, true, Eigen::Isometry3d::Identity(), limits).reason, "inlier_ratio");
  overlap.inlier_count = 50;
  overlap.inlier_ratio = 0.5;
  overlap.inlier_rmse_m = 2.0;
  EXPECT_EQ(evaluateMapRegistration(result, overlap, true, Eigen::Isometry3d::Identity(), limits).reason, "inlier_rmse");
  overlap.inlier_rmse_m = 0.1;
  EXPECT_TRUE(evaluateMapRegistration(result, overlap, true, Eigen::Isometry3d::Identity(), limits).accepted);
  result.accepted = false;
  result.reason = "quality_rmse";
  EXPECT_EQ(evaluateMapRegistration(result, overlap, true, Eigen::Isometry3d::Identity(), limits).reason, "surfel_quality_rmse");
  EXPECT_EQ(evaluateMapRegistration(result, overlap, false, Eigen::Isometry3d::Identity(), limits).reason, "quality_rmse");
}

TEST(RegistrationBackend, PclGicpPersistsAndReusesAnUnchangedTarget)
{
  RegistrationBackendConfig config;
  config.method = "gicp";
  config.max_iterations = 10;
  auto backend = createRegistrationBackend(config);
  auto source = gridCloud();
  auto target = gridCloud();

  const auto first = backend->align(source, target, 1, Eigen::Isometry3d::Identity());
  const auto second = backend->align(source, target, 1, Eigen::Isometry3d::Identity());

  EXPECT_STREQ(backend->name(), "gicp");
  EXPECT_TRUE(first.first_call);
  EXPECT_FALSE(first.target_reused);
  EXPECT_EQ(first.target_upload_count, 1U);
  EXPECT_FALSE(second.first_call);
  EXPECT_TRUE(second.target_reused);
  EXPECT_EQ(second.target_upload_count, 1U);
  EXPECT_EQ(second.target_revision, 1U);
}

TEST(RegistrationBackend, RejectsAChangedTargetWithoutRevisionChange)
{
  RegistrationBackendConfig config;
  auto backend = createRegistrationBackend(config);
  auto source = gridCloud();
  auto first_target = gridCloud();
  auto second_target = gridCloud();
  backend->align(source, first_target, 7, Eigen::Isometry3d::Identity());

  EXPECT_THROW(
    backend->align(source, second_target, 7, Eigen::Isometry3d::Identity()),
    std::logic_error);
}

TEST(RegistrationBackend, RejectsARevisionChangeWithoutChangedTarget)
{
  RegistrationBackendConfig config;
  auto backend = createRegistrationBackend(config);
  auto source = gridCloud();
  auto target = gridCloud();
  backend->align(source, target, 7, Eigen::Isometry3d::Identity());

  EXPECT_THROW(
    backend->align(source, target, 8, Eigen::Isometry3d::Identity()),
    std::logic_error);
}

TEST(RegistrationBackend, UnknownBackendNeverFallsBackToPcl)
{
  RegistrationBackendConfig config;
  config.method = "not_a_backend";
  EXPECT_THROW(createRegistrationBackend(config), std::invalid_argument);
}

TEST(RegistrationBackend, UnavailableOptionalBackendFailsExplicitly)
{
  const auto available = availableRegistrationBackends();
  RegistrationBackendConfig config;
  if (std::find(available.begin(), available.end(), "fast_gicp") == available.end()) {
    config.method = "fast_gicp";
    EXPECT_THROW(createRegistrationBackend(config), std::runtime_error);
  }
  if (std::find(available.begin(), available.end(), "fast_vgicp_cuda") == available.end()) {
    config.method = "fast_vgicp_cuda";
    EXPECT_THROW(createRegistrationBackend(config), std::runtime_error);
  }
}

TEST(RegistrationBackend, FastGicpRunsWhenCompiled)
{
  const auto available = availableRegistrationBackends();
  if (std::find(available.begin(), available.end(), "fast_gicp") == available.end()) {
    GTEST_SKIP() << "fast_gicp was intentionally not compiled";
  }
  RegistrationBackendConfig config;
  config.method = "fast_gicp";
  config.max_iterations = 10;
  config.fast_gicp_num_threads = 2;
  auto backend = createRegistrationBackend(config);
  const auto source = gridCloud();
  const auto target = gridCloud();
  const auto result = backend->align(
    source, target, 1, Eigen::Isometry3d::Identity());

  EXPECT_STREQ(backend->name(), "fast_gicp");
  EXPECT_FALSE(backend->isCuda());
  EXPECT_TRUE(result.target_T_source.matrix().allFinite());
  EXPECT_EQ(result.target_upload_count, 1U);
}

TEST(RegistrationBackend, FastVgicpCudaTracksKnownTransformsAcrossTargetRebuild)
{
  const auto available = availableRegistrationBackends();
  if (std::find(available.begin(), available.end(), "fast_vgicp_cuda") ==
    available.end())
  {
    GTEST_SKIP() << "fast_vgicp_cuda was intentionally not compiled";
  }
  RegistrationBackendConfig config;
  config.method = "fast_vgicp_cuda";
  config.max_iterations = 40;
  config.transformation_epsilon = 1.0e-4;
  config.fast_vgicp_cuda_voxel_resolution_m = 0.35;
  config.fast_vgicp_cuda_neighbor_search_method = "DIRECT1";
  config.fast_vgicp_cuda_nearest_neighbor_method = "GPU_BRUTEFORCE";
  auto backend = createRegistrationBackend(config);

  const auto source = asymmetricCloud(1800U);
  Eigen::Isometry3d first_truth = Eigen::Isometry3d::Identity();
  first_truth.linear() =
    (Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.015, Eigen::Vector3d::UnitY())).toRotationMatrix();
  first_truth.translation() = Eigen::Vector3d(0.30, -0.15, 0.05);
  const auto first_target = transformedCloud(*source, first_truth);
  const auto first = backend->align(
    source, first_target, 3, Eigen::Isometry3d::Identity());

  Eigen::Isometry3d second_truth = Eigen::Isometry3d::Identity();
  second_truth.linear() =
    (Eigen::AngleAxisd(-0.035, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.012, Eigen::Vector3d::UnitY())).toRotationMatrix();
  second_truth.translation() = Eigen::Vector3d(-0.22, 0.20, -0.04);
  const auto second_target = transformedCloud(*source, second_truth);
  const auto second = backend->align(
    source, second_target, 4, Eigen::Isometry3d::Identity());
  const auto repeated = backend->align(
    source, second_target, 4, Eigen::Isometry3d::Identity());

  EXPECT_STREQ(backend->name(), "fast_vgicp_cuda");
  EXPECT_TRUE(backend->isCuda());
  EXPECT_TRUE(first.target_T_source.matrix().allFinite());
  EXPECT_LT(
    (first.target_T_source.translation() - first_truth.translation()).norm(), 0.05);
  EXPECT_LT(rotationErrorRadians(first.target_T_source, first_truth), 0.03);
  EXPECT_EQ(first.target_upload_count, 1U);
  EXPECT_FALSE(second.target_reused);
  EXPECT_EQ(second.target_revision, 4U);
  EXPECT_EQ(second.target_upload_count, 2U);
  EXPECT_TRUE(second.target_T_source.matrix().allFinite());
  EXPECT_LT(
    (second.target_T_source.translation() - second_truth.translation()).norm(), 0.05);
  EXPECT_LT(rotationErrorRadians(second.target_T_source, second_truth), 0.03);
  EXPECT_TRUE(repeated.target_reused);
  EXPECT_EQ(repeated.target_upload_count, 2U);
}

}  // namespace
}  // namespace gole::slam_3d_localization
