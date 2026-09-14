// Copyright 2026 Gole Robotics
#include "slam_3d_localization/registration_backend.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
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
