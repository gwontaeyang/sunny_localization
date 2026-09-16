// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__REGISTRATION_PARAMETERS_HPP_
#define SLAM_3D_LOCALIZATION__REGISTRATION_PARAMETERS_HPP_

#include "slam_3d_localization/registration_backend.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <type_traits>

namespace gole::slam_3d_localization
{

// Keep the default YAML minimal. Only the selected algorithm declares its
// optional controls; reuse parameters already declared by the map localizer.
inline void declareSelectedRegistrationParameters(
  rclcpp::Node & node, RegistrationBackendConfig & config)
{
  const auto parameter = [&node](const std::string & name, const auto & fallback) {
      using T = std::decay_t<decltype(fallback)>;
      return node.has_parameter(name) ? node.get_parameter(name).get_value<T>() :
             node.declare_parameter<T>(name, fallback);
    };
  if (config.method == "ndt") {
    config.ndt_resolution_m = parameter("ndt_resolution_m", config.ndt_resolution_m);
    config.ndt_step_size_m = parameter("ndt_step_size_m", config.ndt_step_size_m);
    config.ndt_outlier_ratio = parameter("ndt_outlier_ratio", config.ndt_outlier_ratio);
  }
  if (config.method == "fast_gicp" || config.method == "fast_vgicp" ||
    config.method == "fast_vgicp_cuda")
  {
    config.fast_gicp_num_threads = parameter(
      "fast_gicp_num_threads", config.fast_gicp_num_threads);
    config.fast_gicp_correspondence_randomness = parameter(
      "fast_gicp_correspondence_randomness", config.fast_gicp_correspondence_randomness);
    config.fast_gicp_regularization_method = parameter(
      "fast_gicp_regularization_method", config.fast_gicp_regularization_method);
    config.fast_gicp_rotation_epsilon_rad = parameter(
      "fast_gicp_rotation_epsilon_rad", config.fast_gicp_rotation_epsilon_rad);
  }
  if (config.method == "fast_vgicp") {
    config.fast_vgicp_voxel_resolution_m = parameter(
      "fast_vgicp_voxel_resolution_m", config.fast_vgicp_voxel_resolution_m);
    config.fast_vgicp_neighbor_search_method = parameter(
      "fast_vgicp_neighbor_search_method", config.fast_vgicp_neighbor_search_method);
  }
  if (config.method == "fast_vgicp_cuda") {
    config.fast_vgicp_cuda_voxel_resolution_m = parameter(
      "fast_vgicp_cuda_voxel_resolution_m", config.fast_vgicp_cuda_voxel_resolution_m);
    config.fast_vgicp_cuda_neighbor_search_method = parameter(
      "fast_vgicp_cuda_neighbor_search_method", config.fast_vgicp_cuda_neighbor_search_method);
    config.fast_vgicp_cuda_neighbor_search_radius = parameter(
      "fast_vgicp_cuda_neighbor_search_radius", config.fast_vgicp_cuda_neighbor_search_radius);
    config.fast_vgicp_cuda_nearest_neighbor_method = parameter(
      "fast_vgicp_cuda_nearest_neighbor_method", config.fast_vgicp_cuda_nearest_neighbor_method);
    config.fast_vgicp_cuda_kernel_width = parameter(
      "fast_vgicp_cuda_kernel_width", config.fast_vgicp_cuda_kernel_width);
    config.fast_vgicp_cuda_kernel_max_distance = parameter(
      "fast_vgicp_cuda_kernel_max_distance", config.fast_vgicp_cuda_kernel_max_distance);
  }
}

}  // namespace gole::slam_3d_localization
#endif  // SLAM_3D_LOCALIZATION__REGISTRATION_PARAMETERS_HPP_
