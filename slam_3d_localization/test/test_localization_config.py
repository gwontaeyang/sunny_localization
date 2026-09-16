import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def pipeline_parameters():
    spec = importlib.util.spec_from_file_location(
        "lio_profile", PACKAGE_ROOT / "launch" / "lio_profile.py"
    )
    profile = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(profile)
    return profile.get_lio_pipeline_parameters()


def profile_contract(filename):
    """Read the frozen effective contract plus the visible profile overrides.

    test_minimal_config.cpp separately constructs the actual C++ nodes with
    both files and compares every declared parameter, including omitted keys.
    These checks retain the readable safety/interface assertions below.
    """
    legacy = yaml.safe_load(
        (PACKAGE_ROOT / "test" / "fixtures" / "legacy_config" / filename).read_text()
    )
    current = yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())
    assert current.keys() <= legacy.keys()
    if filename == "slam_3d_lio_gangnam.yaml":
        for node, config in pipeline_parameters().items():
            legacy[node]["ros__parameters"].update(config["ros__parameters"])
        # One intentional change from the frozen profile: the omitted time
        # mode now uses the actual C++ auto default, verified by the node test.
        legacy["slam_3d_lio_odometry"]["ros__parameters"]["point_time_mode"] = "auto"
    for node, config in current.items():
        legacy[node]["ros__parameters"].update(config["ros__parameters"])
    return legacy


def test_profiles_keep_only_operational_and_nondefault_overrides():
    limits = {
        "slam_3d_localization.yaml": {"slam_3d_localizer": 1},
        "slam_3d_localization_avs.yaml": {"slam_3d_localizer": 4},
        "slam_3d_lio_gangnam.yaml": {
            "slam_3d_lio_odometry": 1,
        },
        "slam_3d_localization_indoor_xt32_no_wheel_replay.yaml": {
            "slam_3d_localizer": 44,
        },
    }
    for filename, node_limits in limits.items():
        current = yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())
        legacy = yaml.safe_load(
            (PACKAGE_ROOT / "test" / "fixtures" / "legacy_config" / filename).read_text()
        )
        assert current.keys() == node_limits.keys()
        for node, maximum in node_limits.items():
            parameters = current[node]["ros__parameters"]
            assert len(parameters) <= maximum
            # This cleanup does not retune the visible overrides.
            assert all(
                value == legacy[node]["ros__parameters"][key]
                for key, value in parameters.items()
            )
    lio = profile_contract("slam_3d_lio_gangnam.yaml")[
        "slam_3d_lio_odometry"
    ]["ros__parameters"]
    assert lio["imu_enabled"] is True
    assert lio["imu_bias_estimation_enabled"] is False
    assert lio["wheel_correction_enabled"] is False


def test_default_execution_configs_contain_only_nondefault_sensor_values_and_map():
    lio = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "slam_3d_lio_gangnam.yaml").read_text()
    )["slam_3d_lio_odometry"]["ros__parameters"]
    assert set(lio) == {"imu_position_in_base_m"}
    assert lio["imu_position_in_base_m"] == [0.0, 0.0, 1.20984]
    generic = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "slam_3d_localization.yaml").read_text()
    )["slam_3d_localizer"]["ros__parameters"]
    assert generic == {"map_path": ""}


def test_pipeline_export_uses_the_same_method_as_launch(tmp_path):
    output = tmp_path / "pipeline.yaml"
    subprocess.run(
        [sys.executable, str(PACKAGE_ROOT / "launch" / "lio_profile.py"),
         "--output", str(output)],
        check=True,
    )
    exported = yaml.safe_load(output.read_text())
    assert exported == pipeline_parameters()
    assert exported == json.loads(output.read_text())
    lio = exported["slam_3d_lio_odometry"]["ros__parameters"]
    localizer = exported["slam_3d_localizer"]["ros__parameters"]
    assert lio["odom_topic"] == localizer["odom_topic"]
    assert lio["scan_odom_topic"] == localizer["registration_odom_topic"]
    assert lio["deskewed_cloud_topic"] == localizer["cloud_topic"]
    lio["odom_topic"] = "/changed_by_caller"
    assert pipeline_parameters()["slam_3d_lio_odometry"][
        "ros__parameters"
    ]["odom_topic"] == "/localization_3d/lio/odom"


def test_default_config_has_safe_registration_gates():
    data = profile_contract("slam_3d_localization.yaml")
    params = data["slam_3d_localizer"]["ros__parameters"]

    assert params["map_path"] == ""
    frames = (params["map_frame"], params["odom_frame"], params["base_frame"])
    assert all(frames)
    assert len(set(frames)) == 3
    assert params["registration_method"] in {
        "gicp",
        "voxel_surfel",
        "fast_gicp",
        "fast_vgicp_cuda",
    }
    assert params["registration_method"] == "gicp"
    assert params["input_scan_preprocessed"] is False
    assert params["map_already_voxelized"] is False
    assert params["crop_box_enabled"] is False
    assert params["crop_min_x"] < params["crop_max_x"]
    assert params["crop_min_y"] < params["crop_max_y"]
    assert params["crop_min_z"] < params["crop_max_z"]
    assert params["scan_voxel_leaf_m"] > 0.0
    assert params["map_voxel_leaf_m"] > 0.0
    assert params["local_map_radius_m"] > params["max_correspondence_m"]
    assert 0.0 <= params["local_map_rebuild_distance_m"] < params["local_map_radius_m"]
    assert params["fast_gicp_num_threads"] >= 0
    assert params["fast_gicp_correspondence_randomness"] >= 3
    assert params["fast_gicp_regularization_method"] == "PLANE"
    assert params["fast_gicp_rotation_epsilon_rad"] > 0.0
    assert params["fast_vgicp_cuda_voxel_resolution_m"] > 0.0
    assert params["fast_vgicp_cuda_neighbor_search_method"] in {
        "DIRECT1",
        "DIRECT7",
        "DIRECT27",
        "DIRECT_RADIUS",
    }
    assert params["fast_vgicp_cuda_neighbor_search_radius"] > 0.0
    assert params["fast_vgicp_cuda_nearest_neighbor_method"] in {
        "CPU_PARALLEL_KDTREE",
        "GPU_BRUTEFORCE",
        "GPU_RBF_KERNEL",
    }
    assert params["fast_vgicp_cuda_kernel_width"] > 0.0
    assert params["fast_vgicp_cuda_kernel_max_distance"] > 0.0
    assert isinstance(params["cloud_subscription_reliable"], bool)
    assert params["cloud_subscription_depth"] > 0
    assert params["require_synced_odom_for_update"] is False
    assert params["registration_odom_topic"] == ""
    assert params["require_exact_odom_for_update"] is False
    assert params["odom_sync_pending_cloud_limit"] > 0
    assert params["publish_tf"] is True
    assert params["publish_identity_map_to_odom_before_localization"] is False
    assert params["publish_global_map"] is True
    assert params["global_map_topic"] == "/localization_3d/global_map"
    assert params["fitness_threshold"] > 0.0
    assert 0.0 < params["min_registration_inlier_ratio"] <= 1.0
    assert params["max_registration_inlier_rmse_m"] > 0.0
    assert 0.0 < params["max_correction_rotation_deg"] < 180.0
    assert params["update_min_translation_m"] >= 0.0
    assert 0.0 <= params["update_min_rotation_deg"] < 180.0
    assert params["use_constant_velocity_prediction"] is False
    assert params["constant_velocity_translation_limit_m"] == 0.30
    assert params["constant_velocity_rotation_limit_deg"] == 10.0
    assert params["constant_velocity_max_gap_s"] > 0.0
    assert params["max_cloud_age_s"] == 0.0
    assert params["cloud_timestamp_reset_threshold_s"] > 0.0
    assert params["odom_buffer_duration_s"] > 0.0
    assert (
        0.0
        < params["odom_max_interpolation_gap_s"]
        <= params["odom_buffer_duration_s"]
    )
    assert (
        0.0
        <= params["odom_max_nearest_age_s"]
        <= params["odom_buffer_duration_s"]
    )
    assert params["odom_timestamp_reset_threshold_s"] > 0.0
    assert params["min_scan_points"] > 0
    assert params["min_local_map_points"] > params["min_scan_points"]
    assert params["surfel_voxel_size_m"] > 0.0
    assert params["surfel_min_points_per_voxel"] >= 3
    assert 0.0 <= params["surfel_max_planarity_ratio"] < 1.0
    assert params["surfel_min_tangent_variance"] >= 0.0
    assert params["surfel_max_plane_distance_m"] > 0.0
    assert params["surfel_max_center_distance_m"] > 0.0
    assert params["surfel_min_correspondences"] > 0
    assert 0.0 < params["surfel_min_correspondence_ratio"] <= 1.0
    assert params["surfel_huber_delta_m"] > 0.0
    assert params["surfel_max_iterations"] > 0
    assert params["surfel_max_backtracking_steps"] >= 0
    assert 0.0 < params["surfel_backtracking_factor"] < 1.0
    assert params["surfel_rotation_convergence_rad"] >= 0.0
    assert params["surfel_translation_convergence_m"] >= 0.0
    assert params["surfel_relative_cost_convergence"] >= 0.0
    assert params["surfel_max_rotation_step_rad"] > 0.0
    assert params["surfel_max_translation_step_m"] > 0.0
    assert params["surfel_rotation_scale_m_per_rad"] > 0.0
    assert 0.0 < params["surfel_relative_eigen_threshold"] < 1.0
    assert 1 <= params["surfel_min_observable_rank"] <= 6
    assert params["surfel_max_quality_rmse_m"] > 0.0
    assert params["surfel_max_quality_p95_m"] > 0.0


def test_indoor_xt32_no_wheel_replay_profile_matches_verified_contract():
    data = profile_contract("slam_3d_localization_indoor_xt32_no_wheel_replay.yaml")
    params = data["slam_3d_localizer"]["ros__parameters"]

    # Runtime-only values are supplied by launch, while every algorithm and
    # interface choice below is the effective Sequence20 CV-on contract.
    assert "use_sim_time" not in params
    assert params["map_path"] == ""
    assert params["cloud_topic"] == "/slam_3d/frontend/deskewed_points"
    assert params["odom_topic"] == "/unused/wheel_odom_disabled"
    assert params["registration_method"] == "voxel_surfel"
    assert params["input_scan_preprocessed"] is True
    assert params["map_already_voxelized"] is True
    assert params["crop_box_enabled"] is False
    assert params["cloud_subscription_reliable"] is True
    assert params["cloud_subscription_depth"] == 128
    assert params["publish_tf"] is False
    assert params["publish_local_map"] is False

    assert params["use_constant_velocity_prediction"] is True
    assert params["constant_velocity_translation_limit_m"] == 0.30
    assert params["constant_velocity_rotation_limit_deg"] == 10.0
    assert params["constant_velocity_max_gap_s"] == 0.50
    assert params["cloud_timestamp_reset_threshold_s"] == 1.0

    assert params["max_correspondence_m"] == 1.2
    assert params["min_registration_inlier_ratio"] == 0.25
    assert params["max_registration_inlier_rmse_m"] == 0.45
    assert params["max_correction_translation_m"] == 1.5
    assert params["max_correction_rotation_deg"] == 30.0
    assert params["update_min_translation_m"] == 0.0
    assert params["update_min_rotation_deg"] == 0.0
    assert params["surfel_min_correspondences"] == 150
    assert params["surfel_min_correspondence_ratio"] == 0.08
    assert params["surfel_max_iterations"] == 12
    assert "/odometry/filtered" not in str(params)

    assert {
        key: params[key]
        for key in (
            "surfel_voxel_size_m",
            "surfel_min_points_per_voxel",
            "surfel_max_planarity_ratio",
            "surfel_min_tangent_variance",
            "surfel_max_plane_distance_m",
            "surfel_max_center_distance_m",
            "surfel_min_correspondences",
            "surfel_min_correspondence_ratio",
            "surfel_huber_delta_m",
            "surfel_max_iterations",
            "surfel_max_backtracking_steps",
            "surfel_backtracking_factor",
            "surfel_rotation_convergence_rad",
            "surfel_translation_convergence_m",
            "surfel_relative_cost_convergence",
            "surfel_max_rotation_step_rad",
            "surfel_max_translation_step_m",
            "surfel_rotation_scale_m_per_rad",
            "surfel_relative_eigen_threshold",
            "surfel_min_observable_rank",
            "surfel_max_quality_rmse_m",
            "surfel_max_quality_p95_m",
        )
    } == {
        "surfel_voxel_size_m": 0.6,
        "surfel_min_points_per_voxel": 5,
        "surfel_max_planarity_ratio": 0.25,
        "surfel_min_tangent_variance": 0.0001,
        "surfel_max_plane_distance_m": 0.35,
        "surfel_max_center_distance_m": 1.2,
        "surfel_min_correspondences": 150,
        "surfel_min_correspondence_ratio": 0.08,
        "surfel_huber_delta_m": 0.10,
        "surfel_max_iterations": 12,
        "surfel_max_backtracking_steps": 5,
        "surfel_backtracking_factor": 0.5,
        "surfel_rotation_convergence_rad": 0.0002,
        "surfel_translation_convergence_m": 0.0002,
        "surfel_relative_cost_convergence": 0.00001,
        "surfel_max_rotation_step_rad": 0.20,
        "surfel_max_translation_step_m": 0.50,
        "surfel_rotation_scale_m_per_rad": 1.0,
        "surfel_relative_eigen_threshold": 0.001,
        "surfel_min_observable_rank": 3,
        "surfel_max_quality_rmse_m": 0.30,
        "surfel_max_quality_p95_m": 0.50,
    }


def test_gangnam_profile_defaults_to_raw_imu_without_bias_learning_or_wheel():
    data = profile_contract("slam_3d_lio_gangnam.yaml")
    lio = data["slam_3d_lio_odometry"]["ros__parameters"]
    localizer = data["slam_3d_localizer"]["ros__parameters"]

    assert lio["cloud_topic"] == "/hesai/center/lidar_points"
    assert lio["imu_enabled"] is True
    assert lio["imu_bias_estimation_enabled"] is False
    assert lio["imu_topic"] == "/imu/data"
    assert lio["scan_odom_topic"] == "/localization_3d/lio/scan_odom"
    assert lio["imu_frame"] == "cv7_link"
    assert lio["point_time_field"] == "timestamp"
    assert lio["point_time_mode"] == "auto"
    assert lio["imu_to_base_rpy_deg"] == [0.0, 0.0, 0.0]
    assert lio["imu_position_in_base_m"] == [0.0, 0.0, 1.20984]
    assert lio["wheel_correction_enabled"] is False
    assert lio["publish_tf"] is True

    assert localizer["cloud_topic"] == lio["deskewed_cloud_topic"]
    assert localizer["odom_topic"] == lio["odom_topic"]
    assert localizer["registration_odom_topic"] == lio["scan_odom_topic"]
    assert localizer["require_synced_odom_for_update"] is True
    assert localizer["require_exact_odom_for_update"] is True
    assert localizer["registration_method"] == "gicp"
    assert localizer["use_constant_velocity_prediction"] is False
    assert localizer["publish_tf"] is True
    assert localizer["publish_identity_map_to_odom_before_localization"] is False
    assert localizer["input_scan_preprocessed"] is True
    assert localizer["map_path"] == ""
    assert localizer["odom_frame"] == lio["odom_frame"]
    assert localizer["base_frame"] == lio["base_frame"]


def test_lio_launch_starts_only_lio_and_prior_map_localization():
    launch_text = (
        PACKAGE_ROOT / "launch" / "slam_3d_lio_localization.launch.py"
    ).read_text()

    assert 'executable="slam_3d_lio_odometry_node"' in launch_text
    assert 'executable="slam_3d_localizer_node"' in launch_text
    assert launch_text.count("Node(") == 2
    assert '"slam_3d_lio_gangnam.yaml"' in launch_text
    assert '"gangnam_floor_person_filtered_nav_classified_rgb.pcd"' in launch_text
    assert "odom_estimation" not in launch_text
    assert "robot_localization" not in launch_text
    assert "IncludeLaunchDescription" not in launch_text
    assert 'profile.get_lio_pipeline_parameters()' in launch_text
    for node in ("slam_3d_lio_odometry", "slam_3d_localizer"):
        # C++ tests verify the effective precedence on actual nodes; this check
        # connects their shared default source to the public launch wiring.
        start = launch_text.index(f'pipeline["{node}"]["ros__parameters"]')
        params_file = launch_text.index("params_file,", start)
        runtime = launch_text.index('"use_sim_time": use_sim_time', params_file)
        assert start < params_file < runtime


def test_launch_defaults_to_classified_rgb_map_and_does_not_start_hardware():
    launch_text = (
        PACKAGE_ROOT / "launch" / "slam_3d_localization.launch.py"
    ).read_text()

    assert '"gangnam_floor_person_filtered_nav_classified_rgb.pcd"' in launch_text
    assert 'EnvironmentVariable("HOME")' in launch_text
    assert '"map_path": LaunchConfiguration("map_path")' in launch_text
    assert "hesai" not in launch_text.lower()


def test_node_uses_one_executor_thread_and_preserves_the_odom_callback_group():
    node_text = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer_node.cpp"
    ).read_text()
    header = (
        PACKAGE_ROOT
        / "include"
        / "slam_3d_localization"
        / "slam_3d_localizer.hpp"
    ).read_text()
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()

    assert "rclcpp::executors::StaticSingleThreadedExecutor" in node_text
    assert "use_clock_thread(false)" in node_text
    assert "MultiThreadedExecutor" not in node_text
    assert "odom_callback_group_" in header
    assert "CallbackGroupType::MutuallyExclusive" in implementation
    assert (
        "odom_subscription_options.callback_group = odom_callback_group_;"
        in implementation
    )
    assert "std::chrono::milliseconds(20)" in implementation
    assert "pending_cloud_timer_->cancel();" in implementation
    assert "pending_cloud_timer_->reset();" in implementation


def test_avs_profile_uses_robot_topics_and_classified_map():
    data = profile_contract("slam_3d_localization_avs.yaml")
    params = data["slam_3d_localizer"]["ros__parameters"]

    assert params["map_path"].endswith(
        "/gangnam_floor_person_filtered_nav_classified_rgb.pcd"
    )
    assert params["cloud_topic"] == "/lidar/points_merged"
    assert params["odom_topic"] == "/odom"
    assert params["global_map_topic"] == "/localization_3d/global_map"
    assert params["publish_global_map"] is True
    assert params["publish_identity_map_to_odom_before_localization"] is True


def test_avs_rviz_launch_starts_only_localization_and_visualization():
    launch_text = (
        PACKAGE_ROOT / "launch" / "slam_3d_localization_avs_rviz.launch.py"
    ).read_text()

    assert 'package="slam_3d_localization"' in launch_text
    assert 'package="rviz2"' in launch_text
    assert '"slam_3d_localization_avs.yaml"' in launch_text
    assert '"slam_3d_localization_avs.rviz"' in launch_text
    assert "hesai" not in launch_text.lower()


def test_amcl_compatible_nomotion_update_contract_is_present():
    header = (
        PACKAGE_ROOT
        / "include"
        / "slam_3d_localization"
        / "slam_3d_localizer.hpp"
    ).read_text()
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()

    assert "std_srvs/srv/empty.hpp" in header
    assert '"~/request_nomotion_update"' in implementation
    assert "motion_below_update_threshold" in implementation
    assert "nomotion_update_accepted" in implementation
    assert "initial_pose_update_accepted" in implementation


def test_forced_update_requests_remain_pending_until_registration_acceptance():
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()

    registration = implementation.index(
        "RegistrationOutcome registration = registerScanToMap("
    )
    rejected_branch = implementation.index("if (!accepted) {", registration)
    rejected_return = implementation.index("return;", rejected_branch)
    clear_initial = implementation.index(
        "initial_pose_update_pending_ = false;", registration
    )
    clear_nomotion = implementation.index(
        "nomotion_update_pending_ = false;", registration
    )
    publish_result = implementation.index("publishResult(", clear_initial)

    assert rejected_branch < rejected_return < clear_initial < publish_result
    assert rejected_branch < rejected_return < clear_nomotion < publish_result


def test_constant_velocity_prediction_is_opt_in_and_auditable():
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()

    assert '"use_constant_velocity_prediction"' in implementation
    assert "constant_velocity_predictor_.predict(" in implementation
    assert "constant_velocity_predictor_.recordAccepted(" in implementation
    assert implementation.count("constant_velocity_predictor_.reset();") >= 2
    assert "constant_velocity_predictor_.rebaseAccepted(" in implementation
    assert "cloud_timestamp_reset_count_" in implementation
    assert 'keyValue("prediction_source"' in implementation
    assert '"constant_velocity_status"' in implementation
    assert '"constant_velocity_translation_clamped"' in implementation


def test_map_to_odom_tf_is_anchored_by_localization_and_published_only_from_odom():
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()

    odom_callback_start = implementation.index("void Slam3dLocalizer::odomCallback(")
    odom_callback_end = implementation.index(
        "void Slam3dLocalizer::noMotionUpdateCallback(", odom_callback_start
    )
    odom_callback = implementation[odom_callback_start:odom_callback_end]
    result_publisher_start = implementation.index(
        "void Slam3dLocalizer::publishResult("
    )
    tf_publisher_start = implementation.index(
        "void Slam3dLocalizer::publishMapToOdomTransform(", result_publisher_start
    )
    result_publisher = implementation[result_publisher_start:tf_publisher_start]

    assert (
        "map_T_base_, odom_lookup.odom_T_base, cloud_stamp_ns"
        in implementation
    )
    assert (
        "stamp.nanoseconds(), publish_identity_map_to_odom_before_localization_"
        in odom_callback
    )
    assert "publishMapToOdomTransform(*map_T_odom, stamp);" in odom_callback
    assert "publish_tf_" in odom_callback
    assert "sendTransform" not in result_publisher
    assert implementation.count("tf_broadcaster_->sendTransform(transform);") == 1

    reanchor = implementation.index(
        "map_odom_transform_cache_.reanchor("
    )
    unsynchronized_accept = implementation.index("} else {", reanchor)
    predictor_update = implementation.index(
        "if (use_constant_velocity_prediction_) {", unsynchronized_accept
    )
    assert (
        "map_odom_transform_cache_.clearCorrectionPreserveWatermark();"
        not in implementation[unsynchronized_accept:predictor_update]
    )


def test_map_to_odom_tf_cache_is_invalidated_on_pose_and_timestamp_epoch_resets():
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()

    initial_pose_start = implementation.index(
        "void Slam3dLocalizer::initialPoseCallback("
    )
    odom_start = implementation.index(
        "void Slam3dLocalizer::odomCallback(", initial_pose_start
    )
    cloud_start = implementation.index("void Slam3dLocalizer::cloudCallback(")
    cloud_reset = implementation.index(
        "cloud_timestamp_reset =", cloud_start
    )
    freshness_before = implementation.index("freshness_before", cloud_reset)

    assert (
        "map_odom_transform_cache_.clearCorrectionPreserveWatermark();"
        in implementation[initial_pose_start:odom_start]
    )
    assert "map_odom_transform_cache_.retireIdentityBootstrap();" in implementation[
        initial_pose_start:odom_start
    ]
    assert "map_odom_transform_cache_.resetOdomEpoch();" in implementation[
        odom_start:cloud_start
    ]
    assert (
        "map_odom_transform_cache_.clearCorrectionPreserveWatermark();"
        in implementation[cloud_reset:freshness_before]
    )
    assert "map_odom_transform_cache_.retireIdentityBootstrap();" in implementation[
        cloud_reset:freshness_before
    ]


def test_normal_odom_path_does_not_take_long_running_localization_state_mutex():
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()
    odom_start = implementation.index("void Slam3dLocalizer::odomCallback(")
    odom_end = implementation.index(
        "void Slam3dLocalizer::noMotionUpdateCallback(", odom_start
    )
    odom_callback = implementation[odom_start:odom_end]
    reset_branch = odom_callback.index("if (timestamp_reset) {")
    normal_tf_branch = odom_callback.index("if (publish_tf_) {", reset_branch)

    assert "odom_buffer_mutex_" in odom_callback[:reset_branch]
    assert "state_mutex_" not in odom_callback[:reset_branch]
    assert "state_mutex_" not in odom_callback[normal_tf_branch:]
    assert "tf_cache_mutex_" in odom_callback[normal_tf_branch:]


def test_frames_are_validated_and_odom_requires_exact_frame_ids():
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()
    odom_start = implementation.index("void Slam3dLocalizer::odomCallback(")
    odom_end = implementation.index(
        "void Slam3dLocalizer::noMotionUpdateCallback(", odom_start
    )
    odom_callback = implementation[odom_start:odom_end]

    assert "!map_frame_.empty()" in implementation
    assert "!odom_frame_.empty()" in implementation
    assert "!base_frame_.empty()" in implementation
    assert "map_frame_ != odom_frame_" in implementation
    assert "map_frame_ != base_frame_" in implementation
    assert "odom_frame_ != base_frame_" in implementation
    assert "message->header.frame_id != odom_frame_" in odom_callback
    assert "message->child_frame_id != base_frame_" in odom_callback
    assert "message->header.frame_id.empty()" not in odom_callback
    assert "message->child_frame_id.empty()" not in odom_callback


def test_cloud_freshness_checks_bracket_processing_and_state_commit():
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()
    callback = implementation[implementation.index(
        "void Slam3dLocalizer::cloudCallback("
    ) : implementation.index("void Slam3dLocalizer::publishResult(")]

    cloud_reset_detection = callback.index("cloud_timestamp_reset =")
    cloud_reset_clear = callback.index(
        "map_odom_transform_cache_.clearCorrectionPreserveWatermark();",
        cloud_reset_detection,
    )
    freshness_before = callback.index("freshness_before")
    waiting_before = callback.index("waiting_for_ros_time_before_processing")
    waiting_before_return = callback.index("return;", waiting_before)
    prepare_scan = callback.index("prepareScan(*message)")
    registration = callback.index("registerScanToMap(")
    freshness_after = callback.index("freshness_after", registration)
    waiting_after = callback.index("waiting_for_ros_time_after_registration")
    waiting_after_return = callback.index("return;", waiting_after)
    stale_after = callback.index("if (freshness_after.stale)", freshness_after)
    stale_after_return = callback.index("return;", stale_after)
    timestamp_commit = callback.index(
        "have_last_cloud_stamp_ = true;", stale_after_return
    )
    pose_commit = callback.index("map_T_base_ = result", timestamp_commit)
    tf_anchor = callback.index("map_odom_transform_cache_.reanchor(", pose_commit)

    assert cloud_reset_detection < cloud_reset_clear < freshness_before
    assert freshness_before < waiting_before < waiting_before_return < prepare_scan
    assert prepare_scan < registration < freshness_after
    assert freshness_after < waiting_after < waiting_after_return < stale_after
    assert freshness_after < stale_after < stale_after_return < timestamp_commit
    assert "constant_velocity_predictor_ = constant_velocity_predictor_before;" in callback[
        freshness_after:waiting_after_return
    ]
    assert "constant_velocity_predictor_ = constant_velocity_predictor_before;" in callback[
        stale_after:stale_after_return
    ]
    assert timestamp_commit < pose_commit < tf_anchor
    assert "stale_cloud_before_processing" in callback
    assert "stale_cloud_after_registration" in callback


def test_cloud_freshness_parameter_and_diagnostics_contract():
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()

    assert 'declare_parameter<double>("max_cloud_age_s"' in implementation
    assert "max_cloud_age_s_ < 0.0" in implementation
    assert '"identity_map_to_odom_bootstrap_enabled"' in implementation
    for key in (
        "cloud_freshness_enabled",
        "cloud_age_s",
        "max_cloud_age_s",
        "stale_cloud_drop_stage",
        "stale_cloud_before_processing_count",
        "stale_cloud_after_registration_count",
        "stale_cloud_drop_count",
    ):
        assert f'"{key}"' in implementation


def test_synchronized_odom_update_policy_is_opt_in_and_fail_closed():
    header = (
        PACKAGE_ROOT
        / "include"
        / "slam_3d_localization"
        / "slam_3d_localizer.hpp"
    ).read_text()
    implementation = (
        PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp"
    ).read_text()
    callback = implementation[
        implementation.index("void Slam3dLocalizer::cloudCallback(")
        : implementation.index("void Slam3dLocalizer::publishResult(")
    ]

    assert "bool require_synced_odom_for_update_{false};" in header
    assert (
        'declare_parameter<bool>(\n'
        '    "require_synced_odom_for_update", require_synced_odom_for_update_)'
        in implementation
    )
    odom_lookup = callback.index("odom_buffer_.lookup(cloud_stamp_ns)")
    fail_closed = callback.index(
        "if ((require_synced_odom_for_update_ && !have_synced_odom) ||",
        odom_lookup,
    )
    fail_closed_return = callback.index("return;", fail_closed)
    registration = callback.index("registerScanToMap(", fail_closed_return)

    assert odom_lookup < fail_closed < fail_closed_return < registration
    assert "waiting_for_synchronized_lio_odom" in callback[
        fail_closed:fail_closed_return
    ]
    assert "map_T_base_ =" not in callback[fail_closed:fail_closed_return]
    assert "map_odom_transform_cache_.reanchor(" not in callback[
        fail_closed:fail_closed_return
    ]
    assert '"require_synced_odom_for_update"' in implementation
    assert '"unsynchronized_cloud_drop_count"' in implementation


def test_lio_scan_posterior_uses_separate_exact_sync_channel():
    header = (
        PACKAGE_ROOT
        / "include"
        / "slam_3d_localization"
        / "slam_3d_localizer.hpp"
    ).read_text()
    localizer = (PACKAGE_ROOT / "src" / "slam_3d_localizer.cpp").read_text()
    lio = (PACKAGE_ROOT / "src" / "lio_odometry.cpp").read_text()

    assert "registration_odom_topic_" in header
    assert "require_exact_odom_for_update_" in header
    assert "processPendingClouds" in header
    assert '"registration_odom_topic"' in localizer
    assert "odom_lookup.mode == OdomLookupMode::kExact" in localizer
    assert '"waiting_for_exact_scan_odom"' in localizer
    scan_output = lio.index("bool LioOdometry::publishScanOutputsIfCurrent(")
    revision_gate = lio.index(
        "state_revision_ != expected_state_revision", scan_output
    )
    scan_pose_publish = lio.index(
        "scan_odom_pub_->publish(odometry_message);", revision_gate
    )
    cloud_publish = lio.index(
        "deskewed_cloud_pub_->publish(cloud_message);", scan_pose_publish
    )

    assert revision_gate < scan_pose_publish < cloud_publish
    assert "publishScanOutputsIfCurrent(" in lio[:scan_output]
