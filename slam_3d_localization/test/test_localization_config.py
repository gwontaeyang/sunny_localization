from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_default_config_has_safe_registration_gates():
    data = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "slam_3d_localization.yaml").read_text()
    )
    params = data["slam_3d_localizer"]["ros__parameters"]

    assert params["map_path"] == ""
    assert params["map_frame"] != params["odom_frame"]
    assert params["registration_method"] in {"gicp", "voxel_surfel"}
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
    assert isinstance(params["cloud_subscription_reliable"], bool)
    assert params["cloud_subscription_depth"] > 0
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
    data = yaml.safe_load(
        (
            PACKAGE_ROOT
            / "config"
            / "slam_3d_localization_indoor_xt32_no_wheel_replay.yaml"
        ).read_text()
    )
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


def test_launch_requires_map_path_and_does_not_start_hardware():
    launch_text = (
        PACKAGE_ROOT / "launch" / "slam_3d_localization.launch.py"
    ).read_text()

    assert 'DeclareLaunchArgument("map_path")' in launch_text
    assert '"map_path": LaunchConfiguration("map_path")' in launch_text
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
        "const RegistrationOutcome registration = registerScanToMap("
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
