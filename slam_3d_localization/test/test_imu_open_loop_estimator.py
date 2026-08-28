import importlib.util
import json
import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "estimate_imu_open_loop.py"
SPEC = importlib.util.spec_from_file_location("estimate_imu_open_loop", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
imu_open_loop = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = imu_open_loop
SPEC.loader.exec_module(imu_open_loop)


def make_sample(
    index,
    *,
    dt_s=0.01,
    gyro=(0.0, 0.0, 0.0),
    accel=(0.0, 0.0, -imu_open_loop.GRAVITY_MPS2),
    stamp_ns=None,
):
    if stamp_ns is None:
        stamp_ns = 1_000_000_000 + int(round(index * dt_s * 1.0e9))
    covariance = (0.0,) * 9
    return imu_open_loop.ImuSample(
        stamp_ns=stamp_ns,
        record_time_ns=stamp_ns + 123,
        frame_id="cv7_link",
        angular_velocity_radps=gyro,
        linear_acceleration_mps2=accel,
        angular_velocity_covariance=covariance,
        linear_acceleration_covariance=covariance,
    )


def test_cv7_rx180_maps_y_and_z_vectors_into_base_convention():
    corrected = imu_open_loop.apply_cv7_rx180(
        make_sample(0, gyro=(1.0, 2.0, 3.0), accel=(4.0, 5.0, 6.0))
    )

    assert corrected.angular_velocity_radps == (1.0, -2.0, -3.0)
    assert corrected.linear_acceleration_mps2 == (4.0, -5.0, -6.0)


def test_cv7_rx180_rotates_covariance_and_preserves_unknown_sentinel():
    covariance = (1.0, 2.0, 3.0, 2.0, 4.0, 5.0, 3.0, 5.0, 6.0)
    sample = make_sample(0)
    corrected = imu_open_loop.apply_cv7_rx180(
        imu_open_loop.ImuSample(
            **{
                **sample.__dict__,
                "angular_velocity_covariance": covariance,
                "linear_acceleration_covariance": (-1.0,) + (0.0,) * 8,
            }
        )
    )

    assert corrected.angular_velocity_covariance == (
        1.0,
        -2.0,
        -3.0,
        -2.0,
        4.0,
        5.0,
        -3.0,
        5.0,
        6.0,
    )
    assert corrected.linear_acceleration_covariance == (-1.0,) + (0.0,) * 8


def test_static_cv7_stream_stays_at_rest_and_only_labels_zupt_candidates():
    samples = [make_sample(index) for index in range(301)]
    config = imu_open_loop.EstimatorConfig(
        calibration_duration_s=1.0,
        calibration_min_samples=100,
        max_valid_horizon_s=5.0,
    )

    rows, calibration, filter_stats, run_stats = imu_open_loop.estimate_samples(
        samples, config
    )

    assert calibration.valid
    assert filter_stats.accepted_count == len(samples)
    assert run_stats.zupt_candidate_count > 0
    assert run_stats.zupt_applied_count == 0
    assert all(row["zupt_applied"] == 0 for row in rows)
    assert math.dist(
        (
            rows[-1]["position_x_m"],
            rows[-1]["position_y_m"],
            rows[-1]["position_z_m"],
        ),
        (0.0, 0.0, 0.0),
    ) < 1.0e-9
    assert math.dist(
        (
            rows[-1]["velocity_x_mps"],
            rows[-1]["velocity_y_mps"],
            rows[-1]["velocity_z_mps"],
        ),
        (0.0, 0.0, 0.0),
    ) < 1.0e-9
    assert rows[-1]["position_cov_xx_m2"] > 0.0
    assert rows[-1]["quality_gate_eligible"] == 0
    assert rows[-1]["within_diagnostic_horizon"] == 1
    assert rows[-1]["valid_horizon_s"] == run_stats.last_valid_elapsed_s


def test_constant_acceleration_after_calibration_integrates_relative_motion():
    samples = []
    for index in range(201):
        # Raw CV7 z is negative; Rx180 maps it to +g in corrected base axes.
        raw_accel = (
            (0.0, 0.0, -imu_open_loop.GRAVITY_MPS2)
            if index <= 100
            else (1.0, 0.0, -imu_open_loop.GRAVITY_MPS2)
        )
        samples.append(make_sample(index, accel=raw_accel))
    config = imu_open_loop.EstimatorConfig(
        calibration_duration_s=1.0,
        calibration_min_samples=100,
        max_valid_horizon_s=5.0,
    )

    rows, calibration, _, _ = imu_open_loop.estimate_samples(samples, config)

    assert calibration.valid
    # Trapezoidal endpoint integration gives 0.995 m/s and 0.495 m here.
    assert math.isclose(rows[-1]["velocity_x_mps"], 0.995, abs_tol=2.0e-3)
    assert math.isclose(rows[-1]["position_x_m"], 0.495, abs_tol=2.0e-3)
    assert abs(rows[-1]["position_y_m"]) < 1.0e-9
    assert abs(rows[-1]["position_z_m"]) < 1.0e-9


def test_duplicate_and_nonmonotonic_stamps_are_dropped_without_sorting_in_memory():
    samples = [make_sample(index) for index in range(121)]
    duplicate = make_sample(50)
    backward = make_sample(0, stamp_ns=samples[50].stamp_ns - 1)
    samples[51:51] = [duplicate, backward]
    config = imu_open_loop.EstimatorConfig(
        calibration_duration_s=1.0,
        calibration_min_samples=100,
    )

    rows, calibration, filter_stats, _ = imu_open_loop.estimate_samples(samples, config)

    assert calibration.valid
    assert filter_stats.input_count == 123
    assert filter_stats.accepted_count == 121
    assert filter_stats.duplicate_stamp_count == 1
    assert filter_stats.nonmonotonic_stamp_count == 1
    assert len(rows) == 121


def test_timestamp_gap_preserves_output_but_invalidates_continuous_horizon():
    samples = [make_sample(index) for index in range(111)]
    samples.extend(
        make_sample(index + 200, stamp_ns=3_000_000_000 + index * 10_000_000)
        for index in range(10)
    )
    config = imu_open_loop.EstimatorConfig(
        calibration_duration_s=1.0,
        calibration_min_samples=100,
        max_integration_gap_s=0.05,
    )

    rows, calibration, _, run_stats = imu_open_loop.estimate_samples(samples, config)

    assert calibration.valid
    assert run_stats.integration_gap_count == 1
    first_gap_index = next(
        index for index, row in enumerate(rows) if row["integration_gap"] == 1
    )
    assert rows[first_gap_index]["propagation_valid"] == 0
    assert rows[first_gap_index]["within_valid_horizon"] == 0
    assert rows[first_gap_index]["validity_reason"] == "timestamp_gap"
    assert all(row["zupt_applied"] == 0 for row in rows[first_gap_index:])


def test_anchor_makes_first_pose_exactly_match_supplied_pose(tmp_path):
    half_yaw = math.radians(45.0)
    anchor_path = tmp_path / "anchor.json"
    anchor_path.write_text(
        json.dumps(
            {
                "frame_id": "map",
                "position": {"x": 3.0, "y": -2.0, "z": 1.0},
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(half_yaw),
                    "w": math.cos(half_yaw),
                },
            }
        ),
        encoding="utf-8",
    )
    anchor = imu_open_loop.load_trajectory_anchor(anchor_path)
    config = imu_open_loop.EstimatorConfig(
        calibration_duration_s=1.0,
        calibration_min_samples=100,
    )

    rows, calibration, _, _ = imu_open_loop.estimate_samples(
        [make_sample(index) for index in range(121)], config, anchor
    )

    assert calibration.valid
    first = rows[0]
    assert first["output_frame_id"] == "map"
    assert math.isclose(first["position_x_m"], 3.0, abs_tol=1.0e-12)
    assert math.isclose(first["position_y_m"], -2.0, abs_tol=1.0e-12)
    assert math.isclose(first["position_z_m"], 1.0, abs_tol=1.0e-12)
    assert math.isclose(first["quaternion_z"], math.sin(half_yaw), abs_tol=1.0e-12)
    assert math.isclose(first["quaternion_w"], math.cos(half_yaw), abs_tol=1.0e-12)


def test_stage_trajectory_csv_x_m_columns_are_accepted_as_anchor(tmp_path):
    anchor_path = tmp_path / "stage_trajectory.csv"
    anchor_path.write_text(
        "stamp_ns,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n"
        "123,map,1.5,-0.5,0.25,0,0,0,1\n",
        encoding="utf-8",
    )

    anchor = imu_open_loop.load_trajectory_anchor(anchor_path)

    assert anchor.position_m == (1.5, -0.5, 0.25)
    assert anchor.quaternion_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert anchor.frame_id == "map"


def test_cli_contract_names_required_inputs_and_warns_that_output_is_not_gt():
    text = SCRIPT_PATH.read_text(encoding="utf-8")

    assert '"--bag-db"' in text
    assert '"--trajectory-anchor"' in text
    assert '"--output-dir"' in text
    assert "not ground truth" in text.lower()
    assert '"hard_zero_velocity_applied": False' in text


def test_empty_frame_is_rejected_before_cv7_correction():
    sample = imu_open_loop.ImuSample(
        stamp_ns=1,
        record_time_ns=1,
        frame_id="",
        angular_velocity_radps=(0.0, 0.0, 0.0),
        linear_acceleration_mps2=(0.0, 0.0, -imu_open_loop.GRAVITY_MPS2),
    )

    try:
        imu_open_loop.estimate_samples([sample])
    except RuntimeError as error:
        assert "requires frame 'cv7_link'" in str(error)
    else:
        raise AssertionError("empty frame_id must not receive the CV7 Rx180 correction")


def test_rosbag_metadata_controls_multi_shard_order_and_provenance(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "recording_2.db3").write_bytes(b"two")
    (bag / "recording_10.db3").write_bytes(b"ten")
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  storage_identifier: sqlite3\n"
        "  relative_file_paths:\n"
        "    - recording_10.db3\n"
        "    - recording_2.db3\n"
    )

    paths = imu_open_loop._resolve_database_paths(bag)
    provenance = imu_open_loop._database_provenance(bag)

    assert [path.name for path in paths] == ["recording_10.db3", "recording_2.db3"]
    assert [record["relative_path"] for record in provenance["storage_files"]] == [
        "recording_10.db3",
        "recording_2.db3",
    ]
    assert provenance["storage_order_source"] == "metadata.yaml"
    assert provenance["metadata"]["sha256"]


def test_final_csv_and_metrics_are_published_atomically(tmp_path):
    incomplete = tmp_path / ".imu_open_loop_trajectory.incomplete.csv"
    final = tmp_path / "imu_open_loop_trajectory.csv"
    incomplete.write_text("stamp_ns,valid_horizon_s\n1,\n")

    imu_open_loop._write_final_csv_with_horizon(
        incomplete, final, ["stamp_ns", "valid_horizon_s"], 3.5
    )
    metrics = tmp_path / "imu_open_loop_metrics.json"
    imu_open_loop._write_text_atomic(metrics, '{"ready": true}\n')

    assert not incomplete.exists()
    assert not (tmp_path / ".imu_open_loop_trajectory.csv.finalizing").exists()
    assert "3.500000000" in final.read_text()
    assert json.loads(metrics.read_text()) == {"ready": True}
