import importlib.util
import struct
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "evaluate_localization_replay.py"
SPEC = importlib.util.spec_from_file_location("evaluate_localization_replay", SCRIPT)
evaluation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluation)


def matrix(position=(0, 0, 0), yaw=0):
    return evaluation.transform(position, Rotation.from_euler("z", yaw, degrees=True).as_quat())


def test_composition_rotates_odom_translation_and_scan_points():
    pose = matrix((10, 0, 1), 90) @ matrix((2, 0, 0))
    np.testing.assert_allclose(evaluation.transform_points(pose, [[1, 0, 0]]), [[10, 3, 1]], atol=1e-12)


def test_past_transform_rejects_future_and_stale_data():
    transforms = {100: matrix(), 200: matrix((1, 0, 0))}
    stamps = np.array([100, 200])
    assert evaluation.past_transform(transforms, stamps, 99, 100) is None
    assert evaluation.past_transform(transforms, stamps, 199, 98) is None
    assert evaluation.past_transform(transforms, stamps, 199, 99)[0] == 100
    assert evaluation.past_transform(transforms, stamps, 200, 0)[0] == 200


def test_trajectory_uses_exact_stamps_without_interpolation():
    result = evaluation.exact_trajectory({100: matrix((1, 0, 0)), 200: matrix()}, {100: matrix((2, 0, 0)), 201: matrix()})
    assert list(result) == [100]
    assert result[100][0, 3] == 3


def test_pose_difference_preserves_offset_and_wraps_yaw():
    current = {100: matrix((3, 4, 2), -179), 101: matrix()}
    reference = {100: matrix(yaw=179), 102: matrix()}
    rows, summary = evaluation.pose_differences(current, reference)
    assert len(rows) == 1
    assert rows[0]["xy_m"] == 5
    assert rows[0]["z_m"] == 2
    assert rows[0]["yaw_deg"] == pytest.approx(2)
    assert summary["xy_m"]["rmse"] == 5


def test_trajectory_step_and_stationary_assumption_are_explicit():
    trajectory = {1_000_000_000: matrix(), 2_000_000_000: matrix((0.1, 0, 0)),
                  3_000_000_000: matrix((5, 0, 0))}
    result = evaluation.trajectory_summary(trajectory, 1)
    assert result["path_length_3d_m"] == 5
    assert result["steps_over_1m"] == 1
    assert result["assumed_stationary_prefix"]["count"] == 2
    assert result["assumed_stationary_prefix"]["max_displacement_3d_m"] == pytest.approx(0.1)
    assert "assumed_stationary_prefix" not in evaluation.trajectory_summary(trajectory)


def test_residuals_count_outliers_and_nonfinite_points():
    stats, distances = evaluation.residuals(np.array([[0, 0, 0], [0.5, 0, 0], [10, 0, 0], [np.nan, 0, 0]]), cKDTree([[0, 0, 0]]), 0.5)
    assert stats["input_points"] == 4
    assert stats["finite_points"] == stats["count"] == 3
    assert stats["inliers"] == 2
    assert stats["inlier_fraction"] == pytest.approx(2 / 3)
    assert stats["maximum"] == 10
    np.testing.assert_array_equal(distances, [0, 0.5, 10])


def test_voxel_centroid_uses_all_points_and_negative_cells():
    points = np.array([[0.1, 0, 0], [0.3, 0, 0], [-0.1, 0, 0], [np.inf, 0, 0]])
    np.testing.assert_allclose(evaluation.voxel_centroids(points, 1), [[-0.1, 0, 0], [0.2, 0, 0]])
    with pytest.raises(ValueError):
        evaluation.voxel_centroids(points, 0)


@pytest.mark.parametrize("encoding", ["ascii", "binary", "binary_compressed"])
def test_pcd_encodings_preserve_xyz_and_extra_fields(tmp_path, encoding):
    header = ("FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA " + encoding + "\n").encode()
    points = np.array([[1, 2, 3, 9], [4, 5, 6, 10]], dtype="<f4")
    if encoding == "ascii":
        payload = b"1 2 3 9\n4 5 6 10\n"
    elif encoding == "binary":
        payload = points.tobytes()
    else:
        soa = points.T.copy().tobytes()
        compressed = bytes([len(soa) - 1]) + soa
        payload = struct.pack("<II", len(compressed), len(soa)) + compressed
    path = tmp_path / "map.pcd"
    path.write_bytes(header + payload)
    np.testing.assert_array_equal(evaluation.read_pcd(path), points[:, :3])


def test_lzf_overlap_backreference_and_corruption():
    assert evaluation.lzf_decompress(bytes([0, ord("a"), 64, 0]), 5) == b"aaaaa"
    for payload, length in [(bytes([0]), 1), (bytes([32, 0]), 3), (bytes([0, 65]), 2)]:
        with pytest.raises(ValueError):
            evaluation.lzf_decompress(payload, length)


def test_cloud_xyz_respects_row_padding():
    fields = [SimpleNamespace(name=name, datatype=7, count=1, offset=index * 4) for index, name in enumerate("xyz")]
    message = SimpleNamespace(fields=fields, is_bigendian=False, point_step=12, row_step=16,
                              height=2, width=1, data=struct.pack("<fffIfffI", 1, 2, 3, 0, 4, 5, 6, 0))
    np.testing.assert_array_equal(evaluation.cloud_xyz(message), [[1, 2, 3], [4, 5, 6]])


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError):
        evaluation.transform([0, 0, 0], [0, 0, 0, 0])
