import hashlib
import importlib.util
import json
import sqlite3
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "resolve_latest_slam_stage.py"
SPEC = importlib.util.spec_from_file_location("resolve_latest_slam_stage", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
resolver = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(resolver)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_pcd(path, points=2):
    rows = [f"{index}.0 {index + 1}.0 {index + 2}.0" for index in range(points)]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                f"WIDTH {points}",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                f"POINTS {points}",
                "DATA ascii",
                *rows,
                "",
            ]
        )
    )


def _write_trajectory(path, frame="map"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "stamp_ns,frame_id,child_frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n"
        f"1,{frame},,0,0,0,0,0,0,1\n"
        f"2,{frame},,1,0,0,0,0,0,1\n"
    )


def _write_bag(path, include_odom=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(path)
    try:
        connection.execute(
            "CREATE TABLE topics ("
            "id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
            "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)"
        )
        rows = [
            (1, "/points", "sensor_msgs/msg/PointCloud2", "cdr", ""),
            (2, "/imu/data", "sensor_msgs/msg/Imu", "cdr", ""),
        ]
        if include_odom:
            rows.append((3, "/wheel/odom", "nav_msgs/msg/Odometry", "cdr", ""))
        connection.executemany("INSERT INTO topics VALUES (?, ?, ?, ?, ?)", rows)
        connection.commit()
    finally:
        connection.close()


def _storage_set_sha256(storage):
    digest = _sha256(storage)
    line = f"{digest}  {storage.name}\n".encode()
    return hashlib.sha256(line).hexdigest()


def _make_contract_stage(root, sequence=22):
    stage = root / f"{sequence}_contract_fixture"
    maps = stage / "pcd_full_maps_only"
    inputs = root / "source_inputs"
    datasets = {}
    for dataset in ("bag02", "bag03"):
        map_path = maps / f"{dataset}_full_map.pcd"
        trajectory = inputs / f"{dataset}_trajectory.csv"
        bag = inputs / f"{dataset}.db3"
        _write_pcd(map_path)
        _write_trajectory(trajectory)
        _write_bag(bag)
        datasets[dataset] = {
            "map": {
                "path": str(map_path.relative_to(stage)),
                "sha256": _sha256(map_path),
                "frame": "map",
            },
            "bag": {
                "path": str(bag),
                "storage_set_sha256": _storage_set_sha256(bag),
            },
            "trajectory": {
                "path": str(trajectory),
                "sha256": _sha256(trajectory),
                "frame": "map",
            },
            "point_cloud": {"topic": "/points", "frame": "lidar"},
            "imu": {"topic": "/imu/data", "frame": "imu_link"},
            "wheel_odometry": {"topic": None},
        }
    contract = {
        "schema_version": 1,
        "stage_sequence": sequence,
        "stage_name": stage.name,
        "datasets": datasets,
    }
    contract_path = stage / "localization_inputs.json"
    contract_path.write_text(json.dumps(contract, indent=2) + "\n")
    (stage / "LOCALIZATION_INPUTS_READY").write_text(
        f"{_sha256(contract_path)}  localization_inputs.json\n"
    )
    return stage


def _make_legacy_stage(root):
    stage = root / "21_legacy_fixture"
    inputs = root / "legacy_inputs"
    validation_rows = []
    for dataset in ("bag02", "bag03"):
        selected_map = stage / "pcd_full_maps_only" / f"{dataset}_visibility_rgb.pcd"
        result_dir = stage / "full_results" / dataset
        full_map = result_dir / f"{dataset}_visibility_rgb.pcd"
        parent_map = inputs / f"{dataset}_xt32_cv7_imu_no_wheel_full_map.pcd"
        trajectory = inputs / f"{dataset}_trajectory.csv"
        bag = inputs / f"{dataset}.db3"
        for path in (selected_map, full_map, parent_map):
            _write_pcd(path)
        _write_trajectory(trajectory)
        _write_bag(bag)
        result_dir.mkdir(parents=True, exist_ok=True)
        (result_dir / "replay.log").write_text(
            f"dataset={dataset}\n"
            f"bag={bag}\n"
            f"pcd={parent_map}\n"
            f"trajectory={trajectory}\n"
            "started_utc=2026-01-01T00:00:00Z\n"
            "finished_utc=2026-01-01T00:00:01Z\n"
        )
        metrics = {
            "dataset": dataset,
            "projection": {"input_points": 2},
            "raw_visibility": {
                "bag_path": str(bag),
                "topic": "/points",
                "expected_frame_id": "lidar",
            },
            "outputs": {"visibility_pcd": str(full_map)},
        }
        (result_dir / f"{dataset}_metrics.json").write_text(
            json.dumps(metrics, indent=2) + "\n"
        )
        validation_rows.append(
            {"dataset": dataset, "pcd_points": 2, "validation": "passed"}
        )
    validation_path = stage / "validation" / "validation_report.json"
    validation_path.parent.mkdir(parents=True, exist_ok=True)
    validation_path.write_text(
        json.dumps(
            {"status": "passed", "datasets": validation_rows}, indent=2
        )
        + "\n"
    )
    manifest_lines = []
    for path in sorted(item for item in stage.rglob("*") if item.is_file()):
        relative = path.relative_to(stage).as_posix()
        manifest_lines.append(f"{_sha256(path)}  ./{relative}\n")
    (stage / "manifest.sha256").write_text("".join(manifest_lines))
    return stage


def test_resolves_newest_complete_stage_contract(tmp_path):
    stage = _make_contract_stage(tmp_path)

    result = resolver.resolve(tmp_path)

    assert result["status"] == "ready"
    assert result["stage"]["sequence"] == 22
    assert result["stage"]["path"] == str(stage)
    assert set(result["datasets"]) == {"bag02", "bag03"}
    for dataset in ("bag02", "bag03"):
        assert result["datasets"][dataset]["map"]["pcd_header"]["points"] == 2
        assert result["datasets"][dataset]["imu"] == {
            "topic": "/imu/data",
            "frame": "imu_link",
        }
        assert result["datasets"][dataset]["wheel_odometry"]["available"] is False


def test_newer_non_map_stages_are_skipped_with_audit(tmp_path):
    stage = _make_contract_stage(tmp_path, sequence=22)
    (tmp_path / "23_terrain_only" / "validation").mkdir(parents=True)
    (tmp_path / "24_preview_only").mkdir()

    result = resolver.resolve(tmp_path)

    assert result["stage"]["path"] == str(stage)
    assert result["selection"]["mode"] == "latest_map_producing_stage"
    assert [
        item["name"]
        for item in result["selection"]["skipped_newer_non_map_stages"]
    ] == ["24_preview_only", "23_terrain_only"]


def test_no_map_producing_stage_is_an_error(tmp_path):
    (tmp_path / "22_terrain_only").mkdir()

    with pytest.raises(resolver.ResolverError, match="provides pcd_full_maps_only"):
        resolver.resolve(tmp_path)


def test_newest_incomplete_stage_fails_closed_instead_of_falling_back(tmp_path):
    _make_contract_stage(tmp_path, sequence=22)
    incomplete = tmp_path / "23_incomplete"
    _write_pcd(incomplete / "pcd_full_maps_only" / "bag02_map.pcd")
    _write_pcd(incomplete / "pcd_full_maps_only" / "bag03_map.pcd")

    with pytest.raises(resolver.ResolverError, match="localization input contract"):
        resolver.resolve(tmp_path)


def test_exact_pair_and_declared_hash_are_enforced(tmp_path):
    stage = _make_contract_stage(tmp_path)
    _write_pcd(stage / "pcd_full_maps_only" / "bag01_extra.pcd")
    with pytest.raises(resolver.ResolverError, match="exactly two PCDs"):
        resolver.resolve(tmp_path)

    (stage / "pcd_full_maps_only" / "bag01_extra.pcd").unlink()
    map_path = stage / "pcd_full_maps_only" / "bag02_full_map.pcd"
    map_path.write_text(map_path.read_text() + "0 0 0\n")
    with pytest.raises(resolver.ResolverError, match="map SHA-256 mismatch"):
        resolver.resolve(tmp_path)


def test_pcd_header_and_source_bag_existence_are_enforced(tmp_path):
    stage = _make_contract_stage(tmp_path)
    map_path = stage / "pcd_full_maps_only" / "bag02_full_map.pcd"
    contract_path = stage / "localization_inputs.json"
    contract = json.loads(contract_path.read_text())
    map_path.write_text(map_path.read_text().replace("POINTS 2", "POINTS 3"))
    contract["datasets"]["bag02"]["map"]["sha256"] = _sha256(map_path)
    contract_path.write_text(json.dumps(contract, indent=2) + "\n")
    (stage / "LOCALIZATION_INPUTS_READY").write_text(
        f"{_sha256(contract_path)}  localization_inputs.json\n"
    )
    with pytest.raises(resolver.ResolverError, match=r"WIDTH\*HEIGHT"):
        resolver.resolve(tmp_path)

    _write_pcd(map_path)
    contract["datasets"]["bag02"]["map"]["sha256"] = _sha256(map_path)
    missing_bag = Path(contract["datasets"]["bag02"]["bag"]["path"])
    missing_bag.unlink()
    contract_path.write_text(json.dumps(contract, indent=2) + "\n")
    (stage / "LOCALIZATION_INPUTS_READY").write_text(
        f"{_sha256(contract_path)}  localization_inputs.json\n"
    )
    with pytest.raises(resolver.ResolverError, match="source bag does not exist"):
        resolver.resolve(tmp_path)


def test_stage21_legacy_adapter_uses_replay_metrics_validation_and_manifest(tmp_path):
    stage = _make_legacy_stage(tmp_path)

    result = resolver.resolve(tmp_path)

    assert result["stage"]["adapter"] == "stage21_legacy_lineage"
    assert result["stage"]["path"] == str(stage)
    for dataset in ("bag02", "bag03"):
        resolved = result["datasets"][dataset]
        assert resolved["imu"]["topic"] == "/imu/data"
        assert resolved["imu"]["frame"] == "cv7_link"
        assert resolved["trajectory"]["frame"] == "map"
        assert resolved["wheel_odometry"]["available"] is False


def test_cli_output_is_json_and_check_only_does_not_write(tmp_path, capsys):
    _make_contract_stage(tmp_path)
    output = tmp_path / "resolved" / "inputs.json"
    assert resolver.main(["--root", str(tmp_path), "--output", str(output)]) == 0
    stdout = json.loads(capsys.readouterr().out)
    assert json.loads(output.read_text()) == stdout

    check_only_output = tmp_path / "must_not_exist.json"
    assert (
        resolver.main(["--root", str(tmp_path), "--check-only"])
        == 0
    )
    assert not check_only_output.exists()


def test_ready_marker_must_bind_exact_contract_hash(tmp_path):
    stage = _make_contract_stage(tmp_path)
    (stage / "LOCALIZATION_INPUTS_READY").write_text("")

    with pytest.raises(resolver.ResolverError, match="must contain the contract SHA-256"):
        resolver.resolve(tmp_path)


def test_null_wheel_contract_does_not_promote_unrelated_odometry(tmp_path):
    stage = _make_contract_stage(tmp_path)
    bag = tmp_path / "source_inputs" / "bag02.db3"
    connection = sqlite3.connect(bag)
    try:
        connection.execute(
            "INSERT INTO topics VALUES (?, ?, ?, ?, ?)",
            (3, "/visual_odom", "nav_msgs/msg/Odometry", "cdr", ""),
        )
        connection.commit()
    finally:
        connection.close()
    contract_path = stage / "localization_inputs.json"
    contract = json.loads(contract_path.read_text())
    contract["datasets"]["bag02"]["bag"]["storage_set_sha256"] = (
        _storage_set_sha256(bag)
    )
    contract_path.write_text(json.dumps(contract, indent=2) + "\n")
    (stage / "LOCALIZATION_INPUTS_READY").write_text(
        f"{_sha256(contract_path)}  localization_inputs.json\n"
    )

    result = resolver.resolve(tmp_path)

    wheel = result["datasets"]["bag02"]["wheel_odometry"]
    assert wheel["available"] is False
    assert wheel["topic"] is None
    assert wheel["topics"] == []
    assert wheel["undeclared_odometry_topics"] == ["/visual_odom"]
    assert result["datasets"]["bag02"]["reference_policy"] == (
        "integrate_imu_no_declared_wheel_odometry"
    )


def test_trajectory_reports_stable_sort_policy_and_rejects_nonfinite_pose(tmp_path):
    path = tmp_path / "trajectory.csv"
    path.write_text(
        "stamp_ns,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n"
        "10,map,0,0,0,0,0,0,1\n"
        "9,map,1,0,0,0,0,0,1\n"
        "10,map,2,0,0,0,0,0,1\n"
    )

    info = resolver.read_trajectory(path)

    assert info["timestamp_inversion_count"] == 1
    assert info["timestamp_duplicate_count"] == 1
    assert info["timestamp_order"] == "requires_stable_sort"
    assert info["timestamp_consumer_policy"] == (
        "stable_sort_by_stamp_ns_then_source_row"
    )

    path.write_text(
        "stamp_ns,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n"
        "10,map,nan,0,0,0,0,0,1\n"
    )
    with pytest.raises(resolver.ResolverError, match="non-finite pose data"):
        resolver.read_trajectory(path)


def test_empty_required_pcd_header_value_is_structured_error(tmp_path):
    path = tmp_path / "bad.pcd"
    path.write_text(
        "VERSION\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n"
    )

    with pytest.raises(resolver.ResolverError, match="empty VERSION"):
        resolver.read_pcd_header(path)


def test_rosbag_metadata_controls_storage_order(tmp_path):
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

    paths = resolver._rosbag_storage_files(bag)

    assert [path.name for path in paths] == ["recording_10.db3", "recording_2.db3"]


REAL_STAGE21 = Path(
    "/home/user/analysis_outputs/slam_3d/21_raw_scan_occupancy_path"
)


def _real_stage21_source_bags_available() -> bool:
    if not REAL_STAGE21.is_dir():
        return False
    try:
        for dataset in resolver.DATASETS:
            replay_log = REAL_STAGE21 / "full_results" / dataset / "replay.log"
            lineage = resolver._parse_replay_lineage(replay_log)
            source_bag = Path(lineage["bag"]).expanduser()
            if not source_bag.exists():
                return False
    except (OSError, KeyError, resolver.ResolverError):
        return False
    return True


@pytest.mark.skipif(
    not _real_stage21_source_bags_available(),
    reason="local stage21 evidence or its source bags are unavailable",
)
def test_actual_stage21_resolves_read_only():
    result = resolver.resolve(REAL_STAGE21.parent, "21")

    assert result["status"] == "ready"
    assert result["stage"]["name"] == "21_raw_scan_occupancy_path"
    assert result["datasets"]["bag02"]["map"]["sha256"] == (
        "d2c409f8334b3ec64860e230f743cd95e931c0515613faf40bfba3d0389765df"
    )
    assert result["datasets"]["bag03"]["map"]["sha256"] == (
        "8b6aa5cfa8e52575feaaeef8f2c8c13426b7c00ffb1ac756a4d1c21f4c01e01c"
    )
