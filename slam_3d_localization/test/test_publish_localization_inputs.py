import hashlib
import importlib.util
import json
import sqlite3
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "publish_localization_inputs.py"
SPEC = importlib.util.spec_from_file_location("publish_localization_inputs", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
publisher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(publisher)


def write_pcd(path: Path, points: int = 3) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [f"{index}.0 {index + 1}.0 {index + 2}.0" for index in range(points)]
    path.write_text(
        "\n".join(
            [
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                f"WIDTH {points}",
                "HEIGHT 1",
                f"POINTS {points}",
                "DATA ascii",
                *rows,
                "",
            ]
        )
    )


def write_trajectory(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "stamp_ns,frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n"
        "1,map,0,0,0,0,0,0,1\n"
        "2,map,1,0,0,0,0,0,1\n"
    )


def write_bag(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(path)
    try:
        connection.execute(
            "CREATE TABLE topics ("
            "id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
            "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)"
        )
        connection.executemany(
            "INSERT INTO topics VALUES (?, ?, ?, ?, ?)",
            [
                (1, "/points", "sensor_msgs/msg/PointCloud2", "cdr", ""),
                (2, "/imu/data", "sensor_msgs/msg/Imu", "cdr", ""),
            ],
        )
        connection.commit()
    finally:
        connection.close()


def make_fixture(root: Path):
    stage = root / "24_future_stage"
    stage.mkdir()
    inputs = root / "inputs"
    datasets = {}
    for dataset in publisher.DATASETS:
        map_path = inputs / f"{dataset}.pcd"
        trajectory = inputs / f"{dataset}.csv"
        bag = inputs / f"{dataset}.db3"
        write_pcd(map_path)
        write_trajectory(trajectory)
        write_bag(bag)
        datasets[dataset] = {
            "map_source": str(map_path),
            "trajectory_source": str(trajectory),
            "bag": {"path": str(bag)},
            "point_cloud": {"topic": "/points", "frame": "lidar"},
            "imu": {"topic": "/imu/data", "frame": "imu_link"},
            "wheel_odometry": {"topic": None},
        }
    return stage, {
        "schema_version": 1,
        "stage_sequence": 24,
        "stage_name": stage.name,
        "datasets": datasets,
    }


def test_check_only_validates_without_writing(tmp_path):
    stage, spec = make_fixture(tmp_path)

    result = publisher.publish(stage, spec, check_only=True)

    assert result["status"] == "validated_not_published"
    assert len(result["contract_sha256"]) == 64
    assert result["publishable"] is True
    assert result["target_conflicts"] == []
    assert not any((stage / path).exists() for path in publisher.RESERVED_PATHS)


def test_publish_is_resolver_verified_and_binds_exact_hashes(tmp_path):
    stage, spec = make_fixture(tmp_path)

    result = publisher.publish(stage, spec)
    resolved = publisher.resolver.resolve(tmp_path)

    assert result["status"] == "published_and_resolver_verified"
    assert resolved["stage"]["sequence"] == 24
    assert set(resolved["datasets"]) == set(publisher.DATASETS)
    contract_path = stage / publisher.resolver.CONTRACT_NAME
    marker = (stage / publisher.resolver.READY_MARKER_NAME).read_text().split()[0]
    assert marker == hashlib.sha256(contract_path.read_bytes()).hexdigest()
    for dataset in publisher.DATASETS:
        record = resolved["datasets"][dataset]
        assert record["map"]["pcd_header"]["points"] == 3
        assert record["trajectory"]["pose_count"] == 2
        assert record["wheel_odometry"]["available"] is False


def test_publish_refuses_overwrite_and_bad_topic_leaves_stage_unadvertised(tmp_path):
    stage, spec = make_fixture(tmp_path)
    spec["datasets"]["bag02"]["point_cloud"]["topic"] = "/missing"

    with pytest.raises(publisher.PublisherError, match="lacks LiDAR topic"):
        publisher.publish(stage, spec)
    assert not (stage / "pcd_full_maps_only").exists()

    spec["datasets"]["bag02"]["point_cloud"]["topic"] = "/points"
    publisher.publish(stage, spec)
    checked = publisher.publish(stage, spec, check_only=True)
    assert checked["publishable"] is False
    assert set(checked["target_conflicts"]) == set(publisher.RESERVED_PATHS)
    with pytest.raises(publisher.PublisherError, match="refusing to overwrite"):
        publisher.publish(stage, spec)


def test_publish_rolls_back_nonadvertising_paths_if_atomic_install_fails(
    tmp_path, monkeypatch
):
    stage, spec = make_fixture(tmp_path)
    original_replace = publisher.os.replace
    replace_count = 0

    def fail_before_ready_marker(source, destination):
        nonlocal replace_count
        replace_count += 1
        if replace_count == 3:
            raise OSError("injected rename failure")
        return original_replace(source, destination)

    monkeypatch.setattr(publisher.os, "replace", fail_before_ready_marker)
    with pytest.raises(OSError, match="injected rename failure"):
        publisher.publish(stage, spec)

    assert not any(
        (stage / path).exists() or (stage / path).is_symlink()
        for path in publisher.RESERVED_PATHS
    )


def test_publish_rejects_corrupted_copy_and_leaves_no_reserved_paths(
    tmp_path, monkeypatch
):
    stage, spec = make_fixture(tmp_path)
    original_copy2 = publisher.shutil.copy2

    def corrupt_first_map_copy(source, destination):
        result = original_copy2(source, destination)
        if Path(destination).name == "bag02_full_map.pcd":
            destination = Path(destination)
            destination.write_bytes(destination.read_bytes() + b"corrupt")
        return result

    monkeypatch.setattr(publisher.shutil, "copy2", corrupt_first_map_copy)
    with pytest.raises(publisher.PublisherError, match="bag02 map changed"):
        publisher.publish(stage, spec)

    assert not any(
        (stage / path).exists() or (stage / path).is_symlink()
        for path in publisher.RESERVED_PATHS
    )


def test_publish_rolls_back_if_final_resolver_verification_fails(
    tmp_path, monkeypatch
):
    stage, spec = make_fixture(tmp_path)

    def fail_final_verification(*_args, **_kwargs):
        raise publisher.resolver.ResolverError("injected final verification failure")

    monkeypatch.setattr(publisher.resolver, "resolve", fail_final_verification)
    with pytest.raises(
        publisher.PublisherError, match="injected final verification failure"
    ):
        publisher.publish(stage, spec)

    assert not any(
        (stage / path).exists() or (stage / path).is_symlink()
        for path in publisher.RESERVED_PATHS
    )


def test_cli_reads_duplicate_safe_spec_and_outputs_json(tmp_path, capsys):
    stage, spec = make_fixture(tmp_path)
    spec_path = tmp_path / "spec.json"
    spec_path.write_text(json.dumps(spec))

    assert publisher.main(
        ["--stage", str(stage), "--spec", str(spec_path), "--check-only"]
    ) == 0
    assert json.loads(capsys.readouterr().out)["status"] == "validated_not_published"
