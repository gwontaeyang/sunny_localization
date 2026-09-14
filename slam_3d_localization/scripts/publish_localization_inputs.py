#!/usr/bin/env python3
"""
Publish a fail-closed localization-input contract into a SLAM stage.

The canonical ``pcd_full_maps_only`` directory is renamed into place last. The
latest-stage resolver therefore either skips a stage that has not published its
localization bundle yet, or observes the complete contract; it never sees the
maps first and silently falls back after a partial publish.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
RESOLVER_PATH = SCRIPT_DIR / "resolve_latest_slam_stage.py"
SPEC = importlib.util.spec_from_file_location(
    "slam_3d_localization_stage_resolver", RESOLVER_PATH
)
if SPEC is None or SPEC.loader is None:  # pragma: no cover - installation fault
    raise RuntimeError(f"cannot import resolver: {RESOLVER_PATH}")
resolver = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(resolver)

DATASETS = ("bag02", "bag03")
RESERVED_PATHS = (
    "pcd_full_maps_only",
    "localization_inputs",
    resolver.CONTRACT_NAME,
    resolver.READY_MARKER_NAME,
)


class PublisherError(RuntimeError):
    """Raised when inputs cannot be published without ambiguity."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise PublisherError(message)


def require_object(value: Any, label: str) -> Mapping[str, Any]:
    require(isinstance(value, dict), f"{label} must be an object")
    return value


def require_text(record: Mapping[str, Any], key: str, label: str) -> str:
    value = record.get(key)
    require(
        isinstance(value, str) and bool(value.strip()),
        f"{label}.{key} must be a nonempty string",
    )
    return value


def source_file(record: Mapping[str, Any], key: str, label: str) -> Path:
    unresolved = Path(require_text(record, key, label)).expanduser()
    require(not unresolved.is_symlink(), f"{label} must not be a symlink: {unresolved}")
    path = unresolved.resolve()
    try:
        resolver._require_regular_file(path, label)
    except resolver.ResolverError as exc:
        raise PublisherError(str(exc)) from exc
    return path


def normalized_contract(stage: Path, spec: Mapping[str, Any]) -> Dict[str, Any]:
    try:
        sequence = int(spec.get("stage_sequence"))
    except (TypeError, ValueError) as exc:
        raise PublisherError("stage_sequence must be an integer") from exc
    require(sequence >= resolver.CONTRACT_MIN_SEQUENCE, "stage_sequence must be >= 22")
    stage_name = require_text(spec, "stage_name", "spec")
    require(stage.name == stage_name, f"stage_name mismatch: {stage_name} != {stage.name}")
    try:
        actual_sequence = resolver._stage_sequence(stage)
    except resolver.ResolverError as exc:
        raise PublisherError(str(exc)) from exc
    require(
        sequence == actual_sequence,
        f"stage_sequence mismatch: {sequence} != {actual_sequence}",
    )
    require(spec.get("schema_version") == 1, "schema_version must equal 1")

    raw_datasets = require_object(spec.get("datasets"), "datasets")
    require(
        set(raw_datasets) == set(DATASETS),
        "datasets must contain exactly bag02 and bag03",
    )

    datasets: Dict[str, Any] = {}
    for dataset in DATASETS:
        raw = require_object(raw_datasets[dataset], dataset)
        map_source = source_file(raw, "map_source", f"{dataset}.map")
        trajectory_source = source_file(
            raw, "trajectory_source", f"{dataset}.trajectory"
        )
        bag_section = require_object(raw.get("bag"), f"{dataset}.bag")
        bag_path = Path(
            require_text(bag_section, "path", f"{dataset}.bag")
        ).expanduser().resolve()
        point_cloud = require_object(raw.get("point_cloud"), f"{dataset}.point_cloud")
        imu = require_object(raw.get("imu"), f"{dataset}.imu")
        wheel = require_object(raw.get("wheel_odometry"), f"{dataset}.wheel_odometry")

        point_topic = require_text(point_cloud, "topic", f"{dataset}.point_cloud")
        point_frame = require_text(point_cloud, "frame", f"{dataset}.point_cloud")
        imu_topic = require_text(imu, "topic", f"{dataset}.imu")
        imu_frame = require_text(imu, "frame", f"{dataset}.imu")
        wheel_topic = wheel.get("topic")
        require(
            wheel_topic is None or (isinstance(wheel_topic, str) and wheel_topic),
            f"{dataset}.wheel_odometry.topic must be null or a nonempty string",
        )

        try:
            map_header = resolver.read_pcd_header(map_source)
            trajectory = resolver.read_trajectory(trajectory_source)
            bag = resolver.inspect_bag(bag_path)
            resolver._require_topic(
                bag, point_topic, "sensor_msgs/msg/PointCloud2", "LiDAR"
            )
            resolver._require_topic(bag, imu_topic, "sensor_msgs/msg/Imu", "IMU")
            if wheel_topic is not None:
                resolver._require_topic(
                    bag, wheel_topic, "nav_msgs/msg/Odometry", "wheel odometry"
                )
        except resolver.ResolverError as exc:
            raise PublisherError(str(exc)) from exc

        declared_map_frame = raw.get("map_frame", trajectory["frame"])
        require(
            isinstance(declared_map_frame, str) and declared_map_frame,
            f"{dataset}.map_frame must be a nonempty string",
        )
        require(
            declared_map_frame == trajectory["frame"],
            f"{dataset} map/trajectory frame mismatch: "
            f"{declared_map_frame} != {trajectory['frame']}",
        )

        datasets[dataset] = {
            "map": {
                "path": f"pcd_full_maps_only/{dataset}_full_map.pcd",
                "sha256": resolver.sha256_file(map_source),
                "frame": declared_map_frame,
            },
            "bag": {
                "path": bag["path"],
                "storage_set_sha256": bag["storage_set_sha256"],
            },
            "trajectory": {
                "path": (
                    "localization_inputs/"
                    f"{dataset}_trajectory_post_optimization.csv"
                ),
                "sha256": resolver.sha256_file(trajectory_source),
                "frame": trajectory["frame"],
            },
            "point_cloud": {"topic": point_topic, "frame": point_frame},
            "imu": {"topic": imu_topic, "frame": imu_frame},
            "wheel_odometry": {"topic": wheel_topic},
            "_publisher_sources": {
                "map": str(map_source),
                "trajectory": str(trajectory_source),
                "map_points": map_header["points"],
            },
        }

    return {
        "schema_version": 1,
        "stage_sequence": sequence,
        "stage_name": stage_name,
        "datasets": datasets,
    }


def contract_payload(contract_with_sources: Mapping[str, Any]) -> str:
    contract = json.loads(json.dumps(contract_with_sources))
    for dataset in DATASETS:
        contract["datasets"][dataset].pop("_publisher_sources")
    return json.dumps(contract, ensure_ascii=False, indent=2) + "\n"


def publication_plan(stage: Path, contract_with_sources: Mapping[str, Any]) -> Dict[str, Any]:
    payload = contract_payload(contract_with_sources)
    import hashlib

    contract_sha = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    return {
        "status": "validated_not_published",
        "stage": str(stage),
        "stage_sequence": contract_with_sources["stage_sequence"],
        "stage_name": contract_with_sources["stage_name"],
        "contract_sha256": contract_sha,
        "publish_order": [
            "localization_inputs",
            resolver.CONTRACT_NAME,
            resolver.READY_MARKER_NAME,
            "pcd_full_maps_only (atomic readiness boundary, last)",
        ],
        "datasets": {
            dataset: {
                "map_source": contract_with_sources["datasets"][dataset][
                    "_publisher_sources"
                ]["map"],
                "map_sha256": contract_with_sources["datasets"][dataset]["map"][
                    "sha256"
                ],
                "map_points": contract_with_sources["datasets"][dataset][
                    "_publisher_sources"
                ]["map_points"],
                "trajectory_source": contract_with_sources["datasets"][dataset][
                    "_publisher_sources"
                ]["trajectory"],
                "trajectory_sha256": contract_with_sources["datasets"][dataset][
                    "trajectory"
                ]["sha256"],
                "bag_path": contract_with_sources["datasets"][dataset]["bag"][
                    "path"
                ],
                "bag_storage_set_sha256": contract_with_sources["datasets"][dataset][
                    "bag"
                ]["storage_set_sha256"],
            }
            for dataset in DATASETS
        },
    }


def publish(stage: Path, spec: Mapping[str, Any], check_only: bool = False) -> Dict[str, Any]:
    unresolved_stage = stage.expanduser()
    require(
        unresolved_stage.is_dir() and not unresolved_stage.is_symlink(),
        f"invalid stage directory: {unresolved_stage}",
    )
    stage = unresolved_stage.resolve()
    contract = normalized_contract(stage, spec)
    plan = publication_plan(stage, contract)
    conflicts = [
        name
        for name in RESERVED_PATHS
        if (stage / name).exists() or (stage / name).is_symlink()
    ]
    plan["target_conflicts"] = conflicts
    plan["publishable"] = not conflicts
    if check_only:
        return plan

    require(not conflicts, f"refusing to overwrite reserved stage paths: {conflicts}")

    temporary = Path(tempfile.mkdtemp(prefix=".localization_publish.", dir=stage))
    installed: List[Path] = []
    try:
        maps = temporary / "pcd_full_maps_only"
        trajectories = temporary / "localization_inputs"
        maps.mkdir()
        trajectories.mkdir()
        for dataset in DATASETS:
            sources = contract["datasets"][dataset]["_publisher_sources"]
            copied_map = maps / f"{dataset}_full_map.pcd"
            copied_trajectory = (
                trajectories / f"{dataset}_trajectory_post_optimization.csv"
            )
            shutil.copy2(sources["map"], copied_map)
            shutil.copy2(
                sources["trajectory"],
                copied_trajectory,
            )
            require(
                resolver.sha256_file(copied_map)
                == contract["datasets"][dataset]["map"]["sha256"],
                f"{dataset} map changed while it was being copied",
            )
            require(
                resolver.sha256_file(copied_trajectory)
                == contract["datasets"][dataset]["trajectory"]["sha256"],
                f"{dataset} trajectory changed while it was being copied",
            )

        payload = contract_payload(contract)
        contract_path = temporary / resolver.CONTRACT_NAME
        contract_path.write_text(payload, encoding="utf-8")
        marker_path = temporary / resolver.READY_MARKER_NAME
        marker_path.write_text(
            f"{plan['contract_sha256']}  {resolver.CONTRACT_NAME}\n",
            encoding="utf-8",
        )

        # Non-advertising paths first; canonical map directory is the atomic
        # readiness boundary consumed by discover_stage().
        for source, destination in (
            (trajectories, stage / "localization_inputs"),
            (contract_path, stage / resolver.CONTRACT_NAME),
            (marker_path, stage / resolver.READY_MARKER_NAME),
            (maps, stage / "pcd_full_maps_only"),
        ):
            os.replace(source, destination)
            installed.append(destination)

        try:
            resolved = resolver.resolve(stage.parent, stage.name)
        except resolver.ResolverError as exc:
            raise PublisherError(
                f"published bundle failed resolver verification: {exc}"
            ) from exc
    except Exception:
        # All targets were proven absent before publication, so only paths
        # created by this attempt are removed. The canonical map directory is
        # last and no later operation can expose a partial ready stage.
        for path in reversed(installed):
            if path.is_dir() and not path.is_symlink():
                shutil.rmtree(path)
            elif path.exists() or path.is_symlink():
                path.unlink()
        raise
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)

    plan["status"] = "published_and_resolver_verified"
    plan["resolver_stage"] = resolved["stage"]
    return plan


def parse_arguments(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, type=Path)
    parser.add_argument("--spec", required=True, type=Path)
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="validate sources and print the deterministic plan without writing",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_arguments(argv)
    try:
        spec = resolver._load_json(args.spec.expanduser().resolve())
        result = publish(args.stage, spec, args.check_only)
    except (PublisherError, resolver.ResolverError, OSError) as exc:
        print(f"publish_localization_inputs: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
