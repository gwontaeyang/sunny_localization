#!/usr/bin/env python3
"""Resolve a fail-closed bag02/bag03 localization input bundle.

Automatic selection skips newer analysis stages that do not produce
``pcd_full_maps_only`` at all. Once a stage advertises that directory, it is
the newest map-producing stage and must validate completely; a partial or
invalid map stage never falls back silently to an older map.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import sqlite3
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


DATASETS = ("bag02", "bag03")
STAGE_NAME_RE = re.compile(r"^(?P<sequence>[0-9]+)(?:_|$)")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
LEGACY_SEQUENCE = 21
CONTRACT_MIN_SEQUENCE = 22
CONTRACT_NAME = "localization_inputs.json"
READY_MARKER_NAME = "LOCALIZATION_INPUTS_READY"
LEGACY_IMU_TOPIC = "/imu/data"
LEGACY_IMU_FRAME = "cv7_link"


class ResolverError(RuntimeError):
    """Raised when a stage cannot safely be used for localization."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ResolverError(message)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> Dict[str, Any]:
    def reject_duplicate_keys(
        pairs: Iterable[Tuple[str, Any]],
    ) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ResolverError(f"duplicate JSON key in {path}: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=reject_duplicate_keys,
        )
    except ResolverError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ResolverError(f"cannot read JSON {path}: {exc}") from exc
    _require(isinstance(value, dict), f"JSON root must be an object: {path}")
    return value


def _require_regular_file(path: Path, label: str) -> None:
    _require(path.exists(), f"missing {label}: {path}")
    _require(path.is_file(), f"{label} is not a regular file: {path}")
    _require(not path.is_symlink(), f"{label} must not be a symlink: {path}")
    _require(path.stat().st_size > 0, f"{label} is empty: {path}")


def _stage_sequence(path: Path) -> int:
    match = STAGE_NAME_RE.match(path.name)
    _require(match is not None, f"stage name has no numeric prefix: {path.name}")
    return int(match.group("sequence"))


def discover_stage(root: Path, requested: Optional[str]) -> Path:
    root = root.expanduser().resolve()
    _require(root.is_dir(), f"SLAM stage root does not exist: {root}")

    if requested:
        candidate = Path(requested).expanduser()
        if candidate.is_absolute() or "/" in requested:
            candidate = candidate.resolve()
            _require(candidate.is_dir(), f"requested stage does not exist: {candidate}")
            _stage_sequence(candidate)
            return candidate

        exact = root / requested
        if exact.is_dir():
            _stage_sequence(exact)
            return exact.resolve()

        if requested.isdigit():
            sequence = int(requested)
            matches = [
                child
                for child in root.iterdir()
                if child.is_dir()
                and STAGE_NAME_RE.match(child.name)
                and _stage_sequence(child) == sequence
            ]
            _require(matches, f"no stage with sequence {sequence} under {root}")
            _require(
                len(matches) == 1,
                f"ambiguous stage sequence {sequence}: "
                + ", ".join(sorted(item.name for item in matches)),
            )
            return matches[0].resolve()

        raise ResolverError(f"requested stage does not exist under {root}: {requested}")

    by_sequence: Dict[int, List[Path]] = {}
    for child in root.iterdir():
        if not child.is_dir() or child.is_symlink():
            continue
        match = STAGE_NAME_RE.match(child.name)
        if match is None:
            continue
        by_sequence.setdefault(int(match.group("sequence")), []).append(child)
    _require(by_sequence, f"no numeric SLAM stages found under {root}")

    for sequence in sorted(by_sequence, reverse=True):
        map_candidates = [
            candidate
            for candidate in by_sequence[sequence]
            if (candidate / "pcd_full_maps_only").exists()
            or (candidate / "pcd_full_maps_only").is_symlink()
        ]
        if not map_candidates:
            continue
        _require(
            len(map_candidates) == 1,
            f"ambiguous newest map-producing stage sequence {sequence}: "
            + ", ".join(sorted(item.name for item in map_candidates)),
        )
        # Deliberately do not search an older stage if validation below fails.
        return map_candidates[0].resolve()
    raise ResolverError(
        f"no numeric SLAM stage under {root} provides pcd_full_maps_only"
    )


def find_exact_map_pair(stage: Path) -> Dict[str, Path]:
    map_dir = stage / "pcd_full_maps_only"
    _require(
        map_dir.is_dir() and not map_dir.is_symlink(),
        f"selected map-producing stage is incomplete; invalid {map_dir}",
    )
    pcd_files = sorted(
        path
        for path in map_dir.iterdir()
        if path.is_file() and path.suffix.lower() == ".pcd"
    )
    _require(
        len(pcd_files) == len(DATASETS),
        f"{map_dir} must contain exactly two PCDs (bag02 and bag03); "
        f"found {len(pcd_files)}",
    )

    result: Dict[str, Path] = {}
    for dataset in DATASETS:
        matches = [
            path
            for path in pcd_files
            if re.match(rf"^{dataset}(?:_|$)", path.name) is not None
        ]
        _require(
            len(matches) == 1,
            f"{map_dir} must contain exactly one {dataset}_*.pcd",
        )
        _require(not matches[0].is_symlink(), f"map must not be a symlink: {matches[0]}")
        result[dataset] = matches[0].resolve()
    return result


def read_pcd_header(path: Path) -> Dict[str, Any]:
    _require_regular_file(path, "PCD map")
    fields: Dict[str, List[str]] = {}
    bytes_read = 0
    try:
        with path.open("rb") as stream:
            while bytes_read <= 65536:
                line_bytes = stream.readline()
                if not line_bytes:
                    break
                bytes_read += len(line_bytes)
                try:
                    line = line_bytes.decode("ascii").strip()
                except UnicodeDecodeError as exc:
                    raise ResolverError(
                        f"PCD binary payload started before a DATA header: {path}"
                    ) from exc
                if not line or line.startswith("#"):
                    continue
                tokens = line.split()
                fields[tokens[0].upper()] = tokens[1:]
                if tokens[0].upper() == "DATA":
                    break
    except OSError as exc:
        raise ResolverError(f"cannot read PCD header {path}: {exc}") from exc

    required = {"VERSION", "FIELDS", "SIZE", "TYPE", "WIDTH", "HEIGHT", "POINTS", "DATA"}
    missing = sorted(required - fields.keys())
    _require(not missing, f"incomplete PCD header {path}; missing {', '.join(missing)}")
    empty = sorted(key for key in required if not fields[key])
    _require(not empty, f"incomplete PCD header {path}; empty {', '.join(empty)}")
    pcd_fields = fields["FIELDS"]
    _require(all(axis in pcd_fields for axis in ("x", "y", "z")), f"PCD lacks x/y/z: {path}")
    _require(len(fields["SIZE"]) == len(pcd_fields), f"PCD SIZE/FIELDS mismatch: {path}")
    _require(len(fields["TYPE"]) == len(pcd_fields), f"PCD TYPE/FIELDS mismatch: {path}")
    try:
        width = int(fields["WIDTH"][0])
        height = int(fields["HEIGHT"][0])
        points = int(fields["POINTS"][0])
    except (IndexError, ValueError) as exc:
        raise ResolverError(f"invalid PCD dimensions: {path}") from exc
    _require(width > 0 and height > 0 and points > 0, f"empty PCD dimensions: {path}")
    _require(width * height == points, f"PCD WIDTH*HEIGHT does not equal POINTS: {path}")
    data_encoding = fields["DATA"][0] if fields["DATA"] else ""
    _require(
        data_encoding in {"ascii", "binary", "binary_compressed"},
        f"unsupported PCD DATA encoding in {path}: {data_encoding}",
    )
    return {
        "version": fields["VERSION"][0],
        "fields": pcd_fields,
        "width": width,
        "height": height,
        "points": points,
        "data": data_encoding,
    }


def read_trajectory(path: Path) -> Dict[str, Any]:
    _require_regular_file(path, "trajectory")
    frames = set()
    pose_count = 0
    previous_stamp_ns: Optional[int] = None
    first_source_stamp_ns: Optional[int] = None
    last_source_stamp_ns: Optional[int] = None
    minimum_stamp_ns: Optional[int] = None
    maximum_stamp_ns: Optional[int] = None
    timestamp_inversion_count = 0
    timestamp_duplicate_count = 0
    seen_stamps = set()
    minimum_quaternion_norm = math.inf
    maximum_quaternion_norm = 0.0
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            _require(reader.fieldnames is not None, f"trajectory has no CSV header: {path}")
            required = {"stamp_ns", "frame_id", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"}
            missing = sorted(required - set(reader.fieldnames))
            _require(not missing, f"trajectory {path} lacks columns: {', '.join(missing)}")
            for row in reader:
                pose_count += 1
                frame = (row.get("frame_id") or "").strip()
                _require(frame, f"trajectory contains an empty frame_id: {path}")
                frames.add(frame)
                try:
                    stamp_ns = int(row.get("stamp_ns") or "")
                    pose_values = tuple(
                        float(row.get(column) or "")
                        for column in ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")
                    )
                except (TypeError, ValueError) as exc:
                    raise ResolverError(
                        f"trajectory row {pose_count} contains invalid numeric data: {path}"
                    ) from exc
                _require(
                    stamp_ns > 0,
                    f"trajectory row {pose_count} has a non-positive stamp_ns: {path}",
                )
                _require(
                    all(math.isfinite(value) for value in pose_values),
                    f"trajectory row {pose_count} contains non-finite pose data: {path}",
                )
                quaternion_norm = math.sqrt(sum(value * value for value in pose_values[3:]))
                _require(
                    quaternion_norm > 1.0e-12,
                    f"trajectory row {pose_count} has a zero quaternion: {path}",
                )
                minimum_quaternion_norm = min(minimum_quaternion_norm, quaternion_norm)
                maximum_quaternion_norm = max(maximum_quaternion_norm, quaternion_norm)
                if previous_stamp_ns is not None and stamp_ns < previous_stamp_ns:
                    timestamp_inversion_count += 1
                if stamp_ns in seen_stamps:
                    timestamp_duplicate_count += 1
                seen_stamps.add(stamp_ns)
                previous_stamp_ns = stamp_ns
                if first_source_stamp_ns is None:
                    first_source_stamp_ns = stamp_ns
                last_source_stamp_ns = stamp_ns
                minimum_stamp_ns = (
                    stamp_ns if minimum_stamp_ns is None else min(minimum_stamp_ns, stamp_ns)
                )
                maximum_stamp_ns = (
                    stamp_ns if maximum_stamp_ns is None else max(maximum_stamp_ns, stamp_ns)
                )
    except ResolverError:
        raise
    except (OSError, UnicodeError, csv.Error) as exc:
        raise ResolverError(f"cannot read trajectory {path}: {exc}") from exc
    _require(pose_count > 0, f"trajectory contains no poses: {path}")
    _require(len(frames) == 1, f"trajectory contains mixed frames {sorted(frames)}: {path}")
    return {
        "frame": next(iter(frames)),
        "pose_count": pose_count,
        "first_source_stamp_ns": first_source_stamp_ns,
        "last_source_stamp_ns": last_source_stamp_ns,
        "minimum_stamp_ns": minimum_stamp_ns,
        "maximum_stamp_ns": maximum_stamp_ns,
        "timestamp_inversion_count": timestamp_inversion_count,
        "timestamp_duplicate_count": timestamp_duplicate_count,
        "timestamp_order": (
            "strictly_increasing"
            if timestamp_inversion_count == 0 and timestamp_duplicate_count == 0
            else "requires_stable_sort"
        ),
        "timestamp_consumer_policy": "stable_sort_by_stamp_ns_then_source_row",
        "minimum_quaternion_norm": minimum_quaternion_norm,
        "maximum_quaternion_norm": maximum_quaternion_norm,
    }


def _resolve_declared_path(raw_path: Any, stage: Path, label: str) -> Path:
    _require(isinstance(raw_path, str) and raw_path, f"{label}.path must be a nonempty string")
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        path = stage / path
    return path.resolve()


def _natural_path_key(path: Path) -> Tuple[Any, ...]:
    return tuple(
        int(token) if token.isdigit() else token.lower()
        for token in re.split(r"([0-9]+)", path.name)
    )


def _metadata_storage_files(directory: Path) -> Optional[List[Path]]:
    metadata_path = directory / "metadata.yaml"
    if not metadata_path.exists():
        return None
    _require_regular_file(metadata_path, "rosbag metadata")
    try:
        import yaml
    except ImportError as exc:
        raise ResolverError(
            "python3-yaml is required to honor rosbag metadata storage order"
        ) from exc
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ResolverError(f"cannot read rosbag metadata {metadata_path}: {exc}") from exc
    _require(isinstance(metadata, dict), f"invalid rosbag metadata root: {metadata_path}")
    information = metadata.get("rosbag2_bagfile_information")
    _require(isinstance(information, dict), f"invalid rosbag metadata body: {metadata_path}")
    storage_identifier = information.get("storage_identifier")
    _require(
        storage_identifier in (None, "sqlite3"),
        f"unsupported rosbag storage_identifier {storage_identifier!r}: {metadata_path}",
    )
    relative_paths = information.get("relative_file_paths")
    _require(
        isinstance(relative_paths, list) and relative_paths,
        f"rosbag metadata lacks relative_file_paths: {metadata_path}",
    )

    result: List[Path] = []
    seen = set()
    for index, raw_path in enumerate(relative_paths):
        _require(
            isinstance(raw_path, str) and raw_path,
            f"invalid relative_file_paths[{index}] in {metadata_path}",
        )
        relative = Path(raw_path)
        _require(
            not relative.is_absolute() and ".." not in relative.parts,
            f"unsafe rosbag metadata path {raw_path!r}: {metadata_path}",
        )
        candidate = directory / relative
        _require(candidate.suffix == ".db3", f"non-sqlite rosbag storage: {candidate}")
        _require_regular_file(candidate, "source bag storage")
        resolved = candidate.resolve()
        _require(
            directory == resolved or directory in resolved.parents,
            f"rosbag metadata path escapes bag directory: {candidate}",
        )
        _require(resolved not in seen, f"duplicate rosbag storage path: {candidate}")
        seen.add(resolved)
        result.append(resolved)

    discovered = {
        candidate.resolve()
        for candidate in directory.glob("*.db3")
        if candidate.is_file()
    }
    _require(
        discovered == seen,
        f"rosbag metadata/storage set mismatch under {directory}; "
        f"unlisted={sorted(str(path) for path in discovered - seen)}, "
        f"missing={sorted(str(path) for path in seen - discovered)}",
    )
    return result


def _rosbag_storage_files(path: Path) -> List[Path]:
    _require(path.exists(), f"source bag does not exist: {path}")
    if path.is_file():
        files = [path.resolve()] if path.suffix == ".db3" else []
    elif path.is_dir():
        files = _metadata_storage_files(path)
        if files is None:
            files = sorted(
                (item.resolve() for item in path.glob("*.db3") if item.is_file()),
                key=_natural_path_key,
            )
    else:
        files = []
    _require(files, f"source bag has no sqlite3 storage file: {path}")
    for storage in files:
        _require(storage.stat().st_size > 0, f"source bag storage is empty: {storage}")
    return files


def _read_rosbag_topics(storage_files: Iterable[Path]) -> Dict[str, str]:
    topics: Dict[str, str] = {}
    for storage in storage_files:
        try:
            connection = sqlite3.connect(f"file:{storage}?mode=ro", uri=True)
            try:
                rows = connection.execute("SELECT name, type FROM topics").fetchall()
            finally:
                connection.close()
        except sqlite3.Error as exc:
            raise ResolverError(f"cannot inspect rosbag topics in {storage}: {exc}") from exc
        for name, message_type in rows:
            previous = topics.get(name)
            _require(
                previous in (None, message_type),
                f"topic type changes across bag storage files: {name}",
            )
            topics[name] = message_type
    _require(topics, "source bag contains no topics")
    return topics


def inspect_bag(path: Path) -> Dict[str, Any]:
    path = path.expanduser().resolve()
    storage_files = _rosbag_storage_files(path)
    topics = _read_rosbag_topics(storage_files)
    storage_records = []
    set_digest = hashlib.sha256()
    base = path if path.is_dir() else path.parent
    for storage in storage_files:
        digest = sha256_file(storage)
        relative_name = storage.relative_to(base).as_posix()
        set_digest.update(f"{digest}  {relative_name}\n".encode("utf-8"))
        storage_records.append(
            {
                "path": str(storage),
                "size_bytes": storage.stat().st_size,
                "sha256": digest,
            }
        )
    return {
        "path": str(path.resolve()),
        "storage_files": storage_records,
        "storage_set_sha256": set_digest.hexdigest(),
        "storage_set_hash_format": "sha256sum-lines-v1",
        "storage_order_source": (
            "metadata.yaml"
            if path.is_dir() and (path / "metadata.yaml").is_file()
            else "natural_filename_order"
        ),
        "topics": dict(sorted(topics.items())),
    }


def _require_topic(
    bag: Mapping[str, Any], topic: str, expected_type: str, label: str
) -> None:
    topics = bag["topics"]
    _require(topic in topics, f"source bag lacks {label} topic {topic}")
    _require(
        topics[topic] == expected_type,
        f"{label} topic {topic} has type {topics[topic]}, expected {expected_type}",
    )


def _parse_manifest(path: Path, stage: Path) -> Dict[str, str]:
    _require_regular_file(path, "SHA-256 manifest")
    entries: Dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        raise ResolverError(f"cannot read manifest {path}: {exc}") from exc
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        match = re.fullmatch(r"([0-9a-f]{64}) [ *](.+)", line)
        _require(match is not None, f"malformed manifest line {line_number}: {path}")
        relative = match.group(2)
        if relative.startswith("./"):
            relative = relative[2:]
        relative_path = Path(relative)
        _require(
            not relative_path.is_absolute() and ".." not in relative_path.parts,
            f"unsafe manifest path: {relative}",
        )
        key = relative_path.as_posix()
        _require(key not in entries, f"duplicate manifest entry: {key}")
        resolved = (stage / relative_path).resolve()
        _require(
            resolved == stage or stage in resolved.parents,
            f"manifest entry escapes stage: {relative}",
        )
        entries[key] = match.group(1)
    _require(entries, f"manifest contains no entries: {path}")
    return entries


def _validate_manifest_entry(
    stage: Path, entries: Mapping[str, str], path: Path
) -> str:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(stage).as_posix()
    except ValueError as exc:
        raise ResolverError(f"manifest cannot cover external path: {resolved}") from exc
    _require(relative in entries, f"manifest lacks required entry: {relative}")
    _require_regular_file(resolved, "manifest artifact")
    actual = sha256_file(resolved)
    _require(
        actual == entries[relative],
        f"SHA-256 mismatch for {resolved}: expected {entries[relative]}, got {actual}",
    )
    return actual


def _parse_replay_lineage(path: Path) -> Dict[str, str]:
    _require_regular_file(path, "legacy replay log")
    keys = {"dataset", "bag", "pcd", "trajectory", "started_utc", "finished_utc"}
    result: Dict[str, str] = {}
    try:
        for line in path.read_text(encoding="utf-8", errors="strict").splitlines():
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            if key in keys:
                _require(key not in result, f"duplicate {key}= line in {path}")
                result[key] = value.strip()
    except (OSError, UnicodeError) as exc:
        raise ResolverError(f"cannot read legacy replay log {path}: {exc}") from exc
    for key in keys:
        _require(result.get(key), f"legacy replay log lacks {key}= in {path}")
    return result


def _legacy_result_file(result_dir: Path, suffix: str, label: str) -> Path:
    matches = sorted(result_dir.glob(f"*{suffix}"))
    _require(len(matches) == 1, f"expected exactly one {label} in {result_dir}")
    return matches[0].resolve()


def _legacy_validation(stage: Path) -> Dict[str, Dict[str, Any]]:
    path = stage / "validation" / "validation_report.json"
    report = _load_json(path)
    _require(report.get("status") == "passed", f"legacy validation did not pass: {path}")
    rows = report.get("datasets")
    _require(isinstance(rows, list), f"legacy validation lacks datasets: {path}")
    by_dataset: Dict[str, Dict[str, Any]] = {}
    for row in rows:
        _require(isinstance(row, dict), f"invalid validation dataset row: {path}")
        dataset = row.get("dataset")
        _require(isinstance(dataset, str), f"invalid validation dataset name: {dataset}")
        _require(dataset not in by_dataset, f"duplicate validation dataset: {dataset}")
        by_dataset[dataset] = row
    _require(
        set(by_dataset) == set(DATASETS),
        "legacy validation must cover exactly bag02 and bag03",
    )
    for dataset, row in by_dataset.items():
        _require(row.get("validation") == "passed", f"{dataset} validation did not pass")
        _require(isinstance(row.get("pcd_points"), int), f"{dataset} validation lacks pcd_points")
    return by_dataset


def resolve_legacy_stage21(stage: Path, maps: Mapping[str, Path]) -> Dict[str, Any]:
    manifest_path = stage / "manifest.sha256"
    manifest = _parse_manifest(manifest_path, stage)
    validation_path = stage / "validation" / "validation_report.json"
    validation = _legacy_validation(stage)
    _validate_manifest_entry(stage, manifest, validation_path)

    datasets: Dict[str, Any] = {}
    for dataset in DATASETS:
        result_dir = stage / "full_results" / dataset
        _require(result_dir.is_dir(), f"legacy stage lacks full result for {dataset}")
        replay_path = result_dir / "replay.log"
        metrics_path = _legacy_result_file(result_dir, "_metrics.json", "metrics JSON")
        lineage = _parse_replay_lineage(replay_path)
        metrics = _load_json(metrics_path)
        _require(lineage["dataset"] == dataset, f"replay dataset mismatch for {dataset}")

        raw_visibility = metrics.get("raw_visibility")
        _require(isinstance(raw_visibility, dict), f"metrics lacks raw_visibility: {metrics_path}")
        source_bag = Path(lineage["bag"]).expanduser().resolve()
        metric_bag = Path(str(raw_visibility.get("bag_path", ""))).expanduser().resolve()
        _require(
            source_bag == metric_bag,
            f"bag lineage differs between replay and metrics for {dataset}",
        )
        point_topic = raw_visibility.get("topic")
        point_frame = raw_visibility.get("expected_frame_id")
        _require(
            isinstance(point_topic, str) and point_topic,
            f"metrics lacks LiDAR topic for {dataset}",
        )
        _require(
            isinstance(point_frame, str) and point_frame,
            f"metrics lacks LiDAR frame for {dataset}",
        )

        source_map = Path(lineage["pcd"]).expanduser().resolve()
        source_trajectory = Path(lineage["trajectory"]).expanduser().resolve()
        _require_regular_file(source_map, "legacy parent PCD")
        _require(dataset in source_map.name, f"parent PCD does not match {dataset}: {source_map}")
        _require(
            "cv7" in source_map.name and "imu_no_wheel" in source_map.name,
            f"stage21 legacy IMU/no-wheel lineage is not identifiable: {source_map.name}",
        )
        parent_pcd = read_pcd_header(source_map)
        trajectory_info = read_trajectory(source_trajectory)

        output_record = metrics.get("outputs")
        _require(isinstance(output_record, dict), f"metrics lacks outputs: {metrics_path}")
        output_name = Path(str(output_record.get("visibility_pcd", ""))).name
        _require(output_name, f"metrics lacks visibility_pcd output for {dataset}")
        full_result_pcd = (result_dir / output_name).resolve()
        _require_regular_file(full_result_pcd, "legacy full-result PCD")

        selected_map = maps[dataset]
        selected_header = read_pcd_header(selected_map)
        selected_hash = _validate_manifest_entry(stage, manifest, selected_map)
        full_result_hash = _validate_manifest_entry(stage, manifest, full_result_pcd)
        _require(
            selected_hash == full_result_hash,
            f"convenience PCD is not the validated full PCD for {dataset}",
        )
        _require(
            selected_header["points"] == validation[dataset]["pcd_points"],
            f"PCD point count disagrees with validation for {dataset}",
        )
        projection = metrics.get("projection")
        if isinstance(projection, dict) and isinstance(projection.get("input_points"), int):
            _require(
                selected_header["points"] == projection["input_points"],
                f"PCD point count disagrees with metrics for {dataset}",
            )

        _validate_manifest_entry(stage, manifest, replay_path)
        _validate_manifest_entry(stage, manifest, metrics_path)
        bag = inspect_bag(source_bag)
        _require_topic(bag, point_topic, "sensor_msgs/msg/PointCloud2", "LiDAR")
        _require_topic(bag, LEGACY_IMU_TOPIC, "sensor_msgs/msg/Imu", "IMU")
        odom_topics = sorted(
            name for name, message_type in bag["topics"].items()
            if message_type == "nav_msgs/msg/Odometry"
        )
        _require(
            not odom_topics,
            f"stage21 no-wheel lineage contains odometry topics: {odom_topics}",
        )

        datasets[dataset] = {
            "map": {
                "path": str(selected_map),
                "sha256": selected_hash,
                "frame": trajectory_info["frame"],
                "pcd_header": selected_header,
            },
            "parent_map": {
                "path": str(source_map),
                "sha256": sha256_file(source_map),
                "pcd_header": parent_pcd,
            },
            "bag": bag,
            "trajectory": {
                "path": str(source_trajectory),
                "sha256": sha256_file(source_trajectory),
                **trajectory_info,
            },
            "point_cloud": {"topic": point_topic, "frame": point_frame},
            "imu": {
                "topic": LEGACY_IMU_TOPIC,
                "frame": LEGACY_IMU_FRAME,
                "frame_source": "stage21_cv7_imu_no_wheel_lineage",
            },
            "wheel_odometry": {
                "available": False,
                "topic": None,
                "topics": [],
                "undeclared_odometry_topics": [],
            },
            "reference_policy": "integrate_imu_when_wheel_odometry_is_absent",
            "lineage": {
                "replay_log": str(replay_path),
                "metrics": str(metrics_path),
                "validation": str(validation_path),
            },
        }

    return {
        "status": "ready",
        "resolver_schema_version": 1,
        "stage": {
            "sequence": LEGACY_SEQUENCE,
            "name": stage.name,
            "path": str(stage),
            "adapter": "stage21_legacy_lineage",
            "manifest": {
                "path": str(manifest_path),
                "sha256": sha256_file(manifest_path),
            },
        },
        "datasets": datasets,
    }


def _contract_section(record: Mapping[str, Any], key: str, dataset: str) -> Mapping[str, Any]:
    value = record.get(key)
    _require(isinstance(value, dict), f"{dataset}.{key} must be an object")
    return value


def _contract_text(section: Mapping[str, Any], key: str, label: str) -> str:
    value = section.get(key)
    _require(isinstance(value, str) and value, f"{label}.{key} must be a nonempty string")
    return value


def _contract_hash(section: Mapping[str, Any], label: str) -> str:
    value = _contract_text(section, "sha256", label)
    _require(SHA256_RE.fullmatch(value) is not None, f"{label}.sha256 is not lowercase SHA-256")
    return value


def _validate_ready_marker(marker: Path, contract_path: Path, contract_hash: str) -> None:
    _require(marker.exists(), f"newest stage is incomplete; missing {marker}")
    _require(marker.is_file() and not marker.is_symlink(), f"invalid ready marker: {marker}")
    try:
        content = marker.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError) as exc:
        raise ResolverError(f"cannot read ready marker {marker}: {exc}") from exc
    _require(content, f"ready marker must contain the contract SHA-256: {marker}")
    match = re.fullmatch(
        rf"(?P<sha>[0-9a-f]{{64}})(?: [ *]{re.escape(contract_path.name)})?",
        content,
    )
    _require(match is not None, f"ready marker has invalid SHA-256 binding: {marker}")
    _require(
        match.group("sha") == contract_hash,
        f"ready marker does not match {contract_path}",
    )


def resolve_contract_stage(stage: Path, maps: Mapping[str, Path], sequence: int) -> Dict[str, Any]:
    contract_path = stage / CONTRACT_NAME
    marker_path = stage / READY_MARKER_NAME
    _require_regular_file(contract_path, "localization input contract")
    contract_hash = sha256_file(contract_path)
    _validate_ready_marker(marker_path, contract_path, contract_hash)
    contract = _load_json(contract_path)
    _require(
        contract.get("schema_version") == 1,
        "localization_inputs.json schema_version must be 1",
    )
    if "stage_sequence" in contract:
        _require(
            contract["stage_sequence"] == sequence,
            "contract stage_sequence does not match directory",
        )
    if "stage_name" in contract:
        _require(
            contract["stage_name"] == stage.name,
            "contract stage_name does not match directory",
        )
    records = contract.get("datasets")
    _require(isinstance(records, dict), "contract datasets must be an object")
    _require(
        set(records) == set(DATASETS),
        "contract datasets must contain exactly bag02 and bag03",
    )

    resolved: Dict[str, Any] = {}
    for dataset in DATASETS:
        record = records[dataset]
        _require(isinstance(record, dict), f"contract {dataset} must be an object")
        map_section = _contract_section(record, "map", dataset)
        map_path = _resolve_declared_path(map_section.get("path"), stage, f"{dataset}.map")
        _require(
            map_path == maps[dataset],
            f"contract map path is not the exact {dataset} PCD pair member",
        )
        map_hash = sha256_file(map_path)
        _require(
            map_hash == _contract_hash(map_section, f"{dataset}.map"),
            f"map SHA-256 mismatch: {map_path}",
        )
        map_frame = _contract_text(map_section, "frame", f"{dataset}.map")
        pcd_header = read_pcd_header(map_path)

        trajectory_section = _contract_section(record, "trajectory", dataset)
        trajectory_path = _resolve_declared_path(
            trajectory_section.get("path"), stage, f"{dataset}.trajectory"
        )
        _require_regular_file(trajectory_path, f"{dataset} trajectory")
        trajectory_hash = sha256_file(trajectory_path)
        _require(
            trajectory_hash == _contract_hash(trajectory_section, f"{dataset}.trajectory"),
            f"trajectory SHA-256 mismatch: {trajectory_path}",
        )
        trajectory_info = read_trajectory(trajectory_path)
        declared_trajectory_frame = _contract_text(
            trajectory_section, "frame", f"{dataset}.trajectory"
        )
        _require(
            trajectory_info["frame"] == declared_trajectory_frame,
            f"trajectory frame mismatch for {dataset}",
        )
        _require(
            map_frame == declared_trajectory_frame,
            f"map/trajectory frame mismatch for {dataset}",
        )

        bag_section = _contract_section(record, "bag", dataset)
        bag_path = _resolve_declared_path(bag_section.get("path"), stage, f"{dataset}.bag")
        bag = inspect_bag(bag_path)
        expected_bag_hash = _contract_text(
            bag_section, "storage_set_sha256", f"{dataset}.bag"
        )
        _require(
            SHA256_RE.fullmatch(expected_bag_hash) is not None,
            f"{dataset}.bag.storage_set_sha256 is not lowercase SHA-256",
        )
        _require(
            bag["storage_set_sha256"] == expected_bag_hash,
            f"bag storage-set SHA-256 mismatch for {dataset}",
        )

        point_section = _contract_section(record, "point_cloud", dataset)
        point_topic = _contract_text(point_section, "topic", f"{dataset}.point_cloud")
        point_frame = _contract_text(point_section, "frame", f"{dataset}.point_cloud")
        imu_section = _contract_section(record, "imu", dataset)
        imu_topic = _contract_text(imu_section, "topic", f"{dataset}.imu")
        imu_frame = _contract_text(imu_section, "frame", f"{dataset}.imu")
        _require_topic(bag, point_topic, "sensor_msgs/msg/PointCloud2", "LiDAR")
        _require_topic(bag, imu_topic, "sensor_msgs/msg/Imu", "IMU")

        odom_topics = sorted(
            name for name, message_type in bag["topics"].items()
            if message_type == "nav_msgs/msg/Odometry"
        )
        wheel_section = record.get("wheel_odometry")
        declared_wheel_topic: Optional[str] = None
        if wheel_section is not None:
            _require(
                isinstance(wheel_section, dict),
                f"{dataset}.wheel_odometry must be an object",
            )
            declared_topic = wheel_section.get("topic")
            _require(
                declared_topic is None or isinstance(declared_topic, str),
                f"{dataset}.wheel_odometry.topic must be a string or null",
            )
            if declared_topic:
                _require(
                    declared_topic in odom_topics,
                    f"declared wheel odometry topic is absent: {declared_topic}",
                )
                declared_wheel_topic = declared_topic
        undeclared_odometry_topics = [
            topic for topic in odom_topics if topic != declared_wheel_topic
        ]

        resolved[dataset] = {
            "map": {
                "path": str(map_path),
                "sha256": map_hash,
                "frame": map_frame,
                "pcd_header": pcd_header,
            },
            "bag": bag,
            "trajectory": {
                "path": str(trajectory_path),
                "sha256": trajectory_hash,
                **trajectory_info,
            },
            "point_cloud": {"topic": point_topic, "frame": point_frame},
            "imu": {"topic": imu_topic, "frame": imu_frame},
            "wheel_odometry": {
                "available": declared_wheel_topic is not None,
                "topic": declared_wheel_topic,
                "topics": (
                    [declared_wheel_topic]
                    if declared_wheel_topic is not None
                    else []
                ),
                "undeclared_odometry_topics": undeclared_odometry_topics,
            },
            "reference_policy": (
                "declared_wheel_odometry"
                if declared_wheel_topic is not None
                else "integrate_imu_no_declared_wheel_odometry"
            ),
        }

    return {
        "status": "ready",
        "resolver_schema_version": 1,
        "stage": {
            "sequence": sequence,
            "name": stage.name,
            "path": str(stage),
            "adapter": "localization_inputs_v1",
            "contract": {"path": str(contract_path), "sha256": contract_hash},
            "ready_marker": str(marker_path),
        },
        "datasets": resolved,
    }


def resolve(root: Path, requested_stage: Optional[str] = None) -> Dict[str, Any]:
    stage = discover_stage(root, requested_stage)
    sequence = _stage_sequence(stage)
    maps = find_exact_map_pair(stage)
    if sequence == LEGACY_SEQUENCE:
        result = resolve_legacy_stage21(stage, maps)
    else:
        _require(
            sequence >= CONTRACT_MIN_SEQUENCE,
            f"stage {stage.name} predates the supported stage21 lineage adapter",
        )
        result = resolve_contract_stage(stage, maps, sequence)

    root = root.expanduser().resolve()
    skipped = []
    if requested_stage is None:
        for candidate in root.iterdir():
            match = STAGE_NAME_RE.match(candidate.name)
            if (
                not candidate.is_dir()
                or candidate.is_symlink()
                or match is None
                or int(match.group("sequence")) <= sequence
            ):
                continue
            map_dir = candidate / "pcd_full_maps_only"
            if not map_dir.exists() and not map_dir.is_symlink():
                skipped.append(
                    {
                        "sequence": int(match.group("sequence")),
                        "name": candidate.name,
                        "path": str(candidate.resolve()),
                        "reason": "not_a_map_producing_stage",
                    }
                )
    result["selection"] = {
        "mode": (
            "explicit_stage"
            if requested_stage is not None
            else "latest_map_producing_stage"
        ),
        "policy": (
            "skip newer stages without pcd_full_maps_only; fail closed on the "
            "newest stage that advertises pcd_full_maps_only"
        ),
        "skipped_newer_non_map_stages": sorted(
            skipped, key=lambda item: item["sequence"], reverse=True
        ),
    }
    return result


def _write_json_atomic(path: Path, payload: str) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def parse_arguments(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path.home() / "analysis_outputs" / "slam_3d",
        help="numeric SLAM stage root (default: ~/analysis_outputs/slam_3d)",
    )
    parser.add_argument(
        "--stage",
        help="explicit numeric sequence, stage directory name, or absolute stage path",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="validate and print resolved JSON without writing --output",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="atomically write the resolved JSON to this file",
    )
    arguments = parser.parse_args(argv)
    if arguments.check_only and arguments.output is not None:
        parser.error("--check-only and --output are mutually exclusive")
    return arguments


def main(argv: Optional[List[str]] = None) -> int:
    arguments = parse_arguments(argv)
    try:
        result = resolve(arguments.root, arguments.stage)
        payload = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if arguments.output is not None:
            _write_json_atomic(arguments.output, payload)
        sys.stdout.write(payload)
        return 0
    except (OSError, ResolverError) as exc:
        error = {"status": "error", "error": str(exc)}
        sys.stderr.write(json.dumps(error, sort_keys=True) + "\n")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
