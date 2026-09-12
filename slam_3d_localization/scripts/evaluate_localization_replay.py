#!/usr/bin/env python3
"""Read-only SQLite replay evaluation: map consistency, not absolute accuracy.

Source ROS 2 before running. Requires numpy, scipy and matplotlib. Captures must
contain /tf, LIO odom, scan_odom and deskewed_points. Optional reference CSV uses
stamp_ns,x,y,z,qx,qy,qz,qw in the same map frame; it is NOT assumed ground truth.
No trajectory fitting, interpolation, future TF, or ICP refinement is applied.
"""
from __future__ import annotations

import argparse
import csv
import json
import sqlite3
import struct
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


PREFIX = "/localization_3d/lio/"
WARNING = ("Map residual and baseline/reference differences are consistency proxies, "
           "not ground-truth accuracy, ATE or RPE. A wrong pose may fit repeated geometry. "
           "Processed scans may differ after deskew; raw-source equivalence must be "
           "verified independently. No trajectory fitting or interpolation is applied.")


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform(position, quaternion):
    q = np.asarray(quaternion, dtype=float)
    p = np.asarray(position, dtype=float)
    if not np.isfinite(q).all() or not np.isfinite(p).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("Non-finite pose or invalid quaternion")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(q).as_matrix()
    matrix[:3, 3] = p
    return matrix


def pose_matrix(pose):
    return transform([pose.position.x, pose.position.y, pose.position.z],
                     [pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w])


def transform_points(matrix, points):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def past_transform(transforms, ordered_stamps, stamp, max_gap_ns):
    """Return only a bounded-age transform at or before the query stamp."""
    index = np.searchsorted(ordered_stamps, stamp, side="right") - 1
    if index < 0 or stamp - int(ordered_stamps[index]) > max_gap_ns:
        return None
    selected = int(ordered_stamps[index])
    return selected, transforms[selected]


def exact_trajectory(odometry, transforms):
    return {stamp: transforms[stamp] @ odometry[stamp]
            for stamp in sorted(odometry.keys() & transforms.keys())}


def distribution(values):
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if not len(array):
        return {"count": 0}
    return dict(count=len(array), mean=float(array.mean()),
                rmse=float(np.sqrt(np.mean(array ** 2))),
                **{f"p{p}": float(np.percentile(array, p)) for p in (50, 95, 99)},
                maximum=float(array.max()))


def pose_differences(current, reference):
    """Exact timestamp comparison in the supplied frame, without fitted alignment."""
    rows = []
    for stamp in sorted(current.keys() & reference.keys()):
        pose, prior = current[stamp], reference[stamp]
        delta = pose[:3, 3] - prior[:3, 3]
        angles = Rotation.from_matrix(prior[:3, :3].T @ pose[:3, :3]).as_euler("xyz", degrees=True)
        rows.append(dict(stamp_ns=stamp, xy_m=float(np.linalg.norm(delta[:2])),
                         z_m=float(delta[2]), roll_deg=float(angles[0]),
                         pitch_deg=float(angles[1]), yaw_deg=float(angles[2])))
    return rows, {key: distribution([abs(row[key]) for row in rows])
                  for key in ("xy_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg")}


def trajectory_summary(trajectory, stationary_prefix_seconds=0):
    stamps = sorted(trajectory)
    positions = np.array([trajectory[stamp][:3, 3] for stamp in stamps])
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    angles = [float(np.degrees(Rotation.from_matrix(
        trajectory[left][:3, :3].T @ trajectory[right][:3, :3]).magnitude()))
        for left, right in zip(stamps, stamps[1:])]
    summary = dict(duration_s=(stamps[-1] - stamps[0]) / 1e9,
                   path_length_3d_m=float(steps.sum()),
                   translation_step_m=distribution(steps), rotation_step_deg=distribution(angles),
                   steps_over_1m=int(np.count_nonzero(steps > 1.0)))
    if stationary_prefix_seconds > 0:
        indices = [index for index, stamp in enumerate(stamps)
                   if stamp - stamps[0] <= stationary_prefix_seconds * 1e9]
        selected = positions[indices]
        summary["assumed_stationary_prefix"] = dict(
            warning="User-specified assumption, not a detected stationary interval or ground truth",
            start_ns=stamps[0], end_ns=stamps[indices[-1]],
            requested_duration_s=stationary_prefix_seconds, count=len(indices),
            max_displacement_3d_m=float(np.max(np.linalg.norm(selected - selected[0], axis=1))),
            endpoint_displacement_3d_m=float(np.linalg.norm(selected[-1] - selected[0])))
    return summary


def voxel_centroids(points, resolution):
    if resolution <= 0:
        raise ValueError("Voxel resolution must be positive")
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return points
    keys = np.floor(points / resolution).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(inverse, weights=points[:, axis]) / counts
                            for axis in range(3)])


def residuals(points, tree, threshold):
    """All finite processed points; no correspondence-distance truncation."""
    if threshold <= 0:
        raise ValueError("Inlier threshold must be positive")
    points = np.asarray(points, dtype=float)
    finite = points[np.isfinite(points).all(axis=1)]
    distances = tree.query(finite, k=1)[0] if len(finite) else np.empty(0)
    result = distribution(distances)
    result.update(input_points=len(points), finite_points=len(finite),
                  inliers=int(np.count_nonzero(distances <= threshold)),
                  inlier_fraction=float(np.mean(distances <= threshold)) if len(distances) else None)
    return result, distances


def lzf_decompress(payload, expected):
    """Decode the PCD binary_compressed LZF stream with length checks."""
    output, cursor = bytearray(), 0
    while cursor < len(payload):
        control = payload[cursor]
        cursor += 1
        if control < 32:
            count = control + 1
            if cursor + count > len(payload):
                raise ValueError("Truncated LZF literal")
            output.extend(payload[cursor:cursor + count])
            cursor += count
        else:
            count, offset = control >> 5, (control & 31) << 8
            if count == 7:
                if cursor >= len(payload):
                    raise ValueError("Truncated LZF length")
                count += payload[cursor]
                cursor += 1
            if cursor >= len(payload):
                raise ValueError("Truncated LZF offset")
            offset += payload[cursor] + 1
            cursor += 1
            if offset > len(output):
                raise ValueError("Invalid LZF back reference")
            for _ in range(count + 2):
                output.append(output[-offset])
        if len(output) > expected:
            raise ValueError("LZF exceeds declared size")
    if len(output) != expected:
        raise ValueError("LZF size mismatch")
    return output


def read_pcd(path):
    with Path(path).open("rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("Incomplete PCD header")
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
        names, sizes, kinds = header["FIELDS"], list(map(int, header["SIZE"])), header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(names))))
        count = int(header["POINTS"][0])
        if not (len(names) == len(sizes) == len(kinds) == len(counts)):
            raise ValueError("PCD field metadata length mismatch")
        types = {"F": "f", "U": "u", "I": "i"}
        fields = [(name, "<" + types[kind] + str(size), (n,))
                  for name, size, kind, n in zip(names, sizes, kinds, counts)]
        for name in ("x", "y", "z"):
            if name not in names or counts[names.index(name)] != 1:
                raise ValueError("PCD requires scalar x/y/z fields")
        dtype = np.dtype(fields)
        encoding = header["DATA"][0].lower()
        if encoding == "ascii":
            raw = np.loadtxt(stream, ndmin=2)
            if raw.shape != (count, sum(counts)):
                raise ValueError("PCD ASCII shape mismatch")
            offsets = np.cumsum([0] + counts[:-1])
            return raw[:, [offsets[names.index(name)] for name in ("x", "y", "z")]]
        if encoding == "binary":
            raw = np.frombuffer(stream.read(count * dtype.itemsize), dtype=dtype, count=count)
            return np.column_stack([raw[name][:, 0] for name in ("x", "y", "z")])
        if encoding != "binary_compressed":
            raise ValueError(f"Unsupported PCD encoding: {encoding}")
        lengths = stream.read(8)
        if len(lengths) != 8:
            raise ValueError("Truncated PCD compressed size header")
        compressed, expanded = struct.unpack("<II", lengths)
        if expanded != count * dtype.itemsize:
            raise ValueError("PCD uncompressed size mismatch")
        payload = stream.read(compressed)
        if len(payload) != compressed:
            raise ValueError("Truncated PCD compressed payload")
        data = lzf_decompress(payload, expanded)
        offset, xyz = 0, {}
        for name, size, kind, n in zip(names, sizes, kinds, counts):
            if name in ("x", "y", "z"):
                xyz[name] = np.frombuffer(data, dtype="<" + types[kind] + str(size), count=count, offset=offset)
            offset += count * size * n
        return np.column_stack([xyz[name] for name in ("x", "y", "z")])


def cloud_xyz(message):
    fields = {field.name: field for field in message.fields}
    types = {7: "f4", 8: "f8"}
    selected = [fields[name] for name in ("x", "y", "z")]
    if any(field.datatype not in types or field.count != 1 for field in selected):
        raise ValueError("Processed cloud requires scalar floating-point x/y/z")
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(dict(names=["x", "y", "z"],
                         formats=[endian + types[field.datatype] for field in selected],
                         offsets=[field.offset for field in selected], itemsize=message.point_step))
    raw = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                     strides=(message.row_step, message.point_step))
    return np.column_stack([raw[name].reshape(-1) for name in ("x", "y", "z")])


def read_capture(path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    files = [path] if path.is_file() else sorted(path.glob("*.db3"))
    if not files:
        raise ValueError(f"No SQLite bag files in {path}")
    result = dict(odom={}, scans={}, clouds={}, transforms={}, topic_counts={}, diagnostics={})
    classes = {}
    wanted = {PREFIX + name for name in ("odom", "scan_odom", "deskewed_points", "diagnostics")} | {"/tf", "/localization_3d/diagnostics"}
    for file in files:
        with sqlite3.connect(file.resolve().as_uri() + "?mode=ro", uri=True) as connection:
            topics = {row[0]: row[1:] for row in connection.execute("SELECT id,name,type FROM topics")}
            for topic_id, n in connection.execute("SELECT topic_id,COUNT(*) FROM messages GROUP BY topic_id"):
                name = topics[topic_id][0]
                result["topic_counts"][name] = result["topic_counts"].get(name, 0) + n
            ids = [key for key, value in topics.items() if value[0] in wanted]
            if not ids:
                continue
            query = f"SELECT topic_id,data FROM messages WHERE topic_id IN ({','.join('?' for _ in ids)}) ORDER BY timestamp,id"
            for topic_id, payload in connection.execute(query, ids):
                name, typename = topics[topic_id]
                if typename not in classes:
                    classes[typename] = get_message(typename)
                message = deserialize_message(payload, classes[typename])
                if name.endswith("/diagnostics"):
                    for status in message.status:
                        values = {item.key: item.value for item in status.values}
                        stats = result["diagnostics"].setdefault(name, dict(messages=0, executions=0, accepted=0, reasons={}))
                        stats["messages"] += 1
                        reason = values.get("reason", status.message)
                        stats["reasons"][reason] = stats["reasons"].get(reason, 0) + 1
                        runtime = values.get("registration_runtime_ms", values.get("registration_backend_runtime_ms", "0"))
                        if float(runtime) > 0:
                            stats["executions"] += 1
                            if values.get("state") == "tracking" and "accepted" in reason:
                                stats["accepted"] += 1
                elif name == "/tf":
                    for item in message.transforms:
                        if (item.header.frame_id.lstrip("/"), item.child_frame_id.lstrip("/")) != ("map", "odom"):
                            continue
                        p, q = item.transform.translation, item.transform.rotation
                        result["transforms"][stamp_ns(item.header.stamp)] = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                elif name.endswith("deskewed_points"):
                    if message.header.frame_id.lstrip("/") != "base_link":
                        raise ValueError(f"Unsupported processed cloud frame: {message.header.frame_id}")
                    result["clouds"][stamp_ns(message.header.stamp)] = cloud_xyz(message)
                else:
                    if (message.header.frame_id.lstrip("/"), message.child_frame_id.lstrip("/")) != ("odom", "base_link"):
                        raise ValueError("LIO odometry must be odom→base_link")
                    key = "scans" if name.endswith("scan_odom") else "odom"
                    result[key][stamp_ns(message.header.stamp)] = pose_matrix(message.pose.pose)
    result["trajectory"] = exact_trajectory(result["odom"], result["transforms"])
    if not result["trajectory"]:
        raise ValueError("No exact-stamp map→odom TF × LIO odometry trajectory")
    return result


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        if rows:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def trajectory_rows(trajectory):
    return [dict(stamp_ns=stamp, x=float(matrix[0, 3]), y=float(matrix[1, 3]),
                 z=float(matrix[2, 3]), **dict(zip(("qx", "qy", "qz", "qw"),
                 Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist())))
            for stamp, matrix in sorted(trajectory.items())]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--baseline-capture", type=Path)
    parser.add_argument("--reference-csv", type=Path)
    parser.add_argument("--map-voxel", type=float, default=0.35)
    parser.add_argument("--inlier-distance", type=float, default=0.5)
    parser.add_argument("--max-tf-age", type=float, default=0.1)
    parser.add_argument("--stationary-prefix-seconds", type=float, default=0,
                        help="Optional stationary assumption from first valid trajectory pose; NOT automatically detected")
    args = parser.parse_args()
    if args.max_tf_age < 0 or args.inlier_distance <= 0 or args.stationary_prefix_seconds < 0:
        parser.error("Time bounds must be nonnegative and inlier-distance positive")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if any((args.output_dir / name).exists() for name in ("metrics.json", "comparison.png")):
        parser.error("Refusing to overwrite prior evaluation artifacts")
    data = {"current": read_capture(args.capture)}
    if args.baseline_capture:
        data["baseline"] = read_capture(args.baseline_capture)
    map_points = voxel_centroids(read_pcd(args.map), args.map_voxel)
    if not len(map_points):
        raise ValueError("Evaluation map is empty")
    tree = cKDTree(map_points)
    common = set.intersection(*(set(run["clouds"]) & set(run["scans"]) for run in data.values()))
    if not common:
        raise ValueError("No common exact-stamp processed-cloud/scan-odom pairs to evaluate")
    valid, matrices = [], {name: {} for name in data}
    stamps = {name: np.array(sorted(run["transforms"]), dtype=np.int64) for name, run in data.items()}
    for stamp in sorted(common):
        selected = {name: past_transform(run["transforms"], stamps[name], stamp, int(args.max_tf_age * 1e9))
                    for name, run in data.items()}
        if all(value is not None for value in selected.values()):
            valid.append(stamp)
            for name, value in selected.items():
                matrices[name][stamp] = (value[0], value[1] @ data[name]["scans"][stamp])
    if not valid:
        raise ValueError("No common scans have a non-future TF within max-tf-age")
    summary = dict(warning=WARNING, capture=str(args.capture), baseline_capture=str(args.baseline_capture),
                   map=str(args.map), map_voxel_m=args.map_voxel, map_points=len(map_points),
                   inlier_distance_m=args.inlier_distance, max_tf_age_s=args.max_tf_age,
                   common_scan_stamps=len(common), evaluated_common_stamps=len(valid),
                   unavailable_past_tf_stamps=len(common) - len(valid), variants={})
    residual_rows, pooled = [], {}
    for name, run in data.items():
        distances = []
        for stamp in valid:
            tf_stamp, matrix = matrices[name][stamp]
            stats, values = residuals(transform_points(matrix, run["clouds"][stamp]), tree, args.inlier_distance)
            residual_rows.append(dict(variant=name, stamp_ns=stamp, tf_age_s=(stamp - tf_stamp) / 1e9, **stats))
            distances.append(values)
        pooled[name] = np.concatenate(distances) if distances else np.empty(0)
        summary["variants"][name] = dict(topic_counts=run["topic_counts"], scan_odom_count=len(run["scans"]),
            cloud_count=len(run["clouds"]), trajectory_count=len(run["trajectory"]),
            trajectory=trajectory_summary(run["trajectory"], args.stationary_prefix_seconds),
            registration_diagnostics=run["diagnostics"],
            map_residual_m=distribution(pooled[name]),
            inlier_fraction=float(np.mean(pooled[name] <= args.inlier_distance)) if len(pooled[name]) else None)
        write_csv(args.output_dir / f"{name}_trajectory.csv", trajectory_rows(run["trajectory"]))
    comparisons = {}
    if "baseline" in data:
        comparisons["baseline"] = data["baseline"]["trajectory"]
    if args.reference_csv:
        with args.reference_csv.open() as stream:
            reference_rows = list(csv.DictReader(stream))
            if any(row.get("frame", "map").lstrip("/") != "map" for row in reference_rows):
                raise ValueError("Reference CSV must already be in the same map frame")
            comparisons["reference_not_gt"] = {int(row["stamp_ns"]): transform(
                [float(row[key]) for key in ("x", "y", "z")],
                [float(row[key]) for key in ("qx", "qy", "qz", "qw")]) for row in reference_rows}
        summary["reference_csv"] = str(args.reference_csv)
    deltas = {}
    for name, reference in comparisons.items():
        rows, stats = pose_differences(data["current"]["trajectory"], reference)
        summary[f"difference_vs_{name}_absolute_components"] = stats
        deltas[name] = rows
        write_csv(args.output_dir / f"difference_vs_{name}.csv", rows)
    write_csv(args.output_dir / "scan_residuals.csv", residual_rows)
    (args.output_dir / "metrics.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    make_plot(args.output_dir, map_points, data, residual_rows, deltas, args.capture, args.inlier_distance)
    print(json.dumps(summary, indent=2, allow_nan=False))


def make_plot(output, points, data, residual_rows, deltas, capture, inlier_distance):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    origin = min(min(run["trajectory"]) for run in data.values())
    sampled = points[::max(1, len(points) // 200000)]
    axes[0, 0].scatter(sampled[:, 0], sampled[:, 1], s=0.2, c="gray", alpha=0.25, label="Prior map")
    all_positions = []
    for name, run in data.items():
        trajectory = np.array([matrix[:3, 3] for _, matrix in sorted(run["trajectory"].items())])
        all_positions.append(trajectory)
        axes[0, 0].plot(trajectory[:, 0], trajectory[:, 1], label=name)
        rows = [row for row in residual_rows if row["variant"] == name and row["count"]]
        time = [(row["stamp_ns"] - origin) / 1e9 for row in rows]
        axes[0, 1].plot(time, [row["p50"] for row in rows], label=f"{name} p50")
        axes[0, 1].plot(time, [row["p95"] for row in rows], "--", label=f"{name} p95")
        axes[1, 0].plot(time, [row["inlier_fraction"] * 100 for row in rows], label=name)
    joined = np.concatenate(all_positions)
    lower, upper = joined.min(axis=0), joined.max(axis=0)
    pad = max(5, float(max(upper[:2] - lower[:2])) * 0.15)
    axes[0, 0].set(xlim=(lower[0] - pad, upper[0] + pad), ylim=(lower[1] - pad, upper[1] + pad),
                   xlabel="X [m], map", ylabel="Y [m], map", title="Exact-stamp TF × odometry")
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 1].set(xlabel="Header time from first trajectory pose [s]", ylabel="Nearest-map distance [m]", title="All finite scan points; no residual cutoff")
    axes[1, 0].set(xlabel="Header time from first trajectory pose [s]", ylabel=f"Inliers ≤ {inlier_distance:g} m [%]", ylim=(0, 100), title="Same scan timestamps, fixed evaluation map")
    for name, rows in deltas.items():
        axes[1, 1].plot([(row["stamp_ns"] - origin) / 1e9 for row in rows], [row["xy_m"] for row in rows], label=f"XY vs {name}")
    axes[1, 1].set(xlabel="Header time from first trajectory pose [s]", ylabel="XY difference [m]", title="No alignment; comparison is not ground truth")
    for axis in axes.flat:
        axis.grid(alpha=0.2)
        if axis.get_legend_handles_labels()[0]:
            axis.legend()
    figure.suptitle(f"Localization consistency | {capture.parent.name}/{capture.name}\nNo ground truth: absolute accuracy is NOT verified")
    figure.savefig(output / "comparison.png", dpi=150)
    plt.close(figure)


if __name__ == "__main__":
    main()
