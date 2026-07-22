import json
import math
import os
import signal
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from gole_interfaces.msg import TagInfoArray
from tf2_ros import Buffer, LookupException, TransformException, TransformListener


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def expand_path(path: str) -> str:
    return os.path.abspath(os.path.expandvars(os.path.expanduser(path)))


def quat_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def quat_dot(a: Quaternion, b: Quaternion) -> float:
    return sum(x * y for x, y in zip(a, b))


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return quat_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def quat_rotate(q: Quaternion, v: Vector3) -> Vector3:
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def stddev(values: List[float], avg: float) -> float:
    if len(values) < 2:
        return 0.0
    return math.sqrt(sum((value - avg) ** 2 for value in values) / (len(values) - 1))


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


@dataclass
class TagSample:
    stamp_sec: float
    frame_id: str
    position: Vector3
    orientation: Quaternion


@dataclass
class TagAccumulator:
    tag_id: int
    samples: List[TagSample] = field(default_factory=list)

    def add(self, sample: TagSample) -> None:
        self.samples.append(sample)

    def summary(self) -> Dict:
        positions = [sample.position for sample in self.samples]
        xs = [position[0] for position in positions]
        ys = [position[1] for position in positions]
        zs = [position[2] for position in positions]

        mean_position = (mean(xs), mean(ys), mean(zs))
        mean_orientation = self._average_quaternions(
            [sample.orientation for sample in self.samples]
        )
        distances = [
            math.sqrt(
                (position[0] - mean_position[0]) ** 2
                + (position[1] - mean_position[1]) ** 2
                + (position[2] - mean_position[2]) ** 2
            )
            for position in positions
        ]

        return {
            "id": self.tag_id,
            "sample_count": len(self.samples),
            "frame_id": self.samples[-1].frame_id if self.samples else "",
            "pose": {
                "position": {
                    "x": mean_position[0],
                    "y": mean_position[1],
                    "z": mean_position[2],
                },
                "orientation": {
                    "x": mean_orientation[0],
                    "y": mean_orientation[1],
                    "z": mean_orientation[2],
                    "w": mean_orientation[3],
                },
            },
            "position_stddev": {
                "x": stddev(xs, mean_position[0]),
                "y": stddev(ys, mean_position[1]),
                "z": stddev(zs, mean_position[2]),
                "distance": stddev(distances, mean(distances)),
            },
        }

    @staticmethod
    def _average_quaternions(quaternions: List[Quaternion]) -> Quaternion:
        if not quaternions:
            return 0.0, 0.0, 0.0, 1.0

        reference = quat_normalize(quaternions[0])
        accum = [0.0, 0.0, 0.0, 0.0]
        for quat in quaternions:
            normalized = quat_normalize(quat)
            if quat_dot(reference, normalized) < 0.0:
                normalized = tuple(-value for value in normalized)  # type: ignore[assignment]
            for index, value in enumerate(normalized):
                accum[index] += value

        return quat_normalize(tuple(accum))  # type: ignore[arg-type]


class CreateTagMapNode(Node):
    def __init__(self) -> None:
        super().__init__("create_tag_map_node")

        self.declare_parameter("tag_topic", "/tag_array")
        self.declare_parameter("map_name", "generated_tag_map")
        self.declare_parameter("output_json", "~/.ros/tag_maps/generated_tag_map.json")
        self.declare_parameter("output_yaml", "~/.ros/db/generated_tag_map.yaml")
        self.declare_parameter("save_yaml_for_tag_map_db", True)
        self.declare_parameter("overwrite_existing", False)
        self.declare_parameter("append_existing", True)
        self.declare_parameter("save_raw_samples", True)
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("use_tf_target_frame", True)
        self.declare_parameter("min_samples_per_tag", 1)
        self.declare_parameter("capture_duration_sec", 0.0)
        self.declare_parameter("autosave_period_sec", 5.0)

        self.tag_topic = self.get_parameter("tag_topic").value
        self.map_name = self.get_parameter("map_name").value
        self.output_json = expand_path(self.get_parameter("output_json").value)
        self.output_yaml = expand_path(self.get_parameter("output_yaml").value)
        self.save_yaml_for_tag_map_db = bool(
            self.get_parameter("save_yaml_for_tag_map_db").value
        )
        self.overwrite_existing = bool(self.get_parameter("overwrite_existing").value)
        self.append_existing = bool(self.get_parameter("append_existing").value)
        self.save_raw_samples = bool(self.get_parameter("save_raw_samples").value)
        self.target_frame = self.get_parameter("target_frame").value
        self.use_tf_target_frame = bool(self.get_parameter("use_tf_target_frame").value)
        self.min_samples_per_tag = int(self.get_parameter("min_samples_per_tag").value)
        self.capture_duration_sec = float(
            self.get_parameter("capture_duration_sec").value
        )
        self.autosave_period_sec = float(
            self.get_parameter("autosave_period_sec").value
        )

        self.accumulators: Dict[int, TagAccumulator] = {}
        self.loaded_existing_json = False
        self.json_created_by_this_process = False
        self.yaml_created_by_this_process = False
        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None

        if self.append_existing:
            self.loaded_existing_json = self.load_existing_samples()

        if self.use_tf_target_frame:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().warn(
                "use_tf_target_frame is false. Saved poses will stay in the "
                "incoming tag frame and may not be useful for relocalization."
            )

        self.subscription = self.create_subscription(
            TagInfoArray,
            self.tag_topic,
            self.tag_callback,
            10,
        )

        if self.autosave_period_sec > 0.0:
            self.autosave_timer = self.create_timer(
                self.autosave_period_sec, self.save
            )
        else:
            self.autosave_timer = None

        if self.capture_duration_sec > 0.0:
            self.stop_timer = self.create_timer(
                self.capture_duration_sec, self.finish_capture
            )
        else:
            self.stop_timer = None

        self.get_logger().info(
            f"Saving tag observations from {self.tag_topic} to {self.output_json}"
        )
        self.get_logger().info(
            f"Target frame: {self.target_frame}, use_tf_target_frame={self.use_tf_target_frame}"
        )
        if not self.overwrite_existing:
            self.get_logger().info("Existing output files will not be overwritten.")
        if self.loaded_existing_json:
            sample_total = sum(
                len(accumulator.samples) for accumulator in self.accumulators.values()
            )
            self.get_logger().info(
                f"Loaded {sample_total} existing samples from {self.output_json}"
            )

    def load_existing_samples(self) -> bool:
        if not os.path.exists(self.output_json):
            return False

        try:
            with open(self.output_json, "r", encoding="utf-8") as file:
                document = json.load(file)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Could not load existing tag map JSON: {exc}")
            return False

        raw_samples = document.get("raw_samples")
        if isinstance(raw_samples, dict):
            for tag_id_text, samples in raw_samples.items():
                tag_id = int(tag_id_text)
                accumulator = self.accumulators.setdefault(
                    tag_id,
                    TagAccumulator(tag_id),
                )
                for sample in samples:
                    accumulator.add(
                        TagSample(
                            stamp_sec=float(sample.get("stamp_sec", 0.0)),
                            frame_id=str(sample.get("frame_id", "")),
                            position=self.read_position(sample.get("position", {})),
                            orientation=self.read_orientation(
                                sample.get("orientation", {})
                            ),
                        )
                    )
            return True

        loaded_any = False
        for tag in document.get("tags", []):
            tag_id = int(tag["id"])
            pose = tag.get("pose", {})
            accumulator = self.accumulators.setdefault(tag_id, TagAccumulator(tag_id))
            accumulator.add(
                TagSample(
                    stamp_sec=0.0,
                    frame_id=str(tag.get("frame_id", "")),
                    position=self.read_position(pose.get("position", {})),
                    orientation=self.read_orientation(pose.get("orientation", {})),
                )
            )
            loaded_any = True
        return loaded_any

    @staticmethod
    def read_position(position_doc: Dict) -> Vector3:
        return (
            float(position_doc.get("x", 0.0)),
            float(position_doc.get("y", 0.0)),
            float(position_doc.get("z", 0.0)),
        )

    @staticmethod
    def read_orientation(orientation_doc: Dict) -> Quaternion:
        return quat_normalize(
            (
                float(orientation_doc.get("x", 0.0)),
                float(orientation_doc.get("y", 0.0)),
                float(orientation_doc.get("z", 0.0)),
                float(orientation_doc.get("w", 1.0)),
            )
        )

    def tag_callback(self, msg: TagInfoArray) -> None:
        if not msg.tag_array:
            return

        source_frame = msg.header.frame_id or "unknown"
        transform = None
        output_frame = source_frame

        if self.use_tf_target_frame:
            transform = self.lookup_transform(
                self.target_frame, source_frame, msg.header.stamp
            )
            if transform is None:
                return
            output_frame = self.target_frame

        for tag in msg.tag_array:
            position = (float(tag.point.x), float(tag.point.y), float(tag.point.z))
            orientation = quat_normalize(
                (
                    float(tag.quat.x),
                    float(tag.quat.y),
                    float(tag.quat.z),
                    float(tag.quat.w),
                )
            )

            if transform is not None:
                position, orientation = self.apply_transform(
                    position, orientation, transform
                )

            tag_id = int(tag.id)
            accumulator = self.accumulators.setdefault(
                tag_id, TagAccumulator(tag_id)
            )
            accumulator.add(
                TagSample(
                    stamp_sec=stamp_to_sec(msg.header.stamp),
                    frame_id=output_frame,
                    position=position,
                    orientation=orientation,
                )
            )

    def lookup_transform(self, target_frame: str, source_frame: str, stamp):
        if self.tf_buffer is None:
            return None
        if not source_frame or source_frame == "unknown":
            self.get_logger().warn(
                "Incoming TagInfoArray has an empty header.frame_id; cannot TF-transform it.",
                throttle_duration_sec=5.0,
            )
            return None

        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, stamp)
        except (LookupException, TransformException) as exc:
            self.get_logger().warn(
                f"Could not transform {source_frame} to {target_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    @staticmethod
    def apply_transform(
        position: Vector3, orientation: Quaternion, transform
    ) -> Tuple[Vector3, Quaternion]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_position = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )
        transform_orientation = quat_normalize(
            (
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            )
        )

        rotated_position = quat_rotate(transform_orientation, position)
        mapped_position = (
            rotated_position[0] + transform_position[0],
            rotated_position[1] + transform_position[1],
            rotated_position[2] + transform_position[2],
        )
        mapped_orientation = quat_multiply(transform_orientation, orientation)
        return mapped_position, mapped_orientation

    def finish_capture(self) -> None:
        self.save()
        self.get_logger().info(
            "Capture duration reached. Shutting down create_tag_map_node."
        )
        rclpy.shutdown()

    def save(self) -> None:
        summaries = [
            accumulator.summary()
            for accumulator in sorted(
                self.accumulators.values(), key=lambda item: item.tag_id
            )
            if len(accumulator.samples) >= self.min_samples_per_tag
        ]

        document = {
            "schema_version": 1,
            "map_name": self.map_name,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "source_topic": self.tag_topic,
            "target_frame": self.target_frame if self.use_tf_target_frame else None,
            "min_samples_per_tag": self.min_samples_per_tag,
            "tags": summaries,
        }

        if self.save_raw_samples:
            document["raw_samples"] = {
                str(tag_id): [
                    {
                        "stamp_sec": sample.stamp_sec,
                        "frame_id": sample.frame_id,
                        "position": {
                            "x": sample.position[0],
                            "y": sample.position[1],
                            "z": sample.position[2],
                        },
                        "orientation": {
                            "x": sample.orientation[0],
                            "y": sample.orientation[1],
                            "z": sample.orientation[2],
                            "w": sample.orientation[3],
                        },
                    }
                    for sample in accumulator.samples
                ]
                for tag_id, accumulator in sorted(self.accumulators.items())
            }

        json_saved = self.write_json_if_allowed(document)

        if self.save_yaml_for_tag_map_db:
            self.save_legacy_yaml(summaries)

        sample_total = sum(
            len(accumulator.samples) for accumulator in self.accumulators.values()
        )
        if json_saved:
            self.get_logger().info(
                f"Saved {len(summaries)} tags from {sample_total} samples to {self.output_json}"
            )

    def write_json_if_allowed(self, document: Dict) -> bool:
        can_update_existing = self.loaded_existing_json or self.json_created_by_this_process
        if (
            os.path.exists(self.output_json)
            and not self.overwrite_existing
            and not can_update_existing
        ):
            self.get_logger().warn(
                f"Skipping JSON save because file exists: {self.output_json}. "
                "Set overwrite_existing:=true, append_existing:=true, or choose another output_json."
            )
            return False

        os.makedirs(os.path.dirname(self.output_json), exist_ok=True)
        with open(self.output_json, "w", encoding="utf-8") as file:
            json.dump(document, file, indent=2)
            file.write("\n")
        self.json_created_by_this_process = True
        return True

    def save_legacy_yaml(self, summaries: List[Dict]) -> None:
        can_update_existing = (
            self.loaded_existing_json
            or self.yaml_created_by_this_process
            or self.json_created_by_this_process
        )
        if (
            os.path.exists(self.output_yaml)
            and not self.overwrite_existing
            and not can_update_existing
        ):
            self.get_logger().warn(
                f"Skipping YAML save because file exists: {self.output_yaml}. "
                "Set overwrite_existing:=true or choose another output_yaml."
            )
            return

        os.makedirs(os.path.dirname(self.output_yaml), exist_ok=True)
        with open(self.output_yaml, "w", encoding="utf-8") as file:
            file.write("nodes:\n")
            for tag in summaries:
                position = tag["pose"]["position"]
                file.write(f"  - id: {tag['id']}\n")
                file.write(f"    x: {position['x']:.9f}\n")
                file.write(f"    y: {position['y']:.9f}\n")
            file.write("\nedges: []\n")
        self.yaml_created_by_this_process = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CreateTagMapNode()

    def handle_signal(signum, frame) -> None:
        del signum, frame
        node.get_logger().info("Signal received. Saving tag map before shutdown.")
        node.save()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.save()
        node.destroy_node()


if __name__ == "__main__":
    main()
