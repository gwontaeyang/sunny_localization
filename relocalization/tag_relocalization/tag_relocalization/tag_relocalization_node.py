import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from gole_interfaces.msg import TagInfoArray


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def expand_path(path: str) -> str:
    return os.path.abspath(os.path.expandvars(os.path.expanduser(path)))


def quat_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def quat_conjugate(q: Quaternion) -> Quaternion:
    x, y, z, w = quat_normalize(q)
    return -x, -y, -z, w


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


def quat_to_yaw(q: Quaternion) -> float:
    x, y, z, w = quat_normalize(q)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def average_quaternions(quaternions: List[Quaternion]) -> Quaternion:
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


def average_yaw(quaternions: List[Quaternion]) -> Quaternion:
    if not quaternions:
        return 0.0, 0.0, 0.0, 1.0

    sin_sum = 0.0
    cos_sum = 0.0
    for quat in quaternions:
        yaw = quat_to_yaw(quat)
        sin_sum += math.sin(yaw)
        cos_sum += math.cos(yaw)
    return yaw_to_quat(math.atan2(sin_sum, cos_sum))


@dataclass
class Transform:
    position: Vector3
    orientation: Quaternion

    def inverse(self) -> "Transform":
        inv_orientation = quat_conjugate(self.orientation)
        neg_position = (-self.position[0], -self.position[1], -self.position[2])
        inv_position = quat_rotate(inv_orientation, neg_position)
        return Transform(inv_position, inv_orientation)

    def compose(self, other: "Transform") -> "Transform":
        rotated_position = quat_rotate(self.orientation, other.position)
        position = (
            self.position[0] + rotated_position[0],
            self.position[1] + rotated_position[1],
            self.position[2] + rotated_position[2],
        )
        orientation = quat_multiply(self.orientation, other.orientation)
        return Transform(position, orientation)


@dataclass
class MapTag:
    tag_id: int
    transform: Transform
    sample_count: int
    position_stddev_distance: float
    frame_id: str


class TagRelocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("tag_relocalization_node")

        self.declare_parameter("tag_map_json", "~/.ros/tag_maps/generated_tag_map.json")
        self.declare_parameter("tag_topic", "/tag_array")
        self.declare_parameter("pose_topic", "/odom_estimation_node/set_pose")
        self.declare_parameter("initialpose_topic", "")
        self.declare_parameter("debug_pose_topic", "~/estimated_pose")
        self.declare_parameter("map_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("planar_mode", True)
        self.declare_parameter("publish_debug_pose", True)
        self.declare_parameter("require_base_frame_observation", True)
        self.declare_parameter("relocalize_period_sec", 2.0)
        self.declare_parameter("min_map_samples", 5)
        self.declare_parameter("max_map_position_stddev", 0.20)
        self.declare_parameter("max_detection_distance", 5.0)
        self.declare_parameter("position_covariance", 0.05)
        self.declare_parameter("yaw_covariance", 0.05)

        self.tag_map_json = expand_path(self.get_parameter("tag_map_json").value)
        self.tag_topic = self.get_parameter("tag_topic").value
        self.pose_topic = self.resolve_pose_topic()
        self.debug_pose_topic = self.get_parameter("debug_pose_topic").value
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.planar_mode = bool(self.get_parameter("planar_mode").value)
        self.publish_debug_pose = bool(self.get_parameter("publish_debug_pose").value)
        self.require_base_frame_observation = bool(
            self.get_parameter("require_base_frame_observation").value
        )
        self.relocalize_period_sec = float(
            self.get_parameter("relocalize_period_sec").value
        )
        self.min_map_samples = int(self.get_parameter("min_map_samples").value)
        self.max_map_position_stddev = float(
            self.get_parameter("max_map_position_stddev").value
        )
        self.max_detection_distance = float(
            self.get_parameter("max_detection_distance").value
        )
        self.position_covariance = float(self.get_parameter("position_covariance").value)
        self.yaw_covariance = float(self.get_parameter("yaw_covariance").value)

        self.map_tags = self.load_tag_map(self.tag_map_json)
        self.last_publish_time: Optional[rclpy.time.Time] = None

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.pose_topic,
            10,
        )
        self.debug_pose_pub = self.create_publisher(PoseStamped, self.debug_pose_topic, 10)
        self.subscription = self.create_subscription(
            TagInfoArray,
            self.tag_topic,
            self.tag_callback,
            10,
        )

        self.get_logger().info(
            f"Loaded {len(self.map_tags)} mapped tags from {self.tag_map_json}. "
            f"Publishing relocalization to {self.pose_topic}."
        )

    def resolve_pose_topic(self) -> str:
        pose_topic = str(self.get_parameter("pose_topic").value)
        legacy_topic = str(self.get_parameter("initialpose_topic").value)
        if pose_topic:
            return pose_topic
        if legacy_topic:
            self.get_logger().warn(
                "Parameter initialpose_topic is deprecated. Use pose_topic instead."
            )
            return legacy_topic
        return "/odom_estimation_node/set_pose"

    def load_tag_map(self, path: str) -> Dict[int, MapTag]:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Tag map JSON does not exist: {path}")

        with open(path, "r", encoding="utf-8") as file:
            document = json.load(file)

        default_frame_id = str(document.get("target_frame") or self.map_frame_id)
        map_tags: Dict[int, MapTag] = {}
        for tag in document.get("tags", []):
            pose = tag.get("pose", {})
            position_doc = pose.get("position", {})
            orientation_doc = pose.get("orientation", {})
            position = (
                float(position_doc.get("x", 0.0)),
                float(position_doc.get("y", 0.0)),
                float(position_doc.get("z", 0.0)),
            )
            orientation = quat_normalize(
                (
                    float(orientation_doc.get("x", 0.0)),
                    float(orientation_doc.get("y", 0.0)),
                    float(orientation_doc.get("z", 0.0)),
                    float(orientation_doc.get("w", 1.0)),
                )
            )
            position_stddev = tag.get("position_stddev", {})
            tag_id = int(tag["id"])
            map_tags[tag_id] = MapTag(
                tag_id=tag_id,
                transform=Transform(position, orientation),
                sample_count=int(tag.get("sample_count", 0)),
                position_stddev_distance=float(position_stddev.get("distance", 0.0)),
                frame_id=str(tag.get("frame_id") or default_frame_id),
            )

        return map_tags

    def tag_callback(self, msg: TagInfoArray) -> None:
        if not msg.tag_array or not self.should_publish():
            return

        observation_frame = msg.header.frame_id
        if (
            self.require_base_frame_observation
            and observation_frame
            and observation_frame != self.base_frame_id
        ):
            self.get_logger().warn(
                f"Skipping tag observations in frame '{observation_frame}'. "
                f"Expected '{self.base_frame_id}'.",
                throttle_duration_sec=5.0,
            )
            return

        estimates: List[Transform] = []
        matched_ids: List[int] = []

        for tag in msg.tag_array:
            tag_id = int(tag.id)
            map_tag = self.map_tags.get(tag_id)
            if map_tag is None or not self.is_map_tag_usable(map_tag):
                continue

            observed = Transform(
                (
                    float(tag.point.x),
                    float(tag.point.y),
                    float(tag.point.z),
                ),
                quat_normalize(
                    (
                        float(tag.quat.x),
                        float(tag.quat.y),
                        float(tag.quat.z),
                        float(tag.quat.w),
                    )
                ),
            )
            if self.distance(observed.position) > self.max_detection_distance:
                continue

            map_to_base = map_tag.transform.compose(observed.inverse())
            estimates.append(map_to_base)
            matched_ids.append(tag_id)

        if not estimates:
            return

        estimated_pose = self.average_estimates(estimates)
        self.publish_pose_command(estimated_pose, matched_ids)
        self.last_publish_time = self.get_clock().now()

    def should_publish(self) -> bool:
        if self.last_publish_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_publish_time).nanoseconds * 1.0e-9
        return elapsed >= self.relocalize_period_sec

    def is_map_tag_usable(self, map_tag: MapTag) -> bool:
        if map_tag.sample_count < self.min_map_samples:
            return False
        return map_tag.position_stddev_distance <= self.max_map_position_stddev

    @staticmethod
    def distance(position: Vector3) -> float:
        return math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)

    def average_estimates(self, estimates: List[Transform]) -> Transform:
        count = float(len(estimates))
        position = (
            sum(estimate.position[0] for estimate in estimates) / count,
            sum(estimate.position[1] for estimate in estimates) / count,
            sum(estimate.position[2] for estimate in estimates) / count,
        )
        orientation = (
            average_yaw([estimate.orientation for estimate in estimates])
            if self.planar_mode
            else average_quaternions([estimate.orientation for estimate in estimates])
        )

        if self.planar_mode:
            position = (position[0], position[1], 0.0)

        return Transform(position, orientation)

    def publish_pose_command(self, pose: Transform, matched_ids: List[int]) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame_id
        msg.pose.pose.position.x = pose.position[0]
        msg.pose.pose.position.y = pose.position[1]
        msg.pose.pose.position.z = pose.position[2]
        msg.pose.pose.orientation.x = pose.orientation[0]
        msg.pose.pose.orientation.y = pose.orientation[1]
        msg.pose.pose.orientation.z = pose.orientation[2]
        msg.pose.pose.orientation.w = pose.orientation[3]

        msg.pose.covariance[0] = self.position_covariance
        msg.pose.covariance[7] = self.position_covariance
        msg.pose.covariance[14] = self.position_covariance
        msg.pose.covariance[35] = self.yaw_covariance

        self.pose_pub.publish(msg)
        if self.publish_debug_pose:
            self.publish_debug_pose_msg(msg)

        yaw_deg = math.degrees(quat_to_yaw(pose.orientation))
        self.get_logger().info(
            f"Relocalized from tags {matched_ids}: "
            f"x={pose.position[0]:.3f}, y={pose.position[1]:.3f}, yaw={yaw_deg:.1f} deg"
        )

    def publish_debug_pose_msg(self, pose_command: PoseWithCovarianceStamped) -> None:
        msg = PoseStamped()
        msg.header = pose_command.header
        msg.pose = pose_command.pose.pose
        self.debug_pose_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagRelocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
