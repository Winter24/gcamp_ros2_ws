#!/usr/bin/env python3
"""Replay timestamp-synchronized SQ camera and KITTI-format LiDAR data."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import Header


class SequenceConfig(NamedTuple):
    name: str
    camera_start: int
    camera_end: int


class PlaybackEntry(NamedTuple):
    sequence: str
    timestamp_ns: int
    pointcloud_path: Path
    camera_frame: int
    camera_path: Path


SEQUENCE_CONFIGS = {
    "SQ1": SequenceConfig("SQ1", 600, 10500),
    "SQ2": SequenceConfig("SQ2", 11800, 17630),
}


def map_timestamp_to_camera_frame(
    timestamp_ns: int,
    first_ns: int,
    last_ns: int,
    camera_start: int,
    camera_end: int,
) -> int:
    """Map a source timestamp linearly to the nearest inclusive camera frame."""
    if last_ns <= first_ns:
        raise ValueError("Sequence must contain an increasing timestamp range")
    if not first_ns <= timestamp_ns <= last_ns:
        raise ValueError("Timestamp is outside the sequence range")
    if camera_end < camera_start:
        raise ValueError("Camera frame range is reversed")

    ratio = (timestamp_ns - first_ns) / (last_ns - first_ns)
    mapped = camera_start + ratio * (camera_end - camera_start)
    return math.floor(mapped + 0.5)


def load_kitti_points(path: Path) -> np.ndarray:
    """Load one KITTI-style float32 XYZI point-cloud file."""
    values = np.fromfile(path, dtype=np.float32)
    if values.size % 4 != 0:
        raise ValueError(
            f"Point cloud float count must be a multiple of 4: {path} "
            f"({values.size} values)"
        )
    return values.reshape((-1, 4))


def _read_timestamps(path: Path) -> list[int]:
    if not path.is_file():
        raise FileNotFoundError(f"Timestamp file not found: {path}")
    try:
        timestamps = [int(line.strip()) for line in path.read_text().splitlines() if line.strip()]
    except ValueError as error:
        raise ValueError(f"Invalid nanosecond timestamp in {path}") from error
    if len(timestamps) < 2:
        raise ValueError(f"At least two timestamps are required: {path}")
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise ValueError(f"Timestamps must be strictly increasing: {path}")
    return timestamps


def load_sequence_entries(data_root: Path, config: SequenceConfig) -> list[PlaybackEntry]:
    """Validate and construct one sequence's synchronized playback entries."""
    data_root = Path(data_root)
    sequence_dir = data_root / config.name
    timestamps = _read_timestamps(sequence_dir / "timestamps.txt")
    camera_dir = data_root / "camera frame" / "IMG_1474_frames"
    first_ns, last_ns = timestamps[0], timestamps[-1]
    entries = []

    for index, timestamp_ns in enumerate(timestamps):
        pointcloud_path = sequence_dir / "velodyne" / f"{index:06d}.bin"
        if not pointcloud_path.is_file():
            raise FileNotFoundError(f"Point cloud not found: {pointcloud_path}")

        camera_frame = map_timestamp_to_camera_frame(
            timestamp_ns,
            first_ns,
            last_ns,
            config.camera_start,
            config.camera_end,
        )
        camera_path = camera_dir / f"frame_{camera_frame:06d}.jpg"
        if not camera_path.is_file():
            raise FileNotFoundError(f"Camera frame not found: {camera_path}")

        entries.append(
            PlaybackEntry(
                config.name,
                timestamp_ns,
                pointcloud_path,
                camera_frame,
                camera_path,
            )
        )

    return entries


def _header_from_nanoseconds(timestamp_ns: int, frame_id: str) -> Header:
    header = Header()
    header.stamp.sec, header.stamp.nanosec = divmod(timestamp_ns, 1_000_000_000)
    header.frame_id = frame_id
    return header


def build_messages(
    entry: PlaybackEntry, frame_id: str, include_raw_image: bool = True
) -> tuple[PointCloud2, Image | None, CompressedImage]:
    """Build synchronized ROS messages for one validated playback entry."""
    points = np.ascontiguousarray(load_kitti_points(entry.pointcloud_path), dtype=np.float32)
    image = cv2.imread(str(entry.camera_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not decode camera frame: {entry.camera_path}")

    pointcloud_msg = PointCloud2()
    pointcloud_msg.header = _header_from_nanoseconds(entry.timestamp_ns, frame_id)
    pointcloud_msg.height = 1
    pointcloud_msg.width = points.shape[0]
    pointcloud_msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    pointcloud_msg.is_bigendian = False
    pointcloud_msg.point_step = 16
    pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width
    pointcloud_msg.data = points.tobytes()
    pointcloud_msg.is_dense = bool(np.isfinite(points[:, :3]).all())

    image_msg = None
    if include_raw_image:
        height, width = image.shape[:2]
        image_msg = Image()
        image_msg.header = _header_from_nanoseconds(entry.timestamp_ns, frame_id)
        image_msg.height = height
        image_msg.width = width
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = False
        image_msg.step = width * 3
        image_msg.data = image.tobytes()

    compressed_msg = CompressedImage()
    compressed_msg.header = _header_from_nanoseconds(entry.timestamp_ns, frame_id)
    compressed_msg.format = "jpeg"
    cropped = image[: image.shape[0] // 2, :]
    if cropped.shape[1] > 640:
        scale = 640.0 / cropped.shape[1]
        cropped = cv2.resize(
            cropped,
            (640, max(1, round(cropped.shape[0] * scale))),
            interpolation=cv2.INTER_AREA,
        )
    encoded, jpeg = cv2.imencode(
        ".jpg", cropped, [cv2.IMWRITE_JPEG_QUALITY, 85]
    )
    if not encoded:
        raise ValueError(f"Could not encode camera frame: {entry.camera_path}")
    compressed_msg.data = jpeg.tobytes()
    return pointcloud_msg, image_msg, compressed_msg


def playback_deadline(
    wall_start: float, first_timestamp_ns: int, timestamp_ns: int, rate: float
) -> float:
    """Return an absolute monotonic deadline for a source timestamp."""
    return wall_start + (timestamp_ns - first_timestamp_ns) / 1e9 / rate


def publish_messages(publishers, messages) -> bool:
    """Publish one synchronized trio, returning false during ROS shutdown."""
    try:
        for publisher, message in zip(publishers, messages):
            if publisher is None or message is None:
                continue
            publisher.publish(message)
    except RCLError:
        if not rclpy.ok():
            return False
        raise
    return True


class SqDatasetPlayback(Node):
    """Publish synchronized SQ dataset pairs at their original source timing."""

    def __init__(self) -> None:
        super().__init__("sq_dataset_playback")
        self.declare_parameter("data_root", "/home/winter24/ros2_ws/Data_1/fused/9")
        self.declare_parameter("sequence", "both")
        self.declare_parameter("playback_rate", 1.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("publish_raw_image", False)
        self.declare_parameter("pointcloud_topic", "/fused_points")
        self.declare_parameter("image_topic", "/camera/camera/image_raw")
        self.declare_parameter(
            "compressed_image_topic", "/camera/camera/image_raw/compressed"
        )
        self.declare_parameter("frame_id", "velodyne")

        self.data_root = Path(self.get_parameter("data_root").value)
        sequence = str(self.get_parameter("sequence").value).upper()
        self.playback_rate = float(self.get_parameter("playback_rate").value)
        self.loop_playback = bool(self.get_parameter("loop").value)
        self.publish_raw_image = bool(self.get_parameter("publish_raw_image").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        if self.playback_rate <= 0.0:
            raise ValueError("playback_rate must be positive")
        if sequence not in {"SQ1", "SQ2", "BOTH"}:
            raise ValueError("sequence must be SQ1, SQ2, or both")

        names = ["SQ1", "SQ2"] if sequence == "BOTH" else [sequence]
        self.entries = []
        for name in names:
            self.entries.extend(
                load_sequence_entries(self.data_root, SEQUENCE_CONFIGS[name])
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("pointcloud_topic").value), qos
        )
        self.image_pub = None
        if self.publish_raw_image:
            self.image_pub = self.create_publisher(
                Image, str(self.get_parameter("image_topic").value), qos
            )
        self.compressed_image_pub = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("compressed_image_topic").value),
            qos,
        )
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._playback_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"Loaded {len(self.entries)} synchronized frames ({sequence}), "
            f"playback_rate={self.playback_rate:g}"
        )

    def destroy_node(self):
        self._stop_event.set()
        if self._worker.is_alive() and threading.current_thread() is not self._worker:
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _playback_loop(self) -> None:
        while not self._stop_event.is_set():
            wall_start = time.monotonic()
            first_timestamp = self.entries[0].timestamp_ns
            for index, entry in enumerate(self.entries):
                if self._stop_event.is_set():
                    return
                deadline = playback_deadline(
                    wall_start,
                    first_timestamp,
                    entry.timestamp_ns,
                    self.playback_rate,
                )
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    if self._stop_event.wait(remaining):
                        return

                try:
                    messages = build_messages(
                        entry,
                        self.frame_id,
                        include_raw_image=self.publish_raw_image,
                    )
                except Exception as error:
                    self.get_logger().error(str(error))
                    self._stop_event.set()
                    return
                if not publish_messages(
                    (
                        self.pointcloud_pub,
                        self.image_pub,
                        self.compressed_image_pub,
                    ),
                    messages,
                ):
                    return
                if index == 0 or (index + 1) % 100 == 0:
                    self.get_logger().info(
                        f"Published {index + 1}/{len(self.entries)}: "
                        f"{entry.sequence} LiDAR={entry.pointcloud_path.name} "
                        f"camera=frame_{entry.camera_frame:06d}.jpg"
                    )

            if not self.loop_playback:
                self.get_logger().info("Playback complete")
                return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SqDatasetPlayback()
        rclpy.spin(node)
    except (FileNotFoundError, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"sq_dataset_playback: {error}")
        raise SystemExit(2) from error
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
