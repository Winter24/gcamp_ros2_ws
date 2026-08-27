import importlib.util
from pathlib import Path

import numpy as np
import pytest
import cv2


MODULE_PATH = Path(__file__).parents[1] / "launch" / "sq_dataset_playback.py"
SPEC = importlib.util.spec_from_file_location("sq_dataset_playback", MODULE_PATH)
playback = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(playback)


DATA_ROOT = Path("/home/winter24/ros2_ws/Data_1/fused/9")


def test_timestamp_mapping_hits_endpoints_and_midpoint():
    assert playback.map_timestamp_to_camera_frame(100, 100, 200, 600, 10500) == 600
    assert playback.map_timestamp_to_camera_frame(150, 100, 200, 600, 10500) == 5550
    assert playback.map_timestamp_to_camera_frame(200, 100, 200, 600, 10500) == 10500


def test_real_sequences_have_expected_counts_endpoints_and_gap():
    sq1 = playback.load_sequence_entries(DATA_ROOT, playback.SEQUENCE_CONFIGS["SQ1"])
    sq2 = playback.load_sequence_entries(DATA_ROOT, playback.SEQUENCE_CONFIGS["SQ2"])

    assert len(sq1) == 1487
    assert len(sq2) == 856
    assert sq1[0].camera_frame == 600
    assert sq1[-1].camera_frame == 10500
    assert sq2[0].camera_frame == 11800
    assert sq2[-1].camera_frame == 17630
    assert sq2[0].timestamp_ns - sq1[-1].timestamp_ns == 21_951_904_297


def test_missing_camera_frame_is_rejected(tmp_path):
    sequence = tmp_path / "SQ1"
    (sequence / "velodyne").mkdir(parents=True)
    (sequence / "timestamps.txt").write_text("100\n200\n", encoding="utf-8")
    np.zeros((1, 4), dtype=np.float32).tofile(sequence / "velodyne" / "000000.bin")
    np.zeros((1, 4), dtype=np.float32).tofile(sequence / "velodyne" / "000001.bin")
    (tmp_path / "camera frame" / "IMG_1474_frames").mkdir(parents=True)
    config = playback.SequenceConfig("SQ1", 1, 2)

    with pytest.raises(FileNotFoundError, match="Camera frame"):
        playback.load_sequence_entries(tmp_path, config)


def test_malformed_kitti_pointcloud_is_rejected(tmp_path):
    cloud = tmp_path / "bad.bin"
    np.arange(5, dtype=np.float32).tofile(cloud)

    with pytest.raises(ValueError, match="multiple of 4"):
        playback.load_kitti_points(cloud)


def test_message_pair_uses_identical_source_timestamp(tmp_path):
    cloud = tmp_path / "cloud.bin"
    image = tmp_path / "image.jpg"
    np.array([[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 0.8]], dtype=np.float32).tofile(cloud)
    assert cv2.imwrite(str(image), np.zeros((12, 16, 3), dtype=np.uint8))
    entry = playback.PlaybackEntry("SQ1", 1_786_437_237_101_916_075, cloud, 600, image)

    pointcloud_msg, image_msg, compressed_msg = playback.build_messages(entry, "velodyne")

    stamps = {
        (message.header.stamp.sec, message.header.stamp.nanosec)
        for message in (pointcloud_msg, image_msg, compressed_msg)
    }
    assert stamps == {(1_786_437_237, 101_916_075)}
    assert [field.name for field in pointcloud_msg.fields] == ["x", "y", "z", "intensity"]
    assert pointcloud_msg.width == 2
    assert image_msg.encoding == "bgr8"
    assert (image_msg.width, image_msg.height) == (16, 12)
    assert compressed_msg.format == "jpeg"
    assert bytes(compressed_msg.data).startswith(b"\xff\xd8")


def test_compressed_only_message_build_skips_raw_image(tmp_path):
    cloud = tmp_path / "cloud.bin"
    image = tmp_path / "image.jpg"
    np.array([[1.0, 2.0, 3.0, 0.5]], dtype=np.float32).tofile(cloud)
    source = np.zeros((120, 80, 3), dtype=np.uint8)
    source[:60, :, 1] = 200
    assert cv2.imwrite(str(image), source)
    entry = playback.PlaybackEntry("SQ1", 1_000_000_002, cloud, 600, image)

    pointcloud_msg, image_msg, compressed_msg = playback.build_messages(
        entry, "velodyne", include_raw_image=False
    )

    assert pointcloud_msg.header.stamp.nanosec == 2
    assert image_msg is None
    decoded = cv2.imdecode(np.frombuffer(compressed_msg.data, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[:2] == (60, 80)


def test_playback_deadline_does_not_accumulate_processing_time():
    assert playback.playback_deadline(100.0, 1_000, 1_500, 2.0) == 100.00000025


def test_publish_stops_cleanly_when_ros_context_is_shutdown(monkeypatch):
    class ClosedPublisher:
        def publish(self, _message):
            raise playback.RCLError("publisher context is invalid")

    monkeypatch.setattr(playback.rclpy, "ok", lambda: False)

    assert playback.publish_messages(
        (ClosedPublisher(), ClosedPublisher(), ClosedPublisher()),
        (object(), object(), object()),
    ) is False
