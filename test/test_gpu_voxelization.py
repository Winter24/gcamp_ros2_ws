import importlib.util
from pathlib import Path

import numpy as np
import torch


LAUNCH_DIR = Path(__file__).parents[1] / "launch"
SPEC = importlib.util.spec_from_file_location("gcamp_preprocess", LAUNCH_DIR / "preprocess.py")
PREPROCESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPROCESS)
DETECT_PATH = LAUNCH_DIR / "detect.py"

GEOMETRY = {
    "x_min": -2.0, "x_max": 2.0, "x_res": 1.0,
    "y_min": -2.0, "y_max": 2.0, "y_res": 1.0,
    "z_min": -1.0, "z_max": 1.0, "z_res": 1.0,
}


def legacy_model_tensor(points):
    voxel = PREPROCESS.voxelize(points, GEOMETRY)
    return torch.from_numpy(voxel).float().unsqueeze(0).permute(0, 3, 1, 2)


def test_torch_voxelizer_matches_legacy_binary_occupancy():
    points = np.array([
        [-1.5, -1.5, -0.5],
        [-1.5, -1.5, -0.5],  # duplicate remains binary
        [0.2, 1.2, 0.2],
        [1.998, -0.5, 0.0],
        [3.0, 0.0, 0.0],     # outside ROI
        [np.nan, 0.0, 0.0],
    ], dtype=np.float32)
    actual = PREPROCESS.TorchVoxelizer(torch.device("cpu")).voxelize(points, GEOMETRY)

    assert actual.dtype == torch.float32
    assert actual.shape == (1, 2, 4, 4)
    assert actual.is_contiguous()
    finite = points[~np.isnan(points).any(axis=1)]
    assert torch.equal(actual, legacy_model_tensor(finite))


def test_torch_voxelizer_preserves_strict_epsilon_boundaries():
    points = np.array([
        [-1.9995, 0.0, 0.0],  # rejected: not above min + eps
        [-1.9980, 0.0, 0.0],  # accepted
        [1.9995, 0.0, 0.0],   # rejected: not below max - eps
        [1.9980, 0.0, 0.0],   # accepted
    ], dtype=np.float32)
    voxelizer = PREPROCESS.TorchVoxelizer(torch.device("cpu"))

    assert torch.equal(voxelizer.voxelize(points, GEOMETRY), legacy_model_tensor(points))


def test_reused_buffer_is_cleared_between_frames():
    voxelizer = PREPROCESS.TorchVoxelizer(torch.device("cpu"))
    first = voxelizer.voxelize(np.array([[-1.5, -1.5, -0.5]], np.float32), GEOMETRY)
    assert first.count_nonzero().item() == 1

    second = voxelizer.voxelize(np.array([[1.5, 1.5, 0.5]], np.float32), GEOMETRY)

    assert second.count_nonzero().item() == 1
    assert second[0, 0, 0, 0].item() == 0.0

def test_optional_device_rotation_matches_rotated_input():
    points = np.array([[0.5, 1.5, -0.5]], dtype=np.float32)
    voxelizer = PREPROCESS.TorchVoxelizer(torch.device("cpu"))

    actual = voxelizer.voxelize(points, GEOMETRY, yaw_rad=np.pi / 2.0)
    expected = voxelizer.voxelize(
        PREPROCESS.rotate_points_z(points, np.pi / 2.0), GEOMETRY
    ).clone()

    assert torch.equal(actual, expected)


def test_detector_uses_voxelizer_and_keeps_cpu_fallback():
    source = DETECT_PATH.read_text(encoding="utf-8")

    assert "self.torch_voxelizer = TorchVoxelizer(self.device)" in source
    assert "self.torch_voxelizer.voxelize(" in source
    assert "raw_points, geometry, yaw_rad=yaw_rad" in source
    assert "voxel = voxelize(raw_points, geometry)" in source

def test_cuda_path_stages_full_cloud_and_filters_on_device():
    source = (LAUNCH_DIR / "preprocess.py").read_text(encoding="utf-8")

    assert "pin_memory=True" in source
    assert "pinned[:point_count].to(self.device, non_blocking=True)" in source
    assert "torch.isfinite(gpu_points).all(dim=1)" in source
    assert ").to(torch.int32)" in source
    assert "flat_index = (" in source
    assert "np.unique" not in source

    detector_source = DETECT_PATH.read_text(encoding="utf-8")
    assert "self.device.type != 'cuda'" in detector_source
    assert "raw_points = raw_points[np.isfinite(raw_points).all(axis=1)]" in detector_source

def test_detector_uses_latest_frame_worker_and_single_inference_sync_point():
    source = DETECT_PATH.read_text(encoding="utf-8")

    assert "self._latest_msg = (msg, received_at)" in source
    assert "def _latest_frame_worker(self):" in source
    assert "self._latest_msg = None" in source
    assert "torch.cuda.synchronize(self.device)" not in source
    assert "with torch.inference_mode():" in source
    assert "self._model_cache = {}" in source
    assert "self._xyz_host_buffers = {}" in source
    assert "raw_points[:, 0] = cloud_array['x']" in source
    assert "np.column_stack" not in source

def test_postprocess_has_cuda_rotated_nms_path_with_cpu_fallback():
    source = (LAUNCH_DIR / "postprocess.py").read_text(encoding="utf-8")

    assert "from torchvision.ops import nms_rotated" in source
    assert "pred[\"cls\"].is_cuda" in source
    assert "non_max_suppression(" in source
