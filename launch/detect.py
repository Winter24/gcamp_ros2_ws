#!/usr/bin/env python3
import sys
import os
import math
import rclpy
from rclpy.node import Node
import numpy as np
import time
import torch
from torch import nn
import json
import threading

from ament_index_python.packages import get_package_share_directory

# Get package share directory to locate our modules and models
share_dir = get_package_share_directory('gcamp_gazebo')
launch_dir = os.path.join(share_dir, 'launch')
sys.path.insert(0, launch_dir)

from sensor_msgs_py import point_cloud2 as pc2
from preprocess import (
    TorchVoxelizer,
    rotate_detection_boxes_z,
    rotate_points_z,
    voxelize,
)
from postprocess import filter_pred

from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray, Detection3D,  Detection3DArray, ObjectHypothesisWithPose
from std_msgs.msg import Bool, String
import tensorrt as trt

from core.models.backbones.mobilepixor_coordinate_attention import (
    MobilePixorBackBone as MobilePixorBackBone_CA,
)
from core.models.heads.cnn import Header

# Absolute path to the TensorRT engine (model.plan replaces the old "88epoch" checkpoint)
MODEL_PLAN_PATH = "/home/winter24/ros2_ws/src/Ros2-Autonomous-Hmi/model/model.plan"

# Selected SQ2-axis-fixed continuation epoch 4 for /fused_points.
# It has the best balanced Car AP across the held-out SQ1 and SQ2 validations.
# backbone + standard Header, 35 voxel channels, 3 classes Car/Pedestrian/Cyclist).
# Unlike model.plan this is a plain state_dict, so it runs through the network
# rather than the TensorRT engine.
MODEL_UNITREE_CHECKPOINT_PATH = (
    "/home/winter24/ros2_ws/src/ROS2-3D-LiDAR-Object-Detection/"
    "model/finetune/sq2_axis_fixed_balanced_epoch4_checkpoint_pytorch_fp32"
)
MODEL_UNITREE_PLAN_PATH = (
    "/home/winter24/ros2_ws/src/Ros2-Autonomous-Hmi/"
    "model/fused_epoch4_qat_conservative_c/"
    "fused_epoch4_qat_conservative_c_int8.plan"
)

# The displayed /fused_points ROI is the former right-facing ROI rotated a
# further 30 degrees clockwise. Rotate that physical region +120 degrees into
# the model's unchanged forward tensor, then apply the inverse to predictions.
UNITREE_INFERENCE_YAW_RAD = math.radians(120.0)

# --- unitree /fused_points box tuning ---------------------------------------
# The unitree data was captured on a small robot vehicle, but the model output
# is KITTI-scale (cars ~4m) and the LiDAR mounting yaw differs from KITTI, so
# the raw boxes come out too big and pointing the wrong way. These three knobs
# correct that for the 'unitree' dataset ONLY (kitti/sim are untouched).
#
# HOW TO TUNE: run detect.py and watch the "[unitree box]" log lines — they
# print the decoded yaw (degrees) and length/width (metres) of every box.
# Compare against what you see in the GUI and adjust:
#   * heading points the wrong way  -> change UNITREE_YAW_OFFSET_DEG
#     (try 90, -90 or 180 first; those cover axis-swap / front-back flips)
#   * box too big / too small        -> change UNITREE_SIZE_SCALE
#   * box too tall / too short       -> change UNITREE_BOX_HEIGHT
#   * box sits below / above ground  -> change UNITREE_BOX_Z
UNITREE_YAW_OFFSET_DEG = 0.0   # right-90 inverse mapping already restores yaw
UNITREE_SIZE_SCALE     = 0.75  # multiply model length/width (1.0 = unchanged)
UNITREE_BOX_HEIGHT     = 1.0   # box height in metres (model does not predict z)
UNITREE_BOX_Z          = -0.4  # box bottom z in metres (fused ground is higher
                               # than KITTI's -1.0; raise if box still sinks)

DEFAULT_SCORE_THRESHOLD = 0.3
SIMULATION_SCORE_THRESHOLD = 0.45
UNITREE_SCORE_THRESHOLD = 0.5


class UnitreeCAModel(nn.Module):
    """mobilepixor_CA backbone + standard Header, matching the finetune checkpoint.

    Emits the same {cls, offset, size, yaw} dict as CustomModel/TRTEngine so the
    downstream filter_pred works unchanged.
    """

    def __init__(self, backbone_out_dim: int, num_classes: int):
        super().__init__()
        self.backbone = MobilePixorBackBone_CA()
        self.header = Header(num_classes, backbone_out_dim)

    def forward(self, x):
        return self.header(self.backbone(x))


class TRTEngine:
    """Thin wrapper around a serialized TensorRT engine.

    Presents the same dict-output interface as CustomModel so that the rest
    of the pipeline (filter_pred, lidar_cb) works without modification.
    """

    def __init__(self, plan_path: str, device: torch.device):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(plan_path, "rb") as f:
            self._engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self._context = self._engine.create_execution_context()
        self._device = device
        self._last_event = torch.cuda.Event(enable_timing=True)

        # Pre-allocate output tensors on the GPU so we avoid per-call allocs.
        self._output_bufs: dict = {}
        for i in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(i)
            if self._engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
                shape = tuple(self._engine.get_tensor_shape(name))
                self._output_bufs[name] = torch.empty(
                    shape, dtype=torch.float32, device=device
                )

    def __call__(self, voxel_tensor: torch.Tensor) -> dict:
        """Run inference.

        Args:
            voxel_tensor: float32 CUDA tensor, shape (1, C, H, W), contiguous.

        Returns:
            dict with keys ``cls``, ``offset``, ``size``, ``yaw`` — each a
            CUDA tensor — matching the output format of CustomModel.
        """
        voxel_tensor = voxel_tensor.contiguous()
        self._context.set_tensor_address("voxel", voxel_tensor.data_ptr())
        for name, buf in self._output_bufs.items():
            self._context.set_tensor_address(name, buf.data_ptr())

        stream = torch.cuda.current_stream(self._device).cuda_stream
        self._context.execute_async_v3(stream)
        # The caller owns synchronization/timing so this wrapper can be
        # composed with postprocessing on the same CUDA stream.
        self._last_event.record(torch.cuda.current_stream(self._device))

        return {name: buf for name, buf in self._output_bufs.items()}

    # --- compatibility shims so existing code calling model.eval() / model.to()
    #     doesn't crash when the engine is used instead of nn.Module -----------
    def eval(self):
        return self

    def to(self, device):
        return self

class Eval():
  def __init__(self, data_loader, model, device, img_size, config):
    self.data_loader = data_loader
    self.model = model
    self.model.eval()
    self.device = device
    self.image_size = img_size
    self.config = config

class LiDARPedestrianDetection(Node):
    def __init__(self):
        super().__init__("lidar_detection_node")
        
        self.declare_parameter('points_topic', '/points_raw')
        points_topic = self.get_parameter('points_topic').get_parameter_value().string_value
        self.get_logger().info(f"Subscribing to point cloud topic: {points_topic}")

        self.current_topic = points_topic

        # Keep every Nth point of the dense KITTI cloud before voxelizing to cut
        # per-frame preprocessing cost. Only applied for /kitti/point_cloud.
        self.declare_parameter('kitti_downsample_stride', 4)
        self.kitti_downsample_stride = self.get_parameter(
            'kitti_downsample_stride').get_parameter_value().integer_value

        self.subscription = self.create_subscription(
            PointCloud2,
            points_topic,
            self.lidar_cb,
            1)
            
        self.enabled = True
        self.enable_sub = self.create_subscription(
            Bool,
            '/enable_detection',
            self.enable_cb,
            10)
            
        self.topic_update_sub = self.create_subscription(
            String,
            '/pointcloud_topic_update',
            self.topic_update_cb,
            10)
        
        self.pred_pub = self.create_publisher(
            Detection3DArray,
            "/preds",
            10)

        self.stats_pub = self.create_publisher(
            String,
            "/model/inference_stats",
            10)

        if torch.cuda.is_available():
            self.get_logger().info("Using CUDA GPU for inference")
            self.device = torch.device("cuda")
        else: 
            self.get_logger().info("Using CPU for inference")
            self.device = torch.device("cpu")
        self.torch_voxelizer = TorchVoxelizer(self.device)
        self._voxel_path_logged = False
        self._model_cache = {}
        self._xyz_host_buffers = {}
        self._process_lock = threading.RLock()
        self._latest_msg = None
        self._latest_msg_lock = threading.Lock()
        self._latest_msg_ready = threading.Condition(self._latest_msg_lock)
        self._stop_worker = False
        self._worker = threading.Thread(target=self._latest_frame_worker, daemon=True)
        self._worker.start()
            
        # Default model initialization
        self.dataset_type = None
        if '/fused' in points_topic:
            self.load_model("base_demo_unitree.json", MODEL_UNITREE_PLAN_PATH, "unitree")
        else:
            # Both real KITTI replay (/kitti/point_cloud) and Gazebo sim
            # (/points_raw) run through the same model.plan TensorRT engine,
            # which was built with the KITTI voxel geometry (704x800x35).
            # So both MUST use base_demo_kitti.json — using any other geometry
            # produces the wrong spatial shape and garbage predictions.
            self.load_model("base_demo_kitti.json", MODEL_PLAN_PATH, "kitti")
        active_dataset = self.dataset_type
        active_config = self.config
        active_model = self.model
        # TensorRT is CUDA-only; avoid constructing the alternate engine on the
        # CPU fallback path. CUDA hosts can switch topics without reloading.
        if self.device.type == 'cuda':
            self._preload_model("unitree")
            self._preload_model("kitti")
        self.dataset_type = active_dataset
        self.config = active_config
        self.model = active_model

        self.process_times = []
        self.last_cb_time = None   # for real callback-rate FPS
        self.frame_intervals = []  # rolling window of inter-arrival times
        self.last_input_time = None
        self.input_frame_intervals = []
        self.input_fps = 0.0
        self._frame_count = 0
        
        # Tracker initialization
        self.trackers = {}
        self.next_id = 0
        self.max_missed_frames = 8
        self.distance_threshold = 10.0 # meters
        
    def load_model(self, config_name, model_path, dataset_type):
        cached = self._model_cache.get(dataset_type)
        if cached is not None:
            self.config, self.model = cached
            self.dataset_type = dataset_type
            return

        self.dataset_type = dataset_type
        config_path = os.path.join(launch_dir, config_name)
        self.get_logger().info(f"Loading config from: {config_path}")
        with open(config_path, 'r') as f:
            self.config = json.load(f)

        if dataset_type == 'unitree':
            try:
                if self.device.type != 'cuda':
                    raise RuntimeError("TensorRT requires CUDA")
                if not os.path.isfile(model_path):
                    raise FileNotFoundError(model_path)
                self.get_logger().info(
                    f"Loading unitree TensorRT engine from: {model_path}"
                )
                self.model = TRTEngine(model_path, self.device)
            except Exception as exc:
                self.get_logger().warning(
                    "Falling back to unitree PyTorch checkpoint: "
                    f"{type(exc).__name__}: {exc}"
                )
                model_cfg = self.config['model']
                num_classes = self.config['data']['num_classes']
                model = UnitreeCAModel(model_cfg['backbone_out_dim'], num_classes)
                state_dict = torch.load(
                    MODEL_UNITREE_CHECKPOINT_PATH, map_location=self.device
                )
                # Unwrap common checkpoint containers, then strip DataParallel.
                if isinstance(state_dict, dict):
                    for key in ("state_dict", "model_state_dict", "model"):
                        if key in state_dict and isinstance(state_dict[key], dict):
                            state_dict = state_dict[key]
                            break
                state_dict = {
                    (k[7:] if k.startswith("module.") else k): v
                    for k, v in state_dict.items()
                }
                model.load_state_dict(state_dict, strict=True)
                model.to(self.device)
                model.eval()
                self.model = model
        else:
            self.get_logger().info(f"Loading TensorRT engine from: {model_path}")
            self.model = TRTEngine(model_path, self.device)
        self._model_cache[dataset_type] = (self.config, self.model)

    def _preload_model(self, dataset_type):
        """Load the alternate detector once so topic changes do not stall."""
        if dataset_type in self._model_cache:
            return
        if dataset_type == 'unitree':
            self.load_model("base_demo_unitree.json", MODEL_UNITREE_PLAN_PATH, "unitree")
        else:
            self.load_model("base_demo_kitti.json", MODEL_PLAN_PATH, "kitti")
  
    def enable_cb(self, msg):
        self.enabled = msg.data

    def topic_update_cb(self, msg):
        new_topic = msg.data
        self.get_logger().info(f"Received point cloud topic update: {new_topic}")

        with self._process_lock:
            self.current_topic = new_topic

            if '/fused' in new_topic and self.dataset_type != 'unitree':
                self.get_logger().info("Switching model to unitree finetune TensorRT engine...")
                self.load_model("base_demo_unitree.json", MODEL_UNITREE_PLAN_PATH, "unitree")
            elif '/fused' not in new_topic and self.dataset_type != 'kitti':
                # Both /kitti/point_cloud and /points_raw use KITTI geometry.
                self.get_logger().info("Switching model to KITTI TensorRT engine...")
                self.load_model("base_demo_kitti.json", MODEL_PLAN_PATH, "kitti")
        
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            
        self.subscription = self.create_subscription(
            PointCloud2,
            new_topic,
            self.lidar_cb,
            1)
        self.get_logger().info(f"Successfully subscribed to new point cloud topic: {new_topic}")

    def decode_display_yaw(self, model_yaw):
        """Convert decoded model yaw to the display convention for each model."""
        return float(model_yaw) if self.dataset_type == 'unitree' else -float(model_yaw)

    def extract_bboxes(self, predictions, header):
        import math

        config = self.config['data'][self.dataset_type]
        out_size_factor = self.config['data']['out_size_factor']
        if self.current_topic == '/points_raw':
            thres = SIMULATION_SCORE_THRESHOLD
        elif self.dataset_type == 'unitree':
            thres = UNITREE_SCORE_THRESHOLD
        else:
            thres = DEFAULT_SCORE_THRESHOLD
        nms_thres = 0.1
        boxes = filter_pred(predictions, config, out_size_factor, thres, nms_thres)
        if self.dataset_type == 'unitree' and boxes is not None and len(boxes) > 0:
            # Training maps x_model=-y_lidar, y_model=x_lidar. Restore both
            # centers and headings to the original /fused_points LiDAR frame.
            boxes = rotate_detection_boxes_z(boxes, -UNITREE_INFERENCE_YAW_RAD)
        pred_array = Detection3DArray()
        pred_array.header = header
        pred_array.header.frame_id = header.frame_id

        valid_boxes = []
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                center_x, center_y = float(box[2]), float(box[3])
                distance = (center_x**2 + center_y**2)**0.5
                if distance > 70.0:
                    continue
                valid_boxes.append(box)

        # Spatial filter: The model often predicts 2 disjoint boxes for the front and rear of a car's side.
        # We use longitudinal (dl) and lateral (dw) distance to merge boxes on the SAME car,
        # while PREVENTING the deletion of cars in adjacent lanes.
        filtered_valid_boxes = []
        for box in valid_boxes:
            is_dup = False
            cx, cy = float(box[2]), float(box[3])
            for kbox in filtered_valid_boxes:
                kx, ky = float(kbox[2]), float(kbox[3])
                
                # Calculate local longitudinal and lateral distance
                dx = cx - kx
                dy = cy - ky
                kyaw = float(kbox[6])
                dl = abs(dx * math.cos(kyaw) + dy * math.sin(kyaw))
                dw = abs(-dx * math.sin(kyaw) + dy * math.cos(kyaw))

                # Merge if within 6.0m length-wise and 2.0m width-wise (same car)
                # Adjacent lanes are ~3.7m width-wise, so they will NOT be merged.
                if dl < 6.0 and dw < 2.0:
                    is_dup = True
                    break

            if not is_dup:
                filtered_valid_boxes.append(box)
        valid_boxes = filtered_valid_boxes

        matched_ids = set()
        tracked_boxes = []

        for box in valid_boxes:
            center_x, center_y = float(box[2]), float(box[3])
            best_id = None
            best_dist = self.distance_threshold

            for tid, tracker in self.trackers.items():
                if tid in matched_ids:
                    continue
                dist = ((center_x - tracker['x'])**2 + (center_y - tracker['y'])**2)**0.5
                if dist < best_dist:
                    best_dist = dist
                    best_id = tid

            if best_id is not None:
                alpha = 1.0  # Set to 1.0 to snap instantly to the detection (removes rubber-banding lag)
                self.trackers[best_id]['x'] = alpha * center_x + (1 - alpha) * self.trackers[best_id]['x']
                self.trackers[best_id]['y'] = alpha * center_y + (1 - alpha) * self.trackers[best_id]['y']
                
                # Keep orientation and size smoothing to prevent jitter
                alpha_dim = 0.85
                length, width = float(box[4]), float(box[5])
                self.trackers[best_id]['length'] = alpha_dim * length + (1 - alpha_dim) * self.trackers[best_id]['length']
                self.trackers[best_id]['width'] = alpha_dim * width + (1 - alpha_dim) * self.trackers[best_id]['width']
                
                # SQ1/SQ2 canonical labels use the decoded yaw directly.
                # KITTI/simulation retain their established mirrored display
                # convention through decode_display_yaw().
                yaw = self.decode_display_yaw(box[6])
                old_yaw = self.trackers[best_id]['yaw']

                # Handle 180-degree ambiguity in model yaw output
                diff = (yaw - old_yaw + math.pi) % (2 * math.pi) - math.pi
                if diff > math.pi / 2:
                    yaw -= math.pi
                elif diff < -math.pi / 2:
                    yaw += math.pi
                    
                diff = (yaw - old_yaw + math.pi) % (2 * math.pi) - math.pi
                # Detections are in the moving LiDAR frame. While the ego car
                # turns, a static object's relative yaw changes immediately;
                # smoothing here makes the box lag and appear diagonally offset.
                alpha_yaw = 1.0
                new_yaw = old_yaw + alpha_yaw * diff
                self.trackers[best_id]['yaw'] = (new_yaw + math.pi) % (2 * math.pi) - math.pi
                
                self.trackers[best_id]['missed'] = 0
                matched_ids.add(best_id)
                tracked_boxes.append((best_id, box))
            else:
                new_id = self.next_id
                self.next_id += 1
                length, width = float(box[4]), float(box[5])
                yaw = self.decode_display_yaw(box[6])
                self.trackers[new_id] = {
                    'x': center_x, 'y': center_y,
                    'length': length, 'width': width, 'yaw': yaw,
                    'missed': 0
                }
                matched_ids.add(new_id)
                tracked_boxes.append((new_id, box))

        to_delete = []
        for tid in self.trackers:
            if tid not in matched_ids:
                self.trackers[tid]['missed'] += 1
                if self.trackers[tid]['missed'] > self.max_missed_frames:
                    to_delete.append(tid)
        for tid in to_delete:
            del self.trackers[tid]

        for tid, box in tracked_boxes:
            center_x = self.trackers[tid]['x']
            center_y = self.trackers[tid]['y']
            length = self.trackers[tid]['length']
            width = self.trackers[tid]['width']
            yaw = self.trackers[tid]['yaw']

            # unitree /fused_points: robot-scale correction (see the knobs near
            # MODEL_UNITREE_PATH). kitti/sim keep the raw model output.
            if self.dataset_type == 'unitree':
                # Log the RAW decoded values so the offsets/scale can be tuned
                # against what the GUI shows.
                if self._frame_count % 30 == 0:
                    self.get_logger().info(
                        f"[unitree box] id={tid} cls={int(box[0])} "
                        f"yaw={math.degrees(yaw):.1f}deg "
                        f"L={length:.2f}m W={width:.2f}m"
                    )
                yaw = yaw + math.radians(UNITREE_YAW_OFFSET_DEG)
                yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
                length = length * UNITREE_SIZE_SCALE
                width = width * UNITREE_SIZE_SCALE
                box_height = UNITREE_BOX_HEIGHT
            else:
                box_height = 2.0

            pred = Detection3D()
            pred.header.frame_id = header.frame_id
            pred.bbox.center.position.x = center_x
            pred.bbox.center.position.y = center_y
            # z here is the box BOTTOM (the converter extrudes upward by height).
            # fused/unitree ground sits higher than KITTI's, so it needs its own z.
            if self.dataset_type == 'unitree':
                pred.bbox.center.position.z = UNITREE_BOX_Z
            else:
                pred.bbox.center.position.z = -2.0

            # Revert to standard size mapping based on coordinate analysis
            pred.bbox.size.x = length
            pred.bbox.size.y = width
            pred.bbox.size.z = box_height

            pred.bbox.center.orientation.w = math.cos(yaw / 2.0)
            pred.bbox.center.orientation.x = 0.0
            pred.bbox.center.orientation.y = 0.0
            pred.bbox.center.orientation.z = math.sin(yaw / 2.0)

            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = f"{int(box[0])}_{tid}"
            result.hypothesis.score = float(box[1])
            pred.results.append(result)

            pred_array.detections.append(pred)

        return pred_array

    def lidar_cb(self, msg):
        # Keep only the newest cloud. Inference must never build a backlog of
        # stale frames when the producer is faster than the detector.
        received_at = time.perf_counter()
        with self._latest_msg_ready:
            if self.last_input_time is not None:
                self.input_frame_intervals.append(received_at - self.last_input_time)
                if len(self.input_frame_intervals) > 10:
                    self.input_frame_intervals.pop(0)
                self.input_fps = 1.0 / np.mean(self.input_frame_intervals)
            self.last_input_time = received_at
            self._latest_msg = (msg, received_at)
            self._latest_msg_ready.notify()

    def _latest_frame_worker(self):
        while True:
            with self._latest_msg_ready:
                self._latest_msg_ready.wait_for(
                    lambda: self._stop_worker or self._latest_msg is not None
                )
                if self._stop_worker:
                    return
                msg, received_at = self._latest_msg
                self._latest_msg = None
            try:
                with self._process_lock:
                    self._process_cloud(msg, received_at)
            except Exception as exc:
                self.get_logger().error(f"Detection worker failed: {exc}")

    def _process_cloud(self, msg, received_at=None):
        if not getattr(self, 'enabled', True):
            return
            
        # Input
        start_time = time.perf_counter()
        if received_at is None:
            received_at = start_time
        queue_wait_ms = max(0.0, (start_time - received_at) * 1000.0)
        t0 = start_time
        geometry = self.config['data'][self.dataset_type]['geometry']
        
        # Reuse the compact XYZ host buffer so PointCloud2 decoding does not
        # allocate a new Nx3 array for every frame.
        point_count = msg.width * msg.height
        cloud_array = np.ndarray(
            shape=(point_count,),
            dtype=np.dtype([
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('_', np.uint8, msg.point_step - 12)
            ]),
            buffer=msg.data
        )
        raw_points = self._xyz_host_buffers.get(msg.point_step)
        if raw_points is None or raw_points.shape[0] < point_count:
            raw_points = np.empty((max(point_count * 2, 4096), 3), dtype=np.float32)
            self._xyz_host_buffers[msg.point_step] = raw_points
        raw_points = raw_points[:point_count]
        raw_points[:, 0] = cloud_array['x']
        raw_points[:, 1] = cloud_array['y']
        raw_points[:, 2] = cloud_array['z']
        if self.dataset_type == 'unitree' and self.device.type != 'cuda':
            # Map the exact yellow ROI into the checkpoint's unchanged tensor.
            raw_points = rotate_points_z(raw_points, UNITREE_INFERENCE_YAW_RAD)

        # CUDA stages the complete cloud. Keep legacy downsampling only for the
        # CPU fallback, where preprocessing cost otherwise dominates.
        if (self.device.type != 'cuda'
                and self.current_topic == '/kitti/point_cloud'
                and self.kitti_downsample_stride > 1):
            raw_points = raw_points[::self.kitti_downsample_stride]

        t1 = time.perf_counter()

        if self.device.type == 'cuda':
            if not self._voxel_path_logged:
                self.get_logger().info("Voxelization path: CUDA pinned full-cloud")
                self._voxel_path_logged = True
            yaw_rad = UNITREE_INFERENCE_YAW_RAD if self.dataset_type == 'unitree' else None
            voxel_tensor = self.torch_voxelizer.voxelize(
                raw_points, geometry, yaw_rad=yaw_rad
            )
        else:
            if not self._voxel_path_logged:
                self.get_logger().warn("Voxelization path: CPU fallback")
                self._voxel_path_logged = True
            raw_points = raw_points[np.isfinite(raw_points).all(axis=1)]
            voxel = voxelize(raw_points, geometry)
            voxel_tensor = torch.tensor(voxel).float().unsqueeze(0).to(self.device)
            voxel_tensor = voxel_tensor.permute(0, 3, 1, 2)

        if self.dataset_type == 'kitti':
            # TensorRT engine (model.plan) expects exactly 35 channels (fixed at build time)
            trt_channels = 35
            if voxel_tensor.shape[1] != trt_channels:
                voxel_tensor = voxel_tensor[:, :trt_channels, :, :]
        # (unitree PyTorch model's conv1 already takes the 35 z-bins the
        #  unitree geometry produces, so no channel truncation is needed.)
        t2 = time.perf_counter()

        # Predict and extract to bbox. CUDA events avoid device-wide syncs for
        # timing while still making the reported inference time accurate.
        infer_start = torch.cuda.Event(enable_timing=True) if self.device.type == 'cuda' else None
        if infer_start is not None:
            infer_start.record(torch.cuda.current_stream(self.device))
        with torch.inference_mode():
            predictions = self.model(voxel_tensor)
        if infer_start is not None:
            infer_end = getattr(self.model, '_last_event', None)
            if infer_end is None:
                infer_end = torch.cuda.Event(enable_timing=True)
                infer_end.record(torch.cuda.current_stream(self.device))
            infer_end.synchronize()
            infer_ms = infer_start.elapsed_time(infer_end)
            t3 = time.perf_counter()
        else:
            t3 = time.perf_counter()
            infer_ms = (t3 - t2) * 1000.0

        pred_array = self.extract_bboxes(predictions, msg.header)
        self.pred_pub.publish(pred_array)
        process_time = time.perf_counter() - start_time
        detector_total_latency_ms = queue_wait_ms + process_time * 1000.0
        self.process_times.append(process_time)
        self._frame_count += 1

        # Real pipeline FPS: measure time between successive callbacks
        now = time.perf_counter()
        if self.last_cb_time is not None:
            self.frame_intervals.append(now - self.last_cb_time)
            if len(self.frame_intervals) > 10:
                self.frame_intervals.pop(0)
        self.last_cb_time = now

        display_fps = (1.0 / np.mean(self.frame_intervals)) if self.frame_intervals else 0.0
        inference_fps = 1000.0 / infer_ms if infer_ms > 0.0 else 0.0
        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f"frame={self._frame_count} detections={len(pred_array.detections)} "
                f"fps={display_fps:.2f} process={process_time * 1000.0:.1f}ms "
                f"input_fps={self.input_fps:.2f} infer_fps={inference_fps:.2f} "
                f"queue={queue_wait_ms:.1f}ms total={detector_total_latency_ms:.1f}ms "
                f"infer={infer_ms:.1f}ms"
            )

        if len(self.process_times) == 10:
            avg_proc = np.mean(self.process_times)
            if self._frame_count % 30 == 0:
                self.get_logger().info(f"Avg processing time last 10 frames: {avg_proc*1000:.1f} ms")
            self.process_times.pop(0)
            
        gpu_memory = 0.0
        if torch.cuda.is_available():
            gpu_memory = torch.cuda.memory_allocated(self.device) / (1024 ** 2) # MB

        stats = {
            "fps": float(display_fps),
            "input_fps": float(self.input_fps),
            "pipeline_fps": float(display_fps),
            "inference_fps": float(inference_fps),
            "process_time_ms": float(process_time * 1000),
            "queue_wait_ms": float(queue_wait_ms),
            "detector_total_latency_ms": float(detector_total_latency_ms),
            "points_time_ms": float((t1-t0) * 1000),
            "voxel_time_ms": float((t2-t1) * 1000),
            "infer_time_ms": float(infer_ms),
            "gpu_memory_mb": float(gpu_memory)
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.stats_pub.publish(stats_msg)

    def destroy_node(self):
        with self._latest_msg_ready:
            self._stop_worker = True
            self._latest_msg_ready.notify_all()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detector = LiDARPedestrianDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
