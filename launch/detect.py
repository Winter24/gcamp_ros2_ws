#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
import numpy as np
import time
import torch
from torch import nn
import json

from ament_index_python.packages import get_package_share_directory

# Get package share directory to locate our modules and models
share_dir = get_package_share_directory('gcamp_gazebo')
launch_dir = os.path.join(share_dir, 'launch')
sys.path.insert(0, launch_dir)

from sensor_msgs_py import point_cloud2 as pc2
from preprocess import voxelize
from postprocess import filter_pred

from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray, Detection3D,  Detection3DArray, ObjectHypothesisWithPose
from std_msgs.msg import Bool, String
from core.models.model import CustomModel

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
            
        # Default model initialization
        self.dataset_type = None
        if '/kitti' in points_topic:
            self.load_model("base_demo_kitti.json", "88epoch", "kitti")
        else:
            self.load_model("base_demo.json", "88epoch", "jrdb")

        self.process_times = []
        
        # Tracker initialization
        self.trackers = {}
        self.next_id = 0
        self.max_missed_frames = 8
        self.distance_threshold = 10.0 # meters
        
    def load_model(self, config_name, model_name, dataset_type):
        self.dataset_type = dataset_type
        config_path = os.path.join(launch_dir, config_name)
        self.get_logger().info(f"Loading config from: {config_path}")
        with open(config_path, 'r') as f:
            self.config = json.load(f) 
            
        self.model = CustomModel(self.config["model"], self.config["data"]["num_classes"])  
        
        model_path = os.path.join(launch_dir, model_name)
        self.get_logger().info(f"Loading model weights from: {model_path}")
            
        model_state_dict = torch.load(model_path, map_location=self.device)
        
        checkpoint_weight = model_state_dict['backbone.conv1.weight']
        target_channels = self.model.backbone.conv1.weight.shape[1]
        
        if checkpoint_weight.shape[1] != target_channels:
            self.get_logger().warn(f"[WARNING] Checkpoint weights ({checkpoint_weight.shape[1]} ch) mismatch model architecture ({target_channels} ch).")
            self.get_logger().info(f"[FIX] Synchronizing weights to {target_channels} channels...")
            model_state_dict['backbone.conv1.weight'] = checkpoint_weight[:, :target_channels, :, :]
            
        self.model.load_state_dict(model_state_dict)
        self.model.to(self.device)
        self.model.eval()
  
    def enable_cb(self, msg):
        self.enabled = msg.data

    def topic_update_cb(self, msg):
        new_topic = msg.data
        self.get_logger().info(f"Received point cloud topic update: {new_topic}")
        
        if '/kitti' in new_topic and self.dataset_type != 'kitti':
            self.get_logger().info("Switching model to KITTI weights and config...")
            self.load_model("base_demo_kitti.json", "88epoch", "kitti")
        elif '/kitti' not in new_topic and self.dataset_type != 'jrdb':
            self.get_logger().info("Switching model to JRDB/Gazebo weights and config...")
            self.load_model("base_demo.json", "88epoch", "jrdb")
        
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            
        self.subscription = self.create_subscription(
            PointCloud2,
            new_topic,
            self.lidar_cb,
            1)
        self.get_logger().info(f"Successfully subscribed to new point cloud topic: {new_topic}")

    def cluster_boxes(self, raw_points):
        # Geometric BEV clustering for sim mode. The jrdb checkpoint is
        # out-of-distribution on Gazebo LiDAR (max confidence ~0.04-0.13
        # anywhere, junk outranking real cars), so instead we cluster the
        # occupancy grid and keep car-sized clusters.
        from collections import deque
        R = 16.0     # BEV range (matches sim distance filter)
        res = 0.2    # cell size in meters
        W = int(round(2 * R / res))

        pts = raw_points[(raw_points[:, 2] > -1.2) & (raw_points[:, 2] < 1.0)]
        pts = pts[(np.abs(pts[:, 0]) < R) & (np.abs(pts[:, 1]) < R)]
        d = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[d > 2.8]  # drop ego body
        if len(pts) == 0:
            return []

        gx = np.clip(((pts[:, 0] + R) / res).astype(np.int32), 0, W - 1)
        gy = np.clip(((pts[:, 1] + R) / res).astype(np.int32), 0, W - 1)
        grid = np.zeros((W, W), dtype=np.int32)
        np.add.at(grid, (gx, gy), 1)
        occ = grid >= 2  # noise gate: need >=2 points per cell

        labels = np.full((W, W), -1, dtype=np.int32)
        cell_of_point = gx * W + gy
        boxes = []
        nlab = 0
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for cx, cy in np.argwhere(occ):
            if labels[cx, cy] != -1:
                continue
            comp = []
            q = deque([(cx, cy)])
            labels[cx, cy] = nlab
            while q:
                x, y = q.popleft()
                comp.append((x, y))
                for dx, dy in nbrs:
                    nx2, ny2 = x + dx, y + dy
                    if 0 <= nx2 < W and 0 <= ny2 < W and occ[nx2, ny2] and labels[nx2, ny2] == -1:
                        labels[nx2, ny2] = nlab
                        q.append((nx2, ny2))
            nlab += 1
            comp = np.array(comp)
            cells = set((comp[:, 0] * W + comp[:, 1]).tolist())
            mask = np.isin(cell_of_point, list(cells))
            cpts = pts[mask]
            if len(cpts) < 25:
                continue
            # All cars in this world are parked axis-aligned (yaw = 0 or ±π/2).
            # Min-area rect fitting is noisy on nearly-square footprints and
            # returns diagonal angles. Just check which span is longer.
            xy = cpts[:, :2]
            # Use bbox midpoint, not point mean — the mean is biased toward the
            # near face (far face occluded), which places the box behind the car.
            center = np.array([(xy[:, 0].min() + xy[:, 0].max()) / 2.0,
                               (xy[:, 1].min() + xy[:, 1].max()) / 2.0])
            x_span = float(xy[:, 0].max() - xy[:, 0].min())
            y_span = float(xy[:, 1].max() - xy[:, 1].min())
            if y_span >= x_span:
                length, width = y_span, x_span
                yaw = np.pi / 2.0   # car long axis along Y
            else:
                length, width = x_span, y_span
                yaw = 0.0           # car long axis along X
            ztop = float(cpts[:, 2].max())
            zbot = float(cpts[:, 2].min())
            height = ztop - zbot
            # Car gate. Footprint alone is not enough: pine trees/poles have
            # car-sized footprints. Cars top out near z~0 (roof ~1.5m above
            # ground, sensor at ~1.5m), while trees/poles/fences reach the
            # z=1.0 clip. Grounded check rejects floating foliage clusters.
            if not (2.0 < length < 6.5 and 0.8 < width < 3.0 and height > 0.4
                    and ztop < 0.8 and zbot < -0.6):
                continue
            score = min(1.0, len(cpts) / 300.0)
            boxes.append([0.0, score, float(center[0]), float(center[1]), length, width, yaw])
        return boxes

    def extract_bboxes(self, predictions, header, raw_points=None):
        if self.dataset_type == 'kitti':
            config = self.config['data'][self.dataset_type]
            out_size_factor = self.config['data']['out_size_factor']
            thres = 0.5
            nms_thres = 0.1  # Aggressive NMS threshold to prevent duplicate model outputs
            boxes = filter_pred(predictions, config, out_size_factor, thres, nms_thres)
        else:
            boxes = self.cluster_boxes(raw_points)
        pred_array = Detection3DArray()
        pred_array.header = header
        pred_array.header.frame_id = header.frame_id

        valid_boxes = []
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                center_x, center_y = float(box[2]), float(box[3])
                distance = (center_x**2 + center_y**2)**0.5
                if self.dataset_type == 'kitti':
                    if distance > 70.0 or distance < 3.0:
                        continue
                else:
                    # Min 2.8m: ignore the ego vehicle's own body (self-detection)
                    if distance > 16.0 or distance < 2.8:
                        continue
                valid_boxes.append(box)

        # Spatial filter: The model often predicts 2 disjoint boxes for the front and rear of a car's side.
        # We use longitudinal (dl) and lateral (dw) distance to merge boxes on the SAME car,
        # while PREVENTING the deletion of cars in adjacent lanes.
        filtered_valid_boxes = []
        import math
        for box in valid_boxes:
            is_dup = False
            cx, cy = float(box[2]), float(box[3])
            for kbox in filtered_valid_boxes:
                kx, ky = float(kbox[2]), float(kbox[3])
                
                if self.dataset_type == 'kitti':
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
                else:
                    if ((cx - kx)**2 + (cy - ky)**2)**0.5 < 1.0:
                        is_dup = True
                        break
                        
            if not is_dup:
                filtered_valid_boxes.append(box)
        valid_boxes = filtered_valid_boxes

        matched_ids = set()
        tracked_boxes = []

        import math

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
                
                yaw = float(box[6])
                old_yaw = self.trackers[best_id]['yaw']
                
                # Handle 180-degree ambiguity in model yaw output
                diff = (yaw - old_yaw + math.pi) % (2 * math.pi) - math.pi
                if diff > math.pi / 2:
                    yaw -= math.pi
                elif diff < -math.pi / 2:
                    yaw += math.pi
                    
                diff = (yaw - old_yaw + math.pi) % (2 * math.pi) - math.pi
                new_yaw = old_yaw + alpha * diff
                self.trackers[best_id]['yaw'] = (new_yaw + math.pi) % (2 * math.pi) - math.pi
                
                self.trackers[best_id]['missed'] = 0
                matched_ids.add(best_id)
                tracked_boxes.append((best_id, box))
            else:
                new_id = self.next_id
                self.next_id += 1
                length, width = float(box[4]), float(box[5])
                yaw = float(box[6])
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
            
            pred = Detection3D()
            pred.header.frame_id = header.frame_id
            pred.bbox.center.position.x = center_x
            pred.bbox.center.position.y = center_y
            pred.bbox.center.position.z = -1.0
            
            # Revert to standard size mapping based on coordinate analysis
            pred.bbox.size.x = length
            pred.bbox.size.y = width
            pred.bbox.size.z = 2.0
            
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
        if not getattr(self, 'enabled', True):
            return
            
        # Input
        start_time = time.time()
        t0 = time.time()
        geometry = self.config['data'][self.dataset_type]['geometry']
        
        # High-speed numpy point extraction
        cloud_array = np.ndarray(
            shape=(msg.width * msg.height,),
            dtype=np.dtype([
                ('x', np.float32), ('y', np.float32), ('z', np.float32), 
                ('_', np.uint8, msg.point_step - 12)
            ]),
            buffer=msg.data
        )
        # Drop NaNs safely
        raw_points = np.column_stack((cloud_array['x'], cloud_array['y'], cloud_array['z']))
        raw_points = raw_points[~np.isnan(raw_points).any(axis=1)]
        
        t1 = time.time()

        if self.dataset_type == 'kitti':
            # Convert to bev
            voxel = voxelize(raw_points, geometry)
            voxel_tensor = torch.tensor(voxel).float().unsqueeze(0).to(self.device)
            voxel_tensor = voxel_tensor.permute(0, 3, 1, 2)

            target_channels = self.model.backbone.conv1.weight.shape[1]
            if voxel_tensor.shape[1] != target_channels:
                voxel_tensor = voxel_tensor[:, :target_channels, :, :]
            t2 = time.time()

            # Predict and extract to bbox
            with torch.no_grad():
                predictions = self.model(voxel_tensor)
            t3 = time.time()

            cls_probs = torch.sigmoid(predictions["cls"].squeeze().detach())
            self.get_logger().info(f"Conf: {torch.max(cls_probs):.3f} | Times: points={t1-t0:.3f}s, voxel={t2-t1:.3f}s, infer={t3-t2:.3f}s")
        else:
            # Sim mode uses geometric clustering, not the network
            predictions = None
            t2 = time.time()
            t3 = time.time()

        pred_array = self.extract_bboxes(predictions, msg.header, raw_points)
        self.get_logger().info(f"Detected {len(pred_array.detections)} objects")
        self.pred_pub.publish(pred_array)
        process_time = time.time() - start_time
        self.process_times.append(process_time)
        current_fps = 1/process_time
        self.get_logger().info(f"Processing time: {process_time:.4f}s FPS: {current_fps:.2f}")
        
        display_fps = current_fps
        if len(self.process_times) == 10:
            avg_time_per_frame = np.mean(self.process_times)
            display_fps = 1 / avg_time_per_frame if avg_time_per_frame != 0 else 0  # Calculate FPS
            self.get_logger().info(f"Average FPS for the last 10 frames: {display_fps:.2f}")
            self.process_times.pop(0)
            
        gpu_memory = 0.0
        if torch.cuda.is_available():
            gpu_memory = torch.cuda.memory_allocated(self.device) / (1024 ** 2) # MB

        stats = {
            "fps": float(display_fps),
            "process_time_ms": float(process_time * 1000),
            "points_time_ms": float((t1-t0) * 1000),
            "voxel_time_ms": float((t2-t1) * 1000),
            "infer_time_ms": float((t3-t2) * 1000),
            "gpu_memory_mb": float(gpu_memory)
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.stats_pub.publish(stats_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = LiDARPedestrianDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
