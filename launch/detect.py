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

    def extract_bboxes(self, predictions, header):
        config = self.config['data'][self.dataset_type]
        out_size_factor = self.config['data']['out_size_factor']
        thres = 0.5 if self.dataset_type == 'kitti' else 0.1
        nms_thres = 0.5  # NMS threshold
        
        boxes = filter_pred(predictions, config, out_size_factor, thres, nms_thres)
        pred_array = Detection3DArray()
        pred_array.header = header
        pred_array.header.frame_id = header.frame_id

        for idx, box in enumerate(boxes):
            center_x, center_y = box[2], box[3]
            length, width = box[4], box[5]
            yaw = float(box[6])
            
            # Distance filter
            distance = (center_x**2 + center_y**2)**0.5
            if self.dataset_type == 'kitti':
                if distance > 70.0 or distance < 3.0:
                    continue
            else:
                if distance > 12.0:
                    continue

            pred = Detection3D()
            pred.header.frame_id = header.frame_id
            pred.bbox.center.position.x, pred.bbox.center.position.y, \
            pred.bbox.center.position.z = float(center_x), float(center_y), -1.0
            
            # Fix swapped length/width based on model output coordinates
            pred.bbox.size.x, pred.bbox.size.y, pred.bbox.size.z = float(length), float(width), 2.0
            
            # Apply yaw orientation around Z-axis
            import math
            pred.bbox.center.orientation.w = math.cos(yaw / 2.0)
            pred.bbox.center.orientation.x = 0.0
            pred.bbox.center.orientation.y = 0.0
            pred.bbox.center.orientation.z = math.sin(yaw / 2.0)

            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = str(int(box[0]))
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
            
        pred_array = self.extract_bboxes(predictions, msg.header)
        self.get_logger().info(f"Detected {len(pred_array.detections)} objects")
        self.pred_pub.publish(pred_array)
        process_time = time.time() - start_time
        self.process_times.append(process_time)
        current_fps = 1/process_time
        self.get_logger().info(f"Processing time: {process_time:.4f}s FPS: {current_fps:.2f}")
        if len(self.process_times) == 10:
            avg_time_per_frame = np.mean(self.process_times)
            fps = 1 / avg_time_per_frame if avg_time_per_frame != 0 else 0  # Calculate FPS
            self.get_logger().info(f"Average FPS for the last 10 frames: {fps:.2f}")
            self.process_times.pop(0)
        
def main(args=None):
    rclpy.init(args=args)
    detector = LiDARPedestrianDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
