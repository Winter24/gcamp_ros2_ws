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
from std_msgs.msg import Bool
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
        
        self.pred_pub = self.create_publisher(
            Detection3DArray,
            "/preds",
            10)
        
        config_path = os.path.join(launch_dir, "base_demo.json")
        self.get_logger().info(f"Loading config from: {config_path}")
        with open(config_path, 'r') as f:
            self.config = json.load(f) 
            
        self.model = CustomModel(self.config["model"], self.config["data"]["num_classes"])  
        
        model_path = os.path.join(launch_dir, '88epoch')
        self.get_logger().info(f"Loading model weights from: {model_path}")
        
        if torch.cuda.is_available():
            self.get_logger().info("Using CUDA GPU for inference")
            self.device = torch.device("cuda")
        else: 
            self.get_logger().info("Using CPU for inference")
            self.device = torch.device("cpu")
            
        model_state_dict = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(model_state_dict)
        self.model.to(self.device)
        self.model.eval()

        self.process_times = []
  
    def enable_cb(self, msg):
        self.enabled = msg.data

    def extract_bboxes(self, predictions, header):
        config = self.config['data']['jrdb']
        out_size_factor = self.config['data']['out_size_factor']
        thres = 0.95  # Threshold for filtering predictions (lowered for simulated data)
        nms_thres = 0.5  # NMS threshold
        
        boxes = filter_pred(predictions, config, out_size_factor, thres, nms_thres)
        pred_array = Detection3DArray()
        pred_array.header = header
        pred_array.header.frame_id = header.frame_id

        for idx, box in enumerate(boxes):
            center_x, center_y = box[2], box[3]
            length, width = box[4], box[5]
            
            # Distance filter (ignore noise beyond 15 meters)
            distance = (center_x**2 + center_y**2)**0.5
            if distance > 15.0:
                continue
                
            # Size filter (Humans/Bikes are not > 2.5m and not < 0.2m)
            if width > 2.5 or length > 2.5 or width < 0.2 or length < 0.2:
                continue

            pred = Detection3D()
            pred.header.frame_id = header.frame_id
            pred.bbox.center.position.x, pred.bbox.center.position.y, \
            pred.bbox.center.position.z = center_x, center_y, 1.0
            pred.bbox.size.x, pred.bbox.size.y, pred.bbox.size.z = width, length, 2.0

            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = str(int(box[0]))
            result.hypothesis.score = box[1]
            pred.results.append(result)

            pred_array.detections.append(pred)

        return pred_array

    def lidar_cb(self, msg):
        if not getattr(self, 'enabled', True):
            return
            
        # Input
        start_time = time.time()
        geometry = self.config['data']['jrdb']['geometry']
        raw_points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        raw_points = np.vstack((raw_points['x'], raw_points['y'], raw_points['z'])).T

        # Convert to bev 
        voxel = voxelize(raw_points, geometry) 
        voxel_tensor = torch.tensor(voxel).float().unsqueeze(0).to(self.device)
        voxel_tensor = voxel_tensor.permute(0, 3, 1, 2)
        # Predict and extract to bbox
        with torch.no_grad():
            predictions = self.model(voxel_tensor)
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
