#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
import numpy as np
import time
import cv2
import torch
import torchvision.transforms as transforms

from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String
import json

import sys
ufld_path = '/thesis_ws/Ultra-Fast-Lane-Detection'
if ufld_path not in sys.path:
    sys.path.insert(0, ufld_path)
from model.model import parsingNet

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__("lane_detection_node")
        
        self.declare_parameter('camera_topic', '/kitti/image/color/left/compressed')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        
        self.get_logger().info(f"Subscribing to camera topic: {camera_topic}")
        
        self.subscription = self.create_subscription(
            CompressedImage,
            camera_topic,
            self.image_cb,
            1)
            
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/perception/lane_lines",
            10)

        self.stats_pub = self.create_publisher(
            String,
            "/model/lane_inference_stats",
            10)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using {self.device} for lane detection inference")
        
        self.img_w = 800
        self.img_h = 288
        self.griding_num = 100
        self.cls_num_per_lane = 56
        
        self.lane_trackers = []
        
        # NOTE: Initialize UFLD model here once downloaded.
        # self.model = None
        # self.get_logger().warn("UFLD model not loaded. Please clone the UFLD repository and instantiate 'parsingNet' here.")

        self.model = parsingNet(pretrained=False, backbone='18', cls_dim=(self.griding_num+1, self.cls_num_per_lane, 4), use_aux=False)
        state_dict = torch.load('/thesis_ws/Ultra-Fast-Lane-Detection/model/tusimple_18.pth', map_location=self.device)['model']
        self.model.load_state_dict(state_dict, strict=False)
        self.model.to(self.device)
        self.model.eval()

        self.img_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

        self.row_anchor = [ 64,  68,  72,  76,  80,  84,  88,  92,  96, 100, 104, 108, 112,
            116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164,
            168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216,
            220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268,
            272, 276, 280, 284]

    def image_cb(self, msg):
        start_time = time.time()

        # Decompress image
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().error("Failed to decode compressed image")
            return
            
        original_h, original_w = img.shape[:2]

        # Preprocess
        img_resized = cv2.resize(img, (self.img_w, self.img_h))
        img_tensor = self.img_transform(img_resized).unsqueeze(0).to(self.device)

        infer_time = 0.0
        lanes = []
        
        # Inference
        if self.model is not None:
            t0 = time.time()
            with torch.no_grad():
                out = self.model(img_tensor)
            infer_time = time.time() - t0
            
            # Post-processing UFLD output
            col_sample = np.linspace(0, 800 - 1, self.griding_num)
            col_sample_w = col_sample[1] - col_sample[0]
            
            out_j = out[0].data.cpu().numpy()
            out_j = out_j[:, ::-1, :]
            
            # Calculate softmax using numpy to avoid scipy dependency issues
            logits = out_j[:-1, :, :]
            exp_logits = np.exp(logits - np.max(logits, axis=0, keepdims=True))
            prob = exp_logits / np.sum(exp_logits, axis=0, keepdims=True)
            
            idx = np.arange(self.griding_num) + 1
            idx = idx.reshape(-1, 1, 1)
            loc = np.sum(prob * idx, axis=0)
            out_j = np.argmax(out_j, axis=0)
            loc[out_j == self.griding_num] = 0
            out_j = loc

            # Convert to image coordinates
            for i in range(out_j.shape[1]):
                if np.sum(out_j[:, i] != 0) > 2: # valid lane
                    lane = []
                    for k in range(out_j.shape[0]):
                        if out_j[k, i] > 0:
                            x = int(out_j[k, i] * col_sample_w * original_w / 800)
                            y = int(original_h * (self.row_anchor[self.cls_num_per_lane-1-k] / 288.0))
                            lane.append((x, y))
                    if lane:
                        lanes.append(lane)
        else:
            # Dummy delay to simulate model inference if model isn't loaded
            time.sleep(0.01)
            infer_time = 0.01

        # Process, track, extrapolate and publish MarkerArray
        self.process_and_track_lanes(lanes, msg.header)

        process_time = time.time() - start_time
        
        # Stats
        stats = {
            "fps": 1.0 / process_time if process_time > 0 else 0,
            "process_time_ms": process_time * 1000,
            "infer_time_ms": infer_time * 1000
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.stats_pub.publish(stats_msg)

    def process_and_track_lanes(self, raw_lanes, header):
        lanes_3d = []
        for lane in raw_lanes:
            points_3d = []
            for (u, v) in lane:
                c_x, c_y, f_x, f_y, cam_height = 609.0, 172.0, 721.0, 721.0, 1.65
                if v <= c_y: continue
                z_cam = cam_height * f_y / (v - c_y)
                x_cam = (u - c_x) * z_cam / f_x
                if 0.0 < z_cam < 100.0:
                    points_3d.append((float(z_cam), float(-x_cam)))
            if len(points_3d) >= 3:
                lanes_3d.append(points_3d)
                
        current_coeffs = []
        for pts in lanes_3d:
            pts_arr = np.array(pts)
            try:
                # fit y = A*x + B (straight lines)
                coeffs = np.polyfit(pts_arr[:, 0], pts_arr[:, 1], 1)
                current_coeffs.append(coeffs)
            except:
                pass
                
        alpha = 0.05 # Lower alpha makes it heavily static and smooth
        max_missed = 10
        matched_tracker_indices = set()
        new_trackers = []
        
        for coeffs in current_coeffs:
            best_idx = -1
            best_dist = 2.0 # Max 2m lateral shift to match
            y_eval_curr = np.polyval(coeffs, 10.0)
            
            for i, tracker in enumerate(self.lane_trackers):
                if i in matched_tracker_indices: continue
                y_eval_trk = np.polyval(tracker['coeffs'], 10.0)
                dist = abs(y_eval_curr - y_eval_trk)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
                    
            if best_idx != -1:
                self.lane_trackers[best_idx]['coeffs'] = (1-alpha) * self.lane_trackers[best_idx]['coeffs'] + alpha * coeffs
                self.lane_trackers[best_idx]['missed'] = 0
                matched_tracker_indices.add(best_idx)
            else:
                new_trackers.append({'coeffs': coeffs, 'missed': 0})
                
        for i in range(len(self.lane_trackers)):
            if i not in matched_tracker_indices:
                self.lane_trackers[i]['missed'] += 1
                
        self.lane_trackers = [t for t in self.lane_trackers if t['missed'] <= max_missed]
        self.lane_trackers.extend(new_trackers)
        
        # Sort by closest to the center of the vehicle (lateral distance at x=5m)
        self.lane_trackers.sort(key=lambda t: abs(np.polyval(t['coeffs'], 5.0)))
        
        smooth_lanes = []
        x_eval = np.linspace(-2.0, 80.0, 50)
        # Only render the 2 closest lanes
        for tracker in self.lane_trackers[:2]:
            y_eval = np.polyval(tracker['coeffs'], x_eval)
            lane_pts = []
            for x, y in zip(x_eval, y_eval):
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = 0.0
                lane_pts.append(p)
            smooth_lanes.append(lane_pts)
            
        self.publish_markers(smooth_lanes, header)

    def publish_markers(self, smooth_lanes, header):
        marker_array = MarkerArray()
        
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)
        
        for idx, lane_pts in enumerate(smooth_lanes):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = "usb_cam"
            marker.ns = "lanes"
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.points = lane_pts
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
