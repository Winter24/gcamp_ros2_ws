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

# --- Ultra-Fast-Lane-Detection-V2 repo import ---------------------------------
# Cloned locally to the user's workspace (v2 == model_culane parsingNet API).
UFLD_PATH = '/home/winter24/ros2_ws/Ultra-Fast-Lane-Detection'
UFLD_WEIGHTS = os.path.join(UFLD_PATH, 'weights', 'tusimple_res18.pth')

if UFLD_PATH not in sys.path:
    sys.path.insert(0, UFLD_PATH)

# model_culane -> utils.common -> data.dali_data -> nvidia.dali (training-only,
# not installed and not needed for inference). Stub the module so the import
# chain resolves without pulling in DALI. Only initialize_weights (pure torch)
# is actually used from utils.common.
import types as _types
if 'data.dali_data' not in sys.modules:
    _stub = _types.ModuleType('data.dali_data')
    _stub.TrainCollect = None
    sys.modules['data.dali_data'] = _stub

UFLD_AVAILABLE = False
UFLD_IMPORT_ERROR = None
try:
    from model.model_culane import parsingNet
    UFLD_AVAILABLE = True
except Exception as e:  # noqa: BLE001 - repo missing / import failure, degrade gracefully
    UFLD_IMPORT_ERROR = str(e)


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

        # --- UFLDv2 / TuSimple ResNet18 config (configs/tusimple_res18.py) ----
        self.backbone = '18'
        self.train_width = 800
        self.train_height = 320
        self.crop_ratio = 0.8
        self.num_row = 56          # num_cls_row
        self.num_col = 41          # num_cls_col
        self.num_cell_row = 100    # num_grid_row
        self.num_cell_col = 100    # num_grid_col
        self.num_lanes = 4
        self.local_width = 1

        # Anchors (utils/common.py, Tusimple branch)
        self.row_anchor = np.linspace(160, 710, self.num_row) / 720.0
        self.col_anchor = np.linspace(0, 1, self.num_col)

        self.lane_trackers = []

        self.model = None
        if UFLD_AVAILABLE:
            self._load_model()
        else:
            self.get_logger().error(
                f"UFLDv2 not importable ({UFLD_IMPORT_ERROR}). "
                f"Check repo at {UFLD_PATH}. Lane node will idle (no detections).")

        self.img_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

    def _load_model(self):
        try:
            self.model = parsingNet(
                pretrained=False,
                backbone=self.backbone,
                num_grid_row=self.num_cell_row,
                num_cls_row=self.num_row,
                num_grid_col=self.num_cell_col,
                num_cls_col=self.num_col,
                num_lane_on_row=self.num_lanes,
                num_lane_on_col=self.num_lanes,
                use_aux=False,
                input_height=self.train_height,
                input_width=self.train_width,
                fc_norm=False)

            if not os.path.isfile(UFLD_WEIGHTS):
                self.get_logger().error(
                    f"UFLDv2 weights not found at {UFLD_WEIGHTS}. "
                    f"Download TuSimple ResNet18 weights there. Lane node idling.")
                self.model = None
                return

            ckpt = torch.load(UFLD_WEIGHTS, map_location='cpu')
            state_dict = ckpt['model'] if 'model' in ckpt else ckpt
            compatible = {}
            for k, v in state_dict.items():
                compatible[k[7:] if k.startswith('module.') else k] = v
            self.model.load_state_dict(compatible, strict=False)
            self.model.to(self.device)
            self.model.eval()
            self.get_logger().info(f"UFLDv2 model loaded from {UFLD_WEIGHTS}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Failed to load UFLDv2 model: {e}")
            self.model = None

    def preprocess(self, img_bgr):
        # cv2 gives BGR; model trained on RGB (PIL). Convert.
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        # Resize to (train_height/crop_ratio, train_width) then crop bottom train_height.
        resize_h = int(self.train_height / self.crop_ratio)  # 400
        img_resized = cv2.resize(img_rgb, (self.train_width, resize_h))  # (w, h)
        img_cropped = img_resized[-self.train_height:, :, :]  # bottom 320 rows
        tensor = self.img_transform(img_cropped).unsqueeze(0).to(self.device)
        return tensor

    def pred2coords(self, pred, original_image_width, original_image_height):
        # Ported from repo demo.py pred2coords (soft-argmax over local window).
        num_grid_row = pred['loc_row'].shape[1]
        num_cls_row = pred['loc_row'].shape[2]
        num_grid_col = pred['loc_col'].shape[1]
        num_cls_col = pred['loc_col'].shape[2]

        max_indices_row = pred['loc_row'].argmax(1).cpu()
        valid_row = pred['exist_row'].argmax(1).cpu()
        max_indices_col = pred['loc_col'].argmax(1).cpu()
        valid_col = pred['exist_col'].argmax(1).cpu()

        loc_row = pred['loc_row'].cpu()
        loc_col = pred['loc_col'].cpu()

        coords = []
        row_lane_idx = [1, 2]
        col_lane_idx = [0, 3]

        for i in row_lane_idx:
            tmp = []
            if valid_row[0, :, i].sum() > num_cls_row / 2:
                for k in range(valid_row.shape[1]):
                    if valid_row[0, k, i]:
                        all_ind = torch.tensor(list(range(
                            max(0, max_indices_row[0, k, i] - self.local_width),
                            min(num_grid_row - 1, max_indices_row[0, k, i] + self.local_width) + 1)))
                        out_tmp = (loc_row[0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                        out_tmp = out_tmp / (num_grid_row - 1) * original_image_width
                        tmp.append((int(out_tmp), int(self.row_anchor[k] * original_image_height)))
                if tmp:
                    coords.append(tmp)

        for i in col_lane_idx:
            tmp = []
            if valid_col[0, :, i].sum() > num_cls_col / 4:
                for k in range(valid_col.shape[1]):
                    if valid_col[0, k, i]:
                        all_ind = torch.tensor(list(range(
                            max(0, max_indices_col[0, k, i] - self.local_width),
                            min(num_grid_col - 1, max_indices_col[0, k, i] + self.local_width) + 1)))
                        out_tmp = (loc_col[0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                        out_tmp = out_tmp / (num_grid_col - 1) * original_image_height
                        tmp.append((int(self.col_anchor[k] * original_image_width), int(out_tmp)))
                if tmp:
                    coords.append(tmp)

        return coords

    def image_cb(self, msg):
        start_time = time.time()

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().error("Failed to decode compressed image")
            return

        original_h, original_w = img.shape[:2]

        infer_time = 0.0
        lanes = []

        if self.model is not None:
            img_tensor = self.preprocess(img)
            t0 = time.time()
            with torch.no_grad():
                pred = self.model(img_tensor)
            infer_time = time.time() - t0
            lanes = self.pred2coords(pred, original_w, original_h)
        else:
            time.sleep(0.01)
            infer_time = 0.01

        self.process_and_track_lanes(lanes, msg.header)

        process_time = time.time() - start_time

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
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
