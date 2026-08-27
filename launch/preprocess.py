import numpy as np
import math
import torch

import torch
import numpy as np
from shapely.geometry import Polygon
import json

import torch.nn.functional as F


UNITREE_INFERENCE_YAW_RAD = -np.pi / 2.0


class TorchVoxelizer:
    """Build binary occupancy with filtering and indexing on one Torch device.

    CUDA execution stages the complete XYZ cloud through a reusable pinned host
    buffer. Finite checks, ROI filtering, voxel address arithmetic, and writes
    then stay on the GPU; only the ready model tensor remains after the call.
    """

    _GEOMETRY_NAMES = (
        "x_min", "x_max", "x_res",
        "y_min", "y_max", "y_res",
        "z_min", "z_max", "z_res",
    )

    def __init__(self, device):
        self.device = torch.device(device)
        self._voxel_buffers = {}
        self._geometry_cache = {}
        self._pinned_points = {}
        self._staging_events = {}

    @classmethod
    def _geometry_key(cls, geometry):
        return tuple(float(geometry[name]) for name in cls._GEOMETRY_NAMES)

    def _get_geometry(self, key):
        cached = self._geometry_cache.get(key)
        if cached is not None:
            return cached

        x_min, x_max, x_res, y_min, y_max, y_res, z_min, z_max, z_res = key
        x_size = int((x_max - x_min) / x_res)
        y_size = int((y_max - y_min) / y_res)
        z_size = int((z_max - z_min) / z_res)
        cached = {
            "x_min": x_min, "x_max": x_max, "x_res": x_res,
            "y_min": y_min, "y_max": y_max, "y_res": y_res,
            "z_min": z_min, "z_max": z_max, "z_res": z_res,
            "x_size": x_size, "y_size": y_size, "z_size": z_size,
            "stride_y": x_size, "stride_z": y_size * x_size,
        }
        self._geometry_cache[key] = cached
        return cached

    def _get_voxel_buffer(self, key, geo):
        shape = (1, geo["z_size"], geo["y_size"], geo["x_size"])
        buffer = self._voxel_buffers.get(key)
        if buffer is None or tuple(buffer.shape) != shape:
            buffer = torch.zeros(shape, dtype=torch.float32, device=self.device)
            self._voxel_buffers[key] = buffer
        else:
            buffer.zero_()
        return buffer

    def _stage_full_cloud(self, key, points):
        """Enqueue the complete XYZ cloud from reusable pinned memory."""
        point_count = len(points)
        event = self._staging_events.get(key)
        if event is not None:
            # Do not let the CPU refill pinned pages while their prior DMA is active.
            event.synchronize()

        pinned = self._pinned_points.get(key)
        if pinned is None or pinned.shape[0] < point_count:
            capacity = max(point_count * 2, 4096)
            pinned = torch.empty(
                (capacity, 3), dtype=torch.float32, pin_memory=True
            )
            self._pinned_points[key] = pinned

        source = torch.from_numpy(points)
        pinned[:point_count].copy_(source)
        gpu_points = pinned[:point_count].to(self.device, non_blocking=True)

        if event is None:
            event = torch.cuda.Event()
        event.record(torch.cuda.current_stream(self.device))
        self._staging_events[key] = event
        return gpu_points

    def voxelize(self, points, geometry, yaw_rad=None):
        """Return a contiguous ``(1, Z, Y, X)`` binary occupancy tensor."""
        key = self._geometry_key(geometry)
        geo = self._get_geometry(key)
        voxels = self._get_voxel_buffer(key, geo)
        if len(points) == 0:
            return voxels

        points = np.asarray(points[:, :3], dtype=np.float32)
        if self.device.type == "cuda":
            gpu_points = self._stage_full_cloud(key, points)
        else:
            gpu_points = torch.from_numpy(points).to(self.device)

        if yaw_rad is not None:
            cosine = math.cos(yaw_rad)
            sine = math.sin(yaw_rad)
            x = gpu_points[:, 0].clone()
            y = gpu_points[:, 1].clone()
            gpu_points[:, 0] = cosine * x - sine * y
            gpu_points[:, 1] = sine * x + cosine * y

        eps = 0.001
        finite = torch.isfinite(gpu_points).all(dim=1)
        inside = (
            finite
            & (gpu_points[:, 0] > geo["x_min"] + eps)
            & (gpu_points[:, 0] < geo["x_max"] - eps)
            & (gpu_points[:, 1] > geo["y_min"] + eps)
            & (gpu_points[:, 1] < geo["y_max"] - eps)
            & (gpu_points[:, 2] > geo["z_min"] + eps)
            & (gpu_points[:, 2] < geo["z_max"] - eps)
        )

        valid = gpu_points[inside]
        # Keep address arithmetic in INT32, then widen once because PyTorch's
        # scatter API requires int64 indices.
        x_index = torch.floor(
            (valid[:, 0] - geo["x_min"]) / geo["x_res"]
        ).to(torch.int32)
        y_index = torch.floor(
            (valid[:, 1] - geo["y_min"]) / geo["y_res"]
        ).to(torch.int32)
        z_index = torch.floor(
            (valid[:, 2] - geo["z_min"]) / geo["z_res"]
        ).to(torch.int32)
        flat_index = (
            z_index * geo["stride_z"]
            + y_index * geo["stride_y"]
            + x_index
        )
        voxels.view(-1).scatter_(0, flat_index.long(), 1.0)
        return voxels

def rotate_points_z(points, angle):
    """Return XYZ points rotated around Z without modifying the input."""
    points = np.asarray(points)
    rotated = points.copy()
    cosine = math.cos(angle)
    sine = math.sin(angle)
    rotated[:, 0] = cosine * points[:, 0] - sine * points[:, 1]
    rotated[:, 1] = sine * points[:, 0] + cosine * points[:, 1]
    return rotated


def rotate_detection_boxes_z(boxes, angle):
    """Rotate box centers/headings; box rows are cls, score, x, y, l, w, yaw."""
    boxes = np.asarray(boxes)
    rotated = boxes.copy()
    cosine = math.cos(angle)
    sine = math.sin(angle)
    rotated[:, 2] = cosine * boxes[:, 2] - sine * boxes[:, 3]
    rotated[:, 3] = sine * boxes[:, 2] + cosine * boxes[:, 3]
    rotated[:, 6] = (boxes[:, 6] + angle + math.pi) % (2 * math.pi) - math.pi
    return rotated


def map_right_facing_unitree_roi_to_model(points):
    """Map LiDAR x=-40..40,y=-70.4..0 into checkpoint coordinates."""
    points = np.asarray(points)
    mapped = points.copy()
    mapped[:, 0] = -points[:, 1]
    mapped[:, 1] = points[:, 0]
    return mapped


def voxelize(points, geometry):
    x_min = geometry["x_min"]
    x_max = geometry["x_max"]
    y_min = geometry["y_min"]
    y_max = geometry["y_max"]
    z_min = geometry["z_min"]
    z_max = geometry["z_max"]
    x_res = geometry["x_res"]
    y_res = geometry["y_res"]
    z_res = geometry["z_res"]

    x_size = int((x_max - x_min) / x_res)
    y_size = int((y_max - y_min) / y_res)
    z_size = int((z_max - z_min) / z_res)

    eps = 0.001

    #clip points
    x_indexes = np.logical_and(points[:, 0] > x_min + eps, points[:, 0] < x_max - eps)
    y_indexes = np.logical_and(points[:, 1] > y_min + eps, points[:, 1] < y_max - eps)
    z_indexes = np.logical_and(points[:, 2] > z_min + eps, points[:, 2] < z_max - eps)
    pts = points[np.logical_and(np.logical_and(x_indexes, y_indexes), z_indexes)]

    occupancy_mask = np.zeros((pts.shape[0], 3), dtype = np.int32)
    voxels = np.zeros((x_size, y_size, z_size), dtype = np.float32)
    occupancy_mask[:, 0] = (pts[:, 0] - x_min) // x_res
    occupancy_mask[:, 1] = (pts[:, 1] - y_min) // y_res
    occupancy_mask[:, 2] = (pts[:, 2] - z_min) // z_res

    idxs = np.array([occupancy_mask[:, 0].reshape(-1), occupancy_mask[:, 1].reshape(-1), occupancy_mask[:, 2].reshape( -1)])

    voxels[idxs[0], idxs[1], idxs[2]] = 1
    return np.swapaxes(voxels, 0, 1)


def voxel_to_points(voxel, geometry):
    x_min = geometry["x_min"]
    x_max = geometry["x_max"]
    y_min = geometry["y_min"]
    y_max = geometry["y_max"]
    z_min = geometry["z_min"]
    z_max = geometry["z_max"]
    x_res = geometry["x_res"]
    y_res = geometry["y_res"]
    z_res = geometry["z_res"]

    xs, ys, zs = np.where(voxel.astype(int) == 1)
    points_x = xs + x_res / 2
    points_y = ys + y_res / 2
    points_z = zs + z_res / 2

    points_x = points_x * x_res + x_min
    points_y = points_y * y_res + y_min
    points_z = points_z * z_res + z_min
    #centers = np.array(np.where(voxel.astype(int) == 1)) + np.array([[x_res / 2], [y_res / 2], [z_res / 2]])
    return np.transpose(np.array([points_x, points_y, points_z]))

def trasform_label2metric(label, geometry, ratio=4):
    '''
    :param label: numpy array of shape [..., 2] of coordinates in label map space
    :return: numpy array of shape [..., 2] of the same coordinates in metric space
    '''
    
    metric = np.copy(label).astype(np.float32)
    metric[..., 0] = metric[..., 0] * ratio * geometry["x_res"] + geometry["x_min"]
    metric[..., 1] = metric[..., 1] * ratio * geometry["y_res"] + geometry["y_min"]

    return metric

def transform_metric2label(metric, ratio=4, grid_size=0.1, base_height=100):
    '''
    :param label: numpy array of shape [..., 2] of coordinates in metric space
    :return: numpy array of shape [..., 2] of the same coordinates in label_map space
    '''

    label = (metric / ratio ) / grid_size
    label[..., 1] += base_height
    return label

def get_points_in_a_rotated_box(corners, label_shape=[200, 175]):
    def minY(x0, y0, x1, y1, x):
        if x0 == x1:
            # vertical line, y0 is lowest
            return int(math.floor(y0))

        m = (y1 - y0) / (x1 - x0)

        if m >= 0.0:
            # lowest point is at left edge of pixel column
            return int(math.floor(y0 + m * (x - x0)))
        else:
            # lowest point is at right edge of pixel column
            return int(math.floor(y0 + m * ((x + 1.0) - x0)))


    def maxY(x0, y0, x1, y1, x):
        if x0 == x1:
            # vertical line, y1 is highest
            return int(math.ceil(y1))

        m = (y1 - y0) / (x1 - x0)

        if m >= 0.0:
            # highest point is at right edge of pixel column
            return int(math.ceil(y0 + m * ((x + 1.0) - x0)))
        else:
            # highest point is at left edge of pixel column
            return int(math.ceil(y0 + m * (x - x0)))


    # view_bl, view_tl, view_tr, view_br are the corners of the rectangle
    view = [(corners[i, 0], corners[i, 1]) for i in range(4)]

    pixels = []

    # find l,r,t,b,m1,m2
    l, m1, m2, r = sorted(view, key=lambda p: (p[0], p[1]))
    b, t = sorted([m1, m2], key=lambda p: (p[1], p[0]))
    
    lx, ly = l
    rx, ry = r
    bx, by = b
    tx, ty = t
    m1x, m1y = m1
    m2x, m2y = m2

    xmin = 0
    ymin = 0
    xmax = label_shape[1]
    ymax = label_shape[0]

    # inward-rounded integer bounds
    # note that we're clamping the area of interest to (xmin,ymin)-(xmax,ymax)
    lxi = max(int(math.ceil(lx)), xmin)
    rxi = min(int(math.floor(rx)), xmax)
    byi = max(int(math.ceil(by)), ymin)
    tyi = min(int(math.floor(ty)), ymax)

    x1 = lxi
    x2 = rxi

    for x in range(x1, x2):
        xf = float(x)

        if xf < m1x:
            # Phase I: left to top and bottom
            y1 = minY(lx, ly, bx, by, xf)
            y2 = maxY(lx, ly, tx, ty, xf)

        elif xf < m2x:
            if m1y < m2y:
                # Phase IIa: left/bottom --> top/right
                y1 = minY(bx, by, rx, ry, xf)
                y2 = maxY(lx, ly, tx, ty, xf)

            else:
                # Phase IIb: left/top --> bottom/right
                y1 = minY(lx, ly, bx, by, xf)
                y2 = maxY(tx, ty, rx, ry, xf)

        else:
            # Phase III: bottom/top --> right
            y1 = minY(bx, by, rx, ry, xf)
            y2 = maxY(tx, ty, rx, ry, xf)

        y1 = max(y1, byi)
        y2 = min(y2, tyi)

        for y in range(y1, y2):
            pixels.append((x, y))

    return pixels


def voxel_to_img(voxel):
    voxel = voxel.permute(1, 2, 0)
    max_inds = torch.argmax(voxel, axis = 2)
    img = np.zeros((voxel.shape[0], voxel.shape[1]))
    for i in range(voxel.shape[0]):
        for j in range(voxel.shape[1]):
            idx = max_inds[i][j]
            img[i][j] = int(idx / voxel.shape[2] * 255)

    return img
