# KITTI LiDAR Height Alignment Design

## Goal

Reduce one measurable source of domain mismatch between the Gazebo `/points_raw`
cloud and the KITTI clouds used by the detector by lowering the simulated LiDAR
mounting height from 2.0 m to 1.73 m.

## Scope

- Change only the fixed `velodyne_joint` z translation in `description/vlp_16.xacro`.
- Keep the existing HDL-64E-like scan pattern, range, update rate, and noise settings.
- Keep the KITTI/simulation detection confidence threshold at 0.2 because it is
  already configured to that value in `launch/detect.py`.
- Do not change model weights, voxel geometry, postprocessing, vehicle collision
  geometry, or the Unitree detection path.

## Verification

1. Confirm the source xacro reports a LiDAR z translation of 1.73 m.
2. Process the xacro and confirm the generated robot description contains the
   updated fixed-joint origin.
3. Build the `gcamp_gazebo` package so the installed description is refreshed.
4. Confirm the installed xacro contains the 1.73 m mounting height and the
   KITTI/simulation confidence threshold remains 0.2.

## Runtime Evaluation

After restarting the simulator and detector, compare `/tmp/r7viz_detect.log`
against the previous run. The useful measures are maximum class confidence,
detections per frame, and detection stability for hatchback, SUV, and pickup
vehicles at comparable poses and ranges.
