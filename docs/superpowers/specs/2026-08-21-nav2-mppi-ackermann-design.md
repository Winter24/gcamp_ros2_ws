# Nav2 MPPI Ackermann Design

## Goal

Replace the simulation-only custom Pure Pursuit execution path with the full Nav2 `NavigateToPose` pipeline and `nav2_mppi_controller::MPPIController`, while preserving the HMI `/goal_pose` interface and an easy source-level rollback.

## Navigation pipeline

The HMI continues publishing `geometry_msgs/PoseStamped` on `/goal_pose`. A small relay owns a `NavigateToPose` action client, cancels the previous accepted goal when a newer message arrives, rejects stale callbacks, and sends the newest pose to `navigate_to_pose`. Nav2 uses `SmacPlannerHybrid` for the global path and the `FollowPath` MPPI plugin for local control. The controller output goes through `velocity_smoother` from `cmd_vel_nav` to `/cmd_vel`.

The simulation launch starts map server, AMCL, planner server, controller server, behavior server, BT navigator, velocity smoother, their lifecycle manager, the goal relay, and the existing dynamic map-to-odom broadcaster. It no longer launches `hybrid_pure_pursuit.py`; that file remains installed for rollback.

## Ackermann frame contract

Nav2 and MPPI require +X to be the forward axis. The Prius `chassis` frame has its forward direction at chassis yaw minus 90 degrees. A static zero-translation transform therefore exposes `nav_base` at yaw `-pi/2` relative to `chassis`. Nav2 uses `nav_base` as `robot_base_frame`. Costmap footprints are rotated into this frame, with length on X and width on Y.

## MPPI baseline

`FollowPath` uses `nav2_mppi_controller::MPPIController`, `motion_model: Ackermann`, and `AckermannConstraints.min_turning_r: 3.7`. The initial horizon is four seconds at 10 Hz with 1000 sampled trajectories and one optimizer iteration. Longitudinal velocity permits a conservative reverse range. Critics include constraint, footprint-aware cost, goal, goal angle, path alignment, path following, and path angle. `PreferForwardCritic` is omitted so reverse segments from Smac are not suppressed.

## Behavior tree and failure handling

A project-owned NavigateToPose behavior tree replans at 1 Hz and follows the path without invoking `Spin`, because an Ackermann vehicle cannot rotate in place. Planning or control failure stops navigation and returns failure instead of issuing an incompatible spin recovery. A newer HMI goal cancels the old NavigateToPose goal and starts immediately.

## Verification

Static tests verify MPPI parameters, corrected frame and footprint, full Nav2 launch wiring, absence of custom Pure Pursuit in simulation launch, relay cancellation/stale-result behavior, and a no-Spin BT. Runtime smoke tests verify lifecycle activation, action availability, MPPI plugin loading, `/goal_pose` relay acceptance, `/plan`, and `/cmd_vel` output.

No commit or push is performed.
