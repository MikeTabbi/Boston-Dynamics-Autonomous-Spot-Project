#!/bin/bash
# ============================================================
# Spot Autonomous Exploration - Launch Commands
# ============================================================
# ARCHITECTURE: RTAB-Map RGB-D Visual SLAM
#
#   PHASE 1 — BUILD MAP (RTAB-Map mapping mode)
#     Terminals: 1, 2, 3, 4-MAP
#     Drive Spot manually (or use auto_mapper) to explore.
#     Save map when done via Terminal 4-MAP command.
#
#   PHASE 2 — NAVIGATE (RTAB-Map localization + Nav2)
#     Terminals: 1, 2, 3, 4-LOC, 5, 6A (probe) then 6B (frontier)
#
# Start each terminal in order. Wait ~10s between terminals.
# Credentials and cmd_duration are set via /root/spot_config.yaml.
# ============================================================


# ── TERMINAL 1 ── Spot Driver ──────────────────────────────
docker start -i spot_nav
# (inside container — already sourced in .bashrc)
ros2 launch spot_driver spot_driver.launch.py spot_name:=spot config_file:=/root/spot_config.yaml


# ── TERMINAL 2 ── TF Keep-Alive ────────────────────────────
# Keeps spot/frontleft->spot/frontleft_fisheye TF fresh.
# The Spot driver's register_node loses this TF after startup
# because Spot's hardware clock is ahead of the container clock.
# This script tracks Spot's clock via depth images and republishes
# the TF at Spot's clock rate so depth_registered never drops.
# REQUIRED — start before RTAB-Map.
docker exec -it spot_nav bash

python3 /root/tf_keep_alive.py


# ── TERMINAL 3 ── RTAB-Map (PHASE 1 — Mapping) ─────────────
# Visual SLAM using frontleft RGB-D camera + wheel odometry.
# visual_odometry:=false — uses /spot/odometry (wheel odom) for pose.
# approx_sync:=true — needed because RGB (~2.8 Hz) and depth (~2.3 Hz)
# publish at different rates.
# Watch for "Loop closure detected!" lines — confirms map quality.
# Node count in output should climb steadily (1 Hz rate).
docker exec -it spot_nav bash

# NEW MAP (clears existing map):
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/spot/camera/frontleft/image depth_topic:=/spot/depth_registered/frontleft/image camera_info_topic:=/spot/camera/frontleft/camera_info frame_id:=spot/frontleft_fisheye base_frame_id:=spot/body odom_frame_id:=spot/odom odom_topic:=/spot/odometry visual_odometry:=false approx_sync:=true approx_sync_max_interval:=1.0 subscribe_depth:=true subscribe_rgb:=true rviz:=false rtabmap_args:="--delete_db_on_start --Vis/MinInliers 6 --RGBD/LocalLoopDetectionMaxDiff 2.0 --RGBD/OptimizeMaxError 4.0"

# CONTINUE existing map (loads saved rtabmap.db):
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/spot/camera/frontleft/image depth_topic:=/spot/depth_registered/frontleft/image camera_info_topic:=/spot/camera/frontleft/camera_info frame_id:=spot/frontleft_fisheye base_frame_id:=spot/body odom_frame_id:=spot/odom odom_topic:=/spot/odometry visual_odometry:=false approx_sync:=true approx_sync_max_interval:=1.0 subscribe_depth:=true subscribe_rgb:=true rviz:=false rtabmap_args:="--Vis/MinInliers 6 --RGBD/LocalLoopDetectionMaxDiff 2.0 --RGBD/OptimizeMaxError 4.0"

# Save map when done:
# ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty {}
# Copy map to host:
# docker cp spot_nav:/root/.ros/rtabmap.db /Users/temp/Desktop/spot_nav_repo/rtabmap.db


# ── FOXGLOVE BRIDGE (live visualization) ───────────────────
# Run in a spare terminal. Then open Foxglove Studio and connect to ws://localhost:8765
# 3D panel: Display frame=map, enable /rtabmap/cloud_map, /rtabmap/cloud_obstacles, /rtabmap/map
# Image panel: /spot/camera/frontleft/image
ros2 launch foxglove_bridge foxglove_bridge_launch.xml


# ── TERMINAL 4-MAP ── Drive Spot (PHASE 1) ─────────────────
# Option A — autonomous mapping:
docker exec -it spot_nav bash

python3 -u /root/auto_mapper.py --ros-args -p duration_sec:=600.0 -p forward_speed:=0.10 -p drive_sec:=2.5 -p pause_sec:=1.0 -p wiggle_deg:=15.0 -p wiggle_speed:=0.20 -p cycles_before_turn:=3 -p turn_deg:=90.0 -p obstacle_distance:=0.75 -p forward_cone_deg:=20.0 -p min_obstacle_hits:=5 -p backup_sec:=2.0

# Option B — manual drive with cmd_vel relay:
# ros2 run topic_tools relay /cmd_vel /spot/cmd_vel
# (then use Spot controller or teleop_twist_keyboard)


# ── TERMINAL 3 ── RTAB-Map (PHASE 2 — Localization) ────────
# REPLACE the mapping command with localization mode.
# Loads the saved rtabmap.db, localizes without rebuilding map.
docker exec -it spot_nav bash

ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/spot/camera/frontleft/image depth_topic:=/spot/depth_registered/frontleft/image camera_info_topic:=/spot/camera/frontleft/camera_info frame_id:=spot/frontleft_fisheye base_frame_id:=spot/body odom_frame_id:=spot/odom odom_topic:=/spot/odometry visual_odometry:=false approx_sync:=true approx_sync_max_interval:=1.0 subscribe_depth:=true subscribe_rgb:=true rviz:=false localization:=true


# ── TERMINAL 5 ── cmd_vel Relay (PHASE 2) ──────────────────
# REQUIRED for Nav2 to send velocity commands to Spot.
docker exec -it spot_nav bash

ros2 run topic_tools relay /cmd_vel /spot/cmd_vel


# ── TERMINAL 6 ── Nav2 (PHASE 2) ───────────────────────────
# RTAB-Map provides map->odom TF. Nav2 uses /rtabmap/map as grid.
docker exec -it spot_nav bash

ros2 launch nav2_bringup navigation_launch.py params_file:=/root/nav2_params.yaml use_sim_time:=false


# ── TERMINAL 7A ── Waypoint Probe (run first in Phase 2) ───
docker exec -it spot_nav bash

python3 -u /root/waypoint_probe.py


# ── TERMINAL 7B ── Frontier Explorer (after probe passes) ──
# docker exec -it spot_nav bash
# python3 -u /root/frontier_explore_hybrid.py --ros-args \
#   -p min_frontier_cluster:=8 \
#   -p goal_clearance_cells:=6 \
#   -p max_goal_distance:=6.0 \
#   -p cost_threshold:=120


# ── FOXGLOVE VISUALIZATION ──────────────────────────────────
# Connect Foxglove Studio to ws://localhost:8765
# 3D panel settings:
#   Display frame: map
#   Topics to enable:
#     /rtabmap/cloud_map      — full 3D point cloud
#     /rtabmap/cloud_obstacles — obstacles only
#     /rtabmap/map            — 2D occupancy grid
# Image panel:
#   Topic: /spot/camera/frontleft/image


# ── VERIFY (spare terminal) ─────────────────────────────────
# docker exec -it spot_nav bash

# RTAB-Map receiving data (node count should climb at ~1 Hz)
# ros2 topic echo /rtabmap/info --field nodes_count

# Depth_registered publishing (must be >0 Hz)
# ros2 topic hz /spot/depth_registered/frontleft/image

# TF chain intact (must show: map -> spot/odom -> spot/body)
# ros2 run tf2_tools view_frames

# Wheel odometry publishing
# ros2 topic hz /spot/odometry

# Nav2 ready (Phase 2)
# ros2 action list


# ============================================================
# KEY FILES (inside container at /root/)
# ============================================================
# tf_keep_alive.py    — keeps frontleft->fisheye TF fresh (Spot clock fix)
# auto_mapper.py      — autonomous drive-pause-wiggle mapper
# waypoint_probe.py   — Nav2 validation (2 waypoints)
# nav2_params.yaml    — Nav2 config (allow_unknown:true, controller 5Hz)
# spot_config.yaml    — Spot credentials + cmd_duration:=5.0 (clock fix)
#
# HOST FILES
# /Users/temp/Desktop/spot_nav_repo/ — all project files, synced to GitHub
# /tmp/tf_keep_alive.py              — source (docker cp to container)
#
# ============================================================
# NOTES
# ============================================================
# Clock offset:
#   Spot's hardware clock is ~130s ahead of the container clock.
#   cmd_duration:=5.0 in spot_config.yaml gives commands a 5s validity.
#   tf_keep_alive.py tracks Spot's clock from depth image timestamps
#   and republishes camera TF at Spot's clock rate to prevent expiry.
#
# Why RTAB-Map (not slam_toolbox):
#   slam_toolbox failed due to "hallway problem" — plain walls are
#   featureless in 2D lidar. RTAB-Map uses visual bag-of-words loop
#   closure on RGB images, which works on textured environments.
#   The Agilent Hub has brick walls and equipment — ideal for visual SLAM.
#
# Camera setup:
#   frontleft camera only: RGB 640x480 ~2.8Hz, depth_registered ~2.3Hz
#   depth_registered requires register_node_frontleft (in spot driver)
#   which needs TF spot/frontleft->spot/frontleft_fisheye
#   tf_keep_alive.py prevents this TF from expiring.
#
# RTAB-Map mapping tips:
#   Move slowly (<0.5 m/s). Loop closures need revisiting seen areas.
#   Watch terminal for "Loop closure detected!" lines.
#   WM=1 is normal when stationary — grows as robot moves.
#   --delete_db_on_start clears old map each run (remove for Phase 2).
# ============================================================
