#!/bin/bash
# ============================================================
# Spot Autonomous Exploration - Launch Commands
# ============================================================
# ARCHITECTURE: RTAB-Map RGB-D Visual SLAM
#
#   PHASE 1 — BUILD MAP (RTAB-Map mapping mode)
#     Terminals: T1, T2, T3
#     Drive Spot manually with tablet.
#     Save map + export 2D grid when done.
#
#   PHASE 2 — NAVIGATE (RTAB-Map localization + Nav2)
#     Terminals: T1, T2, T3-LOC, T4, T5, T6
#     Validate with T7A (waypoint probe), then T7B (frontier explorer).
#
# Start each terminal in order. Wait ~10s between terminals.
# ============================================================


# ── TERMINAL 1 ── Spot Driver ──────────────────────────────
docker start -i spot_nav
ros2 launch spot_driver spot_driver.launch.py spot_name:=spot config_file:=/root/spot_config.yaml


# ── TERMINAL 2 ── TF Keep-Alive ────────────────────────────
# REQUIRED — start before RTAB-Map.
# Spot's hardware clock is ~130s ahead of the container clock.
# Republishes frontleft->frontleft_fisheye TF at Spot's clock rate
# so depth_registered never drops.
docker exec -it spot_nav bash

python3 /root/tf_keep_alive.py


# ══════════════════════════════════════════════════════════════
# PHASE 1 — MAPPING
# ══════════════════════════════════════════════════════════════

# ── TERMINAL 3 ── RTAB-Map Mapping ─────────────────────────
# Watch for "Loop closure detected!" — you want multiple before stopping.
# Node count should climb steadily (~1 per second while moving).
docker exec -it spot_nav bash

# ══ OPTION A [USE THIS FOR PRESENTATION] ══════════════════════════════════════════
# NEW MAP → rtabmap_new.db  (keeps rtabmap.db as a backup — run this by default)
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/spot/camera/frontleft/image \
  depth_topic:=/spot/depth_registered/frontleft/image \
  camera_info_topic:=/spot/camera/frontleft/camera_info \
  frame_id:=spot/frontleft_fisheye \
  base_frame_id:=spot/body \
  odom_frame_id:=spot/odom \
  odom_topic:=/spot/odometry \
  visual_odometry:=false \
  approx_sync:=true \
  approx_sync_max_interval:=1.0 \
  subscribe_depth:=true \
  subscribe_rgb:=true \
  rviz:=false \
  database_path:=/root/spot_maps/rtabmap_new.db \
  rtabmap_args:="--delete_db_on_start --Vis/MinInliers 6 --RGBD/LocalLoopDetectionMaxDiff 2.0 --RGBD/OptimizeMaxError 4.0 --Grid/3D false --Grid/FromDepth true --Grid/RangeMax 5.0 --Grid/FootprintHeight 0.5 --Grid/CellSize 0.05"

# ══ OPTION B ══════════════════════════════════════════════════════════════════════
# NEW MAP → rtabmap.db  (overwrites the default database — use if starting completely fresh)
# ros2 launch rtabmap_launch rtabmap.launch.py \
#   rgb_topic:=/spot/camera/frontleft/image \
#   depth_topic:=/spot/depth_registered/frontleft/image \
#   camera_info_topic:=/spot/camera/frontleft/camera_info \
#   frame_id:=spot/frontleft_fisheye \
#   base_frame_id:=spot/body \
#   odom_frame_id:=spot/odom \
#   odom_topic:=/spot/odometry \
#   visual_odometry:=false \
#   approx_sync:=true \
#   approx_sync_max_interval:=1.0 \
#   subscribe_depth:=true \
#   subscribe_rgb:=true \
#   rviz:=false \
#   database_path:=/root/spot_maps/rtabmap.db \
#   rtabmap_args:="--delete_db_on_start --Vis/MinInliers 6 --RGBD/LocalLoopDetectionMaxDiff 2.0 --RGBD/OptimizeMaxError 4.0 --Grid/3D false --Grid/FromDepth true --Grid/RangeMax 5.0 --Grid/FootprintHeight 0.5 --Grid/CellSize 0.05"

# ══ OPTION C ══════════════════════════════════════════════════════════════════════
# CONTINUE existing map in rtabmap.db  (no --delete_db_on_start — resumes prior session)
# ros2 launch rtabmap_launch rtabmap.launch.py \
#   rgb_topic:=/spot/camera/frontleft/image \
#   depth_topic:=/spot/depth_registered/frontleft/image \
#   camera_info_topic:=/spot/camera/frontleft/camera_info \
#   frame_id:=spot/frontleft_fisheye \
#   base_frame_id:=spot/body \
#   odom_frame_id:=spot/odom \
#   odom_topic:=/spot/odometry \
#   visual_odometry:=false \
#   approx_sync:=true \
#   approx_sync_max_interval:=1.0 \
#   subscribe_depth:=true \
#   subscribe_rgb:=true \
#   rviz:=false \
#   database_path:=/root/spot_maps/rtabmap.db \
#   rtabmap_args:="--Vis/MinInliers 6 --RGBD/LocalLoopDetectionMaxDiff 2.0 --RGBD/OptimizeMaxError 4.0 --Grid/3D false --Grid/FromDepth true --Grid/RangeMax 5.0 --Grid/FootprintHeight 0.5 --Grid/CellSize 0.05"


# ── TERMINAL 4 ── Foxglove Bridge (Phase 1) ────────────────
docker exec -it spot_nav bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# Then open Foxglove Studio → connect to ws://localhost:8765
# Fixed frame: map | Enable: /rtabmap/cloud_map, /rtabmap/grid_prob_map


# ── PHASE 1 SAVE ── Run BEFORE stopping Terminal 3 ─────────
# Step 1: Save the database
# ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty {}
#
# Step 2: Export 2D occupancy grid (REQUIRED — do this while T3 is still running)
# ros2 run nav2_map_server map_saver_cli -f /root/spot_maps/rtabmap-2D --ros-args -r /map:=/rtabmap/grid_prob_map -p map_subscribe_transient_local:=true
#
# Step 3: Ctrl+C Terminal 3


# ══════════════════════════════════════════════════════════════
# PHASE 2 — LOCALIZATION + NAVIGATION
# ══════════════════════════════════════════════════════════════

# ── TERMINAL 3 ── RTAB-Map Localization ────────────────────
# Replace the Phase 1 mapping command with this.
# Loads saved rtabmap.db, provides map->odom TF for Nav2.
docker exec -it spot_nav bash

ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/spot/camera/frontleft/image \
  depth_topic:=/spot/depth_registered/frontleft/image \
  camera_info_topic:=/spot/camera/frontleft/camera_info \
  frame_id:=spot/frontleft_fisheye \
  base_frame_id:=spot/body \
  odom_frame_id:=spot/odom \
  odom_topic:=/spot/odometry \
  visual_odometry:=false \
  approx_sync:=true \
  approx_sync_max_interval:=1.0 \
  subscribe_depth:=true \
  subscribe_rgb:=true \
  rviz:=false \
  localization:=true \
  database_path:=/root/spot_maps/rtabmap_new.db \
  rtabmap_args:="--Grid/3D false --Grid/FromDepth true --Grid/RangeMax 5.0 --Grid/FootprintHeight 0.5 --Grid/CellSize 0.05"


# ── TERMINAL 4 ── cmd_vel Relay ─────────────────────────────
# REQUIRED — Nav2 publishes to /cmd_vel, Spot listens on /spot/cmd_vel.
docker exec -it spot_nav bash

ros2 run topic_tools relay /cmd_vel /spot/cmd_vel


# ── TERMINAL 5 ── Map Server ────────────────────────────────
# Serves the exported 2D map as TRANSIENT_LOCAL /map for Nav2.
# Re-export the 2D map after every new mapping run (Phase 1 Save step 2).
docker exec -it spot_nav bash

bash /root/start_map_server.sh
# Then once it's up, run in the same terminal:
# bash /root/activate_map_server.sh


# ── TERMINAL 6 ── Nav2 ──────────────────────────────────────
# Wait for "Managed nodes are active" before proceeding.
docker exec -it spot_nav bash

ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/root/nav2_params.yaml \
  map:=/root/spot_maps/rtabmap-2D.yaml \
  use_sim_time:=false


# ── TERMINAL 7 ── Foxglove Bridge (Phase 2) ────────────────
docker exec -it spot_nav bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# Then open Foxglove Studio → connect to ws://localhost:8765
# Fixed frame: map | Enable: /rtabmap/grid_prob_map, /map, /rtabmap/cloud_obstacles


# ── TERMINAL 8A ── Waypoint Probe (validate first) ──────────
docker exec -it spot_nav bash

python3 -u /root/waypoint_probe.py


# ── TERMINAL 8B ── Frontier Explorer (after probe passes) ───
docker exec -it spot_nav bash

python3 -u /root/frontier_explore_hybrid.py --ros-args \
  -p min_frontier_cluster:=8 \
  -p goal_clearance_cells:=6 \
  -p max_goal_distance:=6.0 \
  -p cost_threshold:=120


# ══════════════════════════════════════════════════════════════
# FOXGLOVE VISUALIZATION
# ══════════════════════════════════════════════════════════════
# Run in a spare terminal, connect Foxglove to ws://localhost:8765
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml
#
# 3D panel settings:
#   Fixed frame: map
#   Topics to enable:
#     /rtabmap/cloud_map        — full 3D point cloud
#     /rtabmap/cloud_obstacles  — live obstacles
#     /rtabmap/grid_prob_map    — 2D occupancy grid
# Image panel:
#   Topic: /spot/camera/frontleft/image


# ══════════════════════════════════════════════════════════════
# DIAGNOSTICS (spare terminal)
# ══════════════════════════════════════════════════════════════
# docker exec -it spot_nav bash

# RTAB-Map node count climbing (~1 Hz while moving):
# ros2 topic echo /rtabmap/info --field nodes_count

# Depth image publishing (must be > 0 Hz):
# ros2 topic hz /spot/depth_registered/frontleft/image

# TF chain intact (must show: map -> spot/odom -> spot/body):
# ros2 run tf2_tools view_frames

# Wheel odometry publishing:
# ros2 topic hz /spot/odometry

# Nav2 actions available (Phase 2):
# ros2 action list


# ══════════════════════════════════════════════════════════════
# KEY FILES (inside container /root/)
# ══════════════════════════════════════════════════════════════
# tf_keep_alive.py           — keeps frontleft->fisheye TF fresh (Spot clock fix)
# autonomous_mapper.py       — autonomous state-machine mapper (Phase 1 option)
# waypoint_probe.py          — Nav2 validation (2 waypoints)
# frontier_explore_hybrid.py — autonomous frontier exploration (Phase 2)
# nav2_params.yaml           — Nav2 config (allow_unknown:true, controller 5Hz)
# spot_config.yaml           — Spot credentials + cmd_duration:=5.0
# start_map_server.sh        — launches nav2_map_server with TRANSIENT_LOCAL /map
# activate_map_server.sh     — configures and activates the map_server lifecycle node
# spot_maps/rtabmap.db       — RTAB-Map database (visual SLAM map)
# spot_maps/rtabmap-2D.pgm   — exported 2D occupancy grid image
# spot_maps/rtabmap-2D.yaml  — exported 2D occupancy grid metadata


# ══════════════════════════════════════════════════════════════
# NOTES
# ══════════════════════════════════════════════════════════════
# Clock offset:
#   Spot's hardware clock is ~130s ahead of the container clock.
#   cmd_duration:=5.0 in spot_config.yaml gives commands a 5s validity window.
#   tf_keep_alive.py republishes the camera TF at Spot's clock rate.
#
# Why RTAB-Map (not slam_toolbox):
#   slam_toolbox failed on featureless walls ("hallway problem").
#   RTAB-Map uses visual bag-of-words loop closure on RGB images.
#   The Agilent Hub has textured walls and equipment — ideal for visual SLAM.
#
# Grid params (--Grid/3D false --Grid/FromDepth true ...):
#   MUST be present in BOTH mapping and localization launch commands.
#   Without them, rtabmap.db stores no 2D grid data and the exported
#   rtabmap-2D.pgm will be all -1 (unknown), breaking Nav2 path planning.
#
# Map export (Phase 1 Save step 2):
#   Must run while T3 mapping is still active (before Ctrl+C).
#   If skipped, Nav2 will report "Robot is out of bounds of the costmap"
#   because the old 2D map has a different coordinate origin.
#
# Mapping tips:
#   Move slowly (<0.3 m/s). Loop closures need revisiting seen areas.
#   Start in a corner facing distinctive features (not a blank wall).
#   Walk full perimeter first, then cross passes through the middle.
#   Return to exact start position at end for loop closure.
# ══════════════════════════════════════════════════════════════
