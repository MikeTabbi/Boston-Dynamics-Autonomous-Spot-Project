#!/bin/bash
# ============================================================
# Spot Autonomous Exploration - Launch Commands
# ============================================================
# TWO-PHASE WORKFLOW:
#
#   PHASE 1 — BUILD MAP
#     Terminals: 1, 2, 3-MAP, 5, 6-MAP
#     auto_mapper drives Spot and saves map when done.
#     Stop Terminals 3-MAP and 6-MAP after map saves.
#
#   PHASE 2 — EXPLORE
#     Terminals: 1, 2, 3-LOC, 4, 5, 6A (probe) then 6B (frontier)
#
# Start each terminal in order. Wait ~5s between terminals.
# Credentials and cmd_duration are set via /root/spot_config.yaml.
# ============================================================


# ── TERMINAL 1 ── Spot Driver ──────────────────────────────
docker start -i spot_nav

source ~/.bashrc
ros2 launch spot_driver spot_driver.launch.py spot_name:=spot config_file:=/root/spot_config.yaml


# ── TERMINAL 2 ── Single Front Camera ──────────────────────
# frontleft depth -> /scan directly. No scan merger.
# output_frame:=spot/body ensures angle=0 = robot forward.
# range_min:=0.85 filters Spot's front-left leg (~0.81m in body frame).
# Use in BOTH phases — identical command each time.
docker exec -it spot_nav bash

source ~/.bashrc
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args -r depth:=/spot/depth/frontleft/image -r depth_camera_info:=/spot/depth/frontleft/camera_info -r scan:=/scan -p range_min:=0.85 -p range_max:=10.0 -p scan_height:=10 -p output_frame:=spot/body


# ── TERMINAL 3-MAP ── SLAM Toolbox (MAPPING mode) ──────────
# PHASE 1 ONLY. Builds map live while auto_mapper drives.
# Stop this terminal after Terminal 6-MAP saves the map.
docker exec -it spot_nav bash

source ~/.bashrc
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/root/slam_params.yaml


# ── TERMINAL 5 ── cmd_vel Relay ────────────────────────────
# REQUIRED in both phases.
docker exec -it spot_nav bash

source ~/.bashrc
ros2 run topic_tools relay /cmd_vel /spot/cmd_vel


# ── TERMINAL 6-MAP ── Autonomous Mapper (PHASE 1) ──────────
# Drives Spot to build the map. No Nav2 required.
# Motion strategy: drive-pause-wiggle
#   drive 2.5s -> stop 1.0s -> wiggle ±15deg -> every 3 cycles: turn 90deg -> repeat
#   obstacle: backup 2s -> turn 90deg -> continue
# Map saves automatically: /root/my_map.pgm + .yaml + .posegraph
docker exec -it spot_nav bash

source ~/.bashrc
python3 -u /root/auto_mapper.py --ros-args -p duration_sec:=600.0 -p forward_speed:=0.10 -p drive_sec:=2.5 -p pause_sec:=1.0 -p wiggle_deg:=15.0 -p wiggle_speed:=0.20 -p cycles_before_turn:=3 -p turn_deg:=90.0 -p obstacle_distance:=0.75 -p forward_cone_deg:=20.0 -p min_obstacle_hits:=5 -p backup_sec:=2.0 -p map_save_path:=/root/my_map
# After this exits: stop Terminal 3-MAP, inspect map, then start Phase 2.

# Copy map to Desktop to inspect:
# docker cp spot_nav:/root/my_map.pgm /Users/temp/Desktop/my_map.pgm


# ── TERMINAL 3-LOC ── SLAM Toolbox (LOCALIZATION mode) ─────
# PHASE 2 ONLY. Use instead of Terminal 3-MAP.
# Loads /root/my_map.posegraph saved by auto_mapper.
# Localizes on the fixed map — does NOT rebuild it.
# Provides map->odom TF to Nav2.
docker exec -it spot_nav bash

source ~/.bashrc
ros2 launch slam_toolbox localization_launch.py slam_params_file:=/root/slam_params_localization.yaml


# ── TERMINAL 4 ── Nav2 (PHASE 2 only) ──────────────────────
# navigation_launch.py only — no map server, no AMCL.
# slam_toolbox (Terminal 3-LOC) provides map->odom TF.
# Do NOT use bringup_launch.py.
docker exec -it spot_nav bash

source ~/.bashrc
ros2 launch nav2_bringup navigation_launch.py params_file:=/root/nav2_params.yaml use_sim_time:=false


# ── TERMINAL 6A ── Waypoint Probe (PHASE 2 — run first) ────
# Edit WAYPOINTS in /root/waypoint_probe.py before running.
# Use RViz 2D Pose Estimate to read map-frame coordinates.
# Run this before 6B — it proves Nav2 + localization work.
docker exec -it spot_nav bash

source ~/.bashrc
python3 -u /root/waypoint_probe.py


# ── TERMINAL 6B ── Frontier Explorer (after probe passes) ──
# Only switch to this once waypoint_probe.py passes all goals cleanly.
# docker exec -it spot_nav bash
# source ~/.bashrc
# python3 -u /root/frontier_explore_hybrid.py --ros-args \
#   -p min_frontier_cluster:=8 \
#   -p goal_clearance_cells:=6 \
#   -p max_goal_distance:=6.0 \
#   -p cost_threshold:=120


# ── VERIFY + CLEAR (spare terminal) ────────────────────────
# docker exec -it spot_nav bash
# source ~/.bashrc
#
# TF chain intact (must show: map -> spot/odom -> spot/body)
# ros2 run tf2_tools view_frames
#
# Scan arriving (~4 Hz)
# ros2 topic hz /scan
#
# Map publishing
# ros2 topic hz /map
#
# Nav2 ready (Phase 2)
# ros2 action list
#
# Clear costmaps if goal aborts (status=6)
# ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}
# ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}
#
# Relay working
# ros2 topic hz /spot/cmd_vel


# ============================================================
# NOTES
# ============================================================
# Clock sync:
#   Container clock is ~130s ahead of Spot. cmd_duration:=5.0 in
#   /root/spot_config.yaml gives commands a 5s validity window.
#   Do NOT pass cmd_duration as a launch arg — must be in config_file.
#
# Single camera:
#   Scan merger abandoned — both front cameras point forward, giving no
#   side coverage. Wall detection at side_angle_deg=35 needs range_min=0.85
#   to filter the leg (~0.81m). obstacle_distance=0.75 is below 0.85 range
#   so obstacle detection uses the filtered scan correctly.
#
# Phase 1 tips:
#   Start Spot in the middle of the room, floor clear.
#   Keep E-Stop handy. duration_sec=600 = 10 min.
#   auto_mapper sweeps 75deg away from wall after each 8s segment —
#   sends Spot diagonally across room for loop closure opportunities.
#   Map saves automatically — inspect .pgm before starting Phase 2.
#
# Phase 2 tips:
#   slam_toolbox LOCALIZATION mode uses .posegraph (not .pgm).
#   Use navigation_launch.py only — not bringup_launch.py.
#   Run waypoint probe first. Clear costmaps if goal aborts (status=6).
#   Only switch to frontier explorer after probe passes cleanly.
# ============================================================
