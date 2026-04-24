#!/bin/bash
# Serves the 2D map export from RTAB-Map as a proper TRANSIENT_LOCAL /map topic.
# Use this instead of map_relay.py during Phase 2 (localization + Nav2).
# Re-export the 2D map after each new mapping run:
#   ros2 run rtabmap_util rtabmap_export_map /root/spot_maps/rtabmap-2D (if available)
# Or use the .pgm/.yaml already exported by RTAB-Map on backup.
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/root/spot_maps/rtabmap-2D.yaml -p use_sim_time:=false
