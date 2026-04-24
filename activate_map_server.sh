#!/bin/bash
# Run this once after start_map_server.sh is up to activate the lifecycle node.
ros2 lifecycle set /map_server configure
sleep 1
ros2 lifecycle set /map_server activate
