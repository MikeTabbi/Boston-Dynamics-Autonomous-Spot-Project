# Boston Dynamics Spot — Autonomous Navigation

ROS2 Humble autonomous mapping and navigation stack for Boston Dynamics Spot, running inside a Docker container (`spot_nav`).

## Architecture

**RTAB-Map RGB-D Visual SLAM** using the front-left depth camera.

- Phase 1: Drive Spot manually to build a visual SLAM map (`rtabmap_new.db`)
- Phase 2: RTAB-Map localizes on the saved map; Nav2 handles path planning; `frontier_explore_hybrid.py` drives autonomous exploration

## Setup

1. Copy `spot_config.yaml.example` → `spot_config.yaml` and fill in your robot credentials.
2. Copy all files into the container:
   ```bash
   docker cp <file> spot_nav:/root/
   ```
3. Follow `launch_commands.sh` terminal by terminal.

## Files

| File | Description |
|---|---|
| `launch_commands.sh` | Full terminal-by-terminal startup reference (Phase 1 + Phase 2) |
| `tf_keep_alive.py` | Republishes camera TF at Spot's clock rate (fixes ~130s clock offset) |
| `autonomous_mapper.py` | Optional autonomous Phase 1 mapper (state machine: forward → backup → turn) |
| `waypoint_probe.py` | Nav2 validation — drives to 2 waypoints, logs ok/fail per goal |
| `frontier_explore_hybrid.py` | Autonomous frontier exploration (Phase 2) |
| `nav2_params.yaml` | Nav2 config (DWB controller, `/rtabmap/cloud_obstacles` costmap layers) |
| `start_rtabmap_map.sh` | One-line Phase 1 mapping launch (writes `rtabmap_new.db`) |
| `start_rtabmap_loc.sh` | One-line Phase 2 localization launch (reads `rtabmap_new.db`) |
| `start_map_server.sh` | Launches nav2_map_server with TRANSIENT_LOCAL `/map` |
| `activate_map_server.sh` | Configures and activates the map_server lifecycle node |
| `spot_config.yaml.example` | Credentials template |

## Key Notes

- **Clock offset:** Spot's hardware clock is ~130s ahead of the container. `tf_keep_alive.py` must run in Phase 1 and Phase 2. `cmd_duration: 5.0` in `spot_config.yaml` gives commands a 5s validity window.
- **Grid params** (`--Grid/3D false --Grid/FromDepth true ...`) must be present in **both** the mapping and localization RTAB-Map commands — without them the exported 2D map is all -1 (unknown) and Nav2 cannot plan paths.
- **Map export** (Phase 1 save step 2) must run while the mapping node is still active, before Ctrl+C. If skipped, the map coordinate origin will be stale and Nav2 will report "Robot is out of bounds of the costmap".
- **Frontier exploration** requires `map_topic:=/rtabmap/grid_prob_map` — the static `/map` from map_server has no unknown cells and will find no frontiers.
- **Loop closures:** Walk the full perimeter, then cross passes, then return to exact start. Aim for 2+ loop closures before saving.
