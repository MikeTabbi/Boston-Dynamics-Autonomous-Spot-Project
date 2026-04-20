# Boston Dynamics Spot — Autonomous Navigation

ROS2 Humble autonomous mapping and navigation stack for Boston Dynamics Spot, running inside a Docker container (`spot_nav`).

## Two-Phase Workflow

### Phase 1 — Build Map
Spot drives autonomously using `auto_mapper.py` while `slam_toolbox` builds a map. No Nav2 required.

### Phase 2 — Explore
Slam_toolbox localizes on the saved map. Nav2 handles path planning. `waypoint_probe.py` validates Nav2 before switching to `frontier_explore_hybrid.py` for autonomous exploration.

## Setup

1. Copy `spot_config.yaml.example` to `spot_config.yaml` and fill in your robot's credentials.
2. Copy all files into the container: `docker cp <file> spot_nav:/root/`
3. Follow `launch_commands.sh` terminal by terminal.

## Files

| File | Description |
|---|---|
| `launch_commands.sh` | Full terminal-by-terminal startup reference |
| `auto_mapper.py` | Autonomous mapping behavior (Phase 1) |
| `waypoint_probe.py` | Nav2 validation — sends sequential waypoints |
| `frontier_explore_hybrid.py` | Autonomous frontier exploration (Phase 2) |
| `scan_merger.py` | Merges /scan_fl + /scan_fr (currently unused — single camera mode) |
| `slam_params.yaml` | slam_toolbox mapping config |
| `slam_params_localization.yaml` | slam_toolbox localization config |
| `nav2_params.yaml` | Nav2 config |
| `spot_config.yaml.example` | Credentials template |
| `depth_crop_relay.py` | Debug tool: crops depth image to a row band |
| `debug_band_compare.py` | Debug tool: compares scan quality between two bands |

## Notes

- Container clock is ~130s ahead of Spot — `cmd_duration: 5.0` in `spot_config.yaml` fixes this.
- Single frontleft camera only (`output_frame:=spot/body`, `range_min:=0.85`). Scan merger abandoned — both front cameras point forward, no side coverage.
- `auto_mapper.py` uses a sweep state (75°) after each wall-follow segment to send Spot diagonally across the room for slam_toolbox loop closure opportunities.
- Nav2: use `navigation_launch.py` only, **not** `bringup_launch.py`.
