# Warehouse Rover — Inter IIT Tech Meet 14.0

Autonomous mecanum-wheeled warehouse rover built in ROS2 (Humble). The robot navigates to racks, extends a telescopic lift through 4 shelf heights, scans QR codes at each height, and logs inventory to a database with a web dashboard.

> ⚠️ **Not the final codebase.** This is a mid-prep / development snapshot. The final competition submission is not published here — under Inter-IIT competition rules, teams cannot make final solution code public, so it was removed from this repo. What remains is earlier work kept for reference and portfolio purposes.

## System Overview

| Package | Role |
|---|---|
| `warehouse_rover_mission_control` | State-machine mission orchestrator: sequences rack waypoints, drives Nav2 goals, triggers lift scans |
| `warehouse_rover_lift_control` | Telescopic lift driver — moves through 4 shelf heights (0.0 / 0.25 / 0.50 / 0.75 m) via a ROS2 service |
| `warehouse_rover_image_processing` / `warehouse_rover_qr_detection` | Multi-pass QR detection pipeline (histogram equalization, adaptive threshold, sharpening, morphology) for robust decoding under warehouse lighting |
| `warehouse_rover_rack_detection` | Rack/shelf pose extraction for waypoint alignment |
| `warehouse_rover_database` | SQLite inventory storage + web dashboard |
| `navigation_setup` | Nav2 stack config — AMCL localization, A* global planner, DWB local controller, static warehouse map |
| `mecanum_in_gazebo` | Gazebo simulation of the mecanum-drive rover |
| `waypoint_recorder` | Utility to capture and save rack waypoints from the live map |
| `warehouse_rover_msgs` | Shared custom message/service definitions |

**Mission flow:** Nav2 drives the rover to each rack → lift controller cycles through 4 heights (~5s/height) → camera captures frames at each height → QR detector decodes inventory tags → results are written to the database → rover proceeds to the next rack. See `src/Inter-IIT-Robotics-Midprep/SYSTEM_OVERVIEW_CONCISE.md` for a full file-by-file breakdown.

## Quick Start

See [`QUICKSTART.md`](QUICKSTART.md) for build/run instructions (ROS2 Humble, Ubuntu 22.04, `colcon build`).

```bash
ros2 launch mecanum_in_gazebo gazebo.launch.py                                  # sim only
ros2 launch warehouse_rover_mission_control master_autonomous_warehouse.launch.py  # full autonomous mission
```

## Repo Notes

- This is a snapshot of the ROS2 workspace `src/` tree, minus build artifacts (`build/`, `install/`, `log/`) and large binaries.
- `src/Inter-IIT-Robotics-Midprep/` was originally its own git checkout; it's included here as plain files (its separate history isn't carried over).
