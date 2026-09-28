void Explore::makePlan() {
    // 1. Get robot's current pose
    auto pose = costmap_client_.getRobotPose();
    
    // 2. Search for frontiers from current position
    auto frontiers = search_.searchFrom(pose.position);
    
    // 3. If no frontiers found, stop exploring
    if (frontiers.empty()) {
        stop(true);
        return;
    }
    
    // 4. Visualize frontiers (optional)
    if (visualize_) {
        visualizeFrontiers(frontiers);
    }
    
    // 5. Find first non-blacklisted frontier
    auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
        [this](const frontier_exploration::Frontier& f) {
            return goalOnBlacklist(f.centroid);
        });
    
    // 6. Check if making progress, blacklist if stuck
    if (timeout && !making_progress) {
        frontier_blacklist_.push_back(target_position);
        makePlan();  // Recursively find new goal
        return;
    }
    
    // 7. Send navigation goal to move_base
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = frontier->centroid;
    move_base_client_->async_send_goal(goal, send_goal_options);
}





/**:
  ros__parameters:
    robot_base_frame: base_link
    return_to_init: true
    costmap_topic: map              # Subscribe to raw SLAM map
    costmap_updates_topic: map_updates
    visualize: true
    planner_frequency: 0.15         # Replan every 6.7 seconds
    progress_timeout: 30.0          # Blacklist goal after 30s
    potential_scale: 3.0            # Distance weight
    orientation_scale: 0.0          # Unused
    gain_scale: 1.0                 # Size weight
    transform_tolerance: 0.3
    min_frontier_size: 0.75         # 75cm minimum















    warehouse_map:
  map_file: "warehouse_floor.pgm"
  resolution: 0.05
  origin: [-10.0, -10.0, 0.0]

racks:
  - id: "rack_001"
    approach_point:
      x: 5.2
      y: 3.7
      theta: 1.57  # facing rack
    rack_center:
      x: 5.9
      y: 3.7
    scan_config:
      heights: [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8]
      lateral_scan: false
    
  - id: "rack_002"
    approach_point:
      x: 5.2
      y: 5.5
      theta: 1.57
    rack_center:
      x: 5.9
      y: 5.5
    scan_config:
      heights: [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8]
      lateral_scan: false

# Optimized visit order (after TSP)
visit_order: ["rack_001", "rack_002", "rack_005", "rack_003", "rack_004"]
```

**This file:**
- ✅ Generated once after verification run
- ✅ Used for all subsequent inventory missions
- ✅ Can be manually edited if needed
- ✅ Contains approach points + scan parameters

---

## Complete Workflow Summary
```
┌─────────────────────────────────────────┐
│ Phase 1: Exploration (m-explore)        │
│ Output: warehouse_floor.pgm + .yaml     │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ Phase 2: Rack Detection (clustering)    │
│ Output: candidate_racks.yaml            │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ Phase 3: Approach Point Calculation     │
│ Output: approach_points.yaml            │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ Phase 4: Route Optimization (greedy)    │
│ Output: optimized_racks.yaml            │
│         (with visit_order)              │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ Phase 5: Verification Run               │
│ - Navigate to each rack in order        │
│ - Perform FULL vertical scan            │
│ - Detect QRs, confirm rack validity     │
│ Output: verified_racks.yaml             │
└─────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────┐
│ Phase 6: Inventory Missions (repeated)  │
│ - Load verified_racks.yaml              │
│ - Navigate in optimized order           │
│ - ALWAYS scan full height range         │
│ - Log all detected QRs with timestamp   │
└─────────────────────────────────────────┘


ros2 run explore_lite explore --ros-args --params-file <path_to_ros_ws>/m-explore-ros2/explore/config/params.yaml