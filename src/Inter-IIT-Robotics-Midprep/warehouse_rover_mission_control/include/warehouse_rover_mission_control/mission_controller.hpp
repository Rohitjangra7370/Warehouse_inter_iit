#ifndef WAREHOUSE_ROVER_MISSION_CONTROL__MISSION_CONTROLLER_HPP_
#define WAREHOUSE_ROVER_MISSION_CONTROL__MISSION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>  // ADD THIS
#include <vector>
#include <string>

namespace warehouse_rover_mission_control
{

enum class MissionState
{
  IDLE,
  NAVIGATING_TO_RACK,
  SCANNING_SHELVES,      // CHANGED from ADJUSTING_LIFT and SCANNING_QR
  RACK_COMPLETE,
  MISSION_COMPLETE,
  FAILED
};

struct RackWaypoint
{
  std::string name;
  double x;
  double y;
  double theta;
  std::vector<double> shelf_heights;  // Keep for reference, but not used directly
};

class MissionController : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MissionController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
  void startMission();

private:
  void loadWaypoints();
  void executeNextRack();
  void navigateToRack(const RackWaypoint & rack);
  void scanRack(const std::string & rack_name);  // CHANGED signature
  
  // Nav2 callbacks
  void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandleNav::WrappedResult & result);
  
  // QR detection callback (kept for monitoring)
  void qrDetectionCallback(const std_msgs::msg::String::SharedPtr msg);
  
  // ROS2 interfaces
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lift_service_client_;  // CHANGED
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  
  // Mission state
  MissionState state_;
  std::vector<RackWaypoint> waypoints_;
  size_t current_rack_idx_;
  
  // Parameters
  double scan_dwell_time_;  // Not used anymore but kept for compatibility
};

}  // namespace warehouse_rover_mission_control

#endif  // WAREHOUSE_ROVER_MISSION_CONTROL__MISSION_CONTROLLER_HPP_