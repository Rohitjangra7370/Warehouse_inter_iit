#include "warehouse_rover_mission_control/mission_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace warehouse_rover_mission_control
{

MissionController::MissionController(const rclcpp::NodeOptions & options)
: Node("mission_controller", options),
  state_(MissionState::IDLE),
  current_rack_idx_(0)
{
  // Parameters (kept for compatibility, but scan timing now controlled by lift service)
  this->declare_parameter<double>("scan_dwell_time", 3.0);
  this->get_parameter("scan_dwell_time", scan_dwell_time_);
  
  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    "navigate_to_pose"
  );
  
  // Lift service client
  lift_service_client_ = this->create_client<std_srvs::srv::Trigger>(
    "/lift_controller/scan_cycle"
  );
  
  // QR detection subscriber (for monitoring only)
  qr_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/qr_detections/status",
    10,
    std::bind(&MissionController::qrDetectionCallback, this, std::placeholders::_1)
  );
  
  // Load waypoints
  loadWaypoints();
  
  RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(this->get_logger(), "║      WAREHOUSE MISSION CONTROLLER STARTED             ║");
  RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════╝");
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "📋 Mission Configuration:");
  RCLCPP_INFO(this->get_logger(), "   Racks to scan: %zu", waypoints_.size());
  RCLCPP_INFO(this->get_logger(), "   Shelves per rack: %zu", waypoints_[0].shelf_heights.size());
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "🔌 Waiting for services...");
  
  // Wait for Nav2
  if (!nav_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(this->get_logger(), "❌ Nav2 action server not available!");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "✅ Nav2 connected");
  
  // Wait for lift service
  if (!lift_service_client_->wait_for_service(5s)) {
    RCLCPP_WARN(this->get_logger(), "⚠️  Lift service not available yet");
  } else {
    RCLCPP_INFO(this->get_logger(), "✅ Lift controller connected");
  }
  
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "✅ Ready to start mission!");
  RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════════════════════");
}

void MissionController::loadWaypoints()
{
  // MEASURED RACK POSITIONS FROM NEW MAP
  RackWaypoint r1;
  r1.name = "RACK_1";
  r1.x = -1.787;
  r1.y = 3.949;
  r1.theta = 0.0 * M_PI / 180.0;  // Faces EAST
  r1.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r1);
  
  RackWaypoint r2;
  r2.name = "RACK_2";
  r2.x = -0.582;
  r2.y = 3.960;
  r2.theta = 0.0 * M_PI / 180.0;  // Faces EAST
  r2.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r2);
  
  RackWaypoint r3;
  r3.name = "RACK_3";
  r3.x = -0.322;
  r3.y = 3.634;
  r3.theta = 180.0 * M_PI / 180.0;  // Faces WEST
  r3.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r3);
  
  RackWaypoint r4;
  r4.name = "RACK_4";
  r4.x = -0.308;
  r4.y = 2.575;
  r4.theta = 180.0 * M_PI / 180.0;  // Faces WEST
  r4.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r4);
  
  RackWaypoint r5;
  r5.name = "RACK_5";
  r5.x = -0.205;
  r5.y = 1.463;
  r5.theta = 180.0 * M_PI / 180.0;  // Faces WEST
  r5.shelf_heights = {0.0, 0.25, 0.50, 0.75};
  waypoints_.push_back(r5);
  
  RCLCPP_INFO(this->get_logger(), "Loaded %zu racks from map measurements", waypoints_.size());
}

void MissionController::startMission()
{
  if (waypoints_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "❌ No waypoints loaded!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(this->get_logger(), "║           STARTING WAREHOUSE SCAN MISSION             ║");
  RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════╝");
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "🎯 Mission Plan:");
  RCLCPP_INFO(this->get_logger(), "   Total racks: %zu", waypoints_.size());
  RCLCPP_INFO(this->get_logger(), "   Shelves per rack: %zu", waypoints_[0].shelf_heights.size());
  RCLCPP_INFO(this->get_logger(), "   Total scan points: %zu", 
              waypoints_.size() * waypoints_[0].shelf_heights.size());
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "🚀 Mission starting...");
  RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════════════════════");
  
  current_rack_idx_ = 0;
  state_ = MissionState::IDLE;
  
  executeNextRack();
}

void MissionController::executeNextRack()
{
  if (current_rack_idx_ >= waypoints_.size()) {
    state_ = MissionState::MISSION_COMPLETE;
    
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║              🎉 MISSION COMPLETE! 🎉                   ║");
    RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "📊 Mission Summary:");
    RCLCPP_INFO(this->get_logger(), "   Racks scanned: %zu", waypoints_.size());
    RCLCPP_INFO(this->get_logger(), "   Total shelves: %zu", 
                waypoints_.size() * waypoints_[0].shelf_heights.size());
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "✅ All racks scanned successfully!");
    RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════════════════════");
    return;
  }
  
  const auto & rack = waypoints_[current_rack_idx_];
  
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "┌────────────────────────────────────────────────────────┐");
  RCLCPP_INFO(this->get_logger(), "│ 🎯 RACK %zu/%zu: %-40s │",
              current_rack_idx_ + 1, waypoints_.size(), rack.name.c_str());
  RCLCPP_INFO(this->get_logger(), "└────────────────────────────────────────────────────────┘");
  
  navigateToRack(rack);
}

void MissionController::navigateToRack(const RackWaypoint & rack)
{
  state_ = MissionState::NAVIGATING_TO_RACK;
  
  RCLCPP_INFO(this->get_logger(), "🚗 Navigating to: (%.2f, %.2f) θ=%.0f°",
              rack.x, rack.y, rack.theta * 180.0 / M_PI);
  
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  
  goal_msg.pose.pose.position.x = rack.x;
  goal_msg.pose.pose.position.y = rack.y;
  goal_msg.pose.pose.position.z = 0.0;
  
  goal_msg.pose.pose.orientation.z = std::sin(rack.theta / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(rack.theta / 2.0);
  
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&MissionController::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&MissionController::feedbackCallback, this, 
              std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&MissionController::resultCallback, this, std::placeholders::_1);
  
  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionController::scanRack(const std::string & rack_name)
{
  state_ = MissionState::SCANNING_SHELVES;
  
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "🔍 Starting shelf scan for %s", rack_name.c_str());
  
  // Wait for lift service if not already available
  if (!lift_service_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(this->get_logger(), "❌ Lift service not available!");
    state_ = MissionState::FAILED;
    return;
  }

  // Call lift service to perform full scan cycle
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  
  RCLCPP_INFO(this->get_logger(), "📞 Requesting lift scan cycle...");
  
  auto result_future = lift_service_client_->async_send_request(request);
  
  // Wait for scan cycle to complete
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "✅ Scan cycle completed: %s", result->message.c_str());
      state_ = MissionState::RACK_COMPLETE;
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️  Scan cycle issue: %s", result->message.c_str());
      state_ = MissionState::RACK_COMPLETE;  // Continue anyway
    }
    
    // Move to next rack
    current_rack_idx_++;
    executeNextRack();
    
  } else {
    RCLCPP_ERROR(this->get_logger(), "❌ Failed to complete scan cycle");
    state_ = MissionState::FAILED;
  }
}

void MissionController::goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "❌ Navigation goal rejected!");
    state_ = MissionState::FAILED;
  } else {
    RCLCPP_INFO(this->get_logger(), "✅ Navigation goal accepted");
  }
}

void MissionController::feedbackCallback(
  GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  auto current_pose = feedback->current_pose.pose;
  RCLCPP_DEBUG(this->get_logger(), 
    "📍 Position: (%.2f, %.2f)", 
    current_pose.position.x, current_pose.position.y);
}

void MissionController::resultCallback(const GoalHandleNav::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "✅ Arrived at rack!");
      
      if (current_rack_idx_ < waypoints_.size()) {
        const auto & rack = waypoints_[current_rack_idx_];
        scanRack(rack.name);
      }
      break;
      
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "❌ Navigation aborted!");
      state_ = MissionState::FAILED;
      break;
      
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "⚠️  Navigation canceled");
      state_ = MissionState::FAILED;
      break;
      
    default:
      RCLCPP_ERROR(this->get_logger(), "❌ Unknown navigation result");
      state_ = MissionState::FAILED;
      break;
  }
}

void MissionController::qrDetectionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Just log for monitoring purposes
  if (state_ == MissionState::SCANNING_SHELVES) {
    if (msg->data == "detected") {
      RCLCPP_DEBUG(this->get_logger(), "📦 QR code detected");
    }
  }
}

}  // namespace warehouse_rover_mission_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<warehouse_rover_mission_control::MissionController>(
    rclcpp::NodeOptions()
  );
  
  // Small delay to ensure all services are ready
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  node->startMission();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}