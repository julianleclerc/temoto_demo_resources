#include <functional>
#include <class_loader/class_loader.hpp>
#include "navigate_to_pose/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::placeholders;

class NavigateToPose : public TemotoAction
{
public:

using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;


NavigateToPose() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());

  // Create a rclcpp::Node object here
  auto node = std::make_shared<rclcpp::Node>("navigate_to_pose_node");

  const std::string action_topic = params_in.robot_name + "/navigate_to_pose";
  navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, action_topic);

  if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "%s action server not available, aborting call for robot navigation", action_topic.c_str());
    // return false;    
  }

}

bool onRun() // REQUIRED
{  

  // Generate the action request
  nav2_msgs::action::NavigateToPose::Goal navigation_goal;
  navigation_goal.pose.pose.position.x = params_in.pose.position.x;
  navigation_goal.pose.pose.position.y = params_in.pose.position.y;
  navigation_goal.pose.pose.position.z = params_in.pose.position.z;


  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigateToPose::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavigateToPose::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavigateToPose::result_callback, this, _1);

  goal_handle_future_ = navigation_action_client_->async_send_goal(navigation_goal);

  


  
  // std::string output = fmt::format("Moving to '{}' \nx = {:<10} r = {}\ny = {:<10} p = {}\nz = {:<10} y = {}",
  //   params_in.robot_name,
  //   params_in.location,
  //   params_in.pose.position.x, params_in.pose.orientation.r,
  //   params_in.pose.position.y, params_in.pose.orientation.p,
  //   params_in.pose.position.z, params_in.pose.orientation.y);

  // TEMOTO_PRINT_OF(output, getName());

  // uint sleep_ms{1000};
  // for (uint i{0}; i<3 && actionOk() ; i++)
  // {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  //   std::cout << ". " << std::flush;
  // }

  // std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  // std::cout << std::endl;

  // TEMOTO_PRINT_OF("Done\n", getName());

  return true;
}

void onPause()
{
  TEMOTO_PRINT_OF("Pausing", getName());
}

void onContinue()
{
  TEMOTO_PRINT_OF("Continuing", getName());
}

void onStop()
{
  TEMOTO_PRINT_OF("Stopping", getName());
}

~NavigateToPose()
{
}


void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Goal accepted by server, waiting for result");
  }
}

void feedback_callback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  std::stringstream ss;

  ss << "distance_remaining: " << feedback->distance_remaining;

  RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "%s", ss.str().c_str());
}

void result_callback(const GoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Unknown result code");
      return;
  }
  
  rclcpp::shutdown();
}

rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;

// Goal Handle and its future
std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;



}; // NavigateToPose class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(NavigateToPose, ActionBase);


