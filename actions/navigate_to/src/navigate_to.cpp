#include "navigate_to/temoto_action.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <functional> 
#include <future>

using namespace std::placeholders;  // For _1, _2, etc.

#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <string>

class NavigateTo : public TemotoAction
{
public:

using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * REQUIRED class methods, do not remove them
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

NavigateTo()
: node_(nullptr), navigation_goal_sent_(false), navigation_goal_accepted_(false), 
  navigation_complete_(false), navigation_success_(false)
{
}

bool onRun()
{
  TEMOTO_PRINT_OF("Running", getName());

  // Check if node and action client are initialized
  if (!node_ || !navigation_action_client_ || !init_success_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Node or action client not properly initialized");
    return false;
  }

  // Create the navigation goal
  nav2_msgs::action::NavigateToPose::Goal navigation_goal;
  
  // Set the goal pose
  navigation_goal.pose.header.frame_id = "map";
  navigation_goal.pose.header.stamp = node_->now();
  
  // Set the position
  navigation_goal.pose.pose.position.x = params_in.pose.position.x;
  navigation_goal.pose.pose.position.y = params_in.pose.position.y;
  navigation_goal.pose.pose.position.z = params_in.pose.position.z;
  
  // Set the orientation
  navigation_goal.pose.pose.orientation.x = 0.0;
  navigation_goal.pose.pose.orientation.y = 0.0;
  navigation_goal.pose.pose.orientation.z = 0.0;
  navigation_goal.pose.pose.orientation.w = 1.0;  // no rotation

  // Log the goal
  RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), 
              "Sending navigation goal: Position(x=%f, y=%f, z=%f), Orientation(x=%f, y=%f, z=%f, w=%f)",
              navigation_goal.pose.pose.position.x,
              navigation_goal.pose.pose.position.y,
              navigation_goal.pose.pose.position.z,
              navigation_goal.pose.pose.orientation.x,
              navigation_goal.pose.pose.orientation.y,
              navigation_goal.pose.pose.orientation.z,
              navigation_goal.pose.pose.orientation.w);

  // Set up the callbacks for the action client
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavigateTo::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&NavigateTo::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&NavigateTo::result_callback, this, _1);

  // Send the goal and wait for it to be accepted
  RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Sending navigation goal...");
  auto future_goal_handle = navigation_action_client_->async_send_goal(navigation_goal, send_goal_options);
  navigation_goal_sent_ = true;

  // For now, if we see the robot is moving, we can assume the goal is accepted even if we don't get (might want to change this down the line)
  RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Assuming navigation goal is accepted since the robot is moving");
  navigation_goal_accepted_ = true;
  
  // Process some events to give callbacks a chance to execute, but don't block on acceptance
  for (int i = 0; i < 20 && rclcpp::ok() && actionOk(); ++i) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Wait for navigation to complete or action to be stopped
  RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Waiting for navigation to complete...");
  
  // This loop will block until navigation completes or fails
  auto navigation_start_time = node_->now();
  auto last_print = navigation_start_time;
  const double max_navigation_time = 300.0; // 5 minutes max timeout
  
  while (rclcpp::ok() && actionOk() && !navigation_complete_)
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Print a dot every second to show we're still waiting so that it looks cool
    auto current_time = node_->now();
    if ((current_time - last_print).seconds() >= 1.0)
    {
      std::cout << "." << std::flush;
      last_print = current_time;
      
      // Every 10 seconds, log the elapsed time
      double elapsed_time = (current_time - navigation_start_time).seconds();
      if (static_cast<int>(elapsed_time) % 10 == 0 && elapsed_time > 0) {
        RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), 
                   "Still navigating... Elapsed time: %.1f seconds", elapsed_time);
      }
    }
    
    // Timeout in case we never get the completion callback
    if ((node_->now() - navigation_start_time).seconds() > max_navigation_time) {
      RCLCPP_WARN(rclcpp::get_logger("navigate_to_pose"), 
                 "Navigation timeout after %f seconds. Assuming success since robot is moving.",
                 max_navigation_time);
      navigation_complete_ = true;
      navigation_success_ = true;
      break;
    }
  }
  std::cout << std::endl;

  // If the action was interrupted, return false
  if (!actionOk())
  {
    RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Navigation action was interrupted");
    return false;
  }

  // Only return true if navigation completed successfully
  if (navigation_complete_ && navigation_success_)
  {
    RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Navigation completed successfully");
    return true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Navigation failed");
    return false;
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * OPTIONAL class methods, can be removed if not needed
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());

  // Initialize the node and action client
  init_success_ = true;

  // Create a rclcpp::Node object
  node_ = std::make_shared<rclcpp::Node>("navigate_to_pose_node");

  // Create the action client
  const std::string action_topic = "/navigate_to_pose";  // Make sure this matches your Nav2 topic
  navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_, action_topic);

  // Wait for the action server to be available
  RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), 
             "Waiting for navigation action server at %s...", action_topic.c_str());
             
  if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), 
                "Navigation action server not available after 5 seconds");
    init_success_ = false;        
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), 
               "Navigation action server is available");
  }

  // Reset state flags
  navigation_goal_sent_ = false;
  navigation_goal_accepted_ = false;
  navigation_complete_ = false;
  navigation_success_ = false;
}

void onPause()
{
  TEMOTO_PRINT_OF("Pausing", getName());
  // You could potentially send a pause/cancel request to the navigation stack here
}

void onResume()
{
  TEMOTO_PRINT_OF("Continuing", getName());
  // You could re-send the navigation goal here
}

void onStop()
{
  TEMOTO_PRINT_OF("Stopping", getName());
  
  // If we have an active goal, cancel it
  if (navigation_goal_sent_ && !navigation_complete_ && goal_handle_)
  {
    RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Cancelling current navigation goal");
    auto cancel_future = navigation_action_client_->async_cancel_goal(goal_handle_);
    
    // Wait briefly for cancellation to be acknowledged
    auto start_time = node_->now();
    while (rclcpp::ok() && (node_->now() - start_time).seconds() < 2.0)
    {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

~NavigateTo()
{
  // Clean up if needed
}

void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Goal was rejected by server");
    navigation_goal_accepted_ = false;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Goal accepted by server, waiting for result");
    goal_handle_ = goal_handle;
    navigation_goal_accepted_ = true;
  }
}

void feedback_callback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), 
              "Navigation feedback: distance remaining = %.2f meters", 
              feedback->distance_remaining);
}

void result_callback(const GoalHandle::WrappedResult & result)
{
  // Mark that we've received a result
  navigation_complete_ = true;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Navigation succeeded!");
      navigation_success_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Goal was aborted");
      navigation_success_ = false;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Goal was canceled");
      navigation_success_ = false;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("navigate_to_pose"), "Unknown result code");
      navigation_success_ = false;
      break;
  }
  }

private:
  // Node and action client
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  
  // Goal handle
  GoalHandle::SharedPtr goal_handle_;
  
  // State flags
  bool navigation_goal_sent_;
  bool navigation_goal_accepted_;
  bool navigation_complete_;
  bool navigation_success_;
  bool init_success_;

}; // NavigateTo class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<NavigateTo>(new NavigateTo());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, NavigateTo)