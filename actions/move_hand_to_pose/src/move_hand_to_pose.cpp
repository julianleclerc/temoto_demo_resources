#include "move_hand_to_pose/temoto_action.hpp"

#include "rclcpp/rclcpp.hpp"
#include <functional> 
#include <future>

// #include <moveit/move_group_interface/move_group_interface.h>

using namespace std::placeholders;  // For _1, _2, etc.

#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <string>

class MoveHandToPose : public TemotoAction
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * REQUIRED class methods, do not remove them
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

MoveHandToPose()
: node_(nullptr)
{
}

bool onRun()
{
  TEMOTO_PRINT_OF("Running", getName());

  
  moveit_msgs::action::MoveGroup::Goal move_group_goal;
  move_group_goal.planning_options.plan_only = false;
  move_group_goal.planning_options.replan = false;
  move_group_goal.request.allowed_planning_time = params_in.max_planning_time;
  move_group_goal.request.max_velocity_scaling_factor = 0.1;
  // move_group_goal.request.max_velocity_scaling_factor = max_velocity_scaling_factor_;


  geometry_msgs::msg::PoseStamped target_pose;

  target_pose.header.frame_id = params_in.pose.frame_id; // "arm0_base_link"
  target_pose.pose.position.x = params_in.pose.position.x ;
  target_pose.pose.position.y = params_in.pose.position.y ;
  target_pose.pose.position.z = params_in.pose.position.z ;
  target_pose.pose.orientation.x = params_in.pose.orientation.x;
  target_pose.pose.orientation.y = params_in.pose.orientation.y;
  target_pose.pose.orientation.z = params_in.pose.orientation.z;
  target_pose.pose.orientation.w = params_in.pose.orientation.w;

  
  // std::string target_link = params_in.target_link;  // arm0_hand
  

  move_group_goal.request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(params_in.target_link, target_pose)
  );
  move_group_goal.request.group_name = params_in.planning_group;
  move_group_goal.request.workspace_parameters.header.frame_id = target_pose.header.frame_id;
  move_group_goal.request.workspace_parameters.header.stamp = node_->now();
  move_group_goal.request.workspace_parameters.min_corner.x = -1e9;
  move_group_goal.request.workspace_parameters.min_corner.y = -1e9;
  move_group_goal.request.workspace_parameters.min_corner.z = -1e9;
  move_group_goal.request.workspace_parameters.max_corner.x = +1e9;
  move_group_goal.request.workspace_parameters.max_corner.y = +1e9;
  move_group_goal.request.workspace_parameters.max_corner.z = +1e9;


  RCLCPP_INFO(rclcpp::get_logger("move_hand_to_pose"), "Sending move group goal to action server");

  move_group_response_future_ = move_group_action_client_->async_send_goal(move_group_goal);
  request_timestamp_ = move_group_goal.request.workspace_parameters.header.stamp;


  // Wait for navigation to complete or action to be stopped
  RCLCPP_INFO(rclcpp::get_logger("move_hand_to_pose"), "Waiting for trajectory to complete...");
    
  // This loop will block until navigation completes or fails
  auto goal_start_time = node_->now();
  auto last_print = goal_start_time;
  const double max_time = 30.0; // 0.5 minutes max timeout
  
  while (rclcpp::ok() && actionOk())
  {
    // If the action was interrupted, return false
    if (!actionOk())
    {
      RCLCPP_INFO(rclcpp::get_logger("move_hand_to_pose"), "Goan action was interrupted");
      return false;
    }
    
    if (move_group_response_future_.valid())
    {
      const auto result = rclcpp::spin_until_future_complete(node_, move_group_response_future_, std::chrono::milliseconds(5));

      switch (result)
      {
        case rclcpp::FutureReturnCode::SUCCESS:
        {
          move_group_goal_handle_ = move_group_response_future_.get();
          move_group_response_future_ = decltype(move_group_response_future_){};
          break;
        }
        case rclcpp::FutureReturnCode::TIMEOUT:
        {
          const double elapsed_seconds = (node_->now() - request_timestamp_).seconds();
          if (elapsed_seconds > 2.0)
          {
            RCLCPP_ERROR(rclcpp::get_logger("move_hand_to_pose"), "Timed out waiting for action server to respond. Aborting MoveHandToPose goal");
            onStop();
            return false;
          }
          break;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED:
        {
          RCLCPP_WARN(rclcpp::get_logger("move_hand_to_pose"), "MoveHandToPose request interrupted. Reporting failed movement");
          onStop();
          return false;
        }
      }
    }

    // Once we reach this part of the function, a goal handle MUST be active
    if (!move_group_goal_handle_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("move_hand_to_pose"), "MoveHandToPose has no active action or action request. This should never happen");
      return false;
    }

    // Check if the action is ongoing, or if it has concluded
    rclcpp::spin_some(node_);
    const int8_t goal_status = move_group_goal_handle_->get_status();
    switch (goal_status)
    {
        // case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        // case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        // case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
            // return BT::NodeStatus::RUNNING;

        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
        {
          RCLCPP_WARN(rclcpp::get_logger("move_hand_to_pose"), "action returned status UNKNOWN, reporting failure");
          [[fallthrough]];
        }
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        {
          RCLCPP_WARN(rclcpp::get_logger("move_hand_to_pose"), "action failed");
          move_group_goal_handle_.reset();
          return false;
        }

        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        {
          RCLCPP_INFO(rclcpp::get_logger("move_hand_to_pose"), "Action complete");
          move_group_goal_handle_.reset();
          return true;
        }
    }
  }
  std::cout << std::endl;

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
  node_ = std::make_shared<rclcpp::Node>("move_hand_to_pose_node");


  move_group_action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "/spot_moveit/move_action");
  



  if(!move_group_action_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("move_hand_to_pose"), "Move Group action client did not respond, aborting MoveHandToPose");
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("move_hand_to_pose"), "Move Group action client found");
  }
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

  if (move_group_response_future_.valid() || move_group_goal_handle_ != nullptr)
  {
    auto cancel_future = move_group_action_client_->async_cancel_all_goals();
    auto response = rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::seconds(1));
    if (response == rclcpp::FutureReturnCode::TIMEOUT || response == rclcpp::FutureReturnCode::INTERRUPTED){
        RCLCPP_FATAL(rclcpp::get_logger("move_hand_to_pose"), "Unable to cancel MoveGroup action request. Robot may move unexpectedly!!!");
    }
  }  
}

~MoveHandToPose()
{
  // Clean up if needed
}



private:
  // Node and action client
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_action_client_;

  std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr> move_group_response_future_;
  rclcpp::Time request_timestamp_{};

  rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr move_group_goal_handle_;
  
  
  bool init_success_;

}; // MoveHandToPose class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<MoveHandToPose>(new MoveHandToPose());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, MoveHandToPose)