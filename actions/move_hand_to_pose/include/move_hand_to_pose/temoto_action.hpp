#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

#include "move_hand_to_pose/input_parameters.hpp"

#include <boost/config.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/dll/alias.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit/kinematic_constraints/utils.h>

/**
 * @brief Class that integrates TeMoto Base Subsystem specific and Action Engine specific codebases.
 *
 */
class TemotoAction : public ActionBase
{
public:

  TemotoAction()
  {}

  /**
   * @brief Get the Name of the action
   *
   * @return const std::string&
   */
  const std::string& getName()
  {
    return getUmrfNodeConst().getFullName();
  }

  virtual void updateParameters(const ActionParameters& parameters_in)
  {
  }

  input_parameters_t params_in;

private:

  void getInputParameters()
  {
    const auto& params{getUmrfNodeConst().getInputParameters()};

    params_in.robot_name = params.getParameterData<std::string>("robot_name");
    params_in.planning_group = params.getParameterData<std::string>("planning_group");
    params_in.target_link = params.getParameterData<std::string>("target_link");
    params_in.pose.frame_id = params.getParameterData<std::string>("pose::frame_id");
    params_in.pose.position.x = params.getParameterData<double>("pose::position::x");
    params_in.pose.position.y = params.getParameterData<double>("pose::position::y");
    params_in.pose.position.z = params.getParameterData<double>("pose::position::z");
    params_in.pose.orientation.x = params.getParameterData<double>("pose::orientation::x");
    params_in.pose.orientation.y = params.getParameterData<double>("pose::orientation::y");
    params_in.pose.orientation.z = params.getParameterData<double>("pose::orientation::z");
    params_in.pose.orientation.w = params.getParameterData<double>("pose::orientation::w");
    params_in.max_planning_time = params.getParameterData<double>("max_planning_time");
  }

  void setOutputParameters()
  {
  }
};