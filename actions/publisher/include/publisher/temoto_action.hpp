#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

#include "publisher/input_parameters.hpp"

#include <boost/config.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/dll/alias.hpp>

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

    params_in.topic = params.getParameterData<std::string>("topic");
    params_in.robot_name = params.getParameterData<std::string>("robot_name");
    params_in.reference_frame = params.getParameterData<std::string>("reference_frame");
    params_in.naviation_pose_2d.x = params.getParameterData<double>("naviation_pose_2d::x");
    params_in.naviation_pose_2d.y = params.getParameterData<double>("naviation_pose_2d::y");
    params_in.naviation_pose_2d.yaw = params.getParameterData<double>("naviation_pose_2d::yaw");

    std::cout << "header - params_in.topic  "  << params_in.topic << std::endl;
  }

  void setOutputParameters()
  {
  }
};
