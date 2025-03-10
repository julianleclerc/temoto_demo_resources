#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

#include "get_coordinates/input_parameters.hpp"
#include "get_coordinates/output_parameters.hpp"


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

  virtual void updateParameters(const ActionParameters& parameters_in) /// How do i update output parameters??
  {
  }
  

  input_parameters_t params_in;
  ouput_parameters_t params_out;

private:

  void getInputParameters()
  {
    const auto& params{getUmrfNodeConst().getInputParameters()};

    params_in.location = params.getParameterData<std::string>("location");
  }

  void setOutputParameters()
  {
    const auto& params{getUmrfNodeConst().setOutputParameters()}; // what is the correct function, .setOutputParameters() correct?

    params_out.pose.position.x = params.getParameterData<double>("pose::position::x");
    params_out.pose.position.y = params.getParameterData<double>("pose::position::y");
    params_out.pose.position.z = params.getParameterData<double>("pose::position::z");
    params_out.pose.orientation.r = params.getParameterData<double>("pose::orientation::r");
    params_out.pose.orientation.p = params.getParameterData<double>("pose::orientation::p");
    params_out.pose.orientation.y = params.getParameterData<double>("pose::orientation::y");
  }
};