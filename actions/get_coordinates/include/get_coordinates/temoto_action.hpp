#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

#include "get_coordinates/input_parameters.hpp"
#include "get_coordinates/output_parameters.hpp"

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
  output_parameters_t params_out;

private:

  void getInputParameters()
  {
    const auto& params{getUmrfNodeConst().getInputParameters()};

    params_in.location = params.getParameterData<std::string>("location");
  }

  void setOutputParameters()
  {
    auto& params{getUmrfNode().getOutputParametersNc()};

    params.setParameter("position::x", "number", boost::any(params_out.position.x));
    params.setParameter("position::y", "number", boost::any(params_out.position.y));
    params.setParameter("position::z", "number", boost::any(params_out.position.z));
    params.setParameter("orientation::r", "number", boost::any(params_out.orientation.r));
    params.setParameter("orientation::p", "number", boost::any(params_out.orientation.p));
    params.setParameter("orientation::y", "number", boost::any(params_out.orientation.y));
  }
};