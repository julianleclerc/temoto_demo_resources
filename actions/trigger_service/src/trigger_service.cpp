#include <functional>
#include <class_loader/class_loader.hpp>
#include "trigger_service/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>

using namespace std::placeholders;

class TriggerService : public TemotoAction
{
public:
  TriggerService() // REQUIRED
  : node_(nullptr)
  {
  }

  void onInit()
  {
    TEMOTO_PRINT_OF("Initializing", getName());

    // Create a rclcpp::Node object
    node_ = std::make_shared<rclcpp::Node>("trigger_service_node");

    

  }

  bool onRun() // REQUIRED
  {  
    TEMOTO_PRINT_OF("Running", getName());

    trigger_client_ = node_->create_client<std_srvs::srv::Trigger>(params_in.service_name);

    // Wait for the server to be available
    RCLCPP_INFO(rclcpp::get_logger("trigger_service"), 
              "Waiting for server %s...", params_in.service_name.c_str());
              
    while (!trigger_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("trigger_service"), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("trigger_service"), "service not available, waiting again...");
    }


    

    trigger_future_ = trigger_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    if (trigger_future_.has_value()) 
    {      
      if (rclcpp::spin_until_future_complete(node_, trigger_future_.value()) == rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger("trigger_service"), "Service message: %s", trigger_future_->get()->message.c_str());
        return true;  
      } 
      else 
      {
        RCLCPP_ERROR(rclcpp::get_logger("trigger_service"), "Failed to call service");
        return false; 
      }
    }
    else 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No valid trigger future available");
        return false;
    }
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

  ~TriggerService()
  {
  }

private:
  // Node and action client
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;
  std::optional<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId> trigger_future_;

}; // TriggerService class

boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<TriggerService>(new TriggerService());
}

BOOST_DLL_ALIAS(factory, TriggerService)