#include "publisher/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Publisher : public TemotoAction
{
public:
    Publisher() // Constructor
    {
        // Any other initialization you want to perform
    }

    void onInit()
    {
        TEMOTO_PRINT_OF("Initializing", getName());        
    }

    bool onRun()
    {
        node_ = std::make_shared<rclcpp::Node>("navigate_to_pose_node");
        std::cout << "topic input" << params_in.topic << std::endl;
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(params_in.robot_name +"/" + params_in.topic, 10);

        if (!publisher_) {
            RCLCPP_ERROR(rclcpp::get_logger("publisher"), "Publisher is not initialized.");
            return false;
        }

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = params_in.naviation_pose_2d.x;
        message.linear.y = params_in.naviation_pose_2d.y;
        message.angular.z = params_in.naviation_pose_2d.yaw;

        std::cout << "topic input" << params_in.topic << std::endl;

        RCLCPP_INFO(rclcpp::get_logger("publisher"), "Publishing: '%s' '%f' '%f' '%f'",  params_in.topic, 
                                                                                    message.linear.x, 
                                                                                    message.linear.y,
                                                                                    message.angular.z);

        uint sleep_ms{100};
        for (uint i{0}; i<50 && actionOk() ; i++)
        {
            publisher_->publish(message);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            std::cout << ". " << std::flush;
        }

        message.linear.x = 0;
        message.linear.y = 0;
        message.angular.z = 0;
        publisher_->publish(message);

        

        TEMOTO_PRINT_OF("Publishing", getName());

        TEMOTO_PRINT_OF("Done\n", getName());

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

    ~Publisher()
    {
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::shared_ptr<rclcpp::Node> node_;
    int count_ = 0;
};

boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<Publisher>(new Publisher());
}

BOOST_DLL_ALIAS(factory, Publisher)
