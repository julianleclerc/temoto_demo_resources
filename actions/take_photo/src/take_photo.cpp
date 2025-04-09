#include "take_photo/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <memory>

class TakePhoto : public TemotoAction
{
public:
    TakePhoto() // Constructor
    {
        // Any other initialization you want to perform
    }

    void onInit()
    {
        TEMOTO_PRINT_OF("Initializing", getName());        
    }

    bool onRun()
    {
        node_ = std::make_shared<rclcpp::Node>("take_photo");
        std::cout << "topic input: " << params_in.topic << std::endl;
        image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            params_in.topic, 10, std::bind(&TakePhoto::image_callback, this, std::placeholders::_1));

        const double timeout_duration = 5;
        auto start_time = node_->now();

        // Wait until at least one image is received and processed
        while (rclcpp::ok() && actionOk() && !image_received_)
        {
            if (!actionOk())
            {
                RCLCPP_INFO(rclcpp::get_logger("take_photo"), "Action was interrupted");
                return false;
            }

            if ((node_->now() - start_time).seconds() > timeout_duration)
            {
                RCLCPP_WARN(rclcpp::get_logger("take_photo"), "Timeout reached, no image received.");
                return false;
            }
            
            // Spin to process the incoming message
            rclcpp::spin_some(node_);
        }

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

    ~TakePhoto()
    {
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!image_received_)
        {
            std::string file_name = "image_" + std::to_string(node_->now().seconds()) + ".png";
            std::string file_path = params_in.output_dir + "/" + file_name;
            RCLCPP_INFO(rclcpp::get_logger("take_photo"), "file_path: %s ", file_path.c_str());        
            
            try 
            {            
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                cv::imwrite(file_path, cv_ptr->image);
                RCLCPP_INFO(rclcpp::get_logger("take_photo"), "Image saved successfully.");
                image_received_ = true;
            } 
            catch (const cv_bridge::Exception& e) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("take_photo"), "Failed to convert image: %s", e.what());
            }
        }
    }

private:    
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    bool image_received_;
    
};

boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<TakePhoto>(new TakePhoto());
}

BOOST_DLL_ALIAS(factory, TakePhoto)


