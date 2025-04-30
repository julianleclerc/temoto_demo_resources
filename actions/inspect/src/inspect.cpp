#include "inspect/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include "inspect/ai_core.hpp"

using json = nlohmann::json;

class Inspect : public TemotoAction
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * REQUIRED class methods, do not remove them
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

Inspect() : image_received_(false)
{
}

bool onRun()
{
  TEMOTO_PRINT_OF("Running", getName());

  json temoto_log;
  /*
   * Implement your code here
   *
   * Access input parameters via "params_in" member
   * Set output parameters via "params_out" member
   */

  std::string output = fmt::format("Performing inspection for: {}\n", params_in.inspect);
  TEMOTO_PRINT_OF(output, getName());

  // Define I/O
  node_ = std::make_shared<rclcpp::Node>("inspection");
  image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/spot_image_server/rgb/hand_rgb/image", 10, std::bind(&Inspect::image_callback, this, std::placeholders::_1));

  inspection_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "chat_interface_feedback", 10);
  RCLCPP_INFO(node_->get_logger(), "Created publisher on topic: chat_interface_feedback");
    
  display_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/display_feed", 10);
  RCLCPP_INFO(node_->get_logger(), "Created publisher on topic: /display_feed");

  // take picture
  const double timeout_duration = 5;
  auto start_time = node_->now();
  
  // Store the captured image
  cv::Mat captured_image;

  // Wait until at least one image is received and processed
  while (rclcpp::ok() && actionOk() && !image_received_)
  {
      if (!actionOk())
      {
          RCLCPP_INFO(node_->get_logger(), "Action was interrupted");
          return false;
      }

      if ((node_->now() - start_time).seconds() > timeout_duration)
      {
          temoto_log["type"] = "error";
          temoto_log["message"] = "Not able to get images from the camera for inspection, ask user to make sure the camera is correctly publishing to ros2 topic /cam_feed";
          writeLog(temoto_log.dump());
          throw std::runtime_error("Timeout reached, no image received");
      }
      
      // Spin to process the incoming message
      rclcpp::spin_some(node_);
  }

  // Assuming the callback stored the image in captured_image
  if (image_received_ && !captured_image_.empty()) {
    publishImage(captured_image_);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "No valid image received for processing");
    return false;
  }

  std::string instructions = R"(
    You are an advanced computer vision system designed for robot inspection tasks.
    
    INSPECTION CONTEXT:
    You will be given:
    1. An image from the robot's camera
    2. An inspection objective that may be either:
       - A general inspection of an object/area
       - A check for the presence of something specific
    
    IMPORTANT: Your response MUST be a valid JSON string with EXACTLY this format:
    {
      "inspection_message": "Your detailed inspection findings here. Be specific about what you observe and any potential issues.",
      "requires_attention": false,
      "confidence_level": 0.95,
      "inspection_type": "general|presence"
    }
    
    FIELD DESCRIPTIONS:
    - "inspection_message": Detailed analysis of what you see related to the inspection request.
       Make this conversational and informative as it will be relayed directly to the user. Highlight any elements that may be of concern if needed.
    
    - "requires_attention": Boolean (true/false, no quotes):
      for general / security inspections:
       - Set to true if you detect any issues that require human intervention (like hazardous elements)
       - Set to false if everything appears normal and no intervention is needed
      for presence inpsections:
       - Set to true if you identify the presnece of requested object(s)
       - Set to false if you don't indentify the presence of requested object(s)
    
    - "confidence_level": Number between 0 and 1 indicating your confidence in the assessment
        
    - "inspection_type": String indicating which type of inspection was performed:
       - "general" - Overall assessment of an object or area, looking for suspicious or concerning elements
       - "presence" - Checking if something specific is there
    
    ADDITIONAL GUIDELINES:
    - Focus only on the specific inspection request - don't report on unrelated elements
    - If you cannot determine something with confidence, state this explicitly in your inspection_message
    - For presence checks, clearly state whether the object was found or not
    - For security checks, explain what makes something suspicious or not
    - Use natural language in your inspection_message as it will be communicated directly to users
    
    Do NOT include any text outside the JSON structure. Your entire response must be parseable as valid JSON.
  )";

  
  // Create messages for the AI
  std::vector<ai_core::Message> messages;
  
  // System message to define the AI's role
  messages.push_back({
    "system", 
    instructions
  });
        
  // User message to define the inspection target
  std::string user_message = "User input inpsection for: " + params_in.inspect + "\n";
  messages.push_back({
    "user", 
    user_message
  });

  std::string ai_response = ai_core::AIImagePrompt(
    messages,
    captured_image_,  // Use the captured image
    0.3f,    // temperature
    1024,    // max_tokens
    0.0f,    // frequency_penalty
    0.0f     // presence_penalty
  );

  // Check if there was an error in the JSON response
  json response_json;
  try {
    RCLCPP_INFO(node_->get_logger(), "AI response received (size: %zu bytes). Parsing JSON...", ai_response.size());
    RCLCPP_INFO(node_->get_logger(), "Response preview: %s", 
                ai_response.length() > 100 ? (ai_response.substr(0, 97) + "...").c_str() : ai_response.c_str());
    
    // Clean the response to extract only valid JSON
    std::string cleaned_response = cleanLLMJsonResponse(ai_response);
    if (cleaned_response.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to extract valid JSON from LLM response");
      throw std::runtime_error("Failed to extract valid JSON from LLM response");
    }
    
    response_json = json::parse(cleaned_response);
    RCLCPP_INFO(node_->get_logger(), "JSON parsed successfully");
  } catch (const json::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "JSON parsing error: %s", e.what());
    RCLCPP_ERROR(node_->get_logger(), "Raw response: %s", ai_response.c_str());
    
    temoto_log["type"] = "error";
    temoto_log["message"] = "Internal error with llm response inside the inspection action, try again";
    writeLog(temoto_log.dump());
    throw std::runtime_error("Internal error with llm response inside the inspection action");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error processing response: %s", e.what());
    
    temoto_log["type"] = "error";
    temoto_log["message"] = "Internal error with llm response inside the inspection action, try again";
    writeLog(temoto_log.dump());
    throw std::runtime_error("Internal error with llm response inside the inspection action");
  }

  // Check for error indicator -> raise error if yes 
  if (response_json["requires_attention"].get<bool>()) {
    //publishInspectionResult(response_json["inspection_message"]);
    
    std::string error_message = "A concern has been raised in the inspection that requires user intervention (either an element of concern as been raised or an object has been identified): " + 
                                response_json["inspection_message"].get<std::string>() + "\n";
    
    temoto_log["type"] = "error";
    temoto_log["message"] = error_message;
    writeLog(temoto_log.dump());
    throw std::runtime_error("Potential issue raised in the inspection");
  }

  // Publish inspection result
  publishInspectionResult(response_json["inspection_message"]);
  params_out.inspection_result = response_json["inspection_message"];
  std::cout << "Inspection completed successfully" << std::endl;

  return true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * OPTIONAL class methods, can be removed if not needed
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
  // Check API key
  const char* api_key = std::getenv("OPENAI_API_KEY");
  if (api_key == nullptr || strlen(api_key) == 0) {
    TEMOTO_PRINT_OF("WARNING: OPENAI_API_KEY environment variable not set or empty!", getName());
    throw std::runtime_error("API KEY not proprely set, unable to start inspection.");

  } else {
    TEMOTO_PRINT_OF("API key found (length: " + std::to_string(strlen(api_key)) + " characters)", getName());
  }
}

void onPause()
{
  TEMOTO_PRINT_OF("Pausing", getName());
}

void onResume()
{
  TEMOTO_PRINT_OF("Continuing", getName());
}

void onStop()
{
  TEMOTO_PRINT_OF("Stopping", getName());
}

~Inspect()
{
}

void publishInspectionResult(const std::string& inspection) {
  RCLCPP_INFO(node_->get_logger(), "=== PUBLISHING INSPECTION RESULT ===");
  
  try {
    json j;
    j["targets"] = {"David"};
    j["type"] = "response";
    j["message"] = inspection;
    
    std::string json_str = j.dump();
    RCLCPP_INFO(node_->get_logger(), "Created JSON message for chat_interface_feedback: %s", 
                json_str.length() > 100 ? (json_str.substr(0, 97) + "...").c_str() : json_str.c_str());
    
    std_msgs::msg::String msg;
    msg.data = json_str;
    
    RCLCPP_INFO(node_->get_logger(), "Publishing message to chat_interface_feedback (size: %zu bytes)", 
                msg.data.size());
    inspection_publisher_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Message published successfully to chat_interface_feedback");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "ERROR publishing inspection result: %s", e.what());
  }
  
  RCLCPP_INFO(node_->get_logger(), "=== INSPECTION RESULT PUBLISHED ===");
}

void publishImage(const cv::Mat& image) {
  RCLCPP_INFO(node_->get_logger(), "=== PUBLISHING IMAGE TO DISPLAY FEED ===");
  
  try {
    // Encode the image to base64
    RCLCPP_INFO(node_->get_logger(), "Encoding image to base64 (image size: %dx%d)...", 
                image.cols, image.rows);
    std::string encoded_image = encodeImageToBase64(image);
    RCLCPP_INFO(node_->get_logger(), "Image encoded to base64 (encoded size: %zu bytes)", 
                encoded_image.size());
    
    // Create JSON message
    RCLCPP_INFO(node_->get_logger(), "Creating JSON message for display_feed...");
    std::string json_msg = "{\"target\":\"David\",\"name\":\"inspection\",\"image\":\"" 
                          + encoded_image + "\"}";
    
    RCLCPP_INFO(node_->get_logger(), "JSON message created (total size: %zu bytes)", json_msg.size());
    
    // Publish the message
    std_msgs::msg::String msg;
    msg.data = json_msg;
    
    RCLCPP_INFO(node_->get_logger(), "Publishing message to /display_feed...");
    display_publisher_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Message published successfully to /display_feed");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "ERROR publishing image: %s", e.what());
  }
  
  RCLCPP_INFO(node_->get_logger(), "=== IMAGE PUBLISHING COMPLETED ===");
}

std::string encodeImageToBase64(const cv::Mat& image) {
  RCLCPP_INFO(node_->get_logger(), "=== ENCODING IMAGE TO BASE64 ===");
  
  try {
    // Use the ai_core implementation for encoding
    RCLCPP_INFO(node_->get_logger(), "Calling ai_core encoding function...");
    std::string result = ai_core::encodeImageToBase64(image);
    RCLCPP_INFO(node_->get_logger(), "Image encoded successfully to base64 (size: %zu bytes)", result.size());
    
    // Print the first few characters
    if (result.size() > 20) {
      RCLCPP_INFO(node_->get_logger(), "Encoded data begins with: %s...", result.substr(0, 20).c_str());
    }
    
    RCLCPP_INFO(node_->get_logger(), "=== BASE64 ENCODING COMPLETED ===");
    return result;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error encoding image to base64: %s", e.what());
    RCLCPP_INFO(node_->get_logger(), "=== BASE64 ENCODING FAILED ===");
    return "";
  }
}

void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!image_received_)
    {
        try 
        {            
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            captured_image_ = cv_ptr->image;
            image_received_ = true;
            RCLCPP_INFO(node_->get_logger(), "Image received and processed successfully");
        } 
        catch (const cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to convert image: %s", e.what());
        }
    }
}

// Helper function to clean JSON responses from LLMs
std::string cleanLLMJsonResponse(const std::string& raw_response) {
  RCLCPP_INFO(node_->get_logger(), "Cleaning raw LLM response to extract JSON...");
  
  // Find the first opening curly brace
  size_t start_pos = raw_response.find('{');
  if (start_pos == std::string::npos) {
    RCLCPP_ERROR(node_->get_logger(), "No JSON object found in response (no opening brace)");
    return "";
  }
  
  // Find the last closing curly brace
  size_t end_pos = raw_response.rfind('}');
  if (end_pos == std::string::npos) {
    RCLCPP_ERROR(node_->get_logger(), "No JSON object found in response (no closing brace)");
    return "";
  }
  
  // Extract just the JSON object
  if (end_pos <= start_pos) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid JSON structure (closing brace before opening brace)");
    return "";
  }
  
  std::string cleaned_json = raw_response.substr(start_pos, end_pos - start_pos + 1);
  RCLCPP_INFO(node_->get_logger(), "Extracted JSON: %s", 
              cleaned_json.length() > 100 ? (cleaned_json.substr(0, 97) + "...").c_str() : cleaned_json.c_str());
  
  return cleaned_json;
}

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inspection_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_publisher_;
    bool image_received_;
    cv::Mat captured_image_;  // Store the captured image
}; // Inspect class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<Inspect>(new Inspect());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, Inspect)