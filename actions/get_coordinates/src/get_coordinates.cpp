#include "get_coordinates/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <opencv2/opencv.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include "get_coordinates/ai_core.hpp"
#include "get_coordinates/map_builder.hpp"
#include "get_coordinates/llm_solver.hpp"



namespace fs = std::filesystem;
using json = nlohmann::json;

class GetCoordinates : public TemotoAction
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * REQUIRED class methods, do not remove them
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

GetCoordinates() : image_received_(false), resolution_(0.05)
{
    // Initialize origin with default values
    origin_[0] = 0.0;
    origin_[1] = 0.0;
    origin_[2] = 0.0;
}

bool onRun()
{
  TEMOTO_PRINT_OF("Running", getName());
  std::string output = fmt::format("Getting coordinates for: {}\n", params_in.target);
  TEMOTO_PRINT_OF(output, getName());
  
  // Log errors
  json temoto_log;

  // Define I/O
  node_ = std::make_shared<rclcpp::Node>("inspection");
  chat_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "chat_interface_feedback", 10);
  RCLCPP_INFO(node_->get_logger(), "Created publisher on topic: chat_interface_feedback");  

  /*
   * STEP ONE: Setup parameters
   */

  // Find workspace root by getting package directory and navigating up to workspace root
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("get_coordinates");
  RCLCPP_INFO(node_->get_logger(), "Package share directory: %s", package_share_dir.c_str());
  
  // Navigate to workspace root (from install/share/get_coordinates)
  fs::path workspace_path = fs::path(package_share_dir);
  // Go up 4 levels: package_name -> share -> install -> workspace_root
  for (int i = 0; i < 4; i++) {
    workspace_path = workspace_path.parent_path();
  }
  
  // Ensure the path is canonical (resolves symlinks and normalizes the path)
  try {
    workspace_path = fs::canonical(workspace_path);
    RCLCPP_INFO(node_->get_logger(), "Workspace root path: %s", workspace_path.string().c_str());
  } catch (const fs::filesystem_error& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error resolving workspace path: %s", e.what());
    throw std::runtime_error("Failed to resolve workspace path");
  }
  
  // Set data paths
  const std::string DATA_DIR = (workspace_path / "data").string();
  const std::string MAP_PATH = (fs::path(DATA_DIR) / "map.pgm").string();
  const std::string MAP_YAML_PATH = (fs::path(DATA_DIR) / "map.yaml").string();
  const std::string ITEMS_JSON_PATH = (fs::path(DATA_DIR) / "items.json").string();
  
  RCLCPP_INFO(node_->get_logger(), "Data directory: %s", DATA_DIR.c_str());
  RCLCPP_INFO(node_->get_logger(), "Map path: %s", MAP_PATH.c_str());
  RCLCPP_INFO(node_->get_logger(), "Map YAML path: %s", MAP_YAML_PATH.c_str());
  RCLCPP_INFO(node_->get_logger(), "Items JSON path: %s", ITEMS_JSON_PATH.c_str());
  
  // Check if the files exist
  if (!fs::exists(MAP_PATH)) {
    RCLCPP_ERROR(node_->get_logger(), "Map file not found at: %s", MAP_PATH.c_str());
    throw std::runtime_error("Map file not found");
  }
  
  if (!fs::exists(MAP_YAML_PATH)) {
    RCLCPP_ERROR(node_->get_logger(), "Map YAML file not found at: %s", MAP_YAML_PATH.c_str());
    throw std::runtime_error("Map YAML file not found");
  }
  
  if (!fs::exists(ITEMS_JSON_PATH)) {
    RCLCPP_ERROR(node_->get_logger(), "Items JSON file not found at: %s", ITEMS_JSON_PATH.c_str());
    throw std::runtime_error("Items JSON file not found");
  }

  // Initialize map configuration parameters 
  double inflation_radius_m = 0.5;  // inflation radius in meters
  double scale_factor = 2.0;
  double grid_scale = 20.0;  // Setting grid scale to 20 pixels as in Python code

  // if fail to get transform, hardcode robot position to (0, 0)
  json robot_position = {
    {"x", 0.0},
    {"y", 0.0}
  };

  // Create robot position using the RobotTransform struct from map_builder.hpp
  RobotTransform robot_pos{robot_position["x"], robot_position["y"]};

  // Data storage
  json items_data;

  // Output directory for saving images
  std::string output_dir = (fs::path(DATA_DIR) / "debug_GetCoordinates").string();
  
  // Try to create the debug directory
  try {
    if (!fs::exists(output_dir)) {
      fs::create_directories(output_dir);
      RCLCPP_INFO(node_->get_logger(), "Created debug directory: %s", output_dir.c_str());
    }
  } catch (const fs::filesystem_error& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error creating debug directory: %s", e.what());
  }

  // AI stuff
  std::string COORDINATES_METHOD = "oneCoordSearch";
    
  // Load map configuration (will throw on failure)
  loadMapConfig(MAP_YAML_PATH);
  RCLCPP_INFO(node_->get_logger(), "Map configuration loaded: resolution=%f, origin=[%f,%f,%f]", 
              resolution_, origin_[0], origin_[1], origin_[2]);
  
  // Create JSON with map parameters
  json params = {
    {"inflation_radius_m", inflation_radius_m},
    {"scale_factor", scale_factor},
    {"grid_scale", grid_scale},
    {"resolution", resolution_},
    {"origin", {origin_[0], origin_[1], origin_[2]}}
  };

  // Load items data from JSON
  RCLCPP_INFO(node_->get_logger(), "Loading JSON file: %s", ITEMS_JSON_PATH.c_str());
  try {
      items_data = loadJsonFile(ITEMS_JSON_PATH);
      RCLCPP_INFO(node_->get_logger(), "Successfully loaded items_data");
      
      // Inspect the top-level structure
      std::string keys_str = "items_data keys: ";
      for (auto& [key, val] : items_data.items()) {
          keys_str += key + " ";
      }
      RCLCPP_INFO(node_->get_logger(), "%s", keys_str.c_str());
  } catch (const json::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "JSON error loading items_data: %s", e.what());
      throw;
  }

  // Fetch robot pos from transform listner - TO BE IMPLEMENTED

  /*
   * STEP Two: Build the Map
   */
  
  // Load map image
  cv::Mat map;
  try {
    RCLCPP_INFO(node_->get_logger(), "Attempting to load map from: %s", MAP_PATH.c_str());
    map = cv::imread(MAP_PATH, cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load map image: %s", MAP_PATH.c_str());
      throw std::runtime_error("Failed to load map image");
    }
    RCLCPP_INFO(node_->get_logger(), "Map loaded successfully, size: %dx%d", map.cols, map.rows);
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "OpenCV error loading map: %s", e.what());
    throw;
  }
  
  // Try to build map
  std::string map_output_path = (fs::path(DATA_DIR) / "final_robot_map.png").string();
  cv::Mat object_map = MapBuilder::BuildMap(map, params, items_data, robot_pos, map_output_path);
  RCLCPP_INFO(node_->get_logger(), "Map building completed successfully");

  /*
   * STEP THREE: PROMPT LLM
  */

  // Get coordinates
  json llm_solver_response = LLMSolver::getCoordinateOneShot(object_map, params_in.target);
  
  // Check for success
  std::string success = llm_solver_response["success"];
  if (success == "false") {
    std::string message = llm_solver_response["message"];

    RCLCPP_INFO(node_->get_logger(), "Failure to get coordinates: %s", message.c_str());
    nlohmann::json errorObj;
    errorObj["type"] = "error";
    errorObj["message"] = "Get Coordinates was not successful: " + message;
    writeLog(errorObj.dump());
    
    throw std::runtime_error("Get Coordinates was not successful: " + message);
  }  

  // Extract pixel coordinates from the LLM response
  int pixel_x = llm_solver_response["coordinates"]["x"];
  int pixel_y = llm_solver_response["coordinates"]["y"];
  
  // Display Coordinates on map as a simple red dot
  std::string visualization_output_path = (fs::path(DATA_DIR) / "target_visualization.png").string();
  cv::Mat visualization = MapBuilder::displayTargetCoordinate(
      object_map, 
      llm_solver_response, 
      params, 
      visualization_output_path);

  /*
  * STEP FOUR: GET COORDINATES
  */

  // Get realworld coordinates  for nav2 (fix to scale, resolution and origin) <- Solve this

  double world_x = pixel_x * resolution_ + origin_[0];
  double world_y = (map.rows - pixel_y) * resolution_ + origin_[1];
  
  RCLCPP_INFO(node_->get_logger(), "Real-world coordinates for Nav2: x=%f, y=%f", world_x, world_y);
  
  // Add coordinates to the output parameters
  json coordinates_json = {
    {"x", world_x},
    {"y", world_y},
    {"target_id", llm_solver_response["target_id"]}
  };
  
  // Create a response message for the user
  std::string target_id = llm_solver_response["target_id"];
  std::string reasoning = llm_solver_response["message"];
  
  // Create the response JSON for the final step
  json response_json;
  response_json["success"] = llm_solver_response["success"];
  response_json["real_world_coordinates"] = {{"x", world_x}, {"y", world_y}};
  response_json["pixel_coordinates"] = {{"x", pixel_x}, {"y", pixel_y}};
  response_json["target_id"] = target_id;
  response_json["message"] = reasoning;
  
  // FIX: Convert JSON to string for logging
  RCLCPP_INFO(node_->get_logger(), "Response JSON: %s", response_json.dump().c_str());

  /*
   * FINAL STEP: RETURN RESULTS
   */

  // Publish inspection result
  publishResult(response_json["message"]);
  
  // FIX: Store all the information in the inspection_result field as a JSON string
  params_out.inspection_result = response_json.dump();
  RCLCPP_INFO(node_->get_logger(), "Inspection completed successfully");

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
    throw std::runtime_error("API KEY not properly set, unable to start inspection.");
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

~GetCoordinates()
{
}

// Helper function to load map configuration from YAML file
bool loadMapConfig(const std::string& yaml_path) {
  RCLCPP_INFO(node_->get_logger(), "Loading map config from: %s", yaml_path.c_str());
  std::ifstream file(yaml_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open YAML file: %s", yaml_path.c_str());
    throw std::runtime_error("Failed to open map YAML file");
  }
  
  // Simple YAML parser for known format
  std::string line;
  bool resolution_found = false;
  bool origin_found = false;
  
  while (std::getline(file, line)) {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') continue;
    
    if (line.find("resolution:") != std::string::npos) {
      resolution_ = std::stod(line.substr(line.find(":") + 1));
      resolution_found = true;
      RCLCPP_INFO(node_->get_logger(), "Found resolution: %f", resolution_);
    }
    else if (line.find("origin:") != std::string::npos) {
      std::string origin_str = line.substr(line.find("[") + 1, line.find("]") - line.find("[") - 1);
      std::istringstream iss(origin_str);
      std::string token;
      int i = 0;
      
      while (std::getline(iss, token, ',') && i < 3) {
        origin_[i++] = std::stod(token);
      }
      
      origin_found = true;
      RCLCPP_INFO(node_->get_logger(), "Found origin: [%f, %f, %f]", origin_[0], origin_[1], origin_[2]);
    }
  }
  
  if (!resolution_found || !origin_found) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required parameters in YAML file");
    throw std::runtime_error("Missing required parameters in map YAML file");
  }
  
  return true;
}

// Helper function to load JSON file
json loadJsonFile(const std::string& file_path) {
  RCLCPP_INFO(node_->get_logger(), "Loading JSON from file: %s", file_path.c_str());
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open JSON file: %s", file_path.c_str());
    throw std::runtime_error("Failed to open JSON file: " + file_path);
  }
  
  json data;
  try {
    file >> data;
    RCLCPP_INFO(node_->get_logger(), "JSON file loaded successfully");
    return data;
  } catch (const json::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error parsing JSON file: %s", e.what());
    throw;
  }
}

void publishResult(const std::string& message) {
  RCLCPP_INFO(node_->get_logger(), "=== PUBLISHING GETCOORDINATES RESULT ===");
  
  try {
    json j;
    j["targets"] = {"David"};
    j["type"] = "response";
    j["message"] = message;
    
    std::string json_str = j.dump();
    RCLCPP_INFO(node_->get_logger(), "Created JSON message for chat_interface_feedback: %s", 
                json_str.length() > 100 ? (json_str.substr(0, 97) + "...").c_str() : json_str.c_str());
    
    std_msgs::msg::String msg;
    msg.data = json_str;
    
    RCLCPP_INFO(node_->get_logger(), "Publishing message to chat_interface_feedback (size: %zu bytes)", 
                msg.data.size());
    chat_publisher_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Message published successfully to chat_interface_feedback");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "ERROR publishing inspection result: %s", e.what());
  }
  
  RCLCPP_INFO(node_->get_logger(), "=== INSPECTION RESULT PUBLISHED ===");
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chat_publisher_;
    bool image_received_;
    double resolution_;  // Added to store resolution from YAML
    double origin_[3];   // Added to store origin from YAML
}; // GetCoordinates class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<GetCoordinates>(new GetCoordinates());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, GetCoordinates)