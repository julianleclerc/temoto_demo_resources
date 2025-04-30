#include "get_coordinates/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

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

GetCoordinates()
{
}

bool image_received_ = false;
double resolution_ = 0.05;
double origin_[3] = {0.0,0.0,0.0};

std::shared_ptr<rclcpp::Node> node_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chat_publisher_;

std::string getNodeName() const {
  return node_ ? node_->get_name() : "get_coordinates";
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
  
  auto now = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  std::string unique_node_name = "get_coordinates_node_" + std::to_string(timestamp);
  node_ = std::make_shared<rclcpp::Node>(unique_node_name);
  
  const std::string topic = "/chat_interface_feedback";
  chat_publisher_ = node_->create_publisher<std_msgs::msg::String>(topic, 10); 

  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Created publisher on topic: chat_interface_feedback");  
}

bool onRun()
{
  TEMOTO_PRINT_OF("Running", getName());
  std::string output = fmt::format("Getting coordinates for: {}\n", params_in.target);
  TEMOTO_PRINT_OF(output, getName());
  
  // Log errors
  json temoto_log;

  /*
   * STEP ONE: Setup parameters
   */

  // Find workspace root by getting package directory and navigating up to workspace root
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("get_coordinates");
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Package share directory: %s", package_share_dir.c_str());
  
  // Navigate to workspace root (from install/share/get_coordinates)
  fs::path workspace_path = fs::path(package_share_dir);
  // Go up 4 levels: package_name -> share -> install -> workspace_root
  for (int i = 0; i < 4; i++) {
    workspace_path = workspace_path.parent_path();
  }
  
  // Ensure the path is canonical (resolves symlinks and normalizes the path)
  try {
    workspace_path = fs::canonical(workspace_path);
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Workspace root path: %s", workspace_path.string().c_str());
  } catch (const fs::filesystem_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Error resolving workspace path: %s", e.what());
    throw std::runtime_error("Failed to resolve workspace path");
  }
  
  // Set data paths
  const std::string DATA_DIR = (workspace_path / "src" /"data").string();
  const std::string DEBUG_DIR = (workspace_path / "debug").string();
  const std::string DEBUG_SUBDIR = (fs::path(DEBUG_DIR) / "GetCoordinates").string();
  const std::string MAP_PATH = (fs::path(DATA_DIR) / "map.pgm").string();
  const std::string MAP_YAML_PATH = (fs::path(DATA_DIR) / "map.yaml").string();
  const std::string ITEMS_JSON_PATH = (fs::path(DATA_DIR) / "items.json").string();

  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Data directory: %s", DATA_DIR.c_str());
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Debug directory: %s", DEBUG_DIR.c_str());
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "GetCoordinates debug subdirectory: %s", DEBUG_SUBDIR.c_str());
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Map path: %s", MAP_PATH.c_str());
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Map YAML path: %s", MAP_YAML_PATH.c_str());
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Items JSON path: %s", ITEMS_JSON_PATH.c_str());

  // Create debug directories
  try {
    if (!fs::exists(DEBUG_DIR)) {
      fs::create_directories(DEBUG_DIR);
      RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Created debug directory: %s", DEBUG_DIR.c_str());
    }
    
    if (!fs::exists(DEBUG_SUBDIR)) {
      fs::create_directories(DEBUG_SUBDIR);
      RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Created GetCoordinates debug subdirectory: %s", DEBUG_SUBDIR.c_str());
    }
  } catch (const fs::filesystem_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Error creating debug directories: %s", e.what());
  }

  // Check if the files exist
  if (!fs::exists(MAP_PATH)) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Map file not found at: %s", MAP_PATH.c_str());
    throw std::runtime_error("Map file not found");
  }
  
  if (!fs::exists(MAP_YAML_PATH)) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Map YAML file not found at: %s", MAP_YAML_PATH.c_str());
    throw std::runtime_error("Map YAML file not found");
  }
  
  if (!fs::exists(ITEMS_JSON_PATH)) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Items JSON file not found at: %s", ITEMS_JSON_PATH.c_str());
    throw std::runtime_error("Items JSON file not found");
  }

  // Initialize map configuration parameters 
  double inflation_radius_m = 0.2;  // inflation radius in meters
  double scale_factor = 2.0;
  double grid_scale = 20.0;  // Setting grid scale to 20 pixels as in Python code
  double max_polar_distance = 0; // Max distance for the polar coordinate distance in meters

  // if fail to get transform, hardcode robot position to (0, 0)
  json robot_position = {
    {"x", 0.0},
    {"y", 0.0}
  };

  // Create robot position using the RobotTransform struct from map_builder.hpp
  RobotTransform robot_pos{robot_position["x"], robot_position["y"]};

  // Data storage
  json items_data;
    
  // Load map configuration (will throw on failure)
  loadMapConfig(MAP_YAML_PATH);
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Map configuration loaded: resolution=%f, origin=[%f,%f,%f]", 
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
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Loading JSON file: %s", ITEMS_JSON_PATH.c_str());
  try {
      items_data = loadJsonFile(ITEMS_JSON_PATH);
      RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Successfully loaded items_data");
      
      // Inspect the top-level structure - with new format, we should see item IDs directly
      std::string keys_str = "items_data keys: ";
      for (auto& [key, val] : items_data.items()) {
          keys_str += key + " ";
      }
      RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "%s", keys_str.c_str());
  } catch (const json::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "JSON error loading items_data: %s", e.what());
      throw;
  }

  // Fetch robot pos from transform listner - TO BE IMPLEMENTED

  /*
   * STEP Two: Build the Map
   */
  
  // Load map image
  cv::Mat map;
  try {
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Attempting to load map from: %s", MAP_PATH.c_str());
    map = cv::imread(MAP_PATH, cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Failed to load map image: %s", MAP_PATH.c_str());
      throw std::runtime_error("Failed to load map image");
    }
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Map loaded successfully, size: %dx%d", map.cols, map.rows);
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "OpenCV error loading map: %s", e.what());
    throw;
  }
  
  // Try to build map
  std::string map_output_path = (fs::path(DEBUG_SUBDIR) / "final_robot_map.png").string();
  cv::Mat object_map = MapBuilder::BuildMap(map, params, items_data, robot_pos, DEBUG_SUBDIR);
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Map building completed successfully");

/*
   * STEP THREE: PROMPT LLM
  */
  std::string COORDINATES_METHOD = "oneCoordSearch";
  int pixel_x = 0;
  int pixel_y = 0;
  double world_x = 0.0;
  double world_y = 0.0;
  double angle = 0;
  std::string target_id;
  json llm_solver_response;
  // Calculate scaled_resolution
  double scaled_resolution = resolution_ / scale_factor;

  
  if (COORDINATES_METHOD == "oneCoordSearch") {
    // Get coordinates
    llm_solver_response = LLMSolver::getCoordinateOneShot(object_map, items_data, params_in.target);
    
    // Check for success
    std::string success = llm_solver_response["success"];
    if (success == "false") {
        std::string message = llm_solver_response["message"];
        
        RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Failure to get coordinates: %s", message.c_str());
        nlohmann::json errorObj;
        errorObj["type"] = "error";
        errorObj["message"] = "Get Coordinates was not successful: " + message;
        writeLog(errorObj.dump());
        
        throw std::runtime_error("Get Coordinates was not successful: " + message);
    }  

    // Get target object ID and find its position
    target_id = llm_solver_response["target_id"];

    // Check if target exists
    if (items_data.contains(target_id) == false) {
        RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Target object with ID %s not found in items_data", target_id.c_str());
        throw std::runtime_error("Target object not found");
    }

    // Find Coordinate through A*
    double radius_padding = 0.3;  // in meters for extra distance from object
    cv::Point goal_point = MapBuilder::coordinates_astar(map, params, items_data, robot_pos, DEBUG_SUBDIR, target_id, radius_padding);

    pixel_x = static_cast<int>(goal_point.x * scale_factor);
    pixel_y = static_cast<int>(goal_point.y * scale_factor);
    
    json visualization_json = {
        {"target_id", target_id},
        {"coordinates", {{"x", pixel_x}, {"y", pixel_y}}},
        {"success", "true"}
    };
    
    // Display Coordinates on map as a simple red dot
    std::string visualization_output_path = (fs::path(DEBUG_SUBDIR) / "target_visualization.png").string();
    cv::Mat visualization = MapBuilder::displayTargetCoordinate(
        object_map, 
        visualization_json, 
        params, 
        visualization_output_path);

    // Debug final coordinates
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Final original map pixel: (%d, %d)", pixel_x, pixel_y);

    
  };
  
  if(COORDINATES_METHOD == "polarSearch") {
    // Get polar coordinates
    llm_solver_response = LLMSolver::getCoordinatePolar(object_map, items_data, params_in.target);
    // Check for success
    std::string success = llm_solver_response["success"];
    if (success == "false") {
      std::string message = llm_solver_response["message"];

      RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Failure to get coordinates: %s", message.c_str());
      nlohmann::json errorObj;
      errorObj["type"] = "error";
      errorObj["message"] = "Get Coordinates was not successful: " + message;
      writeLog(errorObj.dump());
      
      throw std::runtime_error("Get Coordinates was not successful: " + message);
    }

    // Get polar coordinates from LLM response
    double angle_degrees = llm_solver_response["robot_to_target_angle"].get<double>() - 180;
    double distance_percentage = llm_solver_response["distance"];

    // Log raw angle value for debugging
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Raw angle from LLM: %f degrees", angle_degrees);

    // Normalize angle to 0-360 range
    while (angle_degrees < 0) angle_degrees += 360;
    while (angle_degrees >= 360) angle_degrees -= 360;

    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Normalized angle: %f degrees", angle_degrees);

    // Get target object ID and find its position
    target_id = llm_solver_response["target_id"];
    bool target_found = false;
    double target_world_x = 0.0;
    double target_world_y = 0.0;

    // First, get the target's world coordinates
    if (items_data.contains(target_id)) {
        target_world_x = items_data[target_id]["coordinates"]["x"];
        target_world_y = items_data[target_id]["coordinates"]["y"];
        target_found = true;
        
        RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Found target %s at world coordinates: (%f, %f) meters", 
                    target_id.c_str(), target_world_x, target_world_y);
    }

    if (!target_found) {
        RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Target object with ID %s not found in items_data", target_id.c_str());
        throw std::runtime_error("Target object not found");
    }

    // Convert target world coordinates to pixel coordinates for visualization
    cv::Point target_pixel = MapBuilder::worldToMapCoordinates(target_world_x, target_world_y, params, object_map.rows);
    int target_center_x = target_pixel.x;
    int target_center_y = target_pixel.y;

    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Target pixel coordinates: (%d, %d)", target_center_x, target_center_y);

    // Convert angle to radians - Using standard convention: 0° is East, angles increase counterclockwise
    double angle_radians = angle_degrees * M_PI / 180.0;

    // Calculate distance in world units (meters)
    double max_distance_meters = max_polar_distance; // Usually 1 meter
    double distance_meters = (distance_percentage / 100.0) * max_distance_meters;

    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Distance: %f%% of max (%f m) = %f meters", 
                distance_percentage, max_distance_meters, distance_meters);

                
    /* Get first available coordinate that's white */
    int available_x_coordinate = target_center_x;
    int available_y_coordinate = target_center_y;
    int distance_pixels = 0;
    bool found_white_pixel = false;

    // Start from the target and move outward along the angle until finding a white pixel
    while (!found_white_pixel) {
        // Calculate new coordinates by moving along the angle
        available_x_coordinate = std::round(distance_pixels * std::cos(angle_radians) + target_center_x);
        available_y_coordinate = std::round(distance_pixels * std::sin(angle_radians) + target_center_y);
        
        // Check if coordinates are within the map bounds
        if (available_x_coordinate < 0 || available_x_coordinate >= object_map.cols ||
            available_y_coordinate < 0 || available_y_coordinate >= object_map.rows) {
            // Point is outside map bounds, stop searching
            RCLCPP_WARN(rclcpp::get_logger(getNodeName()), "Reached map boundary while searching for white pixel");
            break;
        }
        
        // Check if the pixel is white
        cv::Vec3b pixel_color = object_map.at<cv::Vec3b>(available_y_coordinate, available_x_coordinate);
        
        // Check if pixel is white (all channels are 255)
        if ((pixel_color[0] > 240 && pixel_color[1] > 240 && pixel_color[2] > 240) || (pixel_color[0] < 10 && pixel_color[1] < 10 && pixel_color[2] > 240)) {
          RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Found white pixel at distance %d pixels", distance_pixels);
          found_white_pixel = true;
          break;
        }
        
        // Move to next pixel along the line
        distance_pixels++;
        
        // Add a safety limit to prevent infinite loops
        if (distance_pixels > 50) {
            RCLCPP_WARN(rclcpp::get_logger(getNodeName()), "Reached maximum search distance without finding white pixel");
            break;
        }
    }

    // Add the extra pixel distance to the original distance
    double extra_distance_meters = (distance_pixels * scaled_resolution);
    distance_meters += extra_distance_meters;

    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Added %f meters from pixel search, new distance: %f meters", 
                extra_distance_meters, distance_meters);

    // Calculate offset in world coordinates (meters)
    double world_offset_x = distance_meters * std::cos(angle_radians);
    double world_offset_y = distance_meters * std::sin(angle_radians);

    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "World coordinate offset: (%f, %f) meters", 
                world_offset_x, world_offset_y);

    // Calculate final position in world coordinates
    double final_world_x = target_world_x + world_offset_x;
    double final_world_y = target_world_y + world_offset_y;

    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Final world coordinates: (%f, %f) meters", 
                final_world_x, final_world_y);

    // Final visual graph
    // Convert final world coordinates to pixel coordinates for visualization
    cv::Point final_pixel = MapBuilder::worldToMapCoordinates(final_world_x, final_world_y, params, object_map.rows);
    pixel_x = final_pixel.x;
    pixel_y = final_pixel.y;

    // Create JSON for robot visualisation for cartesian coordinates
    json cartesian_llm_response = {
        {"target_id", target_id},
        {"coordinates", {{"x", pixel_x}, {"y", pixel_y}}},
        {"success", "true"}
    };

    // Display Coordinates on map as a simple red dot
    std::string visualization_output_path = (fs::path(DEBUG_SUBDIR) / "target_visualization.png").string();
    cv::Mat visualization = MapBuilder::displayTargetCoordinate(
        object_map, 
        cartesian_llm_response, 
        params, 
        visualization_output_path);
  
    // Debug final coordinates
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Target world: (%f, %f)", target_world_x, target_world_y);
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Polar coords: angle=%f°, distance=%f%%", angle_degrees, distance_percentage);
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "World offset: (%f, %f)", world_offset_x, world_offset_y);
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Final world: (%f, %f)", final_world_x, final_world_y);
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Final pixel: (%d, %d)", pixel_x, pixel_y);
  }

  // Create a response message for the user
  target_id = llm_solver_response["target_id"];
  std::string reasoning = llm_solver_response["message"];

  /*
  * STEP FOUR: GET COORDINATES
  */
 
  // Convert pixel coordinates to world coordinates using scaled resolution
  world_x = pixel_x * scaled_resolution + origin_[0];
  world_y = (object_map.rows - pixel_y) * scaled_resolution + origin_[1];
  
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Real-world coordinates for Nav2: x=%f, y=%f", world_x, world_y);  

  // Calculate angle robot -> target
  double target_y = items_data[target_id]["coordinates"]["y"];
  double target_x = items_data[target_id]["coordinates"]["x"];
  double angle_rad = std::atan2(target_y - world_y, target_x - world_x);
  while (angle_rad < 0) angle_rad += 2 * M_PI;
  angle = angle_rad;
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Final angle: (%f)", angle);

  // Debug final coordinates
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Target world: (%f, %f)", target_x, target_y);
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Final world: (%f, %f)", world_x, world_y);
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Final angle: (%f)", angle);
  
  /*
   * FINAL STEP: RETURN RESULTS
   */

  // Publish get_coordinates result
  publishResult(reasoning);
  
  params_out.pose.position.x = world_x;
  params_out.pose.position.y = world_y;
  params_out.pose.position.z = 0;
  params_out.pose.orientation.r = 0;
  params_out.pose.orientation.p = 0;
  params_out.pose.orientation.y = angle;
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Inspection completed successfully");
  
  return true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * OPTIONAL class methods, can be removed if not needed
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


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
  TEMOTO_PRINT_OF("Destroying", getName());
  
  if (node_) {
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Cleaning up node in destructor");
    chat_publisher_.reset();
    node_.reset();
  }
}

bool loadMapConfig(const std::string& yaml_path) {
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Loading map config from: %s", yaml_path.c_str());
  std::ifstream file(yaml_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Failed to open YAML file: %s", yaml_path.c_str());
    throw std::runtime_error("Failed to open map YAML file");
  }
  
  std::string line;
  bool resolution_found = false;
  bool origin_found = false;
  
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    
    if (line.find("resolution:") != std::string::npos) {
      resolution_ = std::stod(line.substr(line.find(":") + 1));
      resolution_found = true;
      RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Found resolution: %f", resolution_);
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
      RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Found origin: [%f, %f, %f]", origin_[0], origin_[1], origin_[2]);
    }
  }
  
  if (!resolution_found || !origin_found) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Missing required parameters in YAML file");
    throw std::runtime_error("Missing required parameters in map YAML file");
  }
  
  return true;
}

json loadJsonFile(const std::string& file_path) {
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Loading JSON from file: %s", file_path.c_str());
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Failed to open JSON file: %s", file_path.c_str());
    throw std::runtime_error("Failed to open JSON file: " + file_path);
  }
  
  json data;
  try {
    file >> data;
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "JSON file loaded successfully");
    return data;
  } catch (const json::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Error parsing JSON file: %s", e.what());
    throw;
  }
}

void publishResult(const std::string& message) {
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "=== PUBLISHING GETCOORDINATES RESULT ===");
  
  try {
    json j;
    j["targets"] = {"David"};
    j["type"] = "response";
    j["message"] = message;
    
    std::string json_str = j.dump();
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Created JSON message for chat_interface_feedback: %s", 
                json_str.length() > 100 ? (json_str.substr(0, 97) + "...").c_str() : json_str.c_str());
    
    std_msgs::msg::String msg;
    msg.data = json_str;
    
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Publishing message to chat_interface_feedback (size: %zu bytes)", msg.data.size());
    chat_publisher_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Message published successfully to chat_interface_feedback");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "ERROR publishing coordinate result: %s", e.what());
  }
  
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "=== COORDINATE RESULT PUBLISHED ===");
} 

std::string encodeImageToBase64(const cv::Mat& image) {
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "=== ENCODING IMAGE TO BASE64 ===");
  
  try {
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Calling ai_core encoding function...");
    std::string result = ai_core::encodeImageToBase64(image);
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Image encoded successfully to base64 (size: %zu bytes)", result.size());
    
    if (result.size() > 20) {
      RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Encoded data begins with: %s...", result.substr(0, 20).c_str());
    }
    
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "=== BASE64 ENCODING COMPLETED ===");
    return result;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Error encoding image to base64: %s", e.what());
    RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "=== BASE64 ENCODING FAILED ===");
    return "";
  }
}

std::string cleanLLMJsonResponse(const std::string& raw_response) {
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Cleaning raw LLM response to extract JSON...");
  
  size_t start_pos = raw_response.find('{');
  if (start_pos == std::string::npos) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "No JSON object found in response (no opening brace)");
    return "";
  }
  
  size_t end_pos = raw_response.rfind('}');
  if (end_pos == std::string::npos) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "No JSON object found in response (no closing brace)");
    return "";
  }
  
  if (end_pos <= start_pos) {
    RCLCPP_ERROR(rclcpp::get_logger(getNodeName()), "Invalid JSON structure (closing brace before opening brace)");
    return "";
  }
  
  std::string cleaned_json = raw_response.substr(start_pos, end_pos - start_pos + 1);
  RCLCPP_INFO(rclcpp::get_logger(getNodeName()), "Extracted JSON: %s", 
              cleaned_json.length() > 100 ? (cleaned_json.substr(0, 97) + "...").c_str() : cleaned_json.c_str());
  
  return cleaned_json;
}

};

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<GetCoordinates>(new GetCoordinates());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, GetCoordinates)