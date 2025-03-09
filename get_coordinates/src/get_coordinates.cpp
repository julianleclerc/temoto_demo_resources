#include <class_loader/class_loader.hpp>
#include "get_coordinates/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>

#include <nlohmann/json.hpp>
#include "get_coordinates/get_coordinates_run.hpp" 

using json = nlohmann::json;

class GetCoordinates : public TemotoAction
{
public:

GetCoordinates() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{
  std::string output = fmt::format("Determining coordinates for navigation to {}", params_in.location);
  TEMOTO_PRINT_OF(output, getName());

  try {
    // Use relative path for data files
    const std::string DATA_DIR = "data";
    const std::string ITEMS_JSON_PATH = DATA_DIR + "/items.json";
    const std::string MAP_PATH = DATA_DIR + "/map.pgm";
    const std::string MAP_YAML_PATH = DATA_DIR + "/map.yaml";   
     
    // Hardcode robot position to (0, 0)
    json robot_position = {
      {"x", 0.0},
      {"y", 0.0}
    };
    
    TEMOTO_PRINT_OF("Calling findCoordinates for: " + params_in.location, getName());
    
    // Call the findCoordinates function from get_coordinates_run.cpp
    json result = findCoordinates(
        MAP_PATH,           // Map image path
        ITEMS_JSON_PATH,    // Items JSON path
        MAP_YAML_PATH,      // Map YAML path
        DATA_DIR,           // Output directory
        params_in.location, // Target object class
        params_in.location, // Object description (using the same value)
        robot_position      // Hardcoded robot position
    );
    
    // Print the result for debugging
    TEMOTO_PRINT_OF("Coordinate search result: " + result.dump(2), getName());
    
    // Check if coordinates were found successfully
    bool success = false;
    if (result.contains("success")) {
      // Convert from string "true"/"false" to bool if necessary
      if (result["success"].is_string()) {
        success = (result["success"] == "true");
      } else if (result["success"].is_boolean()) {
        success = result["success"].get<bool>();
      }
    }
    
    if (success) {
      TEMOTO_PRINT_OF("Successfully found coordinates", getName());
      
      // Set output parameters based on found coordinates
      if (result.contains("coordinates") && result["coordinates"].contains("x") && result["coordinates"].contains("y")) {
        // Extract coordinates
        double x = result["coordinates"]["x"].get<double>();
        double y = result["coordinates"]["y"].get<double>();
        
        // Set the output pose
        if (result.contains("angle")) {
          double angle = result["angle"].get<double>();
          
          // Update the pose in params_out
          params_out.pose.frame_id = "map";
          params_out.pose.position.x = x;
          params_out.pose.position.y = y;
          params_out.pose.position.z = 0.0;
          
          // Set orientation (yaw)
          params_out.pose.orientation.r = 0.0;
          params_out.pose.orientation.p = 0.0;
          params_out.pose.orientation.y = angle;
          
          TEMOTO_PRINT_OF(fmt::format("Target coordinates: x={}, y={}, angle={}", x, y, angle), getName());
        } else {
          // No angle provided, set default orientation
          params_out.pose.frame_id = "map";
          params_out.pose.position.x = x;
          params_out.pose.position.y = y;
          params_out.pose.position.z = 0.0;
          params_out.pose.orientation.r = 0.0;
          params_out.pose.orientation.p = 0.0;
          params_out.pose.orientation.y = 0.0;
          
          TEMOTO_PRINT_OF(fmt::format("Target coordinates: x={}, y={} (no angle provided)", x, y), getName());
        }
      } else {
        TEMOTO_PRINT_OF("Warning: Result contained success flag but no valid coordinates", getName());
      }
    } else {
      std::string error_msg = "Failed to find coordinates";
      if (result.contains("message")) {
        error_msg = result["message"].get<std::string>();
      }
      TEMOTO_PRINT_OF("Error: " + error_msg, getName());
      return false;
    }
    
    TEMOTO_PRINT_OF("Done\n", getName());
    return true;
  } catch (const std::exception& e) {
    TEMOTO_PRINT_OF("Exception: " + std::string(e.what()), getName());
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

~GetCoordinates()
{
}

}; // GetCoordinates class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(GetCoordinates, ActionBase);