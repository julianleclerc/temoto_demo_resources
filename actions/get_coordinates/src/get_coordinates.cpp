#include "get_coordinates/temoto_action.hpp"

#include <fmt/core.h>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>


#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "get_coordinates/get_coordinates_run.hpp" 
#include <ament_index_cpp/get_package_share_directory.hpp>


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

bool onRun()
{
  TEMOTO_PRINT_OF("Running", getName());

  /*
   * Implement your code here
   *
   * Access input parameters via "params_in" member
   * Set output parameters via "params_out" member
   */
  std::string output = fmt::format("Determining coordinates for navigation to {}", params_in.location);
  TEMOTO_PRINT_OF(output, getName());
 
  try {
    // Use relative path for data files
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("get_coordinates");
    
    // Navigate up to workspace root (from install/share/get_coordinates)
    std::filesystem::path workspace_path = std::filesystem::path(package_share_dir) / "../../../..";
    workspace_path = std::filesystem::canonical(workspace_path);
    
    // Set data paths
    const std::string DATA_DIR = (workspace_path / "data").string();
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
          params_out.position.x = x;
          params_out.position.y = y;
          params_out.position.z = 0.0;
          
          // Set orientation (yaw)
          params_out.orientation.r = 0.0;
          params_out.orientation.p = 0.0;
          params_out.orientation.y = angle;
          
          TEMOTO_PRINT_OF(fmt::format("Target coordinates: x={}, y={}, angle={}", x, y, angle), getName());
        } else {
          // No angle provided, set default orientation
          params_out.position.x = x;
          params_out.position.y = y;
          params_out.position.z = 0.0;
          params_out.orientation.r = 0.0;
          params_out.orientation.p = 0.0;
          params_out.orientation.y = 0.0;
          
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

}; // GetCoordinates class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<GetCoordinates>(new GetCoordinates());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, GetCoordinates)