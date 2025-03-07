#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

// Include custom script headers
#include "get_coordinates/getcoord_scalemap_generation.h"
#include "get_coordinates/getcoord_costmap_generation.h"
#include "get_coordinates/getcoord_nonTraversable_generation.h"
#include "get_coordinates/getcoord_grid_generation.h"
#include "get_coordinates/getcoord_objectmap_generation.h"
#include "get_coordinates/getcoord_pixelcoord_return.h"
#include "get_coordinates/getcoord_newcoordmap_generation.h"
#include "get_coordinates/getcoord_origincoord_return.h"
#include "get_coordinates/getcoord_robotmap_generation.h"
#include "get_coordinates/ai_core.h"
#include "get_coordinates/llm_coordinator.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

// Hardcoded paths
const std::string DATA_DIR = "/home/fyier/thesis_temoto/actions/action_test_workspace/src/data";
const std::string ITEMS_JSON_PATH = DATA_DIR + "/items.json";
const std::string MAP_PATH = DATA_DIR + "/map.pgm";
const std::string MAP_YAML_PATH = DATA_DIR + "/map.yaml";

// Define a simple RobotTransform struct to pass robot position
struct RobotTransform {
    double x;
    double y;
};

class CoordinateFinder {
private:
    // Configuration parameters
    float inflation_radius_m = 0.2;
    int scale_factor = 2;
    int grid_scale = 20; // in px
    float resolution = 0.05;
    std::vector<float> origin = {0.0, 0.0, 0.0};

    // Data storage
    json items_data;
    
    // Output directory for saving images
    std::string output_dir;

    // AI stuff
    std::string COORDINATES_METHOD = "oneCoordSearch";
    // Declare the LLM Coordinator
    get_coordinates::LLMCoordinator llm_coordinator;
    // For storing the object map
    cv::Mat object_map;

    // Base64 encoding function
    std::string base64_encode(const unsigned char* data, size_t length) {
        static const char* encoding_table = 
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        static const char padding_char = '=';
        
        std::string encoded;
        encoded.reserve(((length + 2) / 3) * 4);  // Reserve space for the encoded string
        
        for (size_t i = 0; i < length; i += 3) {
            uint32_t octet_a = i < length ? data[i] : 0;
            uint32_t octet_b = i + 1 < length ? data[i + 1] : 0;
            uint32_t octet_c = i + 2 < length ? data[i + 2] : 0;
            
            uint32_t triple = (octet_a << 16) + (octet_b << 8) + octet_c;
            
            encoded.push_back(encoding_table[(triple >> 18) & 0x3F]);
            encoded.push_back(encoding_table[(triple >> 12) & 0x3F]);
            encoded.push_back(encoding_table[(triple >> 6) & 0x3F]);
            encoded.push_back(encoding_table[triple & 0x3F]);
        }
        
        // Add padding if needed
        size_t mod = length % 3;
        if (mod == 1) {
            encoded[encoded.size() - 1] = padding_char;
            encoded[encoded.size() - 2] = padding_char;
        } else if (mod == 2) {
            encoded[encoded.size() - 1] = padding_char;
        }
        
        return encoded;
    }

    // Internal helper methods
    json loadJsonFile(const std::string& filepath) {
        std::cout << "DEBUG LOAD_JSON: About to open file: " << filepath << std::endl;
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "DEBUG LOAD_JSON: Could not open file" << std::endl;
            throw std::runtime_error("Could not open file: " + filepath);
        }
        
        std::cout << "DEBUG LOAD_JSON: File opened successfully, about to parse JSON" << std::endl;
        try {
            json parsed_json = json::parse(file);
            std::cout << "DEBUG LOAD_JSON: JSON parsed successfully" << std::endl;
            return parsed_json;
        } catch (const json::exception& e) {
            std::cerr << "DEBUG LOAD_JSON: JSON parse error: " << e.what() << std::endl;
            throw;
        }
    }

    bool loadMapConfig(const std::string& yaml_path) {
        try {
            YAML::Node config = YAML::LoadFile(yaml_path);
            
            // Extract parameters from the YAML file
            if (config["resolution"]) {
                resolution = config["resolution"].as<float>();
                std::cout << "Loaded resolution: " << resolution << std::endl;
            }
            
            if (config["origin"]) {
                auto yaml_origin = config["origin"].as<std::vector<float>>();
                if (yaml_origin.size() >= 3) {
                    origin = yaml_origin;
                    std::cout << "Loaded origin: [" << origin[0] << ", " << origin[1] << ", " << origin[2] << "]" << std::endl;
                }
            }
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error loading map config: " << e.what() << std::endl;
            return false;
        }
    }

    std::string encodeImageToBase64(const cv::Mat& image) {
        std::vector<uchar> buffer;
        cv::imencode(".png", image, buffer);
        return base64_encode(buffer.data(), buffer.size());
    }
    
    void saveImage(const cv::Mat& image, const std::string& filename) {
        std::string filepath = output_dir + "/" + filename;
        cv::imwrite(filepath, image);
        std::cout << "Saved image to: " << filepath << std::endl;
    }

    // Convert world coordinates to pixel coordinates
    std::pair<int, int> worldToPixel(double x, double y, int height, float res) {
        int pixel_x = static_cast<int>((x - origin[0]) / res);
        int pixel_y = height - static_cast<int>((y - origin[1]) / res) - 1;
        return {pixel_x, pixel_y};
    }

    // Convert pixel coordinates to world coordinates
    std::pair<double, double> pixelToWorld(int x, int y, int height, float res) {
        double world_x = x * res + origin[0];
        double world_y = (height - y - 1) * res + origin[1];
        return {world_x, world_y};
    }

    void initializeLLMCoordinator() {
        std::cout << "DEBUG INIT_LLM: Starting LLM Coordinator initialization" << std::endl;
        
        // Define your system instructions for the LLM
        std::string instructions = R"(
            You are a visual analysis system specialized in locating objects in a map based on descriptions.
            Your task is to identify the pixel coordinates of the requested object in the provided image.
            Respond in JSON format only with the following structure:
            {
                "success": "true" or "false",
                "coordinates": {"x": pixel_x, "y": pixel_y},
                "error": "none" or error_type,
                "message": "additional information if needed"
            }
            )";

        // Check items_data before passing it
        std::cout << "DEBUG INIT_LLM: About to dump items_data" << std::endl;
        std::string items_data_str;
        try {
            items_data_str = items_data.dump();
            std::cout << "DEBUG INIT_LLM: items_data dump successful, length: " << items_data_str.length() << std::endl;
            if (items_data_str.length() > 100) {
                std::cout << "DEBUG INIT_LLM: items_data preview: " << items_data_str.substr(0, 50) << "..." << std::endl;
            } else {
                std::cout << "DEBUG INIT_LLM: items_data: " << items_data_str << std::endl;
            }
        } catch (const json::exception& e) {
            std::cerr << "DEBUG INIT_LLM: JSON error dumping items_data: " << e.what() << std::endl;
            throw;
        }

        // Convert nlohmann::json to Json::Value
        Json::Value jsonCpp_items_data;
        Json::Reader reader;
        
        if (!reader.parse(items_data_str, jsonCpp_items_data)) {
            throw std::runtime_error("Failed to convert items_data to Json::Value");
        }
        
        // Initialize the coordinator with the converted Json::Value
        std::cout << "DEBUG INIT_LLM: About to call llm_coordinator.initialize" << std::endl;
        try {
            llm_coordinator.initialize(jsonCpp_items_data, instructions, items_data_str);
            std::cout << "DEBUG INIT_LLM: Successfully initialized llm_coordinator" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "DEBUG INIT_LLM: Error in llm_coordinator.initialize: " << e.what() << std::endl;
            throw;
        }
    }

public:
    CoordinateFinder(const std::string& items_json_path, const std::string& output_directory, const std::string& map_yaml_path = "") 
        : output_dir(output_directory) {
        std::cout << "DEBUG CONSTRUCTOR: Starting constructor" << std::endl;
        
        // Load items data from JSON
        std::cout << "DEBUG CONSTRUCTOR: About to load JSON file: " << items_json_path << std::endl;
        try {
            items_data = loadJsonFile(items_json_path);
            std::cout << "DEBUG CONSTRUCTOR: Successfully loaded items_data" << std::endl;
            // Inspect the top-level structure
            std::cout << "DEBUG CONSTRUCTOR: items_data keys: ";
            for (auto& [key, val] : items_data.items()) {
                std::cout << key << " ";
            }
            std::cout << std::endl;
        } catch (const json::exception& e) {
            std::cerr << "DEBUG CONSTRUCTOR: JSON error loading items_data: " << e.what() << std::endl;
            throw;
        }
        
        // Load map configuration if YAML path provided
        if (!map_yaml_path.empty() && fs::exists(map_yaml_path)) {
            if (!loadMapConfig(map_yaml_path)) {
                std::cerr << "Warning: Failed to load map config from " << map_yaml_path << std::endl;
                std::cerr << "Using default values: resolution=" << resolution 
                          << ", origin=[" << origin[0] << "," << origin[1] << "," << origin[2] << "]" << std::endl;
            }
        } else {
            std::cerr << "Warning: Map YAML file not found at " << map_yaml_path << std::endl;
            std::cerr << "Using default values: resolution=" << resolution 
                      << ", origin=[" << origin[0] << "," << origin[1] << "," << origin[2] << "]" << std::endl;
        }
        
        // Create JSON file with current parameters
        json params = {
            {"inflation_radius_m", inflation_radius_m},
            {"scale_factor", scale_factor},
            {"grid_scale", grid_scale},
            {"resolution", resolution},
            {"origin", {origin[0], origin[1], origin[2]}}
        };
        
        std::ofstream param_file(output_dir + "/parameters.json");
        param_file << params.dump(4);
        param_file.close();
        
        // Initialize the LLM Coordinator
        std::cout << "DEBUG CONSTRUCTOR: About to initialize LLM Coordinator" << std::endl;
        try {
            initializeLLMCoordinator();
            std::cout << "DEBUG CONSTRUCTOR: Successfully initialized LLM Coordinator" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "DEBUG CONSTRUCTOR: Error initializing LLM Coordinator: " << e.what() << std::endl;
            throw;
        }
    }

    json findCoordinates(const std::string& map_path, const std::string& target_object, const std::string& object_description, const json& robot_position = json()) {
        
        bool success_map_init = false;
        // Declare these variables at the top of the function so they're accessible in all scopes
        cv::Mat scaled_img;
        float scaled_resolution;
        json result;
        
        try {
            // Load map
            cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
            if (map_img.empty()) {
                throw std::runtime_error("Failed to load map image");
            }
            saveImage(map_img, "01_original_map.png");

            // Process 1: Scale map
            std::tie(scaled_img, scaled_resolution) = GetCoordScaleMapGeneration::process(map_img, resolution, scale_factor);
            saveImage(scaled_img, "02_scaled_map.png");

            // Process 2: Generate cost map
            cv::Mat cost_map = GetCoordCostmapGeneration::process(scaled_img, scaled_resolution, inflation_radius_m);
            saveImage(cost_map, "03_cost_map.png");

            // Convert cost_map to color for further processing
            cv::Mat cost_map_color;
            cv::cvtColor(cost_map, cost_map_color, cv::COLOR_GRAY2BGR);

            // Process 3: Darken non-traversable areas
            cv::Mat non_traversable_map;
            
            // If robot position is provided, use it for non-traversable areas
            if (!robot_position.empty() && robot_position.contains("x") && robot_position.contains("y")) {
                // Get robot coordinates
                RobotTransform robot_transform;
                robot_transform.x = robot_position["x"];
                robot_transform.y = robot_position["y"];
                
                // Convert robot world coordinates to pixel coordinates for visualization
                auto robot_pixel = worldToPixel(robot_transform.x, robot_transform.y, scaled_img.rows, scaled_resolution);
                
                // Draw robot position on the cost map for visualization
                cv::circle(cost_map_color, cv::Point(robot_pixel.first, robot_pixel.second), 5, cv::Scalar(0, 0, 255), -1);
                saveImage(cost_map_color, "03b_cost_map_with_robot.png");
                
                // For debugging - create a proper implementation that uses the robot position
                non_traversable_map = GetCoordNonTraversableGeneration::process(cost_map_color, scaled_resolution, origin);
                
                // Mark non-traversable areas in a more visible way for debugging
                cv::Mat debug_map = non_traversable_map.clone();
                
                // Find black pixels (non-traversable areas) and make them red
                for (int y = 0; y < debug_map.rows; y++) {
                    for (int x = 0; x < debug_map.cols; x++) {
                        cv::Vec3b pixel = debug_map.at<cv::Vec3b>(y, x);
                        // If pixel is very dark (non-traversable)
                        if (pixel[0] < 50 && pixel[1] < 50 && pixel[2] < 50) {
                            debug_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 200); // Red in BGR
                        }
                    }
                }
                
                saveImage(debug_map, "04_debug_non_traversable.png");
            } else {
                // No robot position, just use the basic non-traversable function
                non_traversable_map = GetCoordNonTraversableGeneration::process(cost_map_color, scaled_resolution, origin);
            }
            
            saveImage(non_traversable_map, "04_non_traversable_map.png");

            // Process 4: Create grid
            cv::Mat grid_map = GetCoordGridGeneration::process(non_traversable_map, scaled_resolution, grid_scale);
            saveImage(grid_map, "05_grid_map.png");

            // Process 5: Populate map with objects
            object_map = GetCoordObjectMapGeneration::process(grid_map, scaled_resolution, origin, items_data);
            saveImage(object_map, "06_object_map.png");

            // Process 6: Convert items coordinates to pixel coordinates for AI processing
            json pixel_coords = GetCoordPixelCoordReturn::process(
                items_data, scaled_resolution, origin, {object_map.rows, object_map.cols}
            );
            
            // Save the pixel coordinates to a JSON file
            std::ofstream pixel_coords_file(output_dir + "/07_pixel_coordinates.json");
            pixel_coords_file << pixel_coords.dump(4);
            pixel_coords_file.close();

            success_map_init = true;
        
        } catch (const std::exception& e) {
            json error_response = {
                {"success", false},
                {"error", e.what()},
                {"message", "Failed to process coordinates"}
            };
            
            // Save the error to a file
            std::ofstream error_file(output_dir + "/error_log.json");
            error_file << error_response.dump(4);
            error_file.close();
            
            return error_response;
        }

        if (success_map_init == true) {
            
            try {
                
                std::cout << "Debug: Starting LLM coordinate search" << std::endl;
                ////// GET COORDINATES USING LLM HERE //////
                // Prepare request message
                json request_msg = {
                    {"class", target_object},
                    {"description", object_description}
                };

                // Base64 encode the object map for AI processing
                std::vector<uchar> buffer;
                cv::imencode(".jpg", object_map, buffer);
                std::cout << "Debug: Image encoded to buffer size: " << buffer.size() << std::endl;
                
                // Proper base64 encoding
                std::string base64_data = base64_encode(buffer.data(), buffer.size());
                std::cout << "Debug: base64_data length: " << base64_data.length() << std::endl;
                if (base64_data.length() > 40) {
                    std::cout << "Debug: base64_data preview: " << base64_data.substr(0, 20) << "..." 
                            << base64_data.substr(base64_data.length() - 20) << std::endl;
                }

                // Convert nlohmann::json to Json::Value
                Json::Value json_request_msg;
                Json::Value json_encoded_map;
                Json::Reader reader;

                // Convert request_msg
                std::string request_msg_str = request_msg.dump();
                std::cout << "Debug: request_msg_str: " << request_msg_str << std::endl;
                
                if (!reader.parse(request_msg_str, json_request_msg)) {
                    std::string parse_error = "Failed to parse request message to Json::Value";
                    std::cout << "Debug: " << parse_error << std::endl;
                    throw std::runtime_error(parse_error);
                }
                std::cout << "Debug: Successfully parsed request_msg to Json::Value" << std::endl;

                // Correctly create a string JSON value 
                json_encoded_map = Json::Value(base64_data);
                std::cout << "Debug: Created Json::Value for encoded image" << std::endl;

                // Get coordinates using AI
                std::string assistant_reply;
                if (COORDINATES_METHOD == "oneCoordSearch") {
                    std::cout << "Debug: Calling getcoord_search with request and base64 image data" << std::endl;
                    
                    try {
                        // Call the getcoord_search method with the converted Json::Value objects
                        assistant_reply = llm_coordinator.getcoord_search(json_request_msg, json_encoded_map);
                        std::cout << "Debug: Received assistant reply" << std::endl;
                    } catch (const std::exception& e) {
                        std::string error_msg = "Error in getcoord_search: " + std::string(e.what());
                        std::cout << "Debug: " << error_msg << std::endl;
                        throw std::runtime_error(error_msg);
                    }
                }

                // Parse the response
                try {
                    std::cout << "Debug: Assistant reply: " << assistant_reply << std::endl;
                    result = json::parse(assistant_reply);
                    std::cout << "Debug: Successfully parsed assistant reply" << std::endl;
                } catch (const json::parse_error& e) {
                    std::string error_msg = "Failed to parse AI response: " + std::string(e.what());
                    std::cout << "Debug: " << error_msg << std::endl;
                    std::cout << "Debug: Response was: " << assistant_reply << std::endl;
                    throw std::runtime_error(error_msg);
                }

                // Save the AI result to a JSON file
                std::ofstream ai_result_file(output_dir + "/08_ai_search_result.json");
                ai_result_file << result.dump(4);
                ai_result_file.close();
                std::cout << "Debug: Saved AI result to file" << std::endl;
                ////// ------------------------------ //////


            } catch (const std::exception& e) {
                json error_response = {
                    {"success", false},
                    {"error", e.what()},
                    {"message", "Failed to process coordinates"}
                };
                
                // Save the error to a file
                std::ofstream error_file(output_dir + "/error_log.json");
                error_file << error_response.dump(4);
                error_file.close();
                
                return error_response;
            }
            
            try {
                // Check the structure of the result before proceeding
                std::cout << "Debug: Result structure: " << result.dump() << std::endl;
                
                // Process 7: Generate new coordinates map
                cv::Mat new_coords_map;
                float angle_deg;
                std::tie(new_coords_map, angle_deg) = GetCoordNewCoordmapGeneration::process(
                    object_map, scaled_resolution, origin, result, items_data
                );
                saveImage(new_coords_map, "09_new_coords_map.png");
                std::cout << "Debug: Generated new coordinates map" << std::endl;

                // Process 8: Get origin coordinates
                json origin_coords = GetCoordOriginCoordReturn::process(
                    result, scaled_resolution, origin, 
                    {object_map.rows, object_map.cols}, angle_deg
                );
                
                // Debug the structure of origin_coords
                std::cout << "Debug: Origin coords structure: " << origin_coords.dump() << std::endl;
                
                // Save the final coordinates to a JSON file
                std::ofstream final_coords_file(output_dir + "/10_final_coordinates.json");
                final_coords_file << origin_coords.dump(4);
                final_coords_file.close();
                std::cout << "Debug: Saved final coordinates to file" << std::endl;

                return origin_coords;

            } catch (const std::exception& e) {
                json error_response = {
                    {"success", false},
                    {"error", e.what()},
                    {"message", "Failed to process coordinates"}
                };
                
                // Save the error to a file
                std::ofstream error_file(output_dir + "/error_log.json");
                error_file << error_response.dump(4);
                error_file.close();
                
                return error_response;
            }

        } else {
            json error_response = {
                {"success", false},
                {"error", "MapInitError"},
                {"message", "Failed to initialize map for coordinate search"}
            };
            
            // Save the error to a file
            std::ofstream error_file(output_dir + "/error_log.json");
            error_file << error_response.dump(4);
            error_file.close();
            
            return error_response;
        }
    }
};

// Main function
int main(int argc, char** argv) {
    try {
        // Default target object and description if not provided as arguments
        std::string target_object = "table";
        std::string object_description = "wooden table";
        
        // Override defaults if arguments are provided
        if (argc >= 2) {
            target_object = argv[1];
        }
        
        if (argc >= 3) {
            object_description = argv[2];
        }
        
        // Initialize robot position for pathfinding (if needed)
        json robot_position = json::object();
        
        if (argc >= 5) {
            robot_position["x"] = std::stod(argv[3]);
            robot_position["y"] = std::stod(argv[4]);
        }
        
        // Print startup information
        std::cout << "=== Get Coordinates Tool ===" << std::endl;
        std::cout << "Data Directory: " << DATA_DIR << std::endl;
        std::cout << "Map Path: " << MAP_PATH << std::endl;
        std::cout << "Map YAML Path: " << MAP_YAML_PATH << std::endl;
        std::cout << "Items JSON Path: " << ITEMS_JSON_PATH << std::endl;
        std::cout << "Target Object: " << target_object << std::endl;
        std::cout << "Object Description: " << object_description << std::endl;
        if (!robot_position.empty()) {
            std::cout << "Robot Position: x=" << robot_position["x"] << ", y=" << robot_position["y"] << std::endl;
        }
        std::cout << "=========================" << std::endl;
        
        // Check if required files exist
        if (!fs::exists(MAP_PATH)) {
            throw std::runtime_error("Map file not found: " + MAP_PATH);
        }
        
        if (!fs::exists(ITEMS_JSON_PATH)) {
            throw std::runtime_error("Items JSON file not found: " + ITEMS_JSON_PATH);
        }

        // Try-catch with specific handling for JSON errors
        try {
            // Initialize coordinate finder with hardcoded paths
            CoordinateFinder finder(ITEMS_JSON_PATH, DATA_DIR, MAP_YAML_PATH);

            // Find coordinates for the target object
            std::cout << "Debug: Calling findCoordinates..." << std::endl;
            json result = finder.findCoordinates(
                MAP_PATH,          // Map image path
                target_object,     // Target object class
                object_description, // Object description
                robot_position     // Optional robot position
            );

            // Print result
            std::cout << "Coordinate Search Result:\n" 
                    << result.dump(4) << std::endl;
        } catch (const nlohmann::json::exception& e) {
            std::cerr << "JSON Error: " << e.what() << std::endl;
            std::cerr << "Error ID: " << e.id << std::endl;
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}