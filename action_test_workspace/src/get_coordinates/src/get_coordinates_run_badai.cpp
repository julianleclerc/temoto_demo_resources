#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set> // Added missing include
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
#include "get_coordinates/getcoord_pathfind_return.h"
#include "get_coordinates/getcoord_robotmap_generation.h"

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

    // Internal helper methods
    json loadJsonFile(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filepath);
        }
        return json::parse(file);
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
        
        // Base64 encoding (simplified example - in production, use a proper base64 library)
        std::stringstream ss;
        for (uchar c : buffer) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)c;
        }
        return ss.str();
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

public:
    CoordinateFinder(const std::string& items_json_path, const std::string& output_directory, const std::string& map_yaml_path = "") 
        : output_dir(output_directory) {
        // Load items data from JSON
        items_data = loadJsonFile(items_json_path);
        
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
    }

    json findCoordinates(const std::string& map_path, const std::string& target_object, const std::string& object_description, const json& robot_position = json()) {
        try {
            // Load map
            cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
            if (map_img.empty()) {
                throw std::runtime_error("Failed to load map image");
            }
            saveImage(map_img, "01_original_map.png");

            // Process 1: Scale map
            cv::Mat scaled_img;
            float scaled_resolution;
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
            cv::Mat object_map = GetCoordObjectMapGeneration::process(grid_map, scaled_resolution, origin, items_data);
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

        if success_map_init == true {
            
            try {
                ////// GET COORDINATES USING LLM HERE //////


                // Encode map to base64 for AI processing
                std::string encoded_map = encodeImageToBase64(object_map);
                
                // 3 options: 
                // - oneCoordSearch
                // - divCoordSearch
                // - AstarCoordSearch
                COORDINATES_METHOD = "oneCoordSearch"

                if COORDINATES_METHOD == "oneCoordSearch" {
                    // Call the getcoord_search method with the request message and encoded image
                    assistant_reply = self.getcoord_search(request_msg, encoded_object_map)
                }


                // Simulate AI coordinate search (replace with actual AI integration)
                json result = simulateAICoordinateSearch(target_object, object_description, encoded_map);
                
                // Save the AI result to a JSON file
                std::ofstream ai_result_file(output_dir + "/08_ai_search_result.json");
                ai_result_file << result.dump(4);
                ai_result_file.close();


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
                // Process 7: Generate new coordinates map
                cv::Mat new_coords_map;
                float angle_deg;
                std::tie(new_coords_map, angle_deg) = GetCoordNewCoordmapGeneration::process(
                    object_map, scaled_resolution, origin, result, items_data
                );
                saveImage(new_coords_map, "09_new_coords_map.png");

                // Process 8: Get origin coordinates
                json origin_coords = GetCoordOriginCoordReturn::process(
                    result, scaled_resolution, origin, 
                    {object_map.rows, object_map.cols}, angle_deg
                );
                
                // Save the final coordinates to a JSON file
                std::ofstream final_coords_file(output_dir + "/10_final_coordinates.json");
                final_coords_file << origin_coords.dump(4);
                final_coords_file.close();

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

    json findPathToTarget(const std::string& map_path, const json& robot_position, const std::string& target_object) {
        try {
            // First ensure we have valid target coordinates
            json target_coords;
            bool target_found = false;
            
            // Try to find target in items_data first
            for (const auto& [item_class, items_list] : items_data["items"].items()) {
                for (const auto& item : items_list) {
                    if (item["id"] == target_object) {
                        target_coords = {
                            {"success", true},
                            {"target_id", target_object},
                            {"coordinates", {
                                {"x", item["coordinates"]["x"]},
                                {"y", item["coordinates"]["y"]}
                            }}
                        };
                        target_found = true;
                        break;
                    }
                }
                if (target_found) break;
            }
            
            // If not found, use the findCoordinates function to locate it
            if (!target_found) {
                json location_result = findCoordinates(map_path, target_object, "Target object", robot_position);
                if (location_result.contains("success") && location_result["success"] == true &&
                    location_result.contains("coordinates")) {
                    target_coords = location_result;
                } else {
                    return {
                        {"success", false},
                        {"error", "TargetNotFound"},
                        {"message", "Could not find target object coordinates: " + target_object}
                    };
                }
            }
            
            std::cout << "Target coordinates: " << target_coords["coordinates"].dump(4) << std::endl;
            
            // Load map
            cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
            if (map_img.empty()) {
                throw std::runtime_error("Failed to load map image");
            }
            
            // Create path finding map
            cv::Mat scaled_img;
            float scaled_resolution;
            std::tie(scaled_img, scaled_resolution) = GetCoordScaleMapGeneration::process(map_img, resolution, scale_factor);
            
            cv::Mat cost_map = GetCoordCostmapGeneration::process(scaled_img, scaled_resolution, inflation_radius_m);
            cv::Mat path_map;
            cv::cvtColor(cost_map, path_map, cv::COLOR_GRAY2BGR);
            
            // Convert robot coordinates to pixel coordinates
            auto robot_pixel = worldToPixel(robot_position["x"], robot_position["y"], path_map.rows, scaled_resolution);
            auto target_pixel = worldToPixel(target_coords["coordinates"]["x"], target_coords["coordinates"]["y"], path_map.rows, scaled_resolution);
            
            // Draw robot and target on the path map
            cv::circle(path_map, cv::Point(robot_pixel.first, robot_pixel.second), 5, cv::Scalar(0, 0, 255), -1); // Red for robot
            cv::circle(path_map, cv::Point(target_pixel.first, target_pixel.second), 5, cv::Scalar(0, 255, 0), -1); // Green for target
            
            saveImage(path_map, "path_planning_map.png");
            
            // Implement a simple path planning algorithm (A*)
            std::vector<std::pair<int, int>> path = findPathAStar(path_map, robot_pixel, target_pixel);
            
            // Draw the path on the map
            cv::Mat path_visualization = path_map.clone();
            for (const auto& point : path) {
                cv::circle(path_visualization, cv::Point(point.first, point.second), 2, cv::Scalar(255, 0, 0), -1); // Blue for path
            }
            
            saveImage(path_visualization, "path_visualization.png");
            
            // Convert path points to world coordinates
            json path_points = json::array();
            for (const auto& point : path) {
                auto world_coords = pixelToWorld(point.first, point.second, path_map.rows, scaled_resolution);
                path_points.push_back({
                    {"x", world_coords.first},
                    {"y", world_coords.second}
                });
            }
            
            // Save the path to a JSON file
            json path_result = {
                {"success", true},
                {"target_id", target_object},
                {"target_coordinates", target_coords["coordinates"]},
                {"robot_coordinates", robot_position},
                {"path", path_points}
            };
            
            std::ofstream path_file(output_dir + "/path_result.json");
            path_file << path_result.dump(4);
            path_file.close();
            
            return path_result;
            
        } catch (const std::exception& e) {
            json error_response = {
                {"success", false},
                {"error", "PathfindingError"},
                {"message", std::string("Failed to find path: ") + e.what()}
            };
            
            // Save the error to a file
            std::ofstream error_file(output_dir + "/pathfinding_error_log.json");
            error_file << error_response.dump(4);
            error_file.close();
            
            return error_response;
        }
    }

private:
    // A* Pathfinding implementation
    std::vector<std::pair<int, int>> findPathAStar(const cv::Mat& map, std::pair<int, int> start, std::pair<int, int> goal) {
        // Define Node structure for A* search
        struct Node {
            int x, y;
            double g, h, f;
            Node* parent;
            
            Node(int x_, int y_, double g_ = 0, double h_ = 0, Node* parent_ = nullptr)
                : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent(parent_) {}
                
            // For set comparison
            bool operator==(const Node& other) const {
                return x == other.x && y == other.y;
            }
        };
        
        // Create a hash function for Node
        struct NodeHash {
            size_t operator()(const Node& node) const {
                return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
            }
        };
        
        // Create a compare function for priority queue
        struct NodeCompare {
            bool operator()(const Node* a, const Node* b) const {
                return a->f > b->f; // Priority queue is a max heap by default
            }
        };
        
        // Directions for movement (4-connected grid)
        const int dx[4] = {0, 1, 0, -1};
        const int dy[4] = {-1, 0, 1, 0};
        
        // Open list (priority queue) and closed set
        std::priority_queue<Node*, std::vector<Node*>, NodeCompare> openList;
        
        // Use a map to track visited nodes instead of a set
        std::unordered_map<int, std::unordered_map<int, bool>> visited;
        
        // Create start node
        Node* startNode = new Node(start.first, start.second);
        startNode->h = heuristic(start.first, start.second, goal.first, goal.second);
        startNode->f = startNode->h; // g is 0 for start
        
        // Add start node to open list
        openList.push(startNode);
        
        // Track all allocated nodes for cleanup
        std::vector<Node*> allocatedNodes = {startNode};
        
        // Main loop
        while (!openList.empty()) {
            // Get the node with the lowest f score
            Node* current = openList.top();
            openList.pop();
            
            // Check if we've reached the goal
            if (current->x == goal.first && current->y == goal.second) {
                // Reconstruct path
                std::vector<std::pair<int, int>> path;
                while (current != nullptr) {
                    path.push_back({current->x, current->y});
                    current = current->parent;
                }
                
                // Reverse to get path from start to goal
                std::reverse(path.begin(), path.end());
                
                // Clean up allocated nodes
                for (auto* node : allocatedNodes) {
                    delete node;
                }
                
                return path;
            }
            
            // Mark as visited
            visited[current->x][current->y] = true;
            
            // Check all neighbors
            for (int i = 0; i < 4; i++) {
                int nx = current->x + dx[i];
                int ny = current->y + dy[i];
                
                // Skip if already visited or not walkable
                if (visited[nx][ny] || !isWalkable(map, nx, ny)) {
                    continue;
                }
                
                // Calculate g score (cost from start)
                double ng = current->g + 1; // Assuming unit cost for movement
                
                // Create a new neighbor node
                Node* neighborNode = new Node(nx, ny, ng, heuristic(nx, ny, goal.first, goal.second), current);
                allocatedNodes.push_back(neighborNode);
                
                // Add to open list
                openList.push(neighborNode);
            }
        }
        
        // No path found
        std::vector<std::pair<int, int>> emptyPath;
        
        // Clean up allocated nodes
        for (auto* node : allocatedNodes) {
            delete node;
        }
        
        return emptyPath;
    }
    
    // Calculate heuristic (Manhattan distance)
    double heuristic(int x1, int y1, int x2, int y2) {
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    }
    
    // Check if a position is walkable (not an obstacle)
    bool isWalkable(const cv::Mat& map, int x, int y) {
        // Check bounds
        if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
            return false;
        }
        
        // Check if the pixel is not an obstacle (black or dark gray)
        cv::Vec3b pixel = map.at<cv::Vec3b>(y, x);
        return (pixel[0] > 100 || pixel[1] > 100 || pixel[2] > 100);
    }
    
    // Actual AI integration 

    std::string Coordinator::LLM_Search(const std::string& object_class, 
        const std::string& object_description, 
        const std::string& error_log, 
        const Json::Value& object_map) {
            
        // Initialize messages with system instructions
        Json::Value messages(Json::arrayValue);

        // System instruction
        Json::Value systemMsg;
        systemMsg["role"] = "system";

        Json::Value systemContent(Json::arrayValue);
        Json::Value systemTextContent;
        systemTextContent["type"] = "text";
        systemTextContent["text"] = INSTRUCTIONS;
        systemContent.append(systemTextContent);

        systemMsg["content"] = systemContent;
        messages.append(systemMsg);

        // Add the object list
        Json::Value objectListMsg;
        objectListMsg["role"] = "user";

        Json::Value objectListContent(Json::arrayValue);
        Json::Value objectListTextContent;
        objectListTextContent["type"] = "text";
        objectListTextContent["text"] = "The list of objects registered are: " + map_data;
        objectListContent.append(objectListTextContent);

        objectListMsg["content"] = objectListContent;
        messages.append(objectListMsg);

        // Add the map
        Json::Value mapMsg;
        mapMsg["role"] = "user";

        Json::Value mapContent(Json::arrayValue);
        Json::Value mapTextContent;
        mapTextContent["type"] = "text";
        mapTextContent["text"] = "The map is: ";
        mapContent.append(mapTextContent);

        Json::Value mapImageContent;
        mapImageContent["type"] = "image_url";

        Json::Value imageUrl;
        // Assuming object_map is already a base64 string
        std::string base64Map = object_map.asString();
        imageUrl["url"] = "data:image/jpeg;base64," + base64Map;

        mapImageContent["image_url"] = imageUrl;
        mapContent.append(mapImageContent);

        mapMsg["content"] = mapContent;
        messages.append(mapMsg);

        // Add the object and description
        Json::Value objectMsg;
        objectMsg["role"] = "user";

        Json::Value objectContent(Json::arrayValue);
        Json::Value objectTextContent;
        objectTextContent["type"] = "text";
        objectTextContent["text"] = "Return the coordinates for: " + object_class + 
            ", with attributes: " + object_description;
        objectContent.append(objectTextContent);

        objectMsg["content"] = objectContent;
        messages.append(objectMsg);

        // Add error log if it exists
        if (!error_log.empty()) {
        std::string error_info = "\n An error has previously come up, below is the thread of messages between you and the user.\n"
            " Please use this information and try to determine the correct object and return its coordinates.\n"
            " If the object is still not clear, continue until success or until user asks to skip.\n";

        Json::Value errorInfoMsg;
        errorInfoMsg["role"] = "system";

        Json::Value errorInfoContent(Json::arrayValue);
        Json::Value errorInfoTextContent;
        errorInfoTextContent["type"] = "text";
        errorInfoTextContent["text"] = error_info;
        errorInfoContent.append(errorInfoTextContent);

        errorInfoMsg["content"] = errorInfoContent;
        messages.append(errorInfoMsg);

        Json::Value errorLogMsg;
        errorLogMsg["role"] = "assistant";

        Json::Value errorLogContent(Json::arrayValue);
        Json::Value errorLogTextContent;
        errorLogTextContent["type"] = "text";
        errorLogTextContent["text"] = error_log;
        errorLogContent.append(errorLogTextContent);

        errorLogMsg["content"] = errorLogContent;
        messages.append(errorLogMsg);
        }

        // Get response from AI
        try {
        // Call to AI service (this would be implemented separately)
        std::string assistant_reply = ai_core.AI_Image_Prompt(
        Json::FastWriter().write(messages),
        1.0,    // TEMPERATURE
        300,    // MAX_TOKENS
        0.0,    // FREQUENCY_PENALTY
        0.0     // PRESENCE_PENALTY
        );

        log_info("assistant_reply: " + assistant_reply);
        return assistant_reply;
        } 
        catch (const std::exception& e) {
        log_info("Error sending data to LLM: " + std::string(e.what()));

        Json::Value errorResponse;
        errorResponse["success"] = "false";
        errorResponse["error"] = "skip";
        errorResponse["message"] = "Error sending data to LLM: " + std::string(e.what());

        std::string error_reply = Json::FastWriter().write(errorResponse);
        log_info("assistant_reply: " + error_reply);

        return error_reply;
        }
    }
    

    std::string getcoord_search(const Json::Value& message, const Json::Value& object_map) {
        // Get message attributes
        std::string object_class = message.get("class", "").asString();
        std::string object_description = message.get("description", " ").asString();
        std::string error_log = "";
        
        // Check if the object class exists in the data
        Json::Value classes = data.get("classes", Json::Value(Json::arrayValue));
        bool class_found = false;
        
        for (const auto& cls : classes) {
            if (cls.asString() == object_class) {
                class_found = true;
                break;
            }
        }
        
        if (class_found) {
            Json::Value items = data.get("items", Json::Value(Json::objectValue)).get(object_class, Json::Value(Json::arrayValue));
            if (items.size() == -1) { // Note: This condition seems odd in the original code (-1)
                Json::Value item = items[0];
                Json::Value response;
                response["success"] = "true";
                response["coordinates"] = item["coordinates"];
                response["error"] = "none";
                
                std::string assistant_reply = Json::FastWriter().write(response);
                log_info("Sending response: " + assistant_reply);
                return assistant_reply;
            } else {
                // Call LLM_Search and find coordinates
                log_info("Requesting coord from AI for: " + object_class + ", with attributes: " + object_description);
                std::string assistant_reply = LLM_Search(object_class, object_description, error_log, object_map);
                
                Json::Value assistant_reply_json;
                Json::Reader reader;
                reader.parse(assistant_reply, assistant_reply_json);
                
                // Get reply attributes
                std::string success = assistant_reply_json.get("success", "false").asString();
                std::string error = assistant_reply_json.get("error", "").asString();
                std::string error_message = assistant_reply_json.get("message", "").asString();
                
                // Removed ambiguity handling loop as requested
                
                log_debug("Sending response: " + assistant_reply);
                return assistant_reply;
            }
        }
        
        // Object class not found
        log_info("Object class not found");
        Json::Value not_found_response;
        not_found_response["success"] = "false";
        not_found_response["error"] = "notExisting";
        not_found_response["message"] = "The specified object class was not found.";
        
        return Json::FastWriter().write(not_found_response);
    }



    // Simulate AI coordinate search (replace with actual AI integration)
    json simulateAICoordinateSearch(const std::string& target_object, 
                                    const std::string& object_description, 
                                    const std::string& encoded_map) {
        // This is a placeholder for actual AI coordinate search
        // In a real implementation, you'd use an AI service or custom logic
        
        // For testing purposes, find target object in the items_data
        for (const auto& [item_class, items_list] : items_data["items"].items()) {
            for (const auto& item : items_list) {
                if (item["id"] == target_object) {
                    // Object found, return coordinates
                    return {
                        {"success", true},
                        {"target_id", target_object},
                        {"coordinates", {
                            {"x", item["coordinates"]["x"]},
                            {"y", item["coordinates"]["y"]}
                        }},
                        {"message", "Found target object coordinates"}
                    };
                }
            }
        }
        
        // Target not found, return simulated coordinates
        return {
            {"success", true},
            {"target_id", target_object},
            {"coordinates", {{"x", 200}, {"y", 200}}},
            {"message", "Simulated coordinate search result (target not found in items data)"}
        };
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

        // Initialize coordinate finder with hardcoded paths
        CoordinateFinder finder(ITEMS_JSON_PATH, DATA_DIR, MAP_YAML_PATH);

        // Find coordinates for the target object
        json result = finder.findCoordinates(
            MAP_PATH,          // Map image path
            target_object,     // Target object class
            object_description, // Object description
            robot_position     // Optional robot position
        );

        // Print result
        std::cout << "Coordinate Search Result:\n" 
                  << result.dump(4) << std::endl;

        // If robot position is provided, perform pathfinding
        if (!robot_position.empty()) {
            json path_result = finder.findPathToTarget(
                MAP_PATH,        // Map image path
                robot_position,  // Robot position
                target_object    // Target object class
            );
            
            // Print path finding result
            std::cout << "Path Finding Result:\n" 
                      << path_result.dump(4) << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}