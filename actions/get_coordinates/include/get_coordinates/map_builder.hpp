#ifndef MAP_BUILDER_HPP
#define MAP_BUILDER_HPP

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>
#include <map>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <memory>

using json = nlohmann::json;

// Simple struct to represent robot transform
struct RobotTransform {
    double x;
    double y;
    
    RobotTransform(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
};

// A Node class for A* Pathfinding Algorithm
struct AStarNode {
    int x;
    int y;
    double g_cost; // Cost from start to current node
    double h_cost; // Heuristic cost (estimated cost from current to goal)
    double f_cost; // Total cost (g_cost + h_cost)
    std::shared_ptr<AStarNode> parent;

    AStarNode(int x, int y, double g_cost, double h_cost, std::shared_ptr<AStarNode> parent = nullptr)
        : x(x), y(y), g_cost(g_cost), h_cost(h_cost), f_cost(g_cost + h_cost), parent(parent) {}

    // Compare nodes based on f_cost for priority queue
    bool operator>(const AStarNode& other) const {
        if (f_cost == other.f_cost) {
            return h_cost > other.h_cost; // If f_costs are equal, prefer lower h_cost
        }
        return f_cost > other.f_cost;
    }
};

// Hash function for Point to use in unordered_set/map
struct PointHash {
    std::size_t operator()(const cv::Point& point) const {
        return std::hash<int>()(point.x) ^ std::hash<int>()(point.y);
    }
};

// Equals function for Point to use in unordered_set/map
struct PointEquals {
    bool operator()(const cv::Point& a, const cv::Point& b) const {
        return a.x == b.x && a.y == b.y;
    }
};

class MapBuilder {
public:
    /**
     * Builds a map for visualization with all necessary layers
     * 
     * @param base_map Original occupancy grid map (grayscale)
     * @param params Map parameters (resolution, scale_factor, etc.)
     * @param items_data JSON data with object information
     * @param robot_pos Robot position in world coordinates
     * @param path Optional path to save the final map to
     * @return The final processed map as cv::Mat
     */
    static cv::Mat BuildMap(
        const cv::Mat& base_map, 
        const json& params, 
        const json& items_data, 
        const RobotTransform& robot_pos,
        const std::string& path = "");
    
    /**
     * Displays the target coordinate on the map as a simple red dot
     * 
     * @param map_img The map image to draw on
     * @param llm_response JSON response from LLM with target coordinates
     * @param params Map parameters (not used for display, but kept for consistency)
     * @param output_path Optional path to save the visualization to
     * @return The map with the target coordinate displayed as a red dot
     */
    static cv::Mat displayTargetCoordinate(
        const cv::Mat& map_img, 
        const json& llm_response,
        const json& params,
        const std::string& output_path = "");
    
    /**
     * Helper function to convert world coordinates to map pixel coordinates
     * 
     * @param x X coordinate in world frame (meters)
     * @param y Y coordinate in world frame (meters)
     * @param params Map parameters including resolution and origin
     * @param map_height Height of the map in pixels
     * @return Pixel coordinates as cv::Point
     */
    static cv::Point worldToMapCoordinates(double x, double y, const json& params, int map_height);

    /**
     * A* pathfinding algorithm to find a path from robot to target
     * 
     * @param map The original map image
     * @param params Map parameters
     * @param items_data JSON data with object information
     * @param robot_pos Robot position in world coordinates
     * @param map_output_path Path to save the visualization map
     * @param target_id ID of the target object
     * @return The goal coordinate as cv::Point
     */
    static cv::Point coordinates_astar(const cv::Mat& map, const json& params, const json& items_data, 
        const RobotTransform& robot_pos, const std::string& map_output_path, 
        const std::string& target_id, double radius_padding = 0.3);

private:
    // Helper function to scale the map
    static double scaleMap(const cv::Mat& map_img, cv::Mat& scaled_map, const json& params);
    
    // Helper function to generate the cost map
    static cv::Mat generateCostMap(const cv::Mat& map_img, const json& params);
    
    // Helper function to mark non-traversable areas
    static cv::Mat markNonTraversableAreas(const cv::Mat& cost_map, const json& params, const RobotTransform& robot_pos);
    
    // Helper function to generate grid on the map
    static cv::Mat generateGrid(const cv::Mat& map_img, const json& params);
    
    // Helper function to draw objects on the map
    static cv::Mat drawObjectsOnMap(const cv::Mat& map, const json& items_data, const json& params);
    
    // Helper function to draw robot position on the map
    static cv::Mat drawRobotPosition(const cv::Mat& object_map, const json& params, const RobotTransform& robot_pos);
    
    // Helper function to inflate obstacles
    static cv::Mat inflateObstacles(const cv::Mat& map, int inflation_radius_pixels);
};

#endif // MAP_BUILDER_HPP