#ifndef MAP_BUILDER_HPP
#define MAP_BUILDER_HPP

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>
#include <map>

using json = nlohmann::json;

// Simple struct to represent robot transform
struct RobotTransform {
    double x;
    double y;
    
    RobotTransform(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
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
    
    // Helper function to convert world coordinates to map pixel coordinates
    static cv::Point worldToMapCoordinates(double x, double y, const json& params, int map_height);
    
    // Helper function to inflate obstacles
    static cv::Mat inflateObstacles(const cv::Mat& map, int inflation_radius_pixels);
};

#endif // MAP_BUILDER_HPP