#pragma once

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace GetCoordPathfindReturn {
    /**
     * Find a path from robot position to target object
     * 
     * @param object_map The input object map
     * @param resolution The resolution of the map in meters per pixel
     * @param origin The origin coordinates of the map [x, y, z]
     * @param items_data The JSON data with items information
     * @param robot_coords The robot coordinates
     * @param assistant_reply The AI assistant reply containing target ID
     * @return nlohmann::json The result with path finding coordinates
     */
    nlohmann::json process(const cv::Mat& object_map, 
                         double resolution, 
                         const std::vector<float>& origin, 
                         const nlohmann::json& items_data, 
                         const nlohmann::json& robot_coords, 
                         const nlohmann::json& assistant_reply);
}