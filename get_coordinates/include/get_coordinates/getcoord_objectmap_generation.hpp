#pragma once

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace GetCoordObjectMapGeneration {
    /**
     * Process a grid map to generate an object map with items from JSON data
     * 
     * @param grid_map The input grid map
     * @param resolution The resolution of the map in meters per pixel
     * @param origin The origin coordinates of the map [x, y, z]
     * @param items_data The JSON data with items information
     * @return cv::Mat The generated object map with items drawn
     */
    cv::Mat process(const cv::Mat& grid_map, double resolution, 
                  const std::vector<float>& origin, const nlohmann::json& items_data);
}