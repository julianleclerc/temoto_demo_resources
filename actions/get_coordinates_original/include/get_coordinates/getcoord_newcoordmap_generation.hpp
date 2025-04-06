#pragma once

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <utility>

namespace GetCoordNewCoordmapGeneration {
    /**
     * Process an object map to generate a new coordinates map with target location highlighted
     * 
     * @param object_map The input object map
     * @param resolution The resolution of the map in meters per pixel
     * @param origin The origin coordinates of the map [x, y, z]
     * @param result The JSON result containing coordinates
     * @param items_data The JSON data with items information
     * @return std::pair<cv::Mat, float> The generated map with markers and the orientation angle
     */
    std::pair<cv::Mat, float> process(const cv::Mat& object_map, 
                                    double resolution, 
                                    const std::vector<float>& origin, 
                                    const nlohmann::json& result,
                                    const nlohmann::json& items_data);
}