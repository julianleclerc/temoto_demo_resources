#ifndef GETCOORD_OBJECTMAP_GENERATION_H
#define GETCOORD_OBJECTMAP_GENERATION_H

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace GetCoordObjectMapGeneration {
    cv::Mat process(const cv::Mat& grid_map, double resolution, 
                  const std::vector<float>& origin, const nlohmann::json& items_data);
}

#endif // GETCOORD_OBJECTMAP_GENERATION_H