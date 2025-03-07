#ifndef GETCOORD_NEWCOORDMAP_GENERATION_H
#define GETCOORD_NEWCOORDMAP_GENERATION_H

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <utility>

namespace GetCoordNewCoordmapGeneration {
    std::pair<cv::Mat, float> process(const cv::Mat& object_map, 
                                    double resolution, 
                                    const std::vector<float>& origin, 
                                    const nlohmann::json& result,
                                    const nlohmann::json& items_data);
}

#endif // GETCOORD_NEWCOORDMAP_GENERATION_H