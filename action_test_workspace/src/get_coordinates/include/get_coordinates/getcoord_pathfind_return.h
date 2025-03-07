#ifndef GETCOORD_PATHFIND_RETURN_H
#define GETCOORD_PATHFIND_RETURN_H

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace GetCoordPathfindReturn {
    nlohmann::json process(const cv::Mat& object_map, 
                         double resolution, 
                         const std::vector<float>& origin, 
                         const nlohmann::json& items_data, 
                         const nlohmann::json& robot_coords, 
                         const nlohmann::json& assistant_reply);
}

#endif // GETCOORD_PATHFIND_RETURN_H