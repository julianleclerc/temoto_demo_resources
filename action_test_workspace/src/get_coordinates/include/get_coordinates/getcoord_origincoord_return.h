#ifndef GETCOORD_ORIGINCOORD_RETURN_H
#define GETCOORD_ORIGINCOORD_RETURN_H

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <vector>

namespace GetCoordOriginCoordReturn {
    nlohmann::json process(const nlohmann::json& result, 
                         double resolution, 
                         const std::vector<float>& origin, 
                         const std::pair<int, int>& map_shape, 
                         double angle);
}

#endif // GETCOORD_ORIGINCOORD_RETURN_H