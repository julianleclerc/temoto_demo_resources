#ifndef GETCOORD_PIXELCOORD_RETURN_H
#define GETCOORD_PIXELCOORD_RETURN_H

#include <nlohmann/json.hpp>
#include <vector>
#include <utility>

namespace GetCoordPixelCoordReturn {
    nlohmann::json process(const nlohmann::json& items_data, 
                         double resolution, 
                         const std::vector<float>& origin, 
                         const std::pair<int, int>& object_map_shape);
}

#endif // GETCOORD_PIXELCOORD_RETURN_H