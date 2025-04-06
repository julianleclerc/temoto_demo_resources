#pragma once

#include <nlohmann/json.hpp>
#include <vector>
#include <utility>

namespace GetCoordPixelCoordReturn {
    /**
     * Convert world coordinates to pixel coordinates for all items
     * 
     * @param items_data The JSON data with items in world coordinates
     * @param resolution The resolution of the map in meters per pixel
     * @param origin The origin coordinates of the map [x, y, z]
     * @param object_map_shape The shape of the object map [height, width]
     * @return nlohmann::json The items data with pixel coordinates
     */
    nlohmann::json process(const nlohmann::json& items_data, 
                         double resolution, 
                         const std::vector<float>& origin, 
                         const std::pair<int, int>& object_map_shape);
}