#pragma once

#include <nlohmann/json.hpp>
#include <vector>
#include <utility>

namespace GetCoordOriginCoordReturn {
    /**
     * Convert pixel coordinates to world coordinates
     * 
     * @param result The JSON result containing pixel coordinates
     * @param resolution The resolution of the map in meters per pixel
     * @param origin The origin coordinates of the map [x, y, z]
     * @param map_shape The shape of the map [height, width]
     * @param angle The orientation angle in degrees
     * @return nlohmann::json The result with world coordinates
     */
    nlohmann::json process(const nlohmann::json& result, 
                         double resolution, 
                         const std::vector<float>& origin, 
                         const std::pair<int, int>& map_shape, 
                         double angle);
}