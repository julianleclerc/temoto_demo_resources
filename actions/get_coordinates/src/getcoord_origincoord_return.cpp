#include "get_coordinates/getcoord_origincoord_return.hpp"
#include <stdexcept>
#include <string>

namespace GetCoordOriginCoordReturn {
    nlohmann::json process(const nlohmann::json& result, 
                         double resolution, 
                         const std::vector<float>& origin, 
                         const std::pair<int, int>& map_shape, 
                         double angle) {
        try {
            // Create a copy of the input result
            nlohmann::json response = result;
            
            // Extract coordinates
            nlohmann::json& coords = response["coordinates"];
            
            // Check if coordinates exist
            if (!coords.contains("x") || !coords.contains("y")) {
                throw std::runtime_error("Invalid coordinates in result");
            }

            // Extract image coordinates
            double x_img = coords["x"];
            double y_img = coords["y"];

            // Get origin coordinates
            double origin_x = origin[0];
            double origin_y = origin[1];

            // Map dimensions
            int map_height = map_shape.first;

            // Convert image coordinates to world coordinates
            double x_world = x_img * resolution + origin_x;
            double y_world = (map_height - y_img) * resolution + origin_y;

            // Update coordinates in the response
            coords["x"] = x_world;
            coords["y"] = y_world;

            // Add angle to the response
            response["angle"] = angle;

            return response;

        } catch (const std::exception& e) {
            // Create error response
            nlohmann::json error_response = {
                {"success", false},
                {"error", "ConversionError"},
                {"message", std::string("Error converting coordinates: ") + e.what()}
            };

            return error_response;
        }
    }
}