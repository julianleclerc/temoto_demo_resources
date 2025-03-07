#include "get_coordinates/getcoord_pixelcoord_return.h"
#include <cmath>

namespace GetCoordPixelCoordReturn {
    nlohmann::json process(const nlohmann::json& items_data, 
                         double resolution, 
                         const std::vector<float>& origin, 
                         const std::pair<int, int>& object_map_shape) {
        // Extract origin coordinates and map dimensions
        double origin_x = origin[0];
        double origin_y = origin[1];
        int height = object_map_shape.first;
        int width = object_map_shape.second;

        // Create a copy of the input data to modify
        nlohmann::json items_pixel_data;
        
        // Copy classes from input data
        items_pixel_data["classes"] = items_data["classes"];
        
        // Initialize items object
        items_pixel_data["items"] = nlohmann::json::object();

        // Iterate over each class
        for (const auto& item_class : items_data["classes"]) {
            // Initialize empty list for each class
            items_pixel_data["items"][item_class.get<std::string>()] = nlohmann::json::array();

            // Iterate over each item in the class
            for (const auto& item : items_data["items"][item_class.get<std::string>()]) {
                // Get world coordinates
                double world_x = item["coordinates"]["x"];
                double world_y = item["coordinates"]["y"];

                // Convert to pixel coordinates
                int pixel_x = static_cast<int>((world_x - origin_x) / resolution);
                
                // Invert y-axis using the image height
                int pixel_y = height - static_cast<int>((world_y - origin_y) / resolution) - 1;

                // Create a new item with pixel coordinates
                nlohmann::json item_pixel = item;
                item_pixel["coordinates"] = {
                    {"x", pixel_x},
                    {"y", pixel_y}
                };

                // Append to the list
                items_pixel_data["items"][item_class.get<std::string>()].push_back(item_pixel);
            }
        }

        return items_pixel_data;
    }
}