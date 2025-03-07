#include "get_coordinates/getcoord_newcoordmap_generation.h"
#include <cmath>

namespace GetCoordNewCoordmapGeneration {
    std::pair<cv::Mat, float> process(const cv::Mat& object_map, 
                                     double resolution, 
                                     const std::vector<float>& origin, 
                                     const nlohmann::json& result,
                                     const nlohmann::json& items_data) {
        // Create a copy of the object_map
        cv::Mat new_coords_map = object_map.clone();
        
        // Ensure we're working with a color image
        if (new_coords_map.channels() == 1) {
            cv::cvtColor(new_coords_map, new_coords_map, cv::COLOR_GRAY2BGR);
        }

        // Extract coordinates from the result
        int target_x = result["coordinates"]["x"];
        int target_y = result["coordinates"]["y"];
        
        // Map dimensions
        int height = new_coords_map.rows;
        int width = new_coords_map.cols;
        
        // Define a circular marker to highlight the target location
        int marker_radius = 10;
        cv::Scalar inner_color(0, 255, 0);  // Green center
        cv::Scalar outer_color(0, 0, 255);  // Red outline
        float angle_deg = 0.0f;  // Default angle
        
        // Draw the marker
        cv::circle(new_coords_map, cv::Point(target_x, target_y), marker_radius + 2, outer_color, 2);
        cv::circle(new_coords_map, cv::Point(target_x, target_y), marker_radius, inner_color, -1);
        
        // Draw crosshair
        cv::line(new_coords_map, 
                cv::Point(target_x - marker_radius, target_y), 
                cv::Point(target_x + marker_radius, target_y), 
                outer_color, 1);
        cv::line(new_coords_map, 
                cv::Point(target_x, target_y - marker_radius), 
                cv::Point(target_x, target_y + marker_radius), 
                outer_color, 1);
        
        // If we had more information about the target object, we could calculate 
        // an orientation angle here. For now, we'll just use 0.0
        
        return {new_coords_map, angle_deg};
    }
}