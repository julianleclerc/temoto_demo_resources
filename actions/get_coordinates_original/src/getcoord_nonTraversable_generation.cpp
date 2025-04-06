#include "get_coordinates/getcoord_nonTraversable_generation.hpp"
#include <cmath>
#include <queue>
#include <iostream>

namespace GetCoordNonTraversableGeneration {
    cv::Mat process(const cv::Mat& map_img, double resolution, const std::vector<float>& origin) {
        // Create a mutable copy of the map image
        cv::Mat modified_map = map_img.clone();
        
        // Convert to BGR if it's not already
        if (modified_map.channels() == 1) {
            cv::cvtColor(modified_map, modified_map, cv::COLOR_GRAY2BGR);
        }
        
        // Map size in pixels
        int height = modified_map.rows;
        int width = modified_map.cols;
        
        // Create a binary mask where 1=free space, 0=obstacles
        cv::Mat free_space_mask = cv::Mat::zeros(height, width, CV_8UC1);
        
        // Also create an obstacle mask to keep track of black pixels
        cv::Mat obstacle_mask = cv::Mat::zeros(height, width, CV_8UC1);
        
        // Define what colors represent obstacles (dark gray or black)
        // and what colors represent free space (light gray or white)
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                cv::Vec3b pixel = modified_map.at<cv::Vec3b>(y, x);
                
                // If pixel is very light (free space)
                if (pixel[0] >= 240 && pixel[1] >= 240 && pixel[2] >= 240) {
                    free_space_mask.at<uchar>(y, x) = 1;
                }
                // If pixel is very dark (obstacles)
                else if (pixel[0] <= 50 && pixel[1] <= 50 && pixel[2] <= 50) {
                    // Mark it in the obstacle mask
                    obstacle_mask.at<uchar>(y, x) = 1;
                }
                // For gray areas (inflation zone), we'll leave them as they are
            }
        }
        
        // Calculate origin pixel coordinates
        // The origin in the map.yaml is in world coordinates where:
        // - origin[0] is x (corresponds to columns in the image)
        // - origin[1] is y (corresponds to rows in the image)
        // - We need to convert from world coordinates to pixel coordinates
        
        // The origin point in the map is the bottom-left corner of the image
        // in world coordinates, but in the image it's the top-left corner
        // So we need to flip the y-coordinate
        
        // Convert origin from world to pixel coordinates
        int origin_x = static_cast<int>(-origin[0] / resolution);
        int origin_y = static_cast<int>(height - (-origin[1] / resolution));
        
        // Ensure the origin is within the map bounds
        origin_x = std::max(0, std::min(width - 1, origin_x));
        origin_y = std::max(0, std::min(height - 1, origin_y));
        
        std::cout << "Map origin in pixels: (" << origin_x << ", " << origin_y << ")" << std::endl;
        
        // Use the origin as the seed point for flood fill if it's free space
        cv::Point seed_point(origin_x, origin_y);
        
        // If the origin is not in free space, find a nearby free space point
        if (free_space_mask.at<uchar>(seed_point.y, seed_point.x) != 1) {
            // Try to find a free space point near the origin
            const int search_radius = 20; // pixels
            bool found_seed = false;
            
            for (int r = 1; r <= search_radius && !found_seed; ++r) {
                for (int dy = -r; dy <= r && !found_seed; ++dy) {
                    for (int dx = -r; dx <= r && !found_seed; ++dx) {
                        // Only check points at distance r
                        if (std::abs(dx) + std::abs(dy) == r) {
                            int nx = origin_x + dx;
                            int ny = origin_y + dy;
                            
                            // Check bounds
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                if (free_space_mask.at<uchar>(ny, nx) == 1) {
                                    seed_point = cv::Point(nx, ny);
                                    found_seed = true;
                                    std::cout << "Found free space seed near origin: (" << nx << ", " << ny << ")" << std::endl;
                                }
                            }
                        }
                    }
                }
            }
            
            // If still no free space was found near the origin, search the entire map
            if (!found_seed) {
                std::cout << "Could not find free space near origin, searching entire map..." << std::endl;
                for (int y = 0; y < height && !found_seed; ++y) {
                    for (int x = 0; x < width && !found_seed; ++x) {
                        if (free_space_mask.at<uchar>(y, x) == 1) {
                            seed_point = cv::Point(x, y);
                            found_seed = true;
                            std::cout << "Found free space seed elsewhere: (" << x << ", " << y << ")" << std::endl;
                        }
                    }
                }
                
                // If no free space was found at all, just return the original map
                if (!found_seed) {
                    std::cout << "No free space found in the map at all!" << std::endl;
                    return modified_map;
                }
            }
        } else {
            std::cout << "Origin is in free space, using as seed point." << std::endl;
        }
        
        // Draw a marker at the seed point for debugging
        cv::circle(modified_map, seed_point, 3, cv::Scalar(0, 255, 255), -1); // Yellow circle
        
        // Create a mask for the flood fill result
        cv::Mat reachable_mask = cv::Mat::zeros(height, width, CV_8UC1);
        
        // Perform flood fill to identify reachable areas
        std::queue<cv::Point> queue;
        queue.push(seed_point);
        reachable_mask.at<uchar>(seed_point.y, seed_point.x) = 1;
        
        // Directions for 4-connectivity
        const int dx[4] = {0, 1, 0, -1};
        const int dy[4] = {-1, 0, 1, 0};
        
        while (!queue.empty()) {
            cv::Point current = queue.front();
            queue.pop();
            
            // Check neighboring pixels
            for (int i = 0; i < 4; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];
                
                // Check if within bounds
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    // If free space and not already marked as reachable
                    if (free_space_mask.at<uchar>(ny, nx) == 1 && reachable_mask.at<uchar>(ny, nx) == 0) {
                        reachable_mask.at<uchar>(ny, nx) = 1;
                        queue.push(cv::Point(nx, ny));
                    }
                }
            }
        }
        
        // Now identify and color unreachable free space areas as RED
        // But don't color over black pixels (obstacles)
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // If it's free space but not reachable, and NOT an obstacle
                if (free_space_mask.at<uchar>(y, x) == 1 && reachable_mask.at<uchar>(y, x) == 0 
                    && obstacle_mask.at<uchar>(y, x) == 0) {
                    // Mark as non-traversable by making it bright red
                    modified_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Bright red in BGR
                }
            }
        }
        
        // Draw a marker at the origin for reference
        cv::circle(modified_map, cv::Point(origin_x, origin_y), 5, cv::Scalar(0, 255, 0), -1); // Green circle
        
        return modified_map;
    }
}