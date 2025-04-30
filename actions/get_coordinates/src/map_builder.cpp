#include "get_coordinates/map_builder.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <unordered_set>

// Define color constants for visualization
const cv::Scalar COLOR_OBSTACLE(0, 0, 0);       // Black for obstacles
const cv::Scalar COLOR_FREE(255, 255, 255);     // White for free space
const cv::Scalar COLOR_ROBOT(0, 0, 255);        // Blue for robot
const cv::Scalar COLOR_GRID(0, 0, 255);         // Red for grid lines

cv::Mat MapBuilder::BuildMap(
    const cv::Mat& base_map, 
    const json& params, 
    const json& items_data, 
    const RobotTransform& robot_pos,
    const std::string& path) 
{
    // Create debug directory if it doesn't exist
    std::string debug_dir = path;
    try {
        if (!std::filesystem::exists(debug_dir)) {
            std::filesystem::create_directories(debug_dir);
            std::cout << "Created debug directory: " << debug_dir << std::endl;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating debug directory: " << e.what() << std::endl;
    }
    
    // Log the params for debugging
    std::cout << "Map parameters:" << std::endl;
    std::cout << "  Scale Factor: " << params.value("scale_factor", 1.0) << std::endl;
    std::cout << "  Resolution: " << params.value("resolution", 0.05) << std::endl;
    std::cout << "  Inflation Radius: " << params.value("inflation_radius_m", 0.5) << " meters" << std::endl;
    std::cout << "  Grid Scale: " << params.value("grid_scale", 20.0) << " pixels" << std::endl;
    
    // Save the original map
    try {
        cv::imwrite((std::filesystem::path(debug_dir) / "01_base_map.pgm").string(), base_map);
        std::cout << "Saved base map to: " << (std::filesystem::path(debug_dir) / "01_base_map.pgm").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving base map: " << e.what() << std::endl;
    }
    
    // Step 1: Scale the map
    cv::Mat scaled_map;
    double new_resolution = scaleMap(base_map, scaled_map, params);
    
    // Save the scaled map
    try {
        cv::imwrite((std::filesystem::path(debug_dir) / "02_scaled_map.pgm").string(), scaled_map);
        std::cout << "Saved scaled map to: " << (std::filesystem::path(debug_dir) / "02_scaled_map.pgm").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving scaled map: " << e.what() << std::endl;
    }
    
    // Step 2: Generate cost map (inflate obstacles for safety)
    cv::Mat cost_map = generateCostMap(scaled_map, params);
    
    // Save the cost map
    try {
        cv::imwrite((std::filesystem::path(debug_dir) / "03_cost_map.pgm").string(), cost_map);
        std::cout << "Saved cost map to: " << (std::filesystem::path(debug_dir) / "03_cost_map.pgm").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving cost map: " << e.what() << std::endl;
    }
    
    // Step 3: Colorize the cost map
    cv::Mat cost_map_color;
    cv::cvtColor(cost_map, cost_map_color, cv::COLOR_GRAY2BGR);
    
    // Save the colorized cost map
    try {
        cv::imwrite((std::filesystem::path(debug_dir) / "04_cost_map_color.png").string(), cost_map_color);
        std::cout << "Saved colorized cost map to: " << (std::filesystem::path(debug_dir) / "04_cost_map_color.png").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving colorized cost map: " << e.what() << std::endl;
    }
    
    // Step 4: Mark non-traversable areas
    cv::Mat non_traversable_map = markNonTraversableAreas(cost_map_color, params, robot_pos);
    
    // Save the non-traversable map
    try {
        cv::imwrite((std::filesystem::path(debug_dir) / "05_non_traversable_map.png").string(), non_traversable_map);
        std::cout << "Saved non-traversable map to: " << (std::filesystem::path(debug_dir) / "05_non_traversable_map.png").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving non-traversable map: " << e.what() << std::endl;
    }
    
    // Step 5: Draw grid on map
    cv::Mat grid_map = generateGrid(non_traversable_map, params);
    
    // Save the grid map
    try {
        cv::imwrite((std::filesystem::path(debug_dir) / "06_grid_map.png").string(), grid_map);
        std::cout << "Saved grid map to: " << (std::filesystem::path(debug_dir) / "06_grid_map.png").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving grid map: " << e.what() << std::endl;
    }
    
    // Step 6: Draw objects on the map
    cv::Mat object_map = drawObjectsOnMap(grid_map, items_data, params);
    
    // Save the object map
    try {
        cv::imwrite((std::filesystem::path(debug_dir) / "07_object_map.png").string(), object_map);
        std::cout << "Saved object map to: " << (std::filesystem::path(debug_dir) / "07_object_map.png").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving object map: " << e.what() << std::endl;
    }
    
    // Step 7: Draw the robot position
    cv::Mat robot_map = drawRobotPosition(object_map, params, robot_pos);
    
    // Save the robot map
    try {
        cv::imwrite((std::filesystem::path(debug_dir) / "08_robot_map.png").string(), robot_map);
        std::cout << "Saved robot map to: " << (std::filesystem::path(debug_dir) / "08_robot_map.png").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving robot map: " << e.what() << std::endl;
    }
    
    // Save the final map if path is provided
    if (!path.empty()) {
        try {
            cv::imwrite((std::filesystem::path(debug_dir) / "final_robot_map.png").string(), robot_map);
            std::cout << "Saved final robot map to: " << (std::filesystem::path(debug_dir) / "final_robot_map.png").string() << std::endl;
        } catch (const cv::Exception& e) {
            std::cerr << "Error saving final robot map: " << e.what() << std::endl;
        }
    }
    
    return robot_map;
}

double MapBuilder::scaleMap(const cv::Mat& map_img, cv::Mat& scaled_map, const json& params)
{
    // Get scale factor from params
    double scale_factor = params.value("scale_factor", 1.0);
    double resolution = params.value("resolution", 0.05);
    
    std::cout << "Scaling map with factor: " << scale_factor << std::endl;
    std::cout << "Original resolution: " << resolution << " meters/pixel" << std::endl;
    
    // Get original dimensions
    int original_height = map_img.rows;
    int original_width = map_img.cols;
    
    std::cout << "Original map dimensions: " << original_width << "x" << original_height << " pixels" << std::endl;
    
    // Calculate new dimensions
    int new_width = std::max(1, static_cast<int>(original_width * scale_factor));
    int new_height = std::max(1, static_cast<int>(original_height * scale_factor));
    
    std::cout << "New map dimensions: " << new_width << "x" << new_height << " pixels" << std::endl;
    
    // Choose interpolation method based on scale factor
    int interpolation = (scale_factor > 1.0) ? cv::INTER_NEAREST : cv::INTER_AREA;
    
    // If scale_factor is 1.0, just copy the map
    if (std::abs(scale_factor - 1.0) < 0.001) {
        scaled_map = map_img.clone();
        return resolution;
    }
    
    // Perform the scaling
    cv::resize(map_img, scaled_map, cv::Size(new_width, new_height), 0, 0, interpolation);
    
    // Update resolution
    double new_resolution = resolution / scale_factor;
    std::cout << "New resolution: " << new_resolution << " meters/pixel" << std::endl;
    
    // Determine the size for the square image
    int max_dim = std::max(new_width, new_height);
    
    // Initialize a square image with the same depth as the input
    cv::Mat square_img;
    if (map_img.channels() == 3) {
        square_img = cv::Mat::zeros(max_dim, max_dim, CV_8UC3);
    } else {
        square_img = cv::Mat::zeros(max_dim, max_dim, CV_8UC1);
    }
    
    // Copy the scaled image to the square canvas
    cv::Rect roi(0, 0, new_width, new_height);
    scaled_map.copyTo(square_img(roi));
    scaled_map = square_img;
    
    return new_resolution;
}

cv::Mat MapBuilder::generateCostMap(const cv::Mat& map_img, const json& params)
{
    double resolution = params.value("resolution", 0.05);
    double scale_factor = params.value("scale_factor", 1.0);
    double scaled_resolution = resolution / scale_factor;
    double inflation_radius_m = params.value("inflation_radius_m", 0.5);
    
    std::cout << "Generating cost map with inflation radius: " << inflation_radius_m << " meters" << std::endl;
    
    // Convert inflation radius from meters to pixels (accounting for scale)
    int inflation_radius_px = static_cast<int>(std::ceil(inflation_radius_m / scaled_resolution));
    std::cout << "Inflation radius: " << inflation_radius_px << " pixels" << std::endl;
    
    // Create binary map - assuming 0 (black) for obstacles, 255 (white) for free space
    cv::Mat binary_map;
    cv::threshold(map_img, binary_map, 0, 255, cv::THRESH_BINARY);
    
    // Initialize the cost map (white background)
    cv::Mat cost_map = cv::Mat::ones(map_img.size(), CV_8UC1) * 255;
    
    // Identify obstacles
    cv::Mat obstacles = (binary_map == 0);
    
    // Create a mask for distance transform
    cv::Mat distance_mask;
    binary_map.copyTo(distance_mask);
    
    // Compute the distance transform
    cv::Mat dist_transform;
    cv::distanceTransform(distance_mask, dist_transform, cv::DIST_L2, 5);
    
    // Convert distances from pixels to meters
    cv::Mat dist_transform_m = dist_transform * scaled_resolution;
    
    // Define the inflation zone (excluding obstacles)
    cv::Mat inflation_zone;
    cv::compare(dist_transform_m, cv::Scalar(0), inflation_zone, cv::CMP_GT);
    cv::Mat inflation_zone_limit;
    cv::compare(dist_transform_m, cv::Scalar(inflation_radius_m), inflation_zone_limit, cv::CMP_LE);
    cv::Mat inflation_mask;
    cv::bitwise_and(inflation_zone, inflation_zone_limit, inflation_mask);
    
    // Assign BLACK (0) to the inflation zone in the cost map instead of gray (70)
    cost_map.setTo(cv::Scalar(0), inflation_mask);
    
    // Assign black (0) to obstacle cells
    cost_map.setTo(cv::Scalar(0), obstacles);
    
    return cost_map;
}

cv::Mat MapBuilder::markNonTraversableAreas(const cv::Mat& cost_map, const json& params, const RobotTransform& robot_pos)
{
    // Get map size in pixels
    int height = cost_map.rows;
    int width = cost_map.cols;
    
    std::cout << "Marking non-traversable areas on map of size: " << width << "x" << height << " pixels" << std::endl;
    
    // Create a copy of the cost map
    cv::Mat non_traversable_map = cost_map.clone();
    
    // Convert robot's position to pixel coordinates
    double resolution = params.value("resolution", 0.05);
    double scale_factor = params.value("scale_factor", 1.0);
    double scaled_resolution = resolution / scale_factor;
    auto origin = params.value("origin", std::vector<double>{0.0, 0.0, 0.0});
    double origin_x = origin.size() > 0 ? origin[0] : 0.0;
    double origin_y = origin.size() > 1 ? origin[1] : 0.0;
    
    // Get robot position in pixel coordinates
    int robot_pixel_x = static_cast<int>((robot_pos.x - origin_x) / scaled_resolution);
    int robot_pixel_y = height - static_cast<int>((robot_pos.y - origin_y) / scaled_resolution) - 1;
    
    std::cout << "Robot world position: (" << robot_pos.x << ", " << robot_pos.y << ") meters" << std::endl;
    std::cout << "Robot pixel position: (" << robot_pixel_x << ", " << robot_pixel_y << ") px" << std::endl;
    
    // Ensure pixel coordinates are within image bounds
    robot_pixel_x = std::min(std::max(0, robot_pixel_x), width - 1);
    robot_pixel_y = std::min(std::max(0, robot_pixel_y), height - 1);
    
    std::cout << "Adjusted robot pixel position: (" << robot_pixel_x << ", " << robot_pixel_y << ") px" << std::endl;
    
    // Create free space mask - pixels with values close to white (254 or 255)
    cv::Mat free_space_mask;
    cv::inRange(cost_map, cv::Scalar(254, 254, 254), cv::Scalar(255, 255, 255), free_space_mask);
    
    // Prepare image for flood fill
    cv::Mat flood_fill_mask = cv::Mat::zeros(height + 2, width + 2, CV_8UC1);
    cv::Mat flood_fill_img = free_space_mask.clone();
    
    // Perform flood fill starting from robot's position
    cv::Point seed_point(robot_pixel_x, robot_pixel_y);
    cv::floodFill(flood_fill_img, flood_fill_mask, seed_point, 2);
    
    // Create mask of unreachable free spaces
    cv::Mat unreachable_mask = (flood_fill_img != 2) & (free_space_mask > 0);
    
    // Set unreachable free spaces to black in the map
    non_traversable_map.setTo(cv::Scalar(0, 0, 0), unreachable_mask);
    
    return non_traversable_map;
}

cv::Mat MapBuilder::generateGrid(const cv::Mat& map_img, const json& params)
{
    // Get grid scale from params (pixels per grid cell)
    int grid_scale = static_cast<int>(params.value("grid_scale", 20));
    
    std::cout << "Generating grid with scale: " << grid_scale << " pixels" << std::endl;
    
    // Create a copy of the map to draw the grid on
    cv::Mat grid_map = map_img.clone();
    
    // Ensure we're working with a BGR image
    if (grid_map.channels() == 1) {
        cv::cvtColor(grid_map, grid_map, cv::COLOR_GRAY2BGR);
    }
    
    // Extract dimensions
    int height = grid_map.rows;
    int width = grid_map.cols;
    
    // Create a grayscale version for the mask
    cv::Mat map_gray;
    if (map_img.channels() == 3) {
        cv::cvtColor(map_img, map_gray, cv::COLOR_BGR2GRAY);
    } else {
        map_gray = map_img.clone();
    }
    
    // Create binary mask of free space (white)
    cv::Mat binary_mask;
    cv::threshold(map_gray, binary_mask, 254, 255, cv::THRESH_BINARY);
    
    // Generate x positions for vertical grid lines
    for (int x = 0; x < width; x += grid_scale) {
        for (int y = 0; y < height; y++) {
            // Only draw grid lines on free space (white areas)
            if (binary_mask.at<uchar>(y, x) == 255) {
                grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(COLOR_GRID[0], COLOR_GRID[1], COLOR_GRID[2]);
            } else {
                grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(COLOR_GRID[0], COLOR_GRID[1], COLOR_GRID[2]-150);
            }
        }
    }
    
    // Generate y positions for horizontal grid lines
    for (int y = 0; y < height; y += grid_scale) {
        for (int x = 0; x < width; x++) {
            // Only draw grid lines on free space (white areas)
            if (binary_mask.at<uchar>(y, x) == 255) {
                grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(COLOR_GRID[0], COLOR_GRID[1], COLOR_GRID[2]);
            } else {
                grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(COLOR_GRID[0], COLOR_GRID[1], COLOR_GRID[2]-150);
            }
        }
    }
    
    return grid_map;
}

cv::Point MapBuilder::worldToMapCoordinates(double x, double y, const json& params, int map_height) {
    // Get map parameters
    double resolution = params.value("resolution", 0.05);  // meters per pixel
    double scale_factor = params.value("scale_factor", 1.0);  // Apply scale factor
    double scaled_resolution = resolution / scale_factor;
    auto origin = params.value("origin", std::vector<double>{0.0, 0.0, 0.0});
    double origin_x = origin.size() > 0 ? origin[0] : 0.0;
    double origin_y = origin.size() > 1 ? origin[1] : 0.0;
    
    // Convert from world coordinates to pixel coordinates
    int pixel_x = static_cast<int>((x - origin_x) / scaled_resolution);
    
    // In image coordinates, y increases downward
    int pixel_y = map_height - static_cast<int>((y - origin_y) / scaled_resolution) - 1;
    
    std::cout << "Converting world coordinates (" << x << ", " << y << ") to pixel coordinates (" 
              << pixel_x << ", " << pixel_y << ")" << std::endl;
    
    return cv::Point(pixel_x, pixel_y);
}

cv::Mat MapBuilder::drawObjectsOnMap(const cv::Mat& map, const json& items_data, const json& params) {
    cv::Mat result = map.clone();
    int map_height = result.rows;
    
    std::cout << "Drawing objects on map..." << std::endl;
    
    // Create a random number generator for colors
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> color_dist(50, 200);  // Range 50-200 for R,G,B values
    
    // Create a rectangle representing the full image
    cv::Rect image_rect(0, 0, result.cols, result.rows);
    
    // Map to store colors by item ID (to ensure consistent colors)
    std::map<std::string, cv::Scalar> id_colors;
    
    // Store label data for drawing after all bboxes
    struct LabelInfo {
        cv::Point position;
        std::string text;
        cv::Scalar color;
    };
    std::vector<LabelInfo> labels;
    
    // FIRST PASS: Draw all bounding boxes
    // Iterate through all items in the simplified format
    for (auto& [item_id, item] : items_data.items()) {
        try {
            std::cout << "Drawing item: " << item_id << std::endl;
            
            // Get coordinates and dimensions
            const json& coordinates = item["coordinates"];
            const json& dimensions = item["dimensions"];
            
            double x = coordinates["x"];
            double y = coordinates["y"];
            double width = dimensions["width"];
            double height = dimensions["height"];
            
            std::cout << "  World position: (" << x << ", " << y << ") meters" << std::endl;
            std::cout << "  Dimensions: " << width << "x" << height << " meters" << std::endl;
            
            // Convert world coordinates to pixel coordinates
            cv::Point center = worldToMapCoordinates(x, y, params, map_height);
            std::cout << "  Pixel position: (" << center.x << ", " << center.y << ") px" << std::endl;
            
            // Convert dimensions from meters to pixels
            double resolution = params.value("resolution", 0.05);
            double scale_factor = params.value("scale_factor", 1.0);
            double scaled_resolution = resolution / scale_factor;
            int pixel_width = static_cast<int>(width / scaled_resolution);
            int pixel_height = static_cast<int>(height / scaled_resolution);
            
            std::cout << "  Pixel dimensions: " << pixel_width << "x" << pixel_height << " px" << std::endl;
            
            // Define rectangle padding (in pixels)
            int rectangle_padding = 5;
            
            // Calculate rectangle corners
            cv::Point top_left(
                center.x - pixel_width / 2 - rectangle_padding,
                center.y - pixel_height / 2 - rectangle_padding
            );
            
            cv::Point bottom_right(
                center.x + pixel_width / 2 + rectangle_padding,
                center.y + pixel_height / 2 + rectangle_padding
            );
            
            // Ensure coordinates are within map bounds
            cv::Rect object_rect(top_left, bottom_right);
            object_rect &= image_rect;
            
            if (object_rect.width <= 0 || object_rect.height <= 0) {
                std::cout << "  Warning: Object rectangle is outside map bounds, skipping" << std::endl;
                continue;  // Skip if rectangle is outside bounds
            }
            
            // Generate or retrieve color for this item ID
            cv::Scalar color;
            color = cv::Scalar(
                color_dist(gen),  // Blue
                color_dist(gen),  // Green
                color_dist(gen)   // Red
            );
            
            // Draw filled rectangle with transparency
            cv::Mat overlay;
            result.copyTo(overlay);
            cv::rectangle(overlay, object_rect, color, -1);  // -1 for filled rectangle
            
            // Add the filled rectangle with transparency
            double alpha = 0.3;  // 30% opacity
            cv::addWeighted(overlay, alpha, result, 1 - alpha, 0, result);
            
            // Draw rectangle border
            cv::rectangle(result, object_rect, color, 2);  // 2 pixels border width
            
            // Calculate the center of the object bounding box
            cv::Point center_of_box = cv::Point(
                object_rect.x + object_rect.width / 2,
                object_rect.y + object_rect.height / 2
            );
            
            // Store label information for second pass
            // Keep original item_id as label text (no abbreviation)
            std::string label_text = item_id;
            
            // Save label position and text
            labels.push_back({center_of_box, label_text, color});
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing object: " << e.what() << std::endl;
        }
    }
    
    // SECOND PASS: Draw all labels on top
    int font_face = cv::FONT_HERSHEY_PLAIN;  // PLAIN font is more compact
    double font_scale = 0.6;  // Extremely small font
    int thickness = 1;  // Thinnest lines
    
    for (const auto& label : labels) {
        // Get text size to center it properly
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label.text, font_face, font_scale, thickness, &baseline);
        
        // Calculate position to center text in object
        cv::Point text_org(
            label.position.x - text_size.width / 2,
            label.position.y + text_size.height / 2
        );
        
        // Draw white text outline for readability (thin outline)
        cv::putText(result, label.text, 
                  text_org, 
                  font_face, font_scale, 
                  cv::Scalar(240, 240, 240), thickness + 1, cv::LINE_8);
        
        // Draw black text on top
        cv::putText(result, label.text, 
                  text_org, 
                  font_face, font_scale, 
                  cv::Scalar(0, 0, 0), thickness, cv::LINE_8);
    }
    
    return result;
}

cv::Mat MapBuilder::drawRobotPosition(const cv::Mat& object_map, const json& params, const RobotTransform& robot_pos) {
    // Create a copy of the object map
    cv::Mat robot_map = object_map.clone();
    int map_height = robot_map.rows;
    
    std::cout << "Drawing robot position: (" << robot_pos.x << ", " << robot_pos.y << ") meters" << std::endl;
    
    // Get robot's position in map coordinates
    cv::Point robot_point = worldToMapCoordinates(robot_pos.x, robot_pos.y, params, map_height);
    std::cout << "Robot pixel position: (" << robot_point.x << ", " << robot_point.y << ") px" << std::endl;
    
    // Draw the robot position as a filled circle
    int circle_radius = 5; 
    
    // Draw a larger border around the circle for better visibility
    cv::circle(robot_map, robot_point, circle_radius + 2, cv::Scalar(0, 0, 0), -1);
    
    // Draw the robot circle (blue)
    cv::circle(robot_map, robot_point, circle_radius, cv::Scalar(255, 0, 0), -1);
    
    // Add a white dot in the center for better visibility
    cv::circle(robot_map, robot_point, 3, cv::Scalar(240, 240, 240), -1);
        
    return robot_map;
}

cv::Mat MapBuilder::inflateObstacles(const cv::Mat& map, int inflation_radius_pixels) {
    // Create element for dilation
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * inflation_radius_pixels + 1, 2 * inflation_radius_pixels + 1),
        cv::Point(inflation_radius_pixels, inflation_radius_pixels)
    );
    
    // Perform dilation
    cv::Mat inflated_map;
    cv::dilate(map, inflated_map, element);
    
    return inflated_map;
}

cv::Mat MapBuilder::displayTargetCoordinate(
    const cv::Mat& map_img, 
    const json& llm_response,
    const json& params,
    const std::string& output_path)
{
    // Create a copy of the input map
    cv::Mat result = map_img.clone();
    
    std::cout << "Displaying target coordinate on map..." << std::endl;
    
    // Check if the LLM response has the expected structure
    if (!llm_response.contains("coordinates") || 
        !llm_response["coordinates"].contains("x") || 
        !llm_response["coordinates"].contains("y")) {
        std::cerr << "Error: LLM response does not contain valid coordinates" << std::endl;
        return result;
    }
    
    // Get target ID
    std::string target_id = llm_response.contains("target_id") ? 
                            llm_response["target_id"].get<std::string>() : "unknown";
    
    // Get pixel coordinates from LLM response
    int pixel_x = llm_response["coordinates"]["x"];
    int pixel_y = llm_response["coordinates"]["y"];
    
    std::cout << "Target pixel coordinates: (" << pixel_x << ", " << pixel_y << ")" << std::endl;
    
    // Ensure coordinates are within image bounds
    cv::Rect image_rect(0, 0, result.cols, result.rows);
    if (!image_rect.contains(cv::Point(pixel_x, pixel_y))) {
        std::cout << "Warning: Target coordinates are outside map bounds, adjusting..." << std::endl;
        pixel_x = std::min(std::max(0, pixel_x), result.cols - 1);
        pixel_y = std::min(std::max(0, pixel_y), result.rows - 1);
        std::cout << "Adjusted target coordinates: (" << pixel_x << ", " << pixel_y << ")" << std::endl;
    }
    
    cv::Point target_point(pixel_x, pixel_y);
    
    // Draw a simple red dot at the target location
    int radius = 5;
    cv::circle(result, target_point, radius, cv::Scalar(0, 0, 255), -1); // Red filled circle in BGR
    
    // Save the result if path is provided
    if (!output_path.empty()) {
        try {
            cv::imwrite(output_path, result);
            std::cout << "Saved target coordinate visualization to: " << output_path << std::endl;
        } catch (const cv::Exception& e) {
            std::cerr << "Error saving target visualization: " << e.what() << std::endl;
        }
    }
    
    return result;
}

cv::Point MapBuilder::coordinates_astar(
    const cv::Mat& map, 
    const json& params, 
    const json& items_data, 
    const RobotTransform& robot_pos, 
    const std::string& map_output_path, 
    const std::string& target_id,
    double radius_padding) {
    
    std::cout << "Starting A* pathfinding algorithm with circular radius padding..." << std::endl;
    
    // Define wall padding constant in meters
    const double WALL_PADDING_METERS = 0.1; // Can be adjusted as needed
    std::cout << "Using wall padding: " << WALL_PADDING_METERS << " meters" << std::endl;
    
    // Threshold map to black and white
    cv::Mat binary_map;
    cv::threshold(map, binary_map, 200, 255, cv::THRESH_BINARY);
    
    // Convert to color map for visualization
    cv::Mat color_map;
    cv::cvtColor(binary_map, color_map, cv::COLOR_GRAY2BGR);

    // Get parameters with proper type conversion
    float scale_factor = params["scale_factor"].get<float>();
    float grid_scale = params["grid_scale"].get<float>();
    float resolution = params["resolution"].get<float>();
    double origin_x = params["origin"][0].get<double>();
    double origin_y = params["origin"][1].get<double>();
    
    // Calculate wall padding in pixels
    int wall_padding_px = static_cast<int>(WALL_PADDING_METERS / resolution);
    std::cout << "Wall padding in pixels: " << wall_padding_px << " px" << std::endl;
    
    // Apply padding to walls/obstacles (dilate black areas)
    cv::Mat padded_binary_map = binary_map.clone();
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                              cv::Size(2 * wall_padding_px + 1, 2 * wall_padding_px + 1));
    cv::erode(padded_binary_map, padded_binary_map, kernel);
    
    // Save padded map for debugging - FIXED PATH HANDLING
    try {
        cv::imwrite((std::filesystem::path(map_output_path) / "astar_padded_walls.png").string(), padded_binary_map);
        std::cout << "Saved padded wall map to: " << (std::filesystem::path(map_output_path) / "astar_padded_walls.png").string() << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving padded wall map: " << e.what() << std::endl;
    }

    // Convert robot position to pixel coordinates
    int robot_x = static_cast<int>((robot_pos.x - origin_x) / resolution);
    int robot_y = map.rows - static_cast<int>((robot_pos.y - origin_y) / resolution);

    // Process all objects except the target
    for (auto& [id, item] : items_data.items()) {
        if (id == target_id) continue;
        
        float x = item["coordinates"]["x"].get<float>();
        float y = item["coordinates"]["y"].get<float>();
        float width = item["dimensions"]["width"].get<float>();
        float height = item["dimensions"]["height"].get<float>();
        
        // Convert to pixel coordinates
        int px = static_cast<int>((x - origin_x) / resolution);
        int py = map.rows - static_cast<int>((y - origin_y) / resolution) - 1;
        int pw = std::max(1, static_cast<int>(width / resolution));
        int ph = std::max(1, static_cast<int>(height / resolution));
        
        // Add padding to objects too
        pw += 2 * wall_padding_px;
        ph += 2 * wall_padding_px;
        
        cv::rectangle(padded_binary_map, cv::Rect(px - pw/2, py - ph/2, pw, ph), cv::Scalar(0), cv::FILLED);
        cv::rectangle(color_map, cv::Rect(px - pw/2, py - ph/2, pw, ph), cv::Scalar(0, 0, 0), cv::FILLED);
    }

    // Target object variables
    int target_px = 0;
    int target_py = 0;
    int target_radius_px = 0;
    float target_x = 0;
    float target_y = 0;

    // Mark target object (red) - using circle instead of rectangle
    if (items_data.contains(target_id)) {
        auto& target = items_data[target_id];
        target_x = target["coordinates"]["x"].get<float>();
        target_y = target["coordinates"]["y"].get<float>();
        float width = target["dimensions"]["width"].get<float>();
        float height = target["dimensions"]["height"].get<float>();
        
        // Convert to pixel coordinates
        target_px = static_cast<int>((target_x - origin_x) / resolution);
        target_py = map.rows - static_cast<int>((target_y - origin_y) / resolution) - 1;
        
        // Calculate radius based on the larger dimension + padding
        float max_dimension = std::max(width, height) / 2.0f;
        float total_radius = max_dimension + radius_padding; // Add padding in meters
        target_radius_px = static_cast<int>(total_radius / resolution);
        
        std::cout << "Target center: (" << target_px << ", " << target_py << ") px" << std::endl;
        std::cout << "Target radius: " << target_radius_px << " px (includes " 
                  << static_cast<int>(radius_padding / resolution) << " px padding)" << std::endl;
        
        // Draw circle (red for target)
        cv::circle(color_map, cv::Point(target_px, target_py), target_radius_px, cv::Scalar(0, 0, 255), 2);
        
        // Save initialization visualization - FIXED PATH HANDLING
        try {
            cv::imwrite((std::filesystem::path(map_output_path) / "astar_init.png").string(), color_map);
            std::cout << "Saved initialization map to: " << (std::filesystem::path(map_output_path) / "astar_init.png").string() << std::endl;
        } catch (const cv::Exception& e) {
            std::cerr << "Error saving initialization map: " << e.what() << std::endl;
        }

        // A* algorithm implementation
        struct Node {
            cv::Point point;
            float g_cost;
            float h_cost;
            float f_cost() const { return g_cost + h_cost; }
            Node* parent;
            bool operator<(const Node& other) const {
                return f_cost() > other.f_cost();  // For min-heap
            }
        };

        std::priority_queue<Node> open_set;
        std::unordered_map<int, std::unordered_map<int, Node>> all_nodes;

        Node start_node;
        start_node.point = cv::Point(robot_x, robot_y);
        start_node.g_cost = 0;
        start_node.h_cost = std::sqrt(std::pow(target_px - robot_x, 2) + std::pow(target_py - robot_y, 2));
        start_node.parent = nullptr;
        open_set.push(start_node);
        all_nodes[robot_x][robot_y] = start_node;

        const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

        bool found = false;
        Node final_node;

        while (!open_set.empty()) {
            Node current = open_set.top();
            open_set.pop();

            // Calculate distance to target center
            float dist_to_target = std::sqrt(std::pow(current.point.x - target_px, 2) + 
                                            std::pow(current.point.y - target_py, 2));
            
            // Check if we've reached the circular boundary of the target (with padding)
            // We want to be close to but outside the circle
            if (dist_to_target <= target_radius_px + 1 && dist_to_target >= target_radius_px - 1) {
                final_node = current;
                found = true;
                std::cout << "Found path to target boundary at point (" << current.point.x 
                          << ", " << current.point.y << ")" << std::endl;
                std::cout << "Distance to target center: " << dist_to_target << " px" << std::endl;
                break;
            }

            for (int i = 0; i < 8; i++) {
                int nx = current.point.x + dx[i];
                int ny = current.point.y + dy[i];

                if (nx < 0 || ny < 0 || nx >= padded_binary_map.cols || ny >= padded_binary_map.rows)
                    continue;

                // Check if this is a valid (white) pixel in the padded map
                if (padded_binary_map.at<uchar>(ny, nx) != 255)
                    continue;

                // Calculate distance from this point to target center
                float point_to_target = std::sqrt(std::pow(nx - target_px, 2) + std::pow(ny - target_py, 2));
                
                // Skip if the point is inside the target's circle
                if (point_to_target < target_radius_px - 1)
                    continue;

                float new_g = current.g_cost + (i < 4 ? 1.0f : 1.414f);
                
                // Use the difference between the point's distance to target and the desired radius
                // as a heuristic to guide A* toward the circular boundary
                float circle_distance = std::abs(point_to_target - target_radius_px);
                float new_h = circle_distance * 0.5f; // Weight for the circle distance

                if (all_nodes.count(nx) && all_nodes[nx].count(ny)) {
                    if (all_nodes[nx][ny].g_cost <= new_g)
                        continue;
                }

                Node neighbor;
                neighbor.point = cv::Point(nx, ny);
                neighbor.g_cost = new_g;
                neighbor.h_cost = new_h;
                neighbor.parent = &all_nodes[current.point.x][current.point.y];

                open_set.push(neighbor);
                all_nodes[nx][ny] = neighbor;
            }
        }

        if (found) {
            std::vector<cv::Point> path;
            Node* current = &final_node;
            while (current != nullptr) {
                path.push_back(current->point);
                current = current->parent;
            }

            // Draw the path on the visualization map
            for (size_t i = 0; i < path.size() - 1; i++) {
                cv::line(color_map, path[i], path[i + 1], cv::Scalar(0, 255, 0), 2);
            }
            
            // Mark the target point with a different color
            cv::circle(color_map, path[0], 5, cv::Scalar(255, 0, 255), -1);

            // Save final visualization - FIXED PATH HANDLING
            try {
                cv::imwrite((std::filesystem::path(map_output_path) / "astar_final.png").string(), color_map);
                std::cout << "Saved path visualization to: " << (std::filesystem::path(map_output_path) / "astar_final.png").string() << std::endl;
            } catch (const cv::Exception& e) {
                std::cerr << "Error saving final path map: " << e.what() << std::endl;
            }

            // Return the first point in the path (closest to target boundary)
            return path[0];
        } 
        else {
            std::cout << "Could not find path to target boundary..." << std::endl;
        }
    }
    else {
        std::cout << "Target ID not found in items data ... " << std::endl;
    }

    return cv::Point(robot_x, robot_y);
}