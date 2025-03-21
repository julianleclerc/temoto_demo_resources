#include "get_coordinates/getcoord_objectmap_generation.hpp"
#include <random>
#include <cmath>

namespace GetCoordObjectMapGeneration {
    // Helper class for object map generation
    class ObjectMapGenerator {
    private:
        // Random number generator
        std::random_device rd;
        std::mt19937 gen;

        // Generate a random color
        cv::Scalar generateRandomColor() {
            return cv::Scalar(
                gen() % 256,  // Blue
                gen() % 256,  // Green
                gen() % 256   // Red
            );
        }

        // Calculate text color based on background brightness
        cv::Scalar getTextColor(const cv::Scalar& backgroundColor) {
            double brightness = (backgroundColor[0] + backgroundColor[1] + backgroundColor[2]) / 3;
            return (brightness < 128) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 0);
        }

    public:
        ObjectMapGenerator() : gen(rd()) {}

        cv::Mat generateMap(const cv::Mat& robot_map, double resolution, 
                         const std::vector<float>& origin, 
                         const nlohmann::json& items_data) {
            // Create a copy of the robot map
            cv::Mat object_map_color = robot_map.clone();

            // Ensure we're working with a color image
            if (object_map_color.channels() == 1) {
                cv::cvtColor(object_map_color, object_map_color, cv::COLOR_GRAY2BGR);
            }

            // Map dimensions
            int map_height = object_map_color.rows;
            int map_width = object_map_color.cols;

            // Create an overlay image
            cv::Mat overlay = object_map_color.clone();

            // Padding and border properties
            int rectangle_padding = 5;
            int border_thickness = 1;
            cv::Scalar border_color(0, 0, 0);  // Black border

            // Struct to store label information
            struct LabelInfo {
                std::string text;
                cv::Point top_left;
                cv::Point bottom_right;
                cv::Scalar color;
            };
            std::vector<LabelInfo> labels_to_draw;

            // Iterate over items
            for (const auto& [item_class, items_list] : items_data["items"].items()) {
                for (const auto& item : items_list) {
                    std::string item_id = item["id"];
                    
                    // Extract coordinates and dimensions
                    double coord_x = item["coordinates"]["x"];
                    double coord_y = item["coordinates"]["y"];
                    double width = item["dimensions"]["width"];
                    double height = item["dimensions"]["height"];

                    // Convert world coordinates to map pixel coordinates
                    int map_x = static_cast<int>((coord_x - origin[0]) / resolution);
                    int map_y = map_height - static_cast<int>((coord_y - origin[1]) / resolution);

                    // Convert dimensions to pixels
                    int width_px = static_cast<int>(width / resolution);
                    int height_px = static_cast<int>(height / resolution);

                    // Calculate rectangle coordinates with padding
                    cv::Point top_left(
                        std::max(0, std::min(map_width - 1, map_x - width_px / 2 - rectangle_padding)),
                        std::max(0, std::min(map_height - 1, map_y - height_px / 2 - rectangle_padding))
                    );
                    cv::Point bottom_right(
                        std::max(0, std::min(map_width - 1, map_x + width_px / 2 + rectangle_padding)),
                        std::max(0, std::min(map_height - 1, map_y + height_px / 2 + rectangle_padding))
                    );

                    // Generate random color
                    cv::Scalar color = generateRandomColor();

                    // Draw filled rectangle
                    cv::rectangle(overlay, top_left, bottom_right, color, -1);

                    // Draw rectangle border
                    cv::rectangle(overlay, top_left, bottom_right, border_color, border_thickness);

                    // Draw "X" lines inside the rectangle
                    cv::line(overlay, top_left, bottom_right, border_color, 1);
                    cv::line(overlay, 
                        cv::Point(top_left.x, bottom_right.y), 
                        cv::Point(bottom_right.x, top_left.y), 
                        border_color, 1);

                    // Store label information
                    labels_to_draw.push_back({item_id, top_left, bottom_right, color});
                }
            }

            // Draw labels
            for (const auto& label : labels_to_draw) {
                int font = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.3;
                int thickness_text = 1;

                // Calculate text size
                int baseline;
                cv::Size text_size = cv::getTextSize(label.text, font, font_scale, thickness_text, &baseline);

                // Calculate text position
                cv::Point text_pos(
                    (label.top_left.x + label.bottom_right.x) / 2 - text_size.width / 2,
                    (label.top_left.y + label.bottom_right.y) / 2 + text_size.height / 2
                );

                // Determine text and outline colors
                cv::Scalar text_color = getTextColor(label.color);
                cv::Scalar outline_color = (text_color == cv::Scalar(0, 255, 0)) ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);

                // Draw text outline
                cv::putText(overlay, label.text, text_pos, font, font_scale, outline_color, 3, cv::LINE_AA);

                // Draw text
                cv::putText(overlay, label.text, text_pos, font, font_scale, text_color, 1, cv::LINE_AA);
            }

            // Apply transparency
            double alpha = 0.7;
            cv::addWeighted(overlay, alpha, object_map_color, 1 - alpha, 0, object_map_color);

            return object_map_color;
        }
    };

    cv::Mat process(const cv::Mat& grid_map, double resolution, 
                  const std::vector<float>& origin, const nlohmann::json& items_data) {
        ObjectMapGenerator generator;
        return generator.generateMap(grid_map, resolution, origin, items_data);
    }
}