#include "get_coordinates/getcoord_grid_generation.hpp"
#include <vector>

namespace GetCoordGridGeneration {
    cv::Mat process(const cv::Mat& cost_map, double resolution, int grid_scale) {
        // Calculate pixels per grid cell
        int pixels_per_grid_cell = grid_scale;

        // Map dimensions
        int height = cost_map.rows;
        int width = cost_map.cols;

        // Create a copy of the map to draw the grid on
        cv::Mat grid_map = cost_map.clone();

        // Ensure cost_map is in correct format
        cv::Mat cost_map_gray;
        if (grid_map.channels() == 1) {
            cv::cvtColor(grid_map, grid_map, cv::COLOR_GRAY2BGR);
        }

        // Ensure we have a proper grayscale version for analysis
        if (cost_map.channels() == 3) {
            cv::cvtColor(cost_map, cost_map_gray, cv::COLOR_BGR2GRAY);
        } else {
            cost_map_gray = cost_map.clone();
        }

        // Threshold to binary
        cv::Mat binary_map;
        cv::threshold(cost_map_gray, binary_map, 254, 255, cv::THRESH_BINARY);

        // Generate x positions for vertical grid lines
        std::vector<int> x_positions;
        for (int x = 0; x < width; x += pixels_per_grid_cell) {
            x_positions.push_back(x);
        }

        // Draw vertical grid lines
        for (int x : x_positions) {
            for (int y = 0; y < height; y++) {
                if (binary_map.at<uchar>(y, x) == 255) {
                    grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                }
            }
        }

        // Generate y positions for horizontal grid lines
        std::vector<int> y_positions;
        for (int y = 0; y < height; y += pixels_per_grid_cell) {
            y_positions.push_back(y);
        }

        // Draw horizontal grid lines
        for (int y : y_positions) {
            for (int x = 0; x < width; x++) {
                if (binary_map.at<uchar>(y, x) == 255) {
                    grid_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                }
            }
        }

        return grid_map;
    }
}