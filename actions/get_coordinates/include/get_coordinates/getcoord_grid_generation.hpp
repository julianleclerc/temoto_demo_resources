#pragma once

#include <opencv2/opencv.hpp>

namespace GetCoordGridGeneration {
    /**
     * Process a cost map to generate a grid map
     * 
     * @param cost_map The input cost map
     * @param resolution The resolution of the map in meters per pixel
     * @param grid_scale The grid scale in pixels
     * @return cv::Mat The generated grid map with grid lines
     */
    cv::Mat process(const cv::Mat& cost_map, double resolution, int grid_scale);
}