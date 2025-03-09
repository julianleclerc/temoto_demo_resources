#pragma once

#include <opencv2/opencv.hpp>

namespace GetCoordCostmapGeneration {
    /**
     * Process a map image to generate a cost map
     * 
     * @param map_img The input map image
     * @param resolution The resolution of the map in meters per pixel
     * @param inflation_radius_m The inflation radius in meters
     * @return cv::Mat The generated cost map
     */
    cv::Mat process(const cv::Mat& map_img, double resolution, double inflation_radius_m);
}