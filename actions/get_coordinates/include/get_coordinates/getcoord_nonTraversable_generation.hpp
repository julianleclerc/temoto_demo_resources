#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace GetCoordNonTraversableGeneration {
    /**
     * Process a map image to identify and mark non-traversable areas
     * 
     * @param map_img The input map image
     * @param resolution The resolution of the map in meters per pixel
     * @param origin The origin coordinates of the map [x, y, z]
     * @return cv::Mat The map with non-traversable areas marked
     */
    cv::Mat process(const cv::Mat& map_img, double resolution, const std::vector<float>& origin);
}