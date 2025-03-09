#pragma once

#include <opencv2/opencv.hpp>
#include <utility>

namespace GetCoordScaleMapGeneration {
    /**
     * Scale a map image and update its resolution accordingly
     * 
     * @param map_img The input map image
     * @param resolution The original resolution of the map in meters per pixel
     * @param scale_factor The scaling factor to apply
     * @return std::pair<cv::Mat, double> The scaled map and its new resolution
     */
    std::pair<cv::Mat, double> process(const cv::Mat& map_img, double resolution, double scale_factor);
}