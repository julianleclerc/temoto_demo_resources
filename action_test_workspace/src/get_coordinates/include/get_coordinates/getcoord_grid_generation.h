#ifndef GETCOORD_GRID_GENERATION_H
#define GETCOORD_GRID_GENERATION_H

#include <opencv2/opencv.hpp>

namespace GetCoordGridGeneration {
    cv::Mat process(const cv::Mat& cost_map, double resolution, int grid_scale);
}

#endif // GETCOORD_GRID_GENERATION_H