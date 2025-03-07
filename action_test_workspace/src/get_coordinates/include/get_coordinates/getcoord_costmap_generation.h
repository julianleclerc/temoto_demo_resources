#ifndef GETCOORD_COSTMAP_GENERATION_H
#define GETCOORD_COSTMAP_GENERATION_H

#include <opencv2/opencv.hpp>

namespace GetCoordCostmapGeneration {
    cv::Mat process(const cv::Mat& map_img, double resolution, double inflation_radius_m);
}

#endif // GETCOORD_COSTMAP_GENERATION_H