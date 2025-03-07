#ifndef GETCOORD_SCALEMAP_GENERATION_H
#define GETCOORD_SCALEMAP_GENERATION_H

#include <opencv2/opencv.hpp>
#include <utility>

namespace GetCoordScaleMapGeneration {
    std::pair<cv::Mat, double> process(const cv::Mat& map_img, double resolution, double scale_factor);
}

#endif // GETCOORD_SCALEMAP_GENERATION_H