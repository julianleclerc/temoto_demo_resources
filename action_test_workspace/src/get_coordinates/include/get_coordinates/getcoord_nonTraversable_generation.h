#ifndef GETCOORD_NONTRAVERSABLE_GENERATION_H
#define GETCOORD_NONTRAVERSABLE_GENERATION_H

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

namespace GetCoordNonTraversableGeneration {
    // RobotPosition struct definition
    struct RobotPosition {
        struct {
            struct {
                double x;
                double y;
            } translation;
        } transform;
    };

    cv::Mat process(const cv::Mat& map_img, double resolution, 
                   const std::vector<float>& origin);
}

#endif // GETCOORD_NONTRAVERSABLE_GENERATION_H