#ifndef GETCOORD_ROBOTMAP_GENERATION_H
#define GETCOORD_ROBOTMAP_GENERATION_H

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

namespace GetCoordRobotMapGeneration {
    struct Quaternion {
        double x, y, z, w;
    };

    struct Transform {
        struct {
            double x, y, z;
        } translation;
        Quaternion rotation;
    };

    cv::Mat process(const cv::Mat& cost_map, 
                  double resolution, 
                  const std::vector<float>& origin, 
                  const Transform& trans);
}

#endif // GETCOORD_ROBOTMAP_GENERATION_H