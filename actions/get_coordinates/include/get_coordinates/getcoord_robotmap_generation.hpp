#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace GetCoordRobotMapGeneration {
    // Define Quaternion struct
    struct Quaternion {
        double x;
        double y;
        double z;
        double w;
    };

    // Define Vector3 struct
    struct Vector3 {
        double x;
        double y;
        double z;
    };

    // Define Transform struct
    struct Transform {
        Vector3 translation;
        Quaternion rotation;
    };

    /**
     * Process a cost map to generate a robot map with robot position and orientation
     * 
     * @param cost_map The input cost map
     * @param resolution The resolution of the map in meters per pixel
     * @param origin The origin coordinates of the map [x, y, z]
     * @param trans The transform containing robot position and orientation
     * @return cv::Mat The generated robot map with robot position and orientation marked
     */
    cv::Mat process(const cv::Mat& cost_map, 
                   double resolution, 
                   const std::vector<float>& origin, 
                   const Transform& trans);
}