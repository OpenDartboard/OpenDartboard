#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>

namespace math_utils
{
    // Calculates Euclidean distance between two points
    inline double distanceToPoint(const cv::Point &p1, const cv::Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    // Additional math utilities can be added here in the future
    // For example:
    // - angle between points
    // - vector operations
    // - geometric transformations
}
