#pragma once

#include <opencv2/opencv.hpp>

namespace geometry_utils
{
    // Calculate the center point of a frame/image
    inline cv::Point calculateFrameCenter(const cv::Mat &frame)
    {
        return cv::Point(frame.cols / 2, frame.rows / 2);
    }

} // namespace geometry_utils
