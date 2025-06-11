#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

namespace color_utils
{
    // Helper method for color comparison
    inline double colorDifference(const cv::Vec3b &color1, const cv::Vec3b &color2)
    {
        return std::sqrt(
            std::pow(color1[0] - color2[0], 2) +
            std::pow(color1[1] - color2[1], 2) +
            std::pow(color1[2] - color2[2], 2));
    }

    // Extract a color profile along a circle
    inline std::vector<cv::Vec3b> extractColorProfile(
        const cv::Mat &frame,
        const cv::Point &center,
        double radius,
        int samples = 72)
    {
        std::vector<cv::Vec3b> profile;
        if (frame.empty() || frame.channels() < 3 || radius <= 0)
            return profile;

        for (int i = 0; i < samples; i++)
        {
            double angle = i * 2.0 * CV_PI / samples;
            int x = center.x + radius * std::cos(angle);
            int y = center.y + radius * std::sin(angle);

            if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows)
            {
                profile.push_back(frame.at<cv::Vec3b>(y, x));
            }
            else
            {
                profile.push_back(cv::Vec3b(0, 0, 0));
            }
        }

        return profile;
    }
}
