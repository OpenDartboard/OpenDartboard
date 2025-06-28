#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

namespace colors
{
    // Helper method for color comparison using Euclidean distance in BGR space
    // Returns the distance between two colors as a double
    inline double calculateDifference(const Vec3b &color1, const Vec3b &color2)
    {
        return std::sqrt(
            std::pow(color1[0] - color2[0], 2) +
            std::pow(color1[1] - color2[1], 2) +
            std::pow(color1[2] - color2[2], 2));
    }

    // Extract a color profile along a circular path
    // Useful for analyzing dartboard ring colors
    // Returns vector of BGR color values sampled around the circle
    inline vector<Vec3b> extractCircularProfile(
        const Mat &frame,
        const Point &center,
        double radius,
        int samples = 72)
    {
        vector<Vec3b> profile;
        if (frame.empty() || frame.channels() < 3 || radius <= 0)
            return profile;

        for (int i = 0; i < samples; i++)
        {
            double angle = i * 2.0 * CV_PI / samples;
            int x = center.x + radius * std::cos(angle);
            int y = center.y + radius * std::sin(angle);

            if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows)
            {
                profile.push_back(frame.at<Vec3b>(y, x));
            }
            else
            {
                profile.push_back(Vec3b(0, 0, 0)); // Black for out-of-bounds
            }
        }

        return profile;
    }

    // Convert BGR color to HSV color space
    // Useful for color-based dartboard analysis
    inline Vec3b bgrToHsv(const Vec3b &bgrColor)
    {
        Mat bgrMat(1, 1, CV_8UC3, Scalar(bgrColor[0], bgrColor[1], bgrColor[2]));
        Mat hsvMat;
        cvtColor(bgrMat, hsvMat, COLOR_BGR2HSV);
        return hsvMat.at<Vec3b>(0, 0);
    }

    // Check if a color is predominantly red (for dartboard red detection)
    inline bool isRedColor(const Vec3b &color, int hueThreshold = 20, int satThreshold = 60, int valThreshold = 60)
    {
        Vec3b hsv = bgrToHsv(color);
        int hue = hsv[0];
        int sat = hsv[1];
        int val = hsv[2];

        // Red hue wraps around in HSV (0-20 or 160-180)
        bool isRedHue = (hue <= hueThreshold) || (hue >= (180 - hueThreshold));
        return isRedHue && (sat >= satThreshold) && (val >= valThreshold);
    }

    // Check if a color is predominantly green (for dartboard green detection)
    inline bool isGreenColor(const Vec3b &color, int satThreshold = 60, int valThreshold = 60)
    {
        Vec3b hsv = bgrToHsv(color);
        int hue = hsv[0];
        int sat = hsv[1];
        int val = hsv[2];

        // Green hue range in HSV (approximately 40-80)
        bool isGreenHue = (hue >= 40) && (hue <= 80);
        return isGreenHue && (sat >= satThreshold) && (val >= valThreshold);
    }
}
