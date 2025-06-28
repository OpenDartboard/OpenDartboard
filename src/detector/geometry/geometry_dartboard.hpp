#pragma once
#include <opencv2/opencv.hpp>
#include <string>

struct DartboardCalibration; // Forward declaration

// Dartboard geometry and scoring logic class
class GeometryDartboard
{
public:
    // Define standard dartboard segment values (clockwise order)
    static const int standardSegments[20];

    // Calculate score from dart position and calibration
    static std::string calculateScore(const cv::Point &dartPosition, const DartboardCalibration &calib);

    // Format score string based on segment and multiplier
    static std::string formatScore(int value, bool isDouble, bool isTriple, bool isBull);
};
