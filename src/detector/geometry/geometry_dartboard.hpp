#pragma once

#include <opencv2/opencv.hpp>
#include <string>

// Forward declare the structure to avoid circular dependencies
struct DartboardCalibration;

class GeometryDartboard
{
public:
    // Draw the dartboard with correct orientation for this camera
    static void drawCalibrationOverlay(
        cv::Mat &frame,
        const DartboardCalibration &calib,
        bool showDetails = true);

    // Calculate dart score with proper orientation adjustment
    static std::string calculateScore(
        const cv::Point &dartPosition,
        const DartboardCalibration &calib);

    // Format score consistently
    static std::string formatScore(
        int value,
        bool isDouble,
        bool isTriple,
        bool isBull);

private:
    // Standard dartboard segment values (clockwise from top)
    static const int standardSegments[20];

    // Helper function to calculate distance between points
    static double distanceToPoint(const cv::Point &p1, const cv::Point &p2);
};
