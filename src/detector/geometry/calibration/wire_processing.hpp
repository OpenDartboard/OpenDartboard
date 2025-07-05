#pragma once

#include <opencv2/opencv.hpp>
#include <array> // Add this include

using namespace cv;
using namespace std;

// Forward declaration to avoid circular dependency - GLOBAL scope
struct DartboardCalibration;

namespace wire_processing
{
    // Configuration for wire detection methods
    struct WireDetectionConfig
    {
        // Current working method
        bool useHoughLinesDetection = false; // use Hough Lines detection method only

        // Hough line parameters
        int cannyLow = 50;
        int cannyHigh = 150;
        int houghThreshold = 30;
        int houghMinLineLength = 50;
        int houghMaxLineGap = 10;
        float houghAngleTolerance = 15.0f;    // Degrees tolerance for radial lines
        float houghDistanceTolerance = 20.0f; // Pixels tolerance from center
    };

    // Public data structures
    struct WireData
    {
        std::array<Point2f, 20> wireEndpoints;
        int camera_index = -1;
        bool isValid = false;
    };

    // Public interfaces - using global DartboardCalibration
    WireData processWires(const Mat &frame, const Mat &colorMask, const DartboardCalibration &calib, bool enableDebug = false, const WireDetectionConfig &config = WireDetectionConfig());
}