#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

// SIMPLE STRUCT - just data, no methods!
struct DartboardCalibration
{
    // CORE: Bull center (what matters for scoring)
    cv::Point center{0, 0}; // Bull center (true dartboard center)
    int camera_index = -1;  // Which camera this is for

    // THE 6 BOUNDARIES (source of truth for all geometry)
    cv::RotatedRect doubleOuterRing;  // Dartboard edge (was detectedOuterEllipse)
    cv::RotatedRect doubleInnerRing;  // Double ring inner edge (was detectedInnerEllipse)
    cv::RotatedRect tripleOuterRing;  // Triple ring outer edge
    cv::RotatedRect tripleInnerRing;  // Triple ring inner edge
    cv::RotatedRect bullOuterRing;    // Bull outer edge
    cv::RotatedRect bullInnerRing;    // Bull inner edge (bullseye)
    bool hasDetectedEllipses = false; // Source of truth flag

    // PERSPECTIVE INFO: For debugging/quality assessment
    double perspectiveOffsetX = 0.0;         // Bull-ellipse offset X
    double perspectiveOffsetY = 0.0;         // Bull-ellipse offset Y
    double perspectiveOffsetMagnitude = 0.0; // Total offset magnitude

    // ORIENTATION: Still need this for segment alignment
    double orientation = 23.5; // Dartboard orientation (where "20" segment is)
};

// NAMESPACE WITH UTILITY FUNCTIONS
namespace geometry_calibration
{
    // Just calibrate one camera
    DartboardCalibration calibrateSingleCamera(
        const cv::Mat &frame,
        int cameraIdx,
        bool debugMode = false);

    // Calibrate multiple cameras at once
    std::vector<DartboardCalibration> calibrateMultipleCameras(
        const std::vector<cv::Mat> &frames,
        bool debugMode = false,
        int targetWidth = 640,
        int targetHeight = 480);
}