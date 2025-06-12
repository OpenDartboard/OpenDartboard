#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class DartboardCalibration
{
public:
    cv::Point center;       // Center point of dartboard
    double radius;          // Outer radius
    double orientation;     // Orientation in degrees
    double bullRadius;      // Bull's eye radius
    double doubleRingInner; // Inner radius of double ring
    double doubleRingOuter; // Outer radius of double ring
    double tripleRingInner; // Inner radius of triple ring
    double tripleRingOuter; // Outer radius of triple ring
    int camera_index;       // Which camera this calibration is for

    // Default constructor
    DartboardCalibration();

    // Method to calibrate a single camera
    bool calibrateSingleCamera(const cv::Mat &frame, int cameraIdx, bool debugMode = false);

    // Method to find dartboard in a frame
    bool findDartboardCircle(const cv::Mat &frame, cv::Point &center,
                             double &radius, double &orientation,
                             int camera_idx, bool debugMode = false);

    // Static method to create a standard calibration
    static DartboardCalibration createStandardCalibration(
        cv::Point center, double radius, double orientation, int cameraIdx);
};
