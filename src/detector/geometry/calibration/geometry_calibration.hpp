#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class DartboardCalibration
{
public:
    cv::Point center;   // Center point of dartboard
    double radius;      // Equivalent circular radius
    double orientation; // Orientation in degrees

    // Elliptical parameters (always used)
    cv::Size2f axes; // Major and minor axes of the ellipse
    double angle;    // Rotation angle of the ellipse in degrees

    // Standard measurements
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

    // Method to find dartboard in a frame - updated with axes and angle parameters
    bool findDartboardCircle(const cv::Mat &frame, cv::Point &center,
                             double &radius, double &orientation,
                             cv::Size2f &axes, double &angle,
                             int camera_idx, bool debugMode = false);

    // New method: Detect orientation based on radial color profile
    double detectOrientation(const cv::Mat &frame, const cv::Point &center, double radius,
                             const cv::Mat &redMask, const cv::Mat &greenMask, bool debugMode = false);

    // Static method to create a standard calibration - updated with axes and angle parameters
    static DartboardCalibration createStandardCalibration(
        cv::Point center, double radius, double orientation,
        cv::Size2f axes, double angle, int cameraIdx);

    // Static method to calibrate multiple cameras at once
    static std::vector<DartboardCalibration> calibrateMultipleCameras(
        const std::vector<cv::Mat> &frames,
        bool debugMode = false,
        int targetWidth = 640,
        int targetHeight = 480);

    // Helper struct for color samples around the dartboard
    struct ColorSample
    {
        double angle;      // in radians
        double redValue;   // intensity of red color
        double greenValue; // intensity of green color
        double totalValue; // combined value
    };

    // Helper struct for ellipse detection
    struct EllipseParams
    {
        cv::RotatedRect ellipse; // The detected ellipse
        cv::Point center;        // Center point
        double radius;           // Equivalent radius if circular
        double confidence;       // Detection confidence
    };

    // Method to detect elliptical shape
    bool detectEllipticalShape(const cv::Mat &frame,
                               const cv::Mat &redGreenMask,
                               cv::RotatedRect &ellipse);
};
