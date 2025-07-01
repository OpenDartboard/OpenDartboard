#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Forward declaration
struct DartboardCalibration;

namespace perspective_processing
{
    // Standard dartboard dimensions in mm (official tournament spec)
    struct DartboardSpec
    {
        float bullRadius = 6.35f;         // Bull center radius (mm)
        float bull25Radius = 15.9f;       // 25-point ring radius (mm)
        float innerTripleRadius = 99.0f;  // Inner triple ring radius (mm)
        float outerTripleRadius = 107.0f; // Outer triple ring radius (mm)
        float innerDoubleRadius = 162.0f; // Inner double ring radius (mm)
        float outerDoubleRadius = 170.0f; // Outer double ring radius (mm)

        // Output image parameters
        int outputSize = 800;        // Square output image size (pixels)
        float outputRadius = 350.0f; // Board radius in output image (pixels)
    };

    // Helper structures for intersection calculations
    struct RingIntersections
    {
        vector<vector<Point2f>> intersections; // [wireIndex][ringIndex] = intersection point
        vector<vector<Point3f>> modelPoints;   // Corresponding 3D model points
        bool isValid = false;
    };

    // Main processing function - creates front-facing rectified dartboard
    Mat processPerspective(
        const Mat &rawImage,
        const DartboardCalibration &calib,
        bool enableDebug = false,
        const DartboardSpec &spec = DartboardSpec());

    // Helper functions
    Point2f findEllipseLineIntersection(const RotatedRect &ellipse, Point2f lineStart, Point2f lineEnd);
    RingIntersections findAllRingWireIntersections(const DartboardCalibration &calib, const DartboardSpec &spec);
    Mat rectifyBoard(const Mat &image, const Mat &rvec, const Mat &tvec, const Mat &cameraMatrix, const DartboardSpec &spec);

} // namespace perspective_processing
