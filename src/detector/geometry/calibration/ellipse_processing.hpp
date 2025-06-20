#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace ellipse_processing
{
    // Parameters for ray-casting boundary detection
    struct EllipseParams
    {
        // Boundary detection parameters
        int minWhitePixelsThreshold = 1000; // Minimum number of white pixels in the clean mask to consider it valid

        // Debug visualization parameters
        cv::Scalar ellipseColor = cv::Scalar(0, 255, 255); // Color for drawing fitted ellipse (yellow)
        cv::Scalar rayColor = cv::Scalar(255, 0, 255);     // Color for drawing rays (magenta)
        cv::Scalar boundaryColor = cv::Scalar(0, 255, 0);  // Color for boundary points (green)
        int ellipseThickness = 2;                          // Line thickness for ellipse drawing
        int pointRadius = 3;                               // Radius for drawing boundary points
    };

    // CLEAN: Return ellipse directly, handle fallback internally
    cv::RotatedRect processEllipse(
        const std::vector<std::vector<cv::Point>> &contours,
        const cv::Mat &originalFrame,
        const cv::Mat &cleanMask,
        const cv::Point &bullCenter,  // Use bull center as hint
        const cv::Point &frameCenter, // Fallback center
        int camera_idx = 0,
        bool debug_mode = false,
        const EllipseParams &params = EllipseParams());

    // Find boundary points using the clean mask directly
    bool findBoundaryPoints(
        const cv::Mat &cleanMask, // CHANGED: Use clean mask instead of contours
        const cv::Point &center,
        bool debug_mode,
        std::vector<cv::Point> &boundaryPoints,
        std::vector<bool> &rayWasInterpolated,
        std::vector<bool> &finalRayStatus,

        const EllipseParams &params);

    // Fit an ellipse to the boundary points
    bool fitEllipseToBoundary(
        const std::vector<cv::Point> &boundaryPoints,
        cv::RotatedRect &ellipse,
        const EllipseParams &params,
        const cv::Point &center);

} // namespace ellipse_processing
