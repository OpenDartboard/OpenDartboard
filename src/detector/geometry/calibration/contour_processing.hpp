#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace contour_processing
{

    // Parameters for contour processing
    struct ContourParams
    {

        // Morphological enhancement parameters
        int dilationKernelSize = 3;            // Dilation kernel size for connecting broken segments
        int dilationShape = cv::MORPH_ELLIPSE; // Shape of dilation kernel (ellipse for smooth results)

        // Edge detection parameters
        int laplacianKernelSize = 3; // Laplacian kernel size for edge detection
        double edgeThreshold = 10.0; // Threshold for edge detection (higher = fewer edges)

        // OpenCV contour detection parameters
        int contourMode = cv::RETR_LIST;           // Contour retrieval mode (all contours, no hierarchy)
        int contourMethod = cv::CHAIN_APPROX_NONE; // Contour approximation method (store all points)

        // Contour filtering parameters
        double minContourArea = 100.0;        // Minimum contour area to keep (removes tiny noise)
        double maxContourArea = 50000.0;      // Maximum contour area to keep (removes huge blobs)
        double minCircularity = 0.4;          // Minimum circularity for dartboard-like shapes
        double minAspectRatio = 0.3;          // Minimum aspect ratio (removes very thin shapes/text)
        double maxAspectRatio = 3.0;          // Maximum aspect ratio (removes very elongated shapes/text)
        double centerDistanceThreshold = 0.8; // Maximum distance from image center (as ratio of image radius)

        // Debug visualization parameters
        cv::Scalar contourColor = cv::Scalar(0, 255, 255); // Color for drawing contours (yellow)
        int contourThickness = 3;                          // Line thickness for contour drawing
    };

    // Main function - mask in, contours out
    std::vector<std::vector<cv::Point>> processContours(
        const cv::Mat &redGreenFrame,
        const cv::Mat &originalFrame,
        int camera_idx = 0,
        bool debug_mode = false,
        const ContourParams &params = ContourParams());

} // namespace contour_processing
