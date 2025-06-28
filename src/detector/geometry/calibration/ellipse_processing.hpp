#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "mask_processing.hpp" // Include for MaskBundle

namespace ellipse_processing
{

    // Comprehensive parameters for ellipse processing
    struct EllipseParams
    {
        // ===== BASIC PROCESSING PARAMETERS =====
        int minWhitePixelsThreshold = 1000; // Minimum white pixels in mask to attempt ellipse detection

        // ===== DOUBLE RAY TRACE CORE PARAMETERS ===== (Used for doubles ring)
        double angleStepDegrees = 3.0;  // Angular step between rays (120 rays total at 3°)
        int maxRayDistance = 800;       // Maximum pixel distance to cast rays
        int minValidRays = 50;          // Minimum rays needed for reliable ellipse fitting
        int innerRayStartDistance = 10; // Start searching for inner boundary this far from bull center

        // ===== ROLLING BASELINE VALIDATION PARAMETERS =====
        double maxRingWidthJump = 10.0;         // Max pixel jump in ring width between adjacent rays
        double ringWidthOutlierThreshold = 2.0; // Standard deviations for global outlier detection
        int minValidRingMeasurements = 20;      // Minimum valid ring measurements to attempt validation

        // ===== CONTOUR FITTING PARAMETERS ===== (Used for triples & bull)
        double minContourArea = 100.0; // Minimum contour area to consider
        double minAspectRatio = 0.3;   // Minimum aspect ratio for ellipse fitting
        double maxAspectRatio = 3.0;   // Maximum aspect ratio for ellipse fitting
        double minCircularity = 0.2;   // Minimum circularity for ellipse fitting

        // ===== VISUALIZATION PARAMETERS =====
        cv::Scalar rayColor = cv::Scalar(0, 255, 255);           // Yellow color for accepted rays (BGR)
        cv::Scalar boundaryPointColor = cv::Scalar(0, 255, 0);   // Green color for outer boundary points
        cv::Scalar innerPointColor = cv::Scalar(255, 0, 0);      // Blue color for inner boundary points
        cv::Scalar bullCenterColor = cv::Scalar(0, 0, 0);        // Black color for bull center
        cv::Scalar ellipseCenterColor = cv::Scalar(128, 0, 128); // Purple color for ellipse center
        int rayThickness = 1;                                    // Thickness of ray lines in pixels
        int boundaryPointRadius = 4;                             // Radius of boundary point circles
        int centerPointRadius = 4;                               // Radius for center point visualization
        cv::Scalar ellipseColor = cv::Scalar(255, 0, 255);       // Pink color for fitted ellipse (BGR)
        int ellipseThickness = 2;                                // Thickness of ellipse outline
    };

    // Rich boundary data structure with proper dartboard terminology
    struct EllipseBoundaryData
    {
        // RAY TRACING RESULTS (doubles ring - most accurate)
        cv::RotatedRect outerDoubleEllipse; // Dartboard edge (ray traced)
        cv::RotatedRect innerDoubleEllipse; // Double ring inner edge (ray traced)
        bool hasValidDoubles;               // Ray tracing succeeded for doubles
        int validOuterPoints;               // Number of validated outer boundary points
        int validInnerPoints;               // Number of validated inner boundary points

        // CONTOUR FITTING RESULTS (triples & bull rings - efficient)
        cv::RotatedRect outerTripleEllipse; // Triple ring outer edge (contour fitted)
        cv::RotatedRect innerTripleEllipse; // Triple ring inner edge (contour fitted)
        cv::RotatedRect outerBullEllipse;   // 25-point ring (contour fitted)
        cv::RotatedRect innerBullEllipse;   // 50-point bullseye (contour fitted)
        bool hasValidTriples;               // Contour fitting succeeded for triples
        bool hasValidBulls;                 // Contour fitting succeeded for bull rings

        // PERSPECTIVE INFO: For debugging/quality assessment
        double offsetX;         // Horizontal offset between bull and ellipse centers (pixels)
        double offsetY;         // Vertical offset between bull and ellipse centers (pixels)
        double offsetMagnitude; // Total offset distance (pixels)
        double offsetAngle;     // Offset angle in degrees

        // Default constructor for fallback cases
        EllipseBoundaryData() : hasValidDoubles(false), validOuterPoints(0), validInnerPoints(0),
                                hasValidTriples(false), hasValidBulls(false),
                                offsetX(0.0), offsetY(0.0), offsetMagnitude(0.0), offsetAngle(0.0) {}
    };

    // UPDATED: Takes MaskBundle instead of single mask
    EllipseBoundaryData processEllipse(
        const cv::Mat &originalFrame,
        const mask_processing::MaskBundle &masks,
        const cv::Point &bullCenter,
        const cv::Point &frameCenter,
        int camera_idx = 0,
        bool debug_mode = false,
        const EllipseParams &params = EllipseParams());

} // namespace ellipse_processing