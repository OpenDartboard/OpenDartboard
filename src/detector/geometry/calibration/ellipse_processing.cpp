#include "ellipse_processing.hpp"
#include <iostream>
#include <random>
#include <algorithm>
#include <cmath>

using namespace cv;
using namespace std;

namespace ellipse_processing
{
    // MVP Step 1: Use the clean mask directly (skip contour processing)
    bool findBoundaryPoints(
        const Mat &cleanMask, // CHANGED: Use clean mask directly
        const Point &center,
        bool debug_mode,
        vector<Point> &boundaryPoints,
        vector<bool> &rayWasInterpolated,
        vector<bool> &finalRayStatus,
        const EllipseParams &params)
    {

        boundaryPoints.clear();

        cout << "DEBUG: Starting Step 1 - Using Clean Mask from Color Processing" << endl;
        cout << "DEBUG: Clean mask size: " << cleanMask.cols << "x" << cleanMask.rows
             << ", center=(" << center.x << "," << center.y << ")" << endl;

        // Check if we have enough content
        int whitePixels = countNonZero(cleanMask);

        if (whitePixels < params.minWhitePixelsThreshold)
        {
            cout << "DEBUG: Not enough dartboard content in clean mask" << endl;
            return false;
        }

        // Save debug mask to compare with our previous attempts
        static int mask_counter = 0;
        cout << "DEBUG: Step 2 - Ray casting on this clean mask" << endl;

        // TODO: Step 2 - Ray casting will go here
        // TODO: Step 3 - Filtering will go here

        return true; // For now, just indicate we have a good mask
    }

    bool fitEllipseToBoundary(
        const vector<Point> &boundaryPoints,
        RotatedRect &ellipse,
        const EllipseParams &params,
        const Point &center) // FIXED: Add center parameter here too
    {
        if (boundaryPoints.size() < 5) // Need at least 5 points for ellipse
            return false;

        try
        {
            ellipse = fitEllipse(boundaryPoints);
            return true;
        }
        catch (const cv::Exception &e)
        {
            cout << "DEBUG: Ellipse fitting failed: " << e.what() << endl;
            return false;
        }
    }

    // Visualization function with correct vector handling
    Mat createDetailedRayVisualization(
        const Mat &originalFrame,
        const Point &center,
        const vector<Point> &allRayPoints,
        const vector<bool> &rayValid,
        const vector<bool> &rayWasInterpolated,
        const RotatedRect &ellipse,
        const EllipseParams &params)
    {
        Mat visualization = originalFrame.clone();

        // Draw the center with clear marking
        circle(visualization, center, 8, Scalar(255, 255, 255), -1);
        circle(visualization, center, 3, Scalar(0, 0, 0), -1);

        cout << "DEBUG: Visualization - allRayPoints.size()=" << allRayPoints.size()
             << ", rayValid.size()=" << rayValid.size()
             << ", rayWasInterpolated.size()=" << rayWasInterpolated.size() << endl;

        // Draw rays based on actual boundary points found
        for (size_t i = 0; i < allRayPoints.size(); i++)
        {
            Point rayEnd = allRayPoints[i];
            Scalar rayColor, pointColor;
            int thickness = 2;

            // FIXED: Check bounds before accessing vectors
            bool wasInterpolated = (i < rayWasInterpolated.size()) ? rayWasInterpolated[i] : false;
            bool isValid = (i < rayValid.size()) ? rayValid[i] : true; // Default to valid for actual found points

            // Color based on detection status
            if (wasInterpolated)
            {
                // Interpolated/fallback points - ORANGE
                rayColor = Scalar(0, 165, 255);   // Orange
                pointColor = Scalar(0, 255, 255); // Yellow
                thickness = 1;
            }
            else if (isValid)
            {
                // Actually detected boundary points - GREEN
                rayColor = Scalar(0, 255, 0);   // Green
                pointColor = Scalar(0, 200, 0); // Darker green
                thickness = 3;
            }
            else
            {
                // Invalid/rejected points - RED
                rayColor = Scalar(0, 0, 255);   // Red
                pointColor = Scalar(0, 0, 200); // Dark red
                thickness = 1;
            }

            // Draw ray line and point
            line(visualization, center, rayEnd, rayColor, thickness);
            circle(visualization, rayEnd, params.pointRadius + 1, pointColor, -1);
        }

        // Draw the fitted ellipse if valid
        if (ellipse.size.width > 0 && ellipse.size.height > 0)
        {
            cv::ellipse(visualization, ellipse, params.ellipseColor, params.ellipseThickness);

            // ENHANCED: Draw ellipse info
            double area = M_PI * (ellipse.size.width / 2.0) * (ellipse.size.height / 2.0);
            cout << "DEBUG: Drawing ellipse - area=" << area << ", width=" << ellipse.size.width
                 << ", height=" << ellipse.size.height << endl;
        }

        // Enhanced info text with actual values
        putText(visualization, "Points: " + to_string(allRayPoints.size()) + " found",
                Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);

        if (ellipse.size.width > 0)
        {
            double aspectRatio = max(ellipse.size.width, ellipse.size.height) /
                                 min(ellipse.size.width, ellipse.size.height);
            double area = M_PI * (ellipse.size.width / 2.0) * (ellipse.size.height / 2.0);

            putText(visualization,
                    "Ellipse: " + to_string(int(ellipse.size.width / 2)) + "x" + to_string(int(ellipse.size.height / 2)) +
                        " ratio=" + to_string(aspectRatio).substr(0, 4),
                    Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);

            putText(visualization,
                    "Area: " + to_string(int(area)) + " pixels",
                    Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
        }
        else
        {
            putText(visualization, "ELLIPSE FITTING FAILED",
                    Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        }

        putText(visualization, "Green=Detected, Orange=Fallback, Red=Rejected",
                Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

        return visualization;
    }

    // Main function to process the ellipse - CLEAN RETURN
    RotatedRect processEllipse(
        const vector<vector<Point>> &contours,
        const Mat &originalFrame,
        const Mat &cleanMask,
        const Point &bullCenter,
        const Point &frameCenter,
        int camera_idx,
        bool debug_mode,
        const EllipseParams &params)
    {
        vector<Point> allRayPoints;
        vector<bool> rayWasInterpolated;
        vector<bool> actualRayStatus;
        Mat ellipseVisualization;

        // Use the clean mask directly instead of processing contours
        bool boundaryFound = findBoundaryPoints(cleanMask, bullCenter, debug_mode, allRayPoints, rayWasInterpolated, actualRayStatus, params);

        RotatedRect dartboardEllipse;
        bool success = false;

        if (boundaryFound && allRayPoints.size() >= 5)
        {
            try
            {
                dartboardEllipse = fitEllipse(allRayPoints);
                cout << "DEBUG: Fitted ellipse - center=(" << dartboardEllipse.center.x << ", " << dartboardEllipse.center.y
                     << "), size=(" << dartboardEllipse.size.width << ", " << dartboardEllipse.size.height
                     << "), angle=" << dartboardEllipse.angle << endl;
                success = true;
            }
            catch (const cv::Exception &e)
            {
                cout << "DEBUG: Ellipse fitting failed: " << e.what() << endl;
                success = false;
            }
        }
        else
        {
            cout << "DEBUG: Not enough boundary points found for ellipse fitting: " << allRayPoints.size() << endl;
        }

        // HANDLE FALLBACK INTERNALLY
        if (!success)
        {
            cout << "DEBUG: Using fallback ellipse with bull center" << endl;

            // Estimate dartboard size from frame dimensions
            int estimatedRadius = min(originalFrame.cols, originalFrame.rows) / 3;

            // Create fallback ellipse centered on bull
            dartboardEllipse.center = Point2f(bullCenter.x, bullCenter.y);
            dartboardEllipse.size = Size2f(estimatedRadius * 2, estimatedRadius * 2);
            dartboardEllipse.angle = 0;
        }

        if (debug_mode)
        {
            if (success)
            {
                cout << "Camera " << camera_idx << ": ELLIPSE SUCCESS - " << allRayPoints.size() << " boundary points found" << endl;
            }
            else
            {
                cout << "Camera " << camera_idx << ": ELLIPSE FALLBACK - using bull center with estimated size" << endl;
            }

            // Create visualization
            ellipseVisualization = createDetailedRayVisualization(originalFrame, bullCenter, allRayPoints, actualRayStatus, rayWasInterpolated, dartboardEllipse, params);

            system("mkdir -p debug_frames/ellipse_processing");
            imwrite("debug_frames/ellipse_processing/ellipse_result_" + to_string(camera_idx) + ".jpg", ellipseVisualization);
        }

        return dartboardEllipse; // CLEAN: Always return a valid ellipse
    }

} // namespace ellipse_processing