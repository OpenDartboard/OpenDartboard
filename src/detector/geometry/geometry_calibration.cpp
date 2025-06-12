#include "geometry_calibration.hpp"
#include "utils/dartboard_visualization.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

using namespace cv;
using namespace std;

DartboardCalibration::DartboardCalibration()
    : center(0, 0), radius(0), orientation(270.0),
      axes(0, 0), angle(0),
      bullRadius(0), doubleRingInner(0), doubleRingOuter(0),
      tripleRingInner(0), tripleRingOuter(0), camera_index(-1)
{
}

// Method to create a standard calibration
bool DartboardCalibration::calibrateSingleCamera(const cv::Mat &frame, int cameraIdx, bool debugMode)
{
    cout << "DEBUG: Calibrating camera " << cameraIdx + 1 << "..." << endl;

    if (frame.empty())
    {
        cerr << "Empty frame from camera " << cameraIdx + 1 << endl;
        return false;
    }

    cv::Point center;
    double radius;
    double orientation;
    cv::Size2f axes; // Add axes parameter
    double angle;    // Add angle parameter

    bool found = findDartboardCircle(frame, center, radius, orientation, axes, angle, cameraIdx, debugMode);

    if (found && radius > 25)
    {
        // Use helper function to create calibration with all detected parameters
        *this = createStandardCalibration(center, radius, orientation, axes, angle, cameraIdx);
        return true;
    }
    else
    {
        cout << "FAIL: Failed to calibrate camera " << cameraIdx + 1 << endl;
        return false;
    }
}

// Simplified Color-Based Dartboard Detection
bool DartboardCalibration::findDartboardCircle(const cv::Mat &frame, cv::Point &center, double &radius,
                                               double &orientation, cv::Size2f &axes, double &angle,
                                               int camera_idx, bool debug_mode)
{
    // Basic validation
    if (frame.empty() || frame.channels() < 3)
        return false;

    cv::Mat colorFrame = frame.clone();
    cv::Point frameCenter(frame.cols / 2, frame.rows / 2);

    // MODIFICATION 1: Increase ROI radius by 10% to avoid cutting off dartboard edges
    cv::Mat roiMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    // Use 60% of minimum dimension instead of 50% (increase from half to 60% of frame)
    int initialRoiRadius = int(std::min(frame.cols, frame.rows) * 0.6);
    cv::circle(roiMask, frameCenter, initialRoiRadius, cv::Scalar(255), -1);

    // Create ROI-constrained color frame for all subsequent processing
    cv::Mat roiColorFrame;
    colorFrame.copyTo(roiColorFrame, roiMask);

    //---------- STEP 1: CREATE RED AND GREEN ONLY MASK ----------

    // Convert to HSV for better color separation - ONLY PROCESS WITHIN ROI
    cv::Mat hsvFrame;
    cv::cvtColor(roiColorFrame, hsvFrame, cv::COLOR_BGR2HSV);

    // UNIVERSAL approach - use wider ranges that work for ALL cameras
    cv::Mat redMask1, redMask2, redMask, greenMask, whiteMask;

    // Broad red detection that works universally (both primary and wrapped hue ranges)
    cv::inRange(hsvFrame, cv::Scalar(0, 40, 40), cv::Scalar(20, 255, 255), redMask1);
    cv::inRange(hsvFrame, cv::Scalar(160, 40, 40), cv::Scalar(180, 255, 255), redMask2);

    // Detect whitish areas for lighting variations
    cv::inRange(hsvFrame, cv::Scalar(0, 0, 150), cv::Scalar(180, 40, 255), whiteMask);

    // Green detection for triple ring with adaptive range
    cv::inRange(hsvFrame, cv::Scalar(30, 40, 40), cv::Scalar(90, 255, 255), greenMask);

    // Combine masks intelligently
    redMask = redMask1 | redMask2;
    cv::Mat redGreenMask = redMask | greenMask;

    // Add white areas if red/green detection is poor
    int colorCount = cv::countNonZero(redGreenMask);
    if (colorCount < (frame.rows * frame.cols * 0.05))
    { // If less than 5% detected
        redGreenMask = redGreenMask | whiteMask;
    }

    // Create visualization with ROI applied
    cv::Mat redGreenOnly = cv::Mat::zeros(frame.size(), CV_8UC3);
    for (int y = 0; y < redGreenMask.rows; y++)
    {
        for (int x = 0; x < redGreenMask.cols; x++)
        {
            if (redMask.at<uchar>(y, x) > 0)
            {
                redGreenOnly.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red
            }
            else if (greenMask.at<uchar>(y, x) > 0)
            {
                redGreenOnly.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // Green
            }
            else if (whiteMask.at<uchar>(y, x) > 0 && redGreenMask.at<uchar>(y, x) > 0)
            {
                redGreenOnly.at<cv::Vec3b>(y, x) = cv::Vec3b(200, 200, 200); // Light gray
            }
        }
    }

    // Save debug images
    if (debug_mode)
    {
        system("mkdir -p debug_frames");
        cv::imwrite("debug_frames/red_green_only_" + to_string(camera_idx) + ".jpg", redGreenOnly);
        cv::imwrite("debug_frames/red_green_mask_" + to_string(camera_idx) + ".jpg", redGreenMask);
    }

    //---------- STEP 2: ADAPTIVE MASK ENHANCEMENT ----------

    // Detect feature density to determine appropriate processing
    // Using ROI-masked versions for all calculations
    double featureDensity = cv::countNonZero(redGreenMask) / static_cast<double>(cv::countNonZero(roiMask));

    // Choose kernel size based on feature density - larger for sparse features
    int kernelSize = (featureDensity < 0.05) ? 7 : 5;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));

    // Apply more aggressive morphology for sparse features
    int iterations = (featureDensity < 0.05) ? 2 : 1;
    cv::morphologyEx(redGreenMask, redGreenMask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), iterations);

    // Create cleaner mask for circle detection
    cv::Mat circleMask = redGreenMask.clone();
    cv::morphologyEx(circleMask, circleMask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::morphologyEx(circleMask, circleMask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));

    // IMPROVED: Create enhanced contour mask to detect circular features better
    cv::Mat contourMask = redGreenMask.clone();

    // Simple ring enhancement - first dilate slightly to connect broken ring segments
    cv::dilate(redGreenMask, contourMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    // Then extract edges only - this will highlight the rings
    cv::Mat ringEdges;
    cv::Laplacian(contourMask, ringEdges, CV_8U, 3);
    cv::threshold(ringEdges, ringEdges, 10, 255, cv::THRESH_BINARY);

    // Combine original mask with edges to preserve all features while enhancing rings
    contourMask = contourMask | ringEdges;

    // Find contours using the enhanced mask within ROI
    vector<vector<cv::Point>> contours;
    cv::findContours(contourMask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Debug visualization with NO NEW images - use existing contours_*.jpg
    if (debug_mode)
    {
        cv::Mat contourVis = colorFrame.clone();
        // IMPROVED: Make contours much bolder (thickness 3 instead of 1)
        cv::drawContours(contourVis, contours, -1, cv::Scalar(0, 255, 255), 3);
        cv::imwrite("debug_frames/contours_" + to_string(camera_idx) + ".jpg", contourVis);
    }

    //---------- STEP 3: MULTI-STRATEGY DETECTION ----------

    cv::Point bestCenter = frameCenter;
    double bestScore = 0;
    double bestRadius = min(frame.cols, frame.rows) / 3.0;

    // Strategy A: Bull's eye detection
    cv::Mat bullsEye = redMask.clone();
    cv::morphologyEx(bullsEye, bullsEye, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    vector<vector<cv::Point>> bullContours;
    cv::findContours(bullsEye, bullContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Process bull contours
    for (const auto &contour : bullContours)
    {
        double area = cv::contourArea(contour);
        // Adaptive size filtering based on image size
        double minArea = 20;
        double maxArea = frame.total() * 0.005; // 0.5% of image max

        if (area < minArea || area > maxArea)
            continue;

        // Calculate circularity
        double perimeter = cv::arcLength(contour, true);
        double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        if (circularity < 0.5)
            continue;

        cv::Point2f center2f;
        float radius2f;
        cv::minEnclosingCircle(contour, center2f, radius2f);

        // Adaptive scoring based on distance from frame center
        double maxDist = cv::norm(cv::Point(0, 0) - cv::Point(frame.cols, frame.rows)) / 2.0;
        double distToCenter = cv::norm(cv::Point(center2f) - frameCenter);
        double centralityScore = 1.0 - (distToCenter / maxDist);

        // Combined score weighted by circularity and centrality
        double score = circularity * 0.6 + centralityScore * 0.4;

        if (score > bestScore)
        {
            bestScore = score;
            bestCenter = cv::Point(center2f);
            bestRadius = radius2f * 35; // Estimated board radius from bull
        }
    }

    // Strategy B: Circular pattern detection with adaptive parameters
    if (bestScore < 0.5)
    {
        // Adjust parameters based on feature density
        double dp = (featureDensity < 0.05) ? 1.0 : 1.2;
        int minDist = circleMask.rows / ((featureDensity < 0.05) ? 10 : 8);
        int cannyThreshold = (featureDensity < 0.05) ? 70 : 80;
        int accThreshold = (featureDensity < 0.05) ? 20 : 25;

        vector<cv::Vec3f> circles;
        cv::HoughCircles(circleMask, circles, cv::HOUGH_GRADIENT, dp, minDist,
                         cannyThreshold, accThreshold,
                         circleMask.rows / 12, circleMask.rows / 1.5);

        // Process detected circles
        if (!circles.empty())
        {
            double bestCircleScore = 0;
            int bestIdx = -1;

            for (size_t i = 0; i < circles.size(); i++)
            {
                cv::Point circleCenter(cvRound(circles[i][0]), cvRound(circles[i][1]));
                float circleRadius = circles[i][2];

                // Adaptive ROI size based on image dimensions
                int roiSize = max(5, min(20, frame.cols / 30));

                // Check for red content near center (potential bull)
                cv::Rect roi(
                    max(0, circleCenter.x - roiSize),
                    max(0, circleCenter.y - roiSize),
                    min(2 * roiSize, redMask.cols - circleCenter.x + roiSize),
                    min(2 * roiSize, redMask.rows - circleCenter.y + roiSize));

                // Avoid invalid ROIs
                if (roi.width <= 0 || roi.height <= 0)
                    continue;

                cv::Mat centerROI = redMask(roi);
                double centerRedness = cv::countNonZero(centerROI) / static_cast<double>(centerROI.total());

                // Adaptive centrality measure
                double maxDist = cv::norm(cv::Point(0, 0) - cv::Point(frame.cols, frame.rows)) / 2.0;
                double distToCenter = cv::norm(circleCenter - frameCenter);
                double centralityScore = 1.0 - (distToCenter / maxDist);

                // Balance between center proximity and redness
                double circleScore = centralityScore * 0.4 + centerRedness * 0.6;

                if (circleScore > bestCircleScore)
                {
                    bestCircleScore = circleScore;
                    bestIdx = i;
                }
            }

            // Update best detection if circle score is good
            if (bestIdx >= 0 && bestCircleScore > bestScore)
            {
                bestCenter = cv::Point(cvRound(circles[bestIdx][0]), cvRound(circles[bestIdx][1]));
                bestRadius = circles[bestIdx][2];
                bestScore = bestCircleScore;
            }
        }
    }

    // Strategy C: Color density map (universally applicable)
    if (bestScore < 0.4)
    {
        cv::Mat densityMap = cv::Mat::zeros(frame.size(), CV_32F);

        // Create density map with exponential distance weighting
        for (int y = 0; y < redGreenMask.rows; y++)
        {
            for (int x = 0; x < redGreenMask.cols; x++)
            {
                if (redGreenMask.at<uchar>(y, x) > 0)
                {
                    double dist = cv::norm(cv::Point(x, y) - frameCenter);
                    double weight = exp(-dist / (frame.cols / 2.0)) * 10.0;
                    densityMap.at<float>(y, x) = weight;
                }
            }
        }

        // Apply adaptive blur based on feature density
        int blurSize = (featureDensity < 0.05) ? 31 : 21;
        cv::GaussianBlur(densityMap, densityMap, cv::Size(blurSize, blurSize), 0);

        // Find density peak
        cv::Point maxLoc;
        double maxVal;
        cv::minMaxLoc(densityMap, nullptr, &maxVal, nullptr, &maxLoc);

        if (maxVal > 0)
        {
            // Normalized density score
            double densityScore = 0.3 + (maxVal / 20.0);

            if (densityScore > bestScore)
            {
                bestCenter = maxLoc;

                // Calculate radius from feature distribution
                cv::Mat nonZeroCoordinates;
                cv::findNonZero(redGreenMask, nonZeroCoordinates);

                // Compute average distance to colored pixels
                double sumDist = 0;
                int count = 0;
                for (int i = 0; i < nonZeroCoordinates.total(); i++)
                {
                    cv::Point pt = nonZeroCoordinates.at<cv::Point>(i);
                    double dist = cv::norm(pt - bestCenter);
                    if (dist < frame.cols / 2)
                    {
                        sumDist += dist;
                        count++;
                    }
                }

                // Calculate radius with safety check
                bestRadius = (count > 0) ? (sumDist / count) * 1.2 : (frame.cols / 5.0);
                bestScore = densityScore;
            }
        }
    }

    // Use the best center we found
    center = bestCenter;
    radius = bestRadius;

    // After we have detected the center and radius, find the orientation
    orientation = detectOrientation(frame, center, radius, redMask, greenMask, debug_mode);

    // IMPORTANT NEW ADDITION: Detect elliptical shape using the redGreenMask
    cv::RotatedRect ellipse;
    bool ellipseFound = detectEllipticalShape(frame, redGreenMask, ellipse);

    if (ellipseFound)
    {
        // Store ellipse parameters for output params
        axes = ellipse.size;
        angle = ellipse.angle;

        // Debug visualization of ellipse
        if (debug_mode)
        {
            // Create a new visualization matrix specifically for the ellipse debug view
            cv::Mat ellipseVis = colorFrame.clone();
            cv::ellipse(ellipseVis, ellipse, cv::Scalar(0, 255, 255), 2);

            // Add ellipse info
            cv::putText(ellipseVis, "Axes: " + std::to_string(int(ellipse.size.width / 2)) + "x" + std::to_string(int(ellipse.size.height / 2)),
                        cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
            cv::putText(ellipseVis, "Angle: " + std::to_string(int(ellipse.angle)),
                        cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);

            cv::imwrite("debug_frames/ellipse_" + std::to_string(camera_idx) + ".jpg", ellipseVis);
        }
    }
    else
    {
        // If ellipse detection failed, use circular parameters
        axes = cv::Size2f(radius * 2, radius * 2);
        angle = 0;

        if (debug_mode)
        {
            std::cout << "Ellipse detection failed for camera " << camera_idx + 1
                      << ", using circular approximation" << std::endl;
        }
    }

    return true;
}

// Create a standard calibration
DartboardCalibration DartboardCalibration::createStandardCalibration(cv::Point center, double radius,
                                                                     double orientation, cv::Size2f axes,
                                                                     double angle, int cameraIdx)
{
    DartboardCalibration calib;
    calib.center = center;
    calib.radius = radius;
    calib.orientation = orientation;
    calib.camera_index = cameraIdx;

    // Set elliptical parameters
    calib.axes = axes;
    calib.angle = angle;

    // Set standard ring proportions
    calib.bullRadius = radius * 0.07;
    calib.doubleRingInner = radius * 0.92;
    calib.doubleRingOuter = radius;
    calib.tripleRingInner = radius * 0.55;
    calib.tripleRingOuter = radius * 0.63;

    return calib;
}

// New method to detect dartboard orientation
double DartboardCalibration::detectOrientation(const cv::Mat &frame, const cv::Point &center,
                                               double radius, const cv::Mat &redMask,
                                               const cv::Mat &greenMask, bool debugMode)
{
    return 90.0; // Default orientation if invalid inputs
}

// Method to detect elliptical shape of dartboard
bool DartboardCalibration::detectEllipticalShape(const cv::Mat &frame,
                                                 const cv::Mat &redGreenMask,
                                                 cv::RotatedRect &ellipse)
{
    // Create a working copy of the mask and the frame
    cv::Mat workingMask = redGreenMask.clone();
    cv::Mat workingFrame = frame.clone();

    // Step 1: Enhance mask with multi-stage processing
    // Apply morphology to clean up the mask and connect features
    cv::morphologyEx(workingMask, workingMask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)));

    // Try to capture the outer ring with dilate + gradient
    cv::Mat dilatedMask, edgeMask;
    cv::dilate(workingMask, dilatedMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
    cv::morphologyEx(dilatedMask, edgeMask, cv::MORPH_GRADIENT,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // Combine the enhanced features
    workingMask = workingMask | edgeMask;

    // Step 2: Edge enhancement to improve boundary detection
    cv::Mat edges;
    cv::Canny(workingFrame, edges, 50, 150);
    cv::dilate(edges, edges, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    // Combine edge information with our mask where it makes sense
    cv::Mat combinedMask = workingMask.clone();
    for (int y = 0; y < edges.rows; y++)
    {
        for (int x = 0; x < edges.cols; x++)
        {
            if (edges.at<uchar>(y, x) > 0)
            {
                // Check if this edge point is near an existing mask point
                bool nearExisting = false;
                int checkRadius = 15; // Pixels to check around edge point

                for (int cy = max(0, y - checkRadius); cy < min(edges.rows, y + checkRadius + 1); cy++)
                {
                    for (int cx = max(0, x - checkRadius); cx < min(edges.cols, x + checkRadius + 1); cx++)
                    {
                        if (workingMask.at<uchar>(cy, cx) > 0)
                        {
                            nearExisting = true;
                            break;
                        }
                    }
                    if (nearExisting)
                        break;
                }

                // If this edge is near existing content, add it to mask
                if (nearExisting)
                {
                    combinedMask.at<uchar>(y, x) = 255;
                }
            }
        }
    }

    // Step 3: Find contours in the enhanced mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combinedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if (contours.empty())
        return false;

    // Step 4: Process contours to find the best elliptical shape
    cv::RotatedRect bestEllipse;
    double bestScore = -1;

    for (size_t i = 0; i < contours.size(); i++)
    {
        // Skip contours with too few points
        if (contours[i].size() < 6)
            continue;

        double area = cv::contourArea(contours[i]);
        // Skip very small contours
        if (area < 1000)
            continue;

        // Calculate contour metrics to evaluate its quality
        double perimeter = cv::arcLength(contours[i], true);
        double circularity = (4 * CV_PI * area) / (perimeter * perimeter);

        // Try to fit an ellipse
        cv::RotatedRect candidateEllipse;
        try
        {
            candidateEllipse = cv::fitEllipse(contours[i]);

            // Validate the ellipse parameters
            if (candidateEllipse.size.width < 30 || candidateEllipse.size.height < 30)
                continue;

            // Calculate the aspect ratio
            double aspectRatio = max(candidateEllipse.size.width, candidateEllipse.size.height) /
                                 min(candidateEllipse.size.width, candidateEllipse.size.height);

            // Dartboards shouldn't be extremely elongated
            if (aspectRatio > 2.5)
                continue;

            // Score this ellipse based on circularity, area, aspect ratio
            double score = circularity * 0.4 +
                           (1.0 / aspectRatio) * 0.3 +
                           (area / (frame.rows * frame.cols)) * 0.3;

            if (score > bestScore)
            {
                bestScore = score;
                bestEllipse = candidateEllipse;
            }
        }
        catch (...)
        {
            // Skip errors in ellipse fitting
            continue;
        }
    }

    // Step 5: Return the best ellipse if a good fit was found
    if (bestScore > 0)
    {
        ellipse = bestEllipse;
        return true;
    }

    return false;
}

// Multi-camera calibration orchestration
std::vector<DartboardCalibration> DartboardCalibration::calibrateMultipleCameras(const std::vector<cv::Mat> &frames, bool debugMode, int targetWidth, int targetHeight)
{
    std::vector<DartboardCalibration> calibrations;
    cout << "\n===== DARTBOARD CALIBRATION STARTED =====\n";

    if (frames.empty())
    {
        cerr << "No frames provided for calibration" << endl;
        return calibrations; // Return empty vector
    }

    for (size_t cam_idx = 0; cam_idx < frames.size(); cam_idx++)
    {
        if (frames[cam_idx].empty())
        {
            cerr << "WARN: Empty frame from camera " << cam_idx + 1 << endl;
            continue;
        }

        // Create and calibrate each camera
        DartboardCalibration calib;
        if (calib.calibrateSingleCamera(frames[cam_idx], cam_idx, debugMode))
        {
            calibrations.push_back(calib);
        }
        else
        {
            cout << "FAIL: Failed to calibrate camera " << cam_idx + 1 << endl;
        }
    }

    // Show multi-camera view with calibration overlay
    if (debugMode && !calibrations.empty())
    {
        dartboard_visualization::saveMultiCameraDebugView(
            frames,
            calibrations,
            {}, // No detections during calibration
            targetWidth,
            targetHeight,
            false); // Not using competition mode for debugging
    }

    cout << "===== DARTBOARD CALIBRATION COMPLETED =====\n";
    return calibrations;
}
