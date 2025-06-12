#include "geometry_detector.hpp"
#include "geometry_dartboard.hpp"
#include "utils/dartboard_visualization.hpp"
#include "utils/math_utils.hpp"
#include "utils/color_utils.hpp" // Include new color utilities
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;

GeometryDetector::GeometryDetector(bool debug_mode, int target_width, int target_height)
    : initialized(false), calibrated(false), debug_mode(debug_mode),
      target_width(target_width), target_height(target_height)
{
}

bool GeometryDetector::initialize(const string &config_path)
{
    cout << "Initializing geometry detector..." << endl;
    initialized = true;
    calibrated = false;
    return true;
}

bool GeometryDetector::initialize(const string &config_path, const vector<cv::Mat> &initial_frames)
{
    if (!initialize(config_path))
        return false;

    if (!initial_frames.empty())
    {
        cout << "Performing immediate calibration..." << endl;

        calibrated = calibrateDartboard(initial_frames);
        if (calibrated)
        {
            // Save frames as background
            background_frames.clear();
            for (const auto &frame : initial_frames)
                background_frames.push_back(frame.clone());

            cout << "Initial calibration completed successfully" << endl;
        }
        else
        {
            cerr << "Initial calibration failed" << endl;
        }
    }

    return initialized;
}

// Detect darts in the provided frames
vector<DartDetection> GeometryDetector::detectDarts(const vector<cv::Mat> &frames)
{
    // Calibrate if needed
    if (!calibrated)
    {
        calibrated = calibrateDartboard(frames);
        if (!calibrated)
            return {};

        background_frames.clear();
        for (const auto &frame : frames)
            background_frames.push_back(frame.clone());

        return {}; // No darts during calibration
    }

    // Process each camera frame to find darts
    vector<DartDetection> all_detections;

    for (size_t i = 0; i < frames.size() && i < calibrations.size(); i++)
    {
        // Skip invalid frames
        if (frames[i].empty() || i >= background_frames.size() || background_frames[i].empty())
            continue;

        // Find darts in this camera's frame
        Mat curr_processed = preprocessFrame(frames[i]);
        Mat bg_processed = preprocessFrame(background_frames[i]);
        vector<DartDetection> camera_detections = findDarts(curr_processed, bg_processed);

        // Set camera index for each detection
        for (auto &det : camera_detections)
            det.camera_index = i;

        // Add to overall detections
        all_detections.insert(all_detections.end(), camera_detections.begin(), camera_detections.end());
    }

    // Show multi-camera debug view
    if (debug_mode && !all_detections.empty())
    {
        dartboard_visualization::saveMultiCameraDebugView(frames, calibrations, all_detections, target_width, target_height, false);
    }

    return all_detections;
}

// Detect darts using background subtraction and contour analysis
vector<DartDetection> GeometryDetector::findDarts(const Mat &frame, const Mat &background)
{
    if (frame.empty() || background.empty())
        return {};

    // Create grayscale images for processing
    cv::Mat frame_gray = frame.clone();
    cv::Mat bg_gray = background.clone();

    // Compute difference between current frame and background
    Mat diff;
    absdiff(frame_gray, bg_gray, diff);

    // Threshold to find significant changes
    Mat thresh;
    threshold(diff, thresh, 20, 255, THRESH_BINARY);

    // Clean up the binary image
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(thresh, thresh, MORPH_CLOSE, kernel);
    morphologyEx(thresh, thresh, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

    // Find contours in the thresholded image
    vector<vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Process contours to find dart candidates
    vector<DartDetection> all_detections;

    for (const auto &contour : contours)
    {
        // Filter by area and shape
        double area = contourArea(contour);
        if (area < 50)
            continue;

        Rect bbox = boundingRect(contour);
        double aspect_ratio = bbox.width / (double)bbox.height;
        if (aspect_ratio < 0.15 || aspect_ratio > 1.5)
            continue;

        // Find point closest to any dartboard center
        Point best_point(-1, -1);
        double min_dist = numeric_limits<double>::max();
        const DartboardCalibration *closest_calib = nullptr;

        for (const auto &calib : calibrations)
        {
            for (const auto &pt : contour)
            {
                double dist = math_utils::distanceToPoint(pt, calib.center);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    best_point = pt;
                    closest_calib = &calib;
                }
            }
        }

        // Validate dart candidate
        if (best_point.x >= 0 && closest_calib)
        {
            double dist_to_center = math_utils::distanceToPoint(best_point, closest_calib->center);
            if (dist_to_center <= closest_calib->radius * 1.5)
            {
                // Create detection
                DartDetection detection;
                detection.position = best_point;
                detection.confidence = 0.9 - (dist_to_center / (closest_calib->radius * 2.0));
                detection.score = GeometryDartboard::calculateScore(best_point, *closest_calib);
                all_detections.push_back(detection);
            }
        }
    }

    // Remove duplicates
    vector<DartDetection> filtered_detections;
    sort(all_detections.begin(), all_detections.end(),
         [](const DartDetection &a, const DartDetection &b)
         {
             return a.confidence > b.confidence;
         });

    const int DUPLICATE_THRESHOLD = 15;
    for (const auto &det : all_detections)
    {
        bool is_duplicate = false;
        for (const auto &accepted : filtered_detections)
        {
            if (math_utils::distanceToPoint(det.position, accepted.position) < DUPLICATE_THRESHOLD)
            {
                is_duplicate = true;
                break;
            }
        }

        if (!is_duplicate)
            filtered_detections.push_back(det);
    }

    return filtered_detections;
}

// Simple preprocessor that converts to grayscale and blurs
cv::Mat GeometryDetector::preprocessFrame(const cv::Mat &frame, bool preserveColor)
{
    // If we need to preserve color for detection, just apply blur
    if (preserveColor && frame.channels() == 3)
    {
        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
        return blurred;
    }

    // Traditional grayscale conversion for other operations
    cv::Mat gray;
    if (frame.channels() == 3)
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else
        gray = frame.clone();

    // Apply Gaussian blur
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    return gray;
}

// SIMPLIFIED: Calibrate dartboard - just detect circle and use fixed orientation
bool GeometryDetector::calibrateDartboard(const vector<cv::Mat> &frames)
{
    calibrations.clear();
    cout << "\n===== DARTBOARD CALIBRATION STARTED =====\n";

    if (frames.empty())
    {
        cerr << "No frames provided for calibration" << endl;
        return false;
    }

    for (size_t cam_idx = 0; cam_idx < frames.size(); cam_idx++)
    {
        cout << "Calibrating camera " << cam_idx + 1 << "..." << endl;

        if (frames[cam_idx].empty())
        {
            cerr << "Empty frame from camera " << cam_idx + 1 << endl;
            continue;
        }

        // CHANGED: Skip preprocessing - use the original color frame directly
        // This preserves color information for finding red/green rings
        cv::Point center;
        double radius;
        double orientation; // Add orientation parameter
        bool found = findDartboardCircle(frames[cam_idx], center, radius, orientation, cam_idx);

        // Debug logging for calibration
        cout << "DEBUG-CALIB: Processing camera " << cam_idx + 1 << endl;

        // CHANGED: Lower minimum radius threshold from 40 to 25 to accommodate smaller dartboards
        if (found && radius > 25) // Was: radius > 40
        {
            // Use helper function to create calibration with detected orientation
            DartboardCalibration calib = createCalibration(center, radius, orientation, cam_idx);

            // Log before adding to calibrations
            cout << "DEBUG-CALIB: Storing camera " << cam_idx + 1
                 << " center=(" << center.x << "," << center.y << ")" << endl;

            calibrations.push_back(calib);
            cout << "Camera " << cam_idx + 1 << " calibrated: center=("
                 << center.x << "," << center.y << "), radius=" << radius << endl;
        }
        else
        {
            cout << "Failed to calibrate camera " << cam_idx + 1 << endl;
        }
    }

    // Before saving multi-camera view, log all calibrations
    if (!calibrations.empty())
    {
        cout << "DEBUG-CALIB: Final calibration summary:" << endl;
        for (size_t i = 0; i < calibrations.size(); i++)
        {
            cout << "DEBUG-CALIB:   Camera " << i + 1
                 << " center=(" << calibrations[i].center.x << "," << calibrations[i].center.y
                 << "), radius=" << calibrations[i].radius << endl;
        }
    }

    // Show multi-camera view with calibration overlay
    if (debug_mode && !calibrations.empty())
    {
        // Use the centralized visualization function instead
        dartboard_visualization::saveMultiCameraDebugView(
            frames,
            calibrations,
            {}, // No detections during calibration
            target_width,
            target_height,
            false); // Not using competition mode for debugging
    }

    cout << "===== DARTBOARD CALIBRATION COMPLETED =====\n";
    return !calibrations.empty();
}

// Simplified Color-Based Dartboard Detection
bool GeometryDetector::findDartboardCircle(const cv::Mat &frame, cv::Point &center, double &radius, double &orientation, int camera_idx)
{
    // Basic validation
    if (frame.empty() || frame.channels() < 3)
        return false;

    cv::Mat colorFrame = frame.clone();
    cv::Point frameCenter(frame.cols / 2, frame.rows / 2);

    // OPTIMIZATION: Create circular ROI mask centered on frame - MOVED TO BEGINNING
    cv::Mat roiMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    // Start with a generous ROI covering most of the image but excluding corners
    int initialRoiRadius = std::min(frame.cols, frame.rows) / 2.0;
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
        cv::imwrite("debug_frames/red_green_only_" + std::to_string(camera_idx) + ".jpg", redGreenOnly);
        cv::imwrite("debug_frames/red_green_mask_" + std::to_string(camera_idx) + ".jpg", redGreenMask);
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
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contourMask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Debug visualization with NO NEW images - use existing contours_*.jpg
    if (debug_mode)
    {
        cv::Mat contourVis = colorFrame.clone();
        // IMPROVED: Make contours much bolder (thickness 3 instead of 1)
        cv::drawContours(contourVis, contours, -1, cv::Scalar(0, 255, 255), 3);
        cv::imwrite("debug_frames/contours_" + std::to_string(camera_idx) + ".jpg", contourVis);
    }

    //---------- STEP 3: MULTI-STRATEGY DETECTION ----------

    cv::Point bestCenter = frameCenter;
    double bestScore = 0;
    double bestRadius = std::min(frame.cols, frame.rows) / 3.0;

    // Strategy A: Bull's eye detection
    cv::Mat bullsEye = redMask.clone();
    cv::morphologyEx(bullsEye, bullsEye, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    std::vector<std::vector<cv::Point>> bullContours;
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

        std::vector<cv::Vec3f> circles;
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
                int roiSize = std::max(5, std::min(20, frame.cols / 30));

                // Check for red content near center (potential bull)
                cv::Rect roi(
                    std::max(0, circleCenter.x - roiSize),
                    std::max(0, circleCenter.y - roiSize),
                    std::min(2 * roiSize, redMask.cols - circleCenter.x + roiSize),
                    std::min(2 * roiSize, redMask.rows - circleCenter.y + roiSize));

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
    orientation = 270.0; // Standard orientation

    // Final debug visualization
    if (debug_mode)
    {
        // Draw circular ROI on final visualization
        cv::Mat finalVis = colorFrame.clone();
        cv::circle(finalVis, frameCenter, initialRoiRadius, cv::Scalar(70, 70, 70), 1);

        // Draw frame center reference
        cv::circle(finalVis, frameCenter, 5, cv::Scalar(255, 0, 0), -1);
        cv::putText(finalVis, "Frame Center", frameCenter + cv::Point(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

        // Draw detected center and radius
        cv::circle(finalVis, center, 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(finalVis, center, radius, cv::Scalar(0, 255, 255), 2);
        cv::putText(finalVis, "Detected Center", center + cv::Point(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        // Add score info
        cv::putText(finalVis, "Score: " + std::to_string(bestScore).substr(0, 4),
                    cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        cv::imwrite("debug_frames/final_detection_" + std::to_string(camera_idx) + ".jpg", finalVis);
    }

    std::cout << "Camera " << camera_idx + 1 << ": Center=(" << center.x << "," << center.y
              << "), radius=" << radius << ", confidence=" << bestScore << std::endl;

    return true;
}

// Add this helper function to create standard calibration
DartboardCalibration GeometryDetector::createCalibration(cv::Point center, double radius, double orientation, int cameraIdx)
{
    DartboardCalibration calib;
    calib.center = center;
    calib.radius = radius;
    calib.orientation = orientation; // Use the provided orientation
    calib.camera_index = cameraIdx;

    // Set standard ring proportions
    calib.bullRadius = radius * 0.07;
    calib.doubleRingInner = radius * 0.92;
    calib.doubleRingOuter = radius;
    calib.tripleRingInner = radius * 0.55;
    calib.tripleRingOuter = radius * 0.63;

    return calib;
}
