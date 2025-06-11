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

// IMPROVED: More robust color-based dartboard detection that starts from the center
// IMPROVED: More robust color-based dartboard detection that starts from the center
bool GeometryDetector::findDartboardCircle(const cv::Mat &frame, cv::Point &center, double &radius, int camera_idx)
{
    // Enhanced logging for better debugging
    cout << "DEBUG-FIND: Starting circle detection for camera " << camera_idx + 1
         << " (" << frame.cols << "x" << frame.rows << ")" << endl;

    // We need a color image for this approach
    if (frame.empty())
        return false;

    // Must be a color image
    cv::Mat colorFrame;
    if (frame.channels() == 3)
        colorFrame = frame.clone();
    else
    {
        cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
        cerr << "Warning: Using grayscale image for color-based detection" << endl;
    }

    // Create debug visualization
    cv::Mat debugVis;
    if (debug_mode)
        debugVis = colorFrame.clone();

    // STEP 1: Use Hough Circles as an initial detector (more reliable than color-based)
    cv::Mat grayFrame;
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayFrame, grayFrame, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circles;
    // Parameters optimized for dartboard detection
    cv::HoughCircles(grayFrame, circles, cv::HOUGH_GRADIENT, 1,
                     grayFrame.rows / 4,  // Minimum distance between circles
                     100, 30,             // Canny/accumulator thresholds
                     grayFrame.rows / 8,  // Min radius - don't detect tiny circles
                     grayFrame.rows / 2); // Max radius - don't go too large

    cv::Point houghCenter;
    double houghRadius = 0;
    bool foundHough = false;

    if (!circles.empty())
    {
        // Sort circles by size (prefer larger ones as they're more likely the dartboard)
        std::sort(circles.begin(), circles.end(),
                  [](const cv::Vec3f &a, const cv::Vec3f &b)
                  { return a[2] > b[2]; });

        // Use the largest circle that's not too big
        for (const auto &circle : circles)
        {
            float x = circle[0];
            float y = circle[1];
            float r = circle[2];

            // Accept if circle is centered-ish in the frame
            cv::Point frameCenter(frame.cols / 2, frame.rows / 2);
            double centerDist = cv::norm(cv::Point(x, y) - frameCenter);
            if (centerDist < frame.cols / 3)
            {
                houghCenter = cv::Point(x, y);
                houghRadius = r;
                foundHough = true;
                break;
            }
        }
    }

    // Use Hough circle if found, otherwise fall back to frame center
    cv::Point frameCenter(frame.cols / 2, frame.rows / 2);
    cv::Point searchCenter = foundHough ? houghCenter : frameCenter;

    if (debug_mode && foundHough)
    {
        cv::circle(debugVis, houghCenter, houghRadius, cv::Scalar(255, 0, 255), 2);
        cv::putText(debugVis, "Hough", houghCenter - cv::Point(20, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
    }

    // STEP 2: Define search area around detected circle or frame center
    int searchRadius = foundHough ? houghRadius * 1.5 : min(frame.cols, frame.rows) * 0.4;
    cv::Rect centerRegion(
        max(0, searchCenter.x - searchRadius),
        max(0, searchCenter.y - searchRadius),
        min(frame.cols - 1, searchRadius * 2),
        min(frame.rows - 1, searchRadius * 2));

    // Convert to HSV for better color segmentation
    cv::Mat hsvFrame;
    cv::cvtColor(colorFrame, hsvFrame, cv::COLOR_BGR2HSV);

    // STEP 3: Create improved color masks with more precise thresholds
    cv::Mat redMask1, redMask2, redMask, greenMask;

    // Red comes in two ranges in HSV space (more permissive ranges to catch varying lighting)
    cv::inRange(hsvFrame, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), redMask1);
    cv::inRange(hsvFrame, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), redMask2);
    redMask = redMask1 | redMask2;

    // Green ring detection (more permissive range)
    cv::inRange(hsvFrame, cv::Scalar(30, 30, 30), cv::Scalar(90, 255, 255), greenMask);

    // Combined color mask
    cv::Mat colorMask = redMask | greenMask;

    // Apply morphology to clean noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_CLOSE, kernel);

    // STEP 4: Find bull's eye (red circle)
    cv::Mat bullMask = redMask.clone();
    // Apply distance transform to emphasize center points
    cv::Mat distTransform;
    cv::distanceTransform(bullMask, distTransform, cv::DIST_L2, 3);

    // Normalize and threshold to find peaks
    cv::normalize(distTransform, distTransform, 0, 1.0, cv::NORM_MINMAX);
    cv::Mat bullPeaks;
    cv::threshold(distTransform, bullPeaks, 0.5, 1.0, cv::THRESH_BINARY);

    // Convert to 8-bit for findContours
    bullPeaks.convertTo(bullPeaks, CV_8U, 255);

    vector<vector<cv::Point>> bullContours;
    cv::findContours(bullPeaks, bullContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point bullCenter(-1, -1);
    double bullRadius = 0;
    double bestBullScore = 0;

    // First check if we already have a center from Hough circles
    if (foundHough)
    {
        bullCenter = houghCenter;
        // Estimate bullseye radius as 7% of dartboard radius
        bullRadius = houghRadius * 0.07;
        bestBullScore = 1.0;
    }
    else
    {
        // Fall back to contour analysis
        for (const auto &contour : bullContours)
        {
            double area = cv::contourArea(contour);
            if (area < 10)
                continue;

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            // Score based on circularity and proximity to search center
            double circularity = area / (CV_PI * radius * radius);
            double centerDistance = cv::norm(cv::Point2f(searchCenter) - center);
            double centralityScore = 1.0 - (centerDistance / (searchRadius * 1.0));
            double score = circularity * 0.6 + centralityScore * 0.4;

            if (score > bestBullScore && radius < searchRadius * 0.2)
            {
                bestBullScore = score;
                bullCenter = cv::Point(center.x, center.y);
                bullRadius = radius;
            }
        }
    }

    // If we found a bullseye, use it; otherwise use Hough center or fall back to frame center
    if (bullCenter.x > 0 && bestBullScore > 0.3)
    {
        center = bullCenter;
    }
    else if (foundHough)
    {
        center = houghCenter;
    }
    else
    {
        center = frameCenter;
    }

    // STEP 5: Determine dartboard radius using circular Hough transform data or edge analysis
    if (foundHough)
    {
        radius = houghRadius;
    }
    else
    {
        // Use edge detection to find radius from detected center
        cv::Mat edges;
        cv::Canny(grayFrame, edges, 50, 150);

        // Analyze edges radiating from center to find outer circle
        const int NUM_RAYS = 36;
        std::vector<double> edgeDistances;

        for (int i = 0; i < NUM_RAYS; i++)
        {
            double angle = i * (2 * CV_PI / NUM_RAYS);
            double maxDist = min(frame.cols, frame.rows) * 0.45; // Maximum search distance

            for (double r = 20; r < maxDist; r += 0.5)
            {
                int x = center.x + r * cos(angle);
                int y = center.y + r * sin(angle);

                if (x < 0 || y < 0 || x >= edges.cols || y >= edges.rows)
                    break;

                if (edges.at<uchar>(y, x) > 0)
                {
                    edgeDistances.push_back(r);
                    break;
                }
            }
        }

        // Use median distance to avoid outliers
        if (!edgeDistances.empty())
        {
            std::sort(edgeDistances.begin(), edgeDistances.end());
            radius = edgeDistances[edgeDistances.size() / 2]; // Median value
        }
        else
        {
            // Fallback if edge detection fails
            radius = min(frame.cols, frame.rows) / 3.0;
        }
    }

    // Ensure minimum radius
    radius = max(radius, 25.0);

    // STEP 6: Create debug visualization
    if (debug_mode)
    {

        // Create a temporary calibration object for visualization
        DartboardCalibration tempCalib;
        tempCalib.center = center;
        tempCalib.radius = radius;
        tempCalib.orientation = 270.0; // Fixed - segment 20 at top
        tempCalib.camera_index = camera_idx;

        // Set standard ring proportions
        tempCalib.bullRadius = radius * 0.07;
        tempCalib.doubleRingInner = radius * 0.92;
        tempCalib.doubleRingOuter = radius;
        tempCalib.tripleRingInner = radius * 0.55;
        tempCalib.tripleRingOuter = radius * 0.63;

        // Create debug view and scale calibration using shared utility method
        cv::Mat debugVis = cv::Mat(target_height, target_width, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Point offset;

        // Scale calibration for display - use the centralized utility method
        DartboardCalibration displayCalib = dartboard_visualization::scaleCalibrationForDisplay(
            tempCalib, colorFrame, target_width, target_height, offset);

        // Resize the frame and place it on the background
        cv::Mat properlyResized;
        int targetWidth = target_width - 2 * offset.x;
        int targetHeight = target_height - 2 * offset.y;
        cv::resize(colorFrame, properlyResized, cv::Size(targetWidth, targetHeight));
        cv::Mat roi = debugVis(cv::Rect(offset.x, offset.y, targetWidth, targetHeight));
        properlyResized.copyTo(roi);

        // Use the centralized renderer
        dartboard_visualization::drawCalibrationOverlay(debugVis, displayCalib, true);

        // Add camera identifier
        cv::rectangle(debugVis, cv::Point(target_width - 160, 10), cv::Point(target_width - 10, 40),
                      cv::Scalar(0, 0, 0), -1);
        cv::putText(debugVis, "Camera " + to_string(camera_idx + 1) + " Overlay",
                    cv::Point(target_width - 150, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 255), 1);

        // Save debug image
        string dir = "debug_frames/circles";
        system(("mkdir -p " + dir).c_str());
        cv::imwrite(dir + "/camera" + to_string(camera_idx + 1) + "_overlay.jpg", debugVis);
    }

    // Final coordinates with more detailed logging
    cout << "DEBUG-FIND: Final center for camera " << camera_idx + 1 << "=("
         << center.x << "," << center.y << "), radius=" << radius
         << ", offset from frame center=("
         << (center.x - frame.cols / 2) << "," << (center.y - frame.rows / 2) << ")" << endl;

    return true;
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
        bool found = findDartboardCircle(frames[cam_idx], center, radius, cam_idx);

        // Debug logging for calibration
        cout << "DEBUG-CALIB: Processing camera " << cam_idx + 1 << endl;

        // CHANGED: Lower minimum radius threshold from 40 to 25 to accommodate smaller dartboards
        if (found && radius > 25) // Was: radius > 40
        {
            // Always use fixed orientation - standard dartboard
            DartboardCalibration calib;
            calib.center = center;
            calib.radius = radius;
            calib.orientation = 270.0;    // Fixed - segment 20 at top
            calib.camera_index = cam_idx; // CRITICAL: This stores the actual camera index

            // Set standard ring proportions
            calib.bullRadius = radius * 0.07;
            calib.doubleRingInner = radius * 0.92;
            calib.doubleRingOuter = radius;
            calib.tripleRingInner = radius * 0.55;
            calib.tripleRingOuter = radius * 0.63;

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
