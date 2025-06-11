#include "geometry_detector.hpp"
#include "geometry_dartboard.hpp"
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;

GeometryDetector::GeometryDetector(bool debug_mode, int target_width, int target_height)
    : initialized(false), calibrated(false), debug_mode(debug_mode),
      target_width(target_width), target_height(target_height)
{
}

GeometryDetector::~GeometryDetector()
{
    // Nothing to clean up
}

bool GeometryDetector::initialize(const std::string &config_path)
{
    std::cout << "Initializing geometry detector..." << std::endl;
    initialized = true;
    calibrated = false;
    return true;
}

bool GeometryDetector::initialize(const std::string &config_path, const std::vector<cv::Mat> &initial_frames)
{
    if (!initialize(config_path))
        return false;

    if (!initial_frames.empty())
    {
        std::cout << "Performing immediate calibration..." << std::endl;

        calibrated = calibrateDartboard(initial_frames);
        if (calibrated)
        {
            // Save frames as background
            background_frames.clear();
            for (const auto &frame : initial_frames)
                background_frames.push_back(frame.clone());

            std::cout << "Initial calibration completed successfully" << std::endl;
        }
        else
        {
            std::cerr << "Initial calibration failed" << std::endl;
        }
    }

    return initialized;
}

std::vector<DartDetection> GeometryDetector::detectDarts(const std::vector<cv::Mat> &frames)
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
    std::vector<DartDetection> all_detections;

    for (size_t i = 0; i < frames.size() && i < calibrations.size(); i++)
    {
        // Skip invalid frames
        if (frames[i].empty() || i >= background_frames.size() || background_frames[i].empty())
            continue;

        // Find darts in this camera's frame
        Mat curr_processed = preprocessFrame(frames[i]);
        Mat bg_processed = preprocessFrame(background_frames[i]);
        std::vector<DartDetection> camera_detections = findDarts(curr_processed, bg_processed);

        // Set camera index for each detection
        for (auto &det : camera_detections)
            det.camera_index = i;

        // Add to overall detections
        all_detections.insert(all_detections.end(), camera_detections.begin(), camera_detections.end());
    }

    // Show multi-camera debug view
    if (debug_mode && !all_detections.empty())
        saveMultiCameraDebugView(frames, all_detections);

    return all_detections;
}

// Detect darts using background subtraction and contour analysis
std::vector<DartDetection> GeometryDetector::findDarts(const Mat &frame, const Mat &background)
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
    std::vector<std::vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Process contours to find dart candidates
    std::vector<DartDetection> all_detections;

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
        double min_dist = std::numeric_limits<double>::max();
        const DartboardCalibration *closest_calib = nullptr;

        for (const auto &calib : calibrations)
        {
            for (const auto &pt : contour)
            {
                double dist = distanceToPoint(pt, calib.center);
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
            double dist_to_center = distanceToPoint(best_point, closest_calib->center);
            if (dist_to_center <= closest_calib->radius * 1.5)
            {
                // Create detection
                DartDetection detection;
                detection.position = best_point;
                detection.confidence = 0.9 - (dist_to_center / (closest_calib->radius * 2.0));
                detection.score = determineScore(best_point, *closest_calib);
                all_detections.push_back(detection);
            }
        }
    }

    // Remove duplicates
    std::vector<DartDetection> filtered_detections;
    std::sort(all_detections.begin(), all_detections.end(),
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
            if (distanceToPoint(det.position, accepted.position) < DUPLICATE_THRESHOLD)
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

// Wrapper for dartboard renderer drawing function
void GeometryDetector::drawDartboardCalibration(cv::Mat &frame,
                                                const DartboardCalibration &calib,
                                                bool detailed)
{
    // Create a copy with fixed orientation
    DartboardCalibration fixedCalib = calib;
    fixedCalib.orientation = 270.0; // Always use standard orientation
    GeometryDartboard::drawCalibrationOverlay(frame, fixedCalib, detailed);
}

// Wrapper for score calculation
std::string GeometryDetector::determineScore(const cv::Point &dartPosition,
                                             const DartboardCalibration &calib)
{
    return GeometryDartboard::calculateScore(dartPosition, calib);
}

// NEW: Utility method to scale calibration parameters for visualization
DartboardCalibration GeometryDetector::scaleCalibrationForDisplay(
    const DartboardCalibration &calib,
    const cv::Mat &originalFrame,
    int targetWidth, int targetHeight,
    cv::Point &outOffset)
{
    // Create a copy to modify
    DartboardCalibration scaledCalib = calib;

    // Calculate aspect ratio-preserving dimensions
    double aspectRatio = originalFrame.cols / (double)originalFrame.rows;
    int scaledWidth = targetWidth;
    int scaledHeight = static_cast<int>(scaledWidth / aspectRatio);

    // If height exceeds target, scale down proportionally
    if (scaledHeight > targetHeight)
    {
        scaledHeight = targetHeight;
        scaledWidth = static_cast<int>(scaledHeight * aspectRatio);
    }

    // Calculate offset to center frame on black background
    outOffset.x = (targetWidth - scaledWidth) / 2;
    outOffset.y = (targetHeight - scaledHeight) / 2;

    // Calculate scale factors
    double scaleX = scaledWidth / (double)originalFrame.cols;
    double scaleY = scaledHeight / (double)originalFrame.rows;

    // Apply scale and offset to center point
    scaledCalib.center.x = static_cast<int>(scaledCalib.center.x * scaleX) + outOffset.x;
    scaledCalib.center.y = static_cast<int>(scaledCalib.center.y * scaleY) + outOffset.y;

    // Use the smaller scale factor to avoid oval shapes
    double scaleFactor = std::min(scaleX, scaleY);
    scaledCalib.radius *= scaleFactor;

    // Update all ring proportions
    scaledCalib.bullRadius = scaledCalib.radius * 0.07;
    scaledCalib.doubleRingInner = scaledCalib.radius * 0.92;
    scaledCalib.doubleRingOuter = scaledCalib.radius;
    scaledCalib.tripleRingInner = scaledCalib.radius * 0.55;
    scaledCalib.tripleRingOuter = scaledCalib.radius * 0.63;

    return scaledCalib;
}

// Unified visualization function that shows all cameras
void GeometryDetector::saveMultiCameraDebugView(const std::vector<cv::Mat> &frames,
                                                const std::vector<DartDetection> &detections)
{
    if (!debug_mode || frames.empty())
        return;

    // Create resized frames
    std::vector<cv::Mat> resized_frames;
    std::vector<cv::Point> offsets; // Store offsets for each frame
    int max_width = 0, total_height = 0;

    for (const auto &frame : frames)
    {
        cv::Mat resized;
        cv::Point offset;

        if (frame.empty())
        {
            // Use placeholder for missing frames
            resized = cv::Mat(target_height, target_width, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::putText(resized, "Camera disconnected", cv::Point(180, 240),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            offset = cv::Point(0, 0); // No offset for placeholders
        }
        else
        {
            // Create black background image of standard size
            resized = cv::Mat(target_height, target_width, CV_8UC3, cv::Scalar(0, 0, 0));

            // Calculate aspect-ratio preserving dimensions and place the frame
            int targetWidth = target_width;
            int targetHeight = static_cast<int>(targetWidth / (frame.cols / (double)frame.rows));

            if (targetHeight > target_height)
            {
                targetHeight = target_height;
                targetWidth = static_cast<int>(targetHeight * (frame.cols / (double)frame.rows));
            }

            offset.x = (target_width - targetWidth) / 2;
            offset.y = (target_height - targetHeight) / 2;

            // Resize the actual frame
            cv::Mat properlyResized;
            cv::resize(frame, properlyResized, cv::Size(targetWidth, targetHeight));

            // Create ROI and copy the resized frame
            cv::Mat roi = resized(cv::Rect(offset.x, offset.y, targetWidth, targetHeight));
            properlyResized.copyTo(roi);
        }

        resized_frames.push_back(resized);
        offsets.push_back(offset);
        max_width = std::max(max_width, resized.cols);
        total_height += resized.rows;
    }

    // Create combined image
    cv::Mat combined = cv::Mat::zeros(total_height, max_width, CV_8UC3);
    int y_offset = 0;

    std::cout << "DEBUG-MULTIVIEW: Creating combined camera view" << std::endl;

    // Add title to the multi-view image
    cv::putText(combined, "Dartboard Multi-Camera View",
                cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                0.8, cv::Scalar(255, 255, 255), 2);

    // Add each camera view
    for (size_t i = 0; i < resized_frames.size(); i++)
    {
        // Get region for this camera
        cv::Mat region = combined(cv::Rect(0, y_offset,
                                           resized_frames[i].cols,
                                           resized_frames[i].rows));

        // Copy frame
        resized_frames[i].copyTo(region);

        // Draw calibration overlay if available for this camera
        bool found_calibration = false;
        for (size_t calib_idx = 0; calib_idx < calibrations.size(); calib_idx++)
        {
            // Use the stored camera_index to find the right calibration
            if (calibrations[calib_idx].camera_index == i && !frames[i].empty())
            {
                // Use our utility method to get correctly scaled calibration
                cv::Point unused_offset; // We already have the offset
                DartboardCalibration displayCalib = scaleCalibrationForDisplay(
                    calibrations[calib_idx],
                    frames[i],
                    target_width,
                    target_height,
                    unused_offset);

                // Apply the stored offset
                displayCalib.center.x = (displayCalib.center.x - unused_offset.x) + offsets[i].x;
                displayCalib.center.y = (displayCalib.center.y - unused_offset.y) + offsets[i].y;

                std::cout << "DEBUG-MULTIVIEW: Drawing camera " << i + 1
                          << " center=(" << displayCalib.center.x << ","
                          << displayCalib.center.y << "), offset ("
                          << offsets[i].x << "," << offsets[i].y << ")" << std::endl;

                // Use the adjusted calibration for drawing
                drawDartboardCalibration(region, displayCalib, true);
                found_calibration = true;
                break;
            }
        }

        // If no calibration found for this camera, just add the camera label
        if (!found_calibration)
        {
            std::cout << "DEBUG-MULTIVIEW: No calibration for camera " << i + 1 << std::endl;
        }

        y_offset += region.rows;
    }

    // Save the debug view
    std::string dir = "debug_frames";
    system(("mkdir -p " + dir).c_str());
    cv::imwrite(dir + "/dartboard_view.jpg", combined);
}

// Utility for measuring point distances
double GeometryDetector::distanceToPoint(const cv::Point &p1, const cv::Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}
