#include <iostream>
#include <algorithm>

#include "geometry_detector.hpp"
#include "calibration/geometry_calibration.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

GeometryDetector::GeometryDetector(bool debug_mode, int target_width, int target_height, int target_fps)
    : initialized(false), calibrated(false), debug_mode(debug_mode), target_width(target_width), target_height(target_height), target_fps(target_fps)
{
}

bool GeometryDetector::initialize(vector<VideoCapture> &cameras)
{
    // Try to load cached calibration first
    calibrations = cache::geometry::load();
    if (!calibrations.empty())
    {
        // Already calibrated, just set initialized
        log_info("Loaded cached calibration with " + to_string(calibrations.size()) + " cameras");

        // Load background frames
        background_frames = cache::geometry::loadBackgroundFrames();

        // set initialized and calibrated
        initialized = true;
        calibrated = true;
        return true;
    }

    log_info("Capturing frames for calibration...");
    vector<Mat> initial_frames = camera::captureAndAverageFrames(cameras, target_width, target_height, target_fps, 75); // Capture 75 frames for averaging

    if (!initial_frames.empty())
    {
        log_info("Performing immediate calibration...");

        calibrations = geometry_calibration::calibrateMultipleCameras(
            initial_frames,
            debug_mode,
            target_width,
            target_height);

        calibrated = !calibrations.empty();

        if (calibrated)
        {
            // Save frames as background (for dart detection)
            background_frames.clear();
            for (const auto &frame : initial_frames)
                background_frames.push_back(frame.clone());

            log_info("Initial calibration completed successfully");

            // Save calibration for future use
            if (cache::geometry::save(calibrations))
            {
                log_info("Saved calibration");
            }

            // Save background frames for dart detection
            if (cache::geometry::saveBackgroundFrames(background_frames))
            {
                log_info("Saved background frames");
            }

            initialized = true;
            calibrated = true;
        }
        else
        {
            log_error("Initial calibration failed");
            initialized = false;
            calibrated = false;
        }
    }
    else
    {
        log_error("No initial frames captured for calibration");
        initialized = false;
        calibrated = false;
    }

    return initialized;
}

// main function for detecting darts in the provided frames
vector<DartDetection> GeometryDetector::detectDarts(const vector<cv::Mat> &frames)
{
    // Calibrate if needed
    if (!calibrated)
    {
        log_error("Detector not calibrated, performing calibration...");
        return {};
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
        vector<DartDetection> camera_detections = findDarts(curr_processed, bg_processed, i);

        // Set camera index for each detection
        for (auto &det : camera_detections)
            det.camera_index = i;

        // Add to overall detections
        all_detections.insert(all_detections.end(), camera_detections.begin(), camera_detections.end());
    }

    return all_detections;
}

// Detect darts using background subtraction and contour analysis
vector<DartDetection> GeometryDetector::findDarts(const Mat &frame, const Mat &background, int camIndex)
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
    threshold(diff, thresh, 15, 255, THRESH_BINARY);

    // Clean up the binary image
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(thresh, thresh, MORPH_CLOSE, kernel);
    morphologyEx(thresh, thresh, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));

    // Debug visualization
    if (debug_mode)
    {
        system("mkdir -p debug_frames/geometry_detector");
        imwrite("debug_frames/geometry_detector/diff_image_" + to_string(camIndex) + ".jpg", diff);
        imwrite("debug_frames/geometry_detector/thresh_image_" + to_string(camIndex) + ".jpg", thresh);
    }

    // Find contours in the thresholded image
    vector<vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Process contours to find dart candidates
    vector<DartDetection> all_detections;

    // Debug frame for visualization
    Mat debug_frame;
    if (debug_mode)
    {
        debug_frame = frame.clone();
    }

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

        // Debug: Draw all candidate contours
        if (debug_mode)
        {
            drawContours(debug_frame, vector<vector<Point>>{contour}, -1, Scalar(0, 255, 0), 2);
            rectangle(debug_frame, bbox, Scalar(255, 0, 0), 1);
        }

        // Find point closest to any dartboard center
        Point best_point(-1, -1);
        double min_dist = numeric_limits<double>::max();
        const DartboardCalibration *closest_calib = nullptr;

        for (const auto &calib : calibrations)
        {
            for (const auto &pt : contour)
            {
                double dist = math::distanceToPoint(pt, calib.bullCenter);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    best_point = pt;
                    closest_calib = &calib;
                }
            }
        }

        int radius = 50; // hardcoded for now

        // Validate dart candidate
        if (best_point.x >= 0 && closest_calib)
        {
            double dist_to_center = math::distanceToPoint(best_point, closest_calib->bullCenter);
            if (dist_to_center <= radius * 1.5)
            {
                // Create detection
                DartDetection detection;
                detection.position = best_point;
                detection.confidence = 0.9 - (dist_to_center / (radius * 2.0));
                detection.score = calculateScore(best_point, *closest_calib);
                all_detections.push_back(detection);

                // Debug: Mark valid detections
                if (debug_mode)
                {
                    circle(debug_frame, best_point, 10, Scalar(0, 0, 255), 3);
                    putText(debug_frame, detection.score, Point(best_point.x + 15, best_point.y),
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
                }
            }
        }
    }

    // Save debug frame
    if (debug_mode && !debug_frame.empty())
    {
        imwrite("debug_frames/geometry_detector/dart_candidates_" + to_string(camIndex) + ".jpg", debug_frame);
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
            if (math::distanceToPoint(det.position, accepted.position) < DUPLICATE_THRESHOLD)
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

std::string GeometryDetector::calculateScore(
    const cv::Point &dartPosition,
    const DartboardCalibration &calib)
{

    // Dont currenlty work
    return "MISS";

    // -----

    // TODO: the fueture of this code is to calculate the score of a dart throw
    // Check if dart position is valid
}
