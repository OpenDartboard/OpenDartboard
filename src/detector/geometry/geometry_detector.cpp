#include <iostream>
#include <algorithm>

#include "geometry_detector.hpp"
#include "calibration/geometry_calibration.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

// Constructor
GeometryDetector::GeometryDetector(bool debug_mode, int target_width, int target_height, int target_fps)
    : initialized(false), calibrated(false), debug_mode(debug_mode), target_width(target_width), target_height(target_height), target_fps(target_fps)
{
}

// Main process method - simplified to basic structure
DetectorResult GeometryDetector::process(const vector<Mat> &frames)
{
#ifdef DEBUG_VIA_VIDEO_INPUT
    if (!frames.empty())
    {
        Mat combined_raw = debug::createCombinedFrame(frames, "RAW");
        raw_streamer->push(combined_raw);
    }
#endif

    DetectorResult result;

    if (!calibrated || frames.empty())
    {
        return result;
    }

    // Process motion session - all motion logic is now handled in motion_processing
    motion_processing::MotionResult motion_result = motion_processing::processMotion(frames, background_frames, debug_mode);

    // Set motion detection result
    result.motion_detected = motion_result.dart_detected;

    // Log dart detection events
    if (motion_result.dart_detected)
    {
        log_info("DART DETECTED! Ready for dart localization and scoring.");
        // TODO: This is where you'd do dart localization using current frames
    }

    return result;
}

// Main initialization method
bool GeometryDetector::initialize(vector<VideoCapture> &cameras)
{
#ifdef DEBUG_VIA_VIDEO_INPUT
    // Initialize multiple debug streamers
    raw_streamer = make_unique<streamer>(8080, cameras[0].get(cv::CAP_PROP_FPS));
    cv::Mat startup_img_raw(target_height, target_width, CV_8UC3, cv::Scalar::all(0));
    cv::putText(startup_img_raw, "Raw Cameras", {50, 100}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 255, 0}, 2);
#endif

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
    vector<Mat> initial_frames = camera::captureAndAverageFrames(cameras, 30); // Capture 75 frames for averaging

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
