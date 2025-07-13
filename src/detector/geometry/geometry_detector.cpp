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
// Motion detection function - similar to camera::detectMotion but with state
vector<bool> GeometryDetector::detectMotion(const vector<Mat> &frames)
{

    // Initialize previous frames on first run
    if (previous_frames.empty())
    {
        // Direct assignment
        previous_frames = frames;
    }

    // Detect current motion using camera utility
    vector<bool> current_motions = {false, false, false};

    // Motion detection for each camera (based on camera.hpp detectMotion)
    for (size_t i = 0; i < frames.size() && i < previous_frames.size(); i++)
    {
        if (frames[i].empty())
            continue;

        // Convert to grayscale
        Mat prev_gray, curr_gray;
        cvtColor(previous_frames[i], prev_gray, COLOR_BGR2GRAY);
        cvtColor(frames[i], curr_gray, COLOR_BGR2GRAY);

        // Calculate absolute difference
        Mat diff;
        absdiff(prev_gray, curr_gray, diff);

        // Apply threshold
        Mat thresh;
        threshold(diff, thresh, 25, 255, THRESH_BINARY);

        // Apply morphological operations to reduce noise
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(thresh, thresh, MORPH_CLOSE, kernel);

        // Debug: Save motion detection images
        // if (debug_mode)
        // {
        //     system("mkdir -p debug_frames/geometry_detector/motion/");
        //     imwrite("debug_frames/geometry_detector/motion/diff_cam_" + to_string(i) + ".jpg", diff);
        //     imwrite("debug_frames/geometry_detector/motion/thresh_cam_" + to_string(i) + ".jpg", thresh);
        //     imwrite("debug_frames/geometry_detector/motion/kernel_cam" + to_string(i) + ".jpg", kernel);
        // }

        // Count motion pixels and calculate ratio
        int motion_pixels = countNonZero(thresh);
        int total_pixels = thresh.rows * thresh.cols;
        double motion_ratio = (double)motion_pixels / total_pixels;
        double threshold_ratio = 0.05; // 5% of pixels changed

        if (motion_ratio > threshold_ratio)
        {
            current_motions[i] = true;
        }
    }

    return current_motions;
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

    if (previous_frames.empty())
    {
        // Direct assignment
        previous_frames = frames;
    }

    // Check if we have any motions detected in any of the frames (cameras)
    vector<bool> motions = GeometryDetector::detectMotion(frames);
    auto now = chrono::steady_clock::now();

    // Check if any camera detects motion this frame
    bool any_motion_this_frame = count(motions.begin(), motions.end(), true) > 0;

    // Check if we're in cooldown period
    auto cooldown_elapsed = chrono::duration_cast<chrono::milliseconds>(now - last_session_end_time).count();
    bool in_cooldown = cooldown_elapsed < COOLDOWN_PERIOD_MS;

    if (any_motion_this_frame && !is_moving && !in_cooldown)
    {
        // Start new motion session
        is_moving = true;
        motion_session_start_time = now;
        fill(are_moving.begin(), are_moving.end(), false);
        fill(was_moving_in_session.begin(), was_moving_in_session.end(), false);
        log_debug("Motion session started");
    }
    else if (any_motion_this_frame && !is_moving && in_cooldown)
    {
        cout << "DEBUG: Motion detected but in cooldown period (" << cooldown_elapsed << "ms/" << COOLDOWN_PERIOD_MS << "ms)" << endl;
    }

    if (is_moving)
    {
        // Update camera states during active session
        for (size_t i = 0; i < motions.size(); i++)
        {
            if (motions[i])
            {
                are_moving[i] = true;
                was_moving_in_session[i] = true;
            }
            else
            {
                are_moving[i] = false;
            }
        }

        // Check session end conditions IMMEDIATELY after updating states
        auto session_duration = chrono::duration_cast<chrono::milliseconds>(now - motion_session_start_time).count();
        int any_left_moving = count(are_moving.begin(), are_moving.end(), true);
        int cameras_that_moved = count(was_moving_in_session.begin(), was_moving_in_session.end(), true);
        int total_cameras = motions.size();

        // DEBUG: Show which specific cameras are moving and which participated
        cout << "DEBUG: duration=" << session_duration << "ms";
        cout << " | Currently moving: ";
        for (size_t i = 0; i < are_moving.size(); i++)
        {
            if (are_moving[i])
                cout << "cam" << i << " ";
        }
        cout << " | Participated: ";
        for (size_t i = 0; i < was_moving_in_session.size(); i++)
        {
            if (was_moving_in_session[i])
                cout << "cam" << i << " ";
        }
        cout << " | Total: " << cameras_that_moved << "/" << total_cameras;

        // Check success condition FIRST (before timeout)
        if (any_left_moving == 0 && cameras_that_moved == total_cameras)
        {
            cout << " -> ENDING SESSION (success)" << endl;
            log_debug("Motion session ended (all cameras participated and stopped) - duration: " + to_string(session_duration) + "ms, cameras participated: " + to_string(cameras_that_moved) + "/" + to_string(total_cameras));
            is_moving = false;
            last_session_end_time = now; // Start cooldown period
            // TODO: This is where you'd capture frames and do dart detection
        }
        else if (session_duration >= MAX_MOTION_SESSION_MS)
        {
            cout << " -> ENDING SESSION (timeout)" << endl;
            log_debug("Motion session ended (maximum timeout reached) - duration: " + to_string(session_duration) + "ms, cameras participated: " + to_string(cameras_that_moved) + "/" + to_string(total_cameras));
            is_moving = false;
            last_session_end_time = now; // Start cooldown period
            // TODO: This is where you'd capture frames and do dart detection
        }
        else
        {
            // cout << " -> continuing..." << endl;
        }
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