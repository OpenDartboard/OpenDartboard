#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>

using namespace cv;
using namespace std;

namespace motion_processing
{
    // Motion detection parameters
    struct MotionParams
    {
        double threshold_ratio = 0.05;      // 5% of pixels changed
        int binary_threshold = 25;          // Threshold for binary difference
        int morph_kernel_size = 5;          // Size of morphological kernel
        int morph_type = MORPH_RECT;        // Type of morphological kernel (MORPH_RECT, MORPH_ELLIPSE)
        int max_session_duration_ms = 2000; // Force end session after this time
        int cooldown_period_ms = 2300;      // Wait time after session ends before allowing new session
    };

    // Session result information
    struct MotionResult
    {
        bool session_active = false;       // Is a motion session currently active
        bool session_ended = false;        // Did a session just end this frame
        string end_reason = "";            // Why the session ended ("success", "timeout")
        int session_duration_ms = 0;       // How long the session lasted
        int cameras_participated = 0;      // How many cameras participated
        int total_cameras = 0;             // Total number of cameras
        vector<bool> currently_moving;     // Which cameras are currently moving
        vector<bool> participated_cameras; // Which cameras participated in this session
    };

    // Process motion detection with complete session management
    MotionResult processMotion(
        const vector<Mat> &current_frames,
        bool debug_mode = false,
        const MotionParams &params = MotionParams());

    // Detect motion in the current frames without session management
    vector<bool> detectMotion(
        const vector<Mat> &current_frames,
        bool debug_mode = false,
        const MotionParams &params = MotionParams());

} // namespace motion_processing
