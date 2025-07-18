#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>

using namespace cv;
using namespace std;

namespace motion_processing
{
    // Motion data for a single camera
    struct MotionData
    {
        bool motion_detected = false; // Boolean result (for backward compatibility)
        double motion_ratio = 0.0;    // Actual motion intensity (0.0 to 1.0)
        int motion_pixels = 0;        // Number of pixels that changed
        int total_pixels = 0;         // Total pixels in frame
    };

    // Motion detection parameters
    struct MotionParams
    {
        // detectMotion params
        int morph_type = MORPH_RECT;   // Type of morphological kernel (MORPH_RECT, MORPH_ELLIPSE)
        int morph_kernel_size = 4;     // Size of morphological kernel
        double threshold_ratio = 0.05; // 5% of pixels changed
        int binary_threshold = 10;     // Threshold for binary difference
        int blur_kernel_size = 5;      // Size of Gaussian blur kernel
        int blur_sigma_x = 10;         // Sigma for Gaussian blur in X direction
        int blur_sigma_y = 10;         // Sigma for Gaussian blur in Y direction

        // Event-based dart detection params
        double spike_threshold = 0.08;     // High motion intensity that indicates potential dart hit
        double low_threshold = 0.001;      // Low motion intensity for stability detection
        int min_cameras_for_event = 2;     // Minimum cameras that must participate in dart event
        int spike_window_frames = 10;      // Frames to wait for other cameras to join spike
        int stability_frames = 10;         // Consecutive low-motion frames needed for stability
        int max_event_duration_ms = 10000; // Maximum time for dart event (safety timeout)
        int cooldown_period_ms = 1000;     // Cooldown after dart detection
    };

    // Event states for dart detection
    enum class DartEventState
    {
        IDLE,           // No motion detected
        SPIKE_DETECTED, // High motion detected, waiting for pattern confirmation
        STABILIZING,    // Waiting for motion to settle down
        END,            // Motion event finished,
        COOLDOWN        // Preventing immediate re-detection
    };

    // Session result information
    struct MotionResult
    {
        bool motion_finished = false;                        // Did a motion event just finish
        DartEventState current_state = DartEventState::IDLE; // Current detection state
        int detection_duration_ms = 0;                       // How long current detection has been running
    };

    // Process motion detection with event-based dart landing detection
    MotionResult processMotion(
        const vector<Mat> &current_frames,
        const vector<Mat> &background_frames,
        bool debug_mode = false,
        const MotionParams &params = MotionParams());

} // namespace motion_processing
