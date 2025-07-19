#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace dart_processing
{
    // Dart board state - exactly as you described
    enum class DartBoardState
    {
        CLEAN,  // No darts, matches background
        DART_1, // 1 dart on board
        DART_2, // 2 darts on board
        DART_3  // 3 darts on board
    };

    // Parameters for dart state detection
    struct DartParams
    {
        // background comparison parameters
        double background_diff_threshold = 20; // Minimum difference to consider a pixel changed

        // Frame processing parameters
        int blur_kernel_size = 5;  // Fill dart gaps
        int dilate_iterations = 3; // Control dart expansion
        int erode_iterations = 2;  // Control noise removal

        // Simple morphological operations
        int morph_kernel_size = 4; // Size of morphological kernel

        // Statbility frames
        int stability_frames = 6; // Frames needed to confirm state change / (3 cameras * 2 frames per camera)
    };

    // Per-camera detection result
    struct CameraDetectionResult
    {
        DartBoardState detected_state = DartBoardState::CLEAN;
        int total_changed_pixels = 0;              // Total changed pixels
        double change_ratio = 0.0;                 // Percentage of changed pixels
        int total_pixels = 0;                      // Total pixels in frame
        Point2f tip_position = Point2f(-1, -1);    // Position of dart tip if found
        Point2f center_position = Point2f(-1, -1); // Center of biggest dart shape
        bool tip_found = false;                    // Was tip found in this frame
    };

    // Result of dart state detection
    struct DartStateResult
    {
        DartBoardState current_state = DartBoardState::CLEAN;  // Current dartboard state
        DartBoardState previous_state = DartBoardState::CLEAN; // Previous dartboard state
        bool state_changed = false;                            // Did the state change this frame
        int confidence_frames = 0;                             // How many frames we've been confident in this state
        vector<CameraDetectionResult> camera_results;          // Results from each camera
    };

    // Process dart state detection using background comparison on all 3 cameras
    DartStateResult processDartState(
        const vector<Mat> &current_frames,
        const vector<Mat> &background_frames,
        bool movement_finished = false,
        bool debug_mode = false,
        const DartParams &params = DartParams());

} // namespace dart_processing