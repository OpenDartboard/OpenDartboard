#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "../detector_interface.hpp"
#include "calibration/geometry_calibration.hpp"
#include "streamer.hpp"

using namespace cv;
using namespace std;

class GeometryDetector : public DetectorInterface
{
public:
    GeometryDetector(bool debug_mode, int target_width, int target_height, int target_fps);
    virtual ~GeometryDetector() = default;

    virtual bool initialize(vector<VideoCapture> &cameras) override;
    virtual bool isInitialized() const override { return initialized; }

    // Process method that handles motion + detection
    virtual DetectorResult process(const vector<Mat> &frames) override;

protected:
    bool initialized;
    bool calibrated;
    bool debug_mode;
    int target_width;
    int target_height;
    int target_fps;
    bool is_moving = false;
    vector<DartboardCalibration> calibrations;
    vector<Mat> background_frames;
    vector<Mat> previous_frames;
    vector<bool> are_moving = {false, false, false};            // Track motion state for each camera
    vector<bool> was_moving_in_session = {false, false, false}; // Has moved during this session

    // Motion session timing
    chrono::steady_clock::time_point motion_session_start_time;
    chrono::steady_clock::time_point last_session_end_time;
    const int MAX_MOTION_SESSION_MS = 2000; // Force end after 2 seconds
    const int COOLDOWN_PERIOD_MS = 2300;    // Wait 2.3s after session ends before allowing new session

    vector<bool> detectMotion(const vector<Mat> &frames);

// Debugging streamers for visual output
#ifdef DEBUG_VIA_VIDEO_INPUT
    unique_ptr<streamer> raw_streamer; // Port 8080 - Raw camera feeds
#endif
};