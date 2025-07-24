#pragma once

#include <opencv2/opencv.hpp>
#include "dart_processing.hpp"
#include "../calibration/geometry_calibration.hpp"

using namespace cv;
using namespace std;

namespace score_processing
{
    // Score result for a single dart
    struct ScoreResult
    {
        string score = "MISS";                        // Dart score (S20, D5, T17, BULL, etc.)
        Point2f dartboard_position = Point2f(-1, -1); // Position on dartboard coordinate system
        Point2f pixel_position = Point2f(-1, -1);     // Original pixel position
        Point2f center_position = Point2f(-1, -1);    // Dart center position
        float confidence = 0.0f;                      // Scoring confidence
        int camera_index = -1;                        // Which camera detected this
        bool valid = false;                           // Is this a valid score result
    };

    // Process dart scoring from tip detection results
    ScoreResult processScore(
        const vector<Mat> &background_frames,
        const dart_processing::DartStateResult &dart_result,
        const vector<DartboardCalibration> &calib,
        bool debug_mode = false);

} // namespace score_processing
