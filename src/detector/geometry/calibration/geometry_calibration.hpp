#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "ellipse_processing.hpp"
#include "wire_processing.hpp"        // Include full definition for WireData
#include "orientation_processing.hpp" // Include orientation data

using namespace std;
using namespace cv;

struct DartboardCalibration
{
    // CORE: Basic calibration data
    Point bullCenter{0, 0};  // Bull center (true dartboard center)
    Point frameCenter{0, 0}; // Frame center
    int camera_index = -1;   // Which camera this is for
    int capture_width = -1;  // Width of the captured frame
    int capture_height = -1; // Height of the captured frame
    uint64_t timestamp = 0;  // Unix timestamp of when this calibration was captured

    // ELLIPSES: Dartboard shape and rings
    ellipse_processing::EllipseBoundaryData ellipses; // Contains all ellipse data

    // WIRE DATA: Contains wire positions for segment alignment
    wire_processing::WireData wires; // Contains wire positions and segment numbers

    // ORIENTATION: Dartboard rotation and "20" segment position
    orientation_processing::OrientationData orientation; // Where is the "20" segment?
};

// NAMESPACE WITH UTILITY FUNCTIONS
namespace geometry_calibration
{
    // Calibrate multiple cameras at once
    vector<DartboardCalibration> calibrateMultipleCameras(const vector<Mat> &frames, bool debugMode = false, int targetWidth = 640, int targetHeight = 480);
}