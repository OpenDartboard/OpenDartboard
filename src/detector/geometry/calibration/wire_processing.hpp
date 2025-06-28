#pragma once

#include <opencv2/opencv.hpp>

#include "geometry_calibration.hpp"

using namespace cv;
using namespace std;

namespace wire_processing
{
    // Public data structures
    struct WireData
    {
        vector<Point2f> wireEndpoints;
        vector<int> segmentNumbers;
        int camera_index = -1;
        bool isValid = false;
    };

    // Public interfaces
    WireData processWires(const Mat &frame, const Mat &colorMask, const DartboardCalibration &calib, bool enableDebug = false);
}