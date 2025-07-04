#pragma once

#include "../detector_interface.hpp"
#include "calibration/geometry_calibration.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

class GeometryDetector : public DetectorInterface
{
public:
    GeometryDetector(bool debug_mode, int target_width, int target_height, int target_fps);
    virtual ~GeometryDetector() = default;

    virtual bool initialize(vector<VideoCapture> &cameras) override;
    virtual bool isInitialized() const override { return initialized; }
    virtual vector<DartDetection> detectDarts(const vector<Mat> &frames) override;

protected:
    bool initialized;
    bool calibrated;
    bool debug_mode;
    int target_width;
    int target_height;
    int target_fps;
    vector<DartboardCalibration> calibrations;
    vector<Mat> background_frames;

    // Detector methods - removed calibrateDartboard
    vector<DartDetection> findDarts(const Mat &frame, const Mat &background, int camIndex);
    Mat preprocessFrame(const Mat &frame, bool preserveColor = false);
    string calculateScore(const Point &dartPosition, const DartboardCalibration &calib);
};
