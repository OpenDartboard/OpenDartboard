#pragma once

#include "../dart_detector.hpp"
#include "geometry_calibration.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

class GeometryDetector : public DartDetector
{
public:
    GeometryDetector(bool debug_mode = false, int target_width = 640, int target_height = 480);
    virtual ~GeometryDetector() = default;

    virtual bool initialize(const std::string &config_path) override;
    bool initialize(const std::string &config_path, const std::vector<cv::Mat> &initial_frames);
    virtual bool isInitialized() const override { return initialized; }
    virtual std::vector<DartDetection> detectDarts(const std::vector<cv::Mat> &frames) override;

protected:
    bool initialized;
    bool calibrated;
    bool debug_mode;
    int target_width;
    int target_height;
    std::vector<DartboardCalibration> calibrations;
    std::vector<cv::Mat> background_frames;

    // Detector methods - removed calibrateDartboard
    std::vector<DartDetection> findDarts(const cv::Mat &frame, const cv::Mat &background);
    cv::Mat preprocessFrame(const cv::Mat &frame, bool preserveColor = false);
};
