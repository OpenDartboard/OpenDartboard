#pragma once
#include "../dart_detector.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Dartboard calibration data structure
struct DartboardCalibration
{
    cv::Point center;
    double radius;
    double orientation = 270.0; // Always use standard orientation
    double bullRadius;
    double doubleRingInner;
    double doubleRingOuter;
    double tripleRingInner;
    double tripleRingOuter;
    int camera_index = 0;
};

class GeometryDetector : public DartDetector
{
public:
    // Updated constructor to accept target resolution
    GeometryDetector(bool debug_mode = false, int target_width = 640, int target_height = 480);
    ~GeometryDetector() override;

    bool initialize(const std::string &config_path) override;
    bool initialize(const std::string &config_path, const std::vector<cv::Mat> &initial_frames);
    std::vector<DartDetection> detectDarts(const std::vector<cv::Mat> &frames) override;
    bool isInitialized() const override { return initialized; }

private:
    bool initialized;
    bool calibrated;
    bool debug_mode;
    int target_width;  // Target width for visualizations
    int target_height; // Target height for visualizations
    cv::Mat dartboard_map;
    std::vector<DartboardCalibration> calibrations;
    std::vector<cv::Mat> background_frames;
    const int segment_values[20] = {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};

    // Core methods
    bool calibrateDartboard(const std::vector<cv::Mat> &frames);
    std::vector<DartDetection> findDarts(const cv::Mat &frame, const cv::Mat &background);

    // Helpers
    bool findDartboardCircle(const cv::Mat &frame, cv::Point &center, double &radius, int camera_idx = 0);
    cv::Mat preprocessFrame(const cv::Mat &frame, bool preserveColor = false);
    void saveMultiCameraDebugView(const std::vector<cv::Mat> &frames,
                                  const std::vector<DartDetection> &detections);

    // Utility functions
    double distanceToPoint(const cv::Point &p1, const cv::Point &p2);
    double colorDifference(const cv::Vec3b &color1, const cv::Vec3b &color2);
    std::vector<cv::Vec3b> extractColorProfile(const cv::Mat &frame,
                                               const cv::Point &center,
                                               double radius,
                                               int samples = 72);

    // NEW: Utility for scaling calibration for visualization
    DartboardCalibration scaleCalibrationForDisplay(
        const DartboardCalibration &calib,
        const cv::Mat &originalFrame,
        int targetWidth, int targetHeight,
        cv::Point &outOffset);

    // Interface with renderer
    void drawDartboardCalibration(cv::Mat &frame, const DartboardCalibration &calib,
                                  bool detailed = false);
    std::string determineScore(const cv::Point &dartPosition, const DartboardCalibration &calib);
};
