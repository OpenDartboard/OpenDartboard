#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

struct DartDetection
{
    cv::Point position;   // Position of the dart tip
    float confidence;     // How confident we are (0-1)
    std::string score;    // The detected score (e.g., "D20", "S5")
    int camera_index = 0; // Which camera detected this dart
};

// Abstract interface for any dart detection method
class DartDetector
{
public:
    virtual ~DartDetector() = default;

    // Initialize the detector
    virtual bool initialize(const std::string &model_path) = 0;

    // Process frames and return detected darts
    virtual std::vector<DartDetection> detectDarts(const std::vector<cv::Mat> &frames) = 0;

    // Whether the detector is ready
    virtual bool isInitialized() const = 0;
};
