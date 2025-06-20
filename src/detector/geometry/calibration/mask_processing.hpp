#pragma once

#include <opencv2/opencv.hpp>

namespace mask_processing
{
    // Parameters for mask processing - for future enhancements
    struct MaskParams
    {
        // Basic threshold parameters
        int binaryThreshold = 1; // Threshold value for binary conversion (any pixel > this becomes white)
    };

    // Simple utility to convert colored frame to binary mask
    // Input: redGreenFrame (with red/green/gray pixels) → Output: binary mask (white/black)
    cv::Mat processMask(
        const cv::Mat &redGreenFrame,
        int camera_idx = 0,
        bool debug_mode = false,
        const MaskParams &params = MaskParams());

} // namespace mask_processing
