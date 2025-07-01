#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace mask_processing
{
    // Parameters for mask processing
    struct MaskParams
    {
        int binaryThreshold = 1; // Threshold for binary mask creation

        // Preprocessing parameters (moved from ellipse_processing)
        int maskCloseKernelSize = 7;  // Kernel size for morphological closing
        int maskOpenKernelSize = 3;   // Kernel size for morphological opening
        int maskDilateKernelSize = 5; // Kernel size for dilation
    };

    // Bundle of all masks using proper dartboard terminology
    struct MaskBundle
    {
        Mat fullMask;         // Raw mask before any processing/carving
        Mat doublesMask;      // Doubles ring area (preprocessed for ellipse detection)
        Mat triplesMask;      // Triples ring area
        Mat outerBullMask;    // 25-point ring (single bull)
        Mat bullMask;         // 50-point bullseye (double bull) - carved red area
        bool isValid = false; // Flag if mask generation succeeded
    };

    // Return bundle of masks using dartboard terminology
    MaskBundle processMask(
        const Mat &redGreenFrame,
        Point bullCenter,
        int camera_idx,
        bool debug_mode = false,
        const MaskParams &params = MaskParams());

} // namespace mask_processing
