#pragma once

#include <opencv2/opencv.hpp>

// @definitions: ROI = "Region of Interest" - Focus processing on dartboard area to:
// -- Improve performance (process fewer pixels)
// -- Reduce noise (ignore irrelevant background)
// -- Focus on dartboard area (ignore walls, furniture, etc.)

namespace roi_processing
{
    // All ROI parameters in header (clean approach)
    struct ROIParams
    {
        // ROI size and shape - FIXED: More reasonable values
        float roiSizePercent = 0.8f;                // Use 80% of frame dimensions
        cv::Point customCenter = cv::Point(-1, -1); // Optional custom center (-1,-1 = use frame center)

        // Perspective correction for angled dartboard view - FIXED: Better proportions
        float horizontalScale = 0.95f;  // Slightly narrower (was 1.1f - too wide!)
        float verticalScale = 1.1f;     // Taller to capture full dartboard height (was 0.9f - too short!)
        float perspectiveMargin = 1.0f; // No extra margin (was 1.2x!)

        // Future enhancement options
        bool adaptiveROI = false;    // Future: adjust ROI based on dartboard detection
        float adaptiveMargin = 0.1f; // Future: extra margin for adaptive ROI
    };

    // Main function: Extract ROI region from frame
    // Input: full frame → Output: cropped frame focused on dartboard area
    cv::Mat processROI(
        const cv::Mat &frame,
        bool debug_mode = false,
        int camera_idx = 0,
        const ROIParams &params = ROIParams());

} // namespace roi_processing
