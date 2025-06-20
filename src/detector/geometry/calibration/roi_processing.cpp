#include "roi_processing.hpp"
#include "../utils/geometry_utils.hpp"
#include <algorithm>

using namespace cv;
using namespace std;

// @defentions: ROI = "Region of Interest" - it's about focusing processing on a specific area of the image to:
// -- Improve performance (process fewer pixels)
// -- Reduce noise (ignore irrelevant background)
// -- Focus on dartboard area (ignore walls, furniture, etc.)

namespace roi_processing
{
    Mat processROI(const Mat &frame, bool debug_mode, int camera_idx, const ROIParams &params)
    {
        // Determine center point
        Point center = (params.customCenter.x >= 0) ? params.customCenter : geometry_utils::calculateFrameCenter(frame);

        // Create elliptical ROI based on ACTUAL frame dimensions
        Mat mask = Mat::zeros(frame.size(), CV_8UC1);

        // Calculate ellipse size based on frame dimensions and scale factors
        int ellipseWidth = int(frame.cols * params.roiSizePercent * params.horizontalScale);
        int ellipseHeight = int(frame.rows * params.roiSizePercent * params.verticalScale);

        // Apply perspective margin
        ellipseWidth = int(ellipseWidth * params.perspectiveMargin);
        ellipseHeight = int(ellipseHeight * params.perspectiveMargin);

        // FIXED: OpenCV ellipse() expects semi-axes (half-widths), not full dimensions
        Size ellipseSize(ellipseWidth / 2, ellipseHeight / 2);
        ellipse(mask, center, ellipseSize, 0, 0, 360, Scalar(255), -1);

        // Apply mask and return cropped result
        Mat roiFrame;
        frame.copyTo(roiFrame, mask);

        // Debug output
        if (debug_mode)
        {
            system("mkdir -p debug_frames/roi_processing");
            imwrite("debug_frames/roi_processing/roi_frame_" + to_string(camera_idx) + ".jpg", roiFrame);
        }

        return roiFrame;
    }

} // namespace roi_processing