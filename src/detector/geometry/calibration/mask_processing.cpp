#include "mask_processing.hpp"

using namespace cv;

namespace mask_processing
{
    Mat processMask(const Mat &redGreenFrame, int camera_idx, bool debug_mode, const MaskParams &params)
    {
        Mat mask;

        // Convert colored frame to grayscale
        cvtColor(redGreenFrame, mask, COLOR_BGR2GRAY);

        // Threshold: any non-black pixel becomes white
        threshold(mask, mask, params.binaryThreshold, 255, THRESH_BINARY);

        // Debug output (consistent with other modules)
        if (debug_mode)
        {
            system("mkdir -p debug_frames/mask_processing");
            imwrite("debug_frames/mask_processing/mask_" + std::to_string(camera_idx) + ".jpg", mask);
        }

        return mask;
    }

} // namespace mask_processing
