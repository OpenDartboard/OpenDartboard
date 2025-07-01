#include "contour_processing.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

// NOTES: CURRENTLY THIS MODULE IS NOT USED IN THE NEW PIPELINE

namespace contour_processing
{
    Mat visualizeContours(
        const Mat &originalFrame,
        const vector<vector<Point>> &contours,
        const ContourParams &params)
    {

        Mat visualization = originalFrame.clone();
        drawContours(
            visualization,
            contours,
            -1, // Draw all contours
            params.contourColor,
            params.contourThickness);

        return visualization;
    }

    vector<vector<Point>> processContours(
        const Mat &redGreenFrame,
        const Mat &originalFrame,
        int camera_idx,
        bool debug_mode,
        const ContourParams &params)
    {
        // Find all contours in the mask
        vector<vector<Point>> allContours;
        // convert to black and white mask
        Mat mask;
        cvtColor(redGreenFrame, mask, COLOR_BGR2GRAY); // Convert colored frame to grayscale
        threshold(mask, mask, 1, 255, THRESH_BINARY);  // Threshold: any non-black pixel becomes white
        findContours(mask.clone(), allContours, params.contourMode, params.contourMethod);

        // Filter contours based on area
        vector<vector<Point>> filteredContours;
        for (const auto &contour : allContours)
        {
            double area = contourArea(contour);
            if (area >= params.minContourArea)
            {
                filteredContours.push_back(contour);
            }
        }

        // Sort by area (largest first)
        sort(filteredContours.begin(), filteredContours.end(),
             [](const vector<Point> &a, const vector<Point> &b)
             {
                 return contourArea(a) > contourArea(b);
             });

        // Debug output with optional visualization
        if (debug_mode)
        {
            log_debug("Found " + log_string(filteredContours.size()) + " contours for camera " + log_string(camera_idx));

            // Create and save visualization
            Mat contourVis = visualizeContours(originalFrame, filteredContours, params);
            system("mkdir -p debug_frames/contour_processing");
            imwrite("debug_frames/contour_processing/contours_" + to_string(camera_idx) + ".jpg", contourVis);
        }

        return filteredContours;
    }

} // namespace contour_processing