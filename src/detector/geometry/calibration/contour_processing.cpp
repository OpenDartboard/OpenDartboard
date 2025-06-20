#include "contour_processing.hpp"
#include <iostream>

using namespace cv;
using namespace std;

namespace contour_processing
{
    vector<vector<Point>> processContours( // FIXED: Correct function name
        const Mat &mask,
        const Mat &originalFrame, // FIXED: Add missing parameter
        int camera_idx,
        bool debug_mode,
        const ContourParams &params)
    {
        // Find all contours in the mask
        vector<vector<Point>> allContours;
        cv::findContours(mask.clone(), allContours, params.contourMode, params.contourMethod);

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
            cout << "DEBUG: Found " << filteredContours.size() << " contours for camera " << camera_idx << endl;

            // Create and save visualization
            Mat contourVis = visualizeContours(originalFrame, filteredContours, params);
            system("mkdir -p debug_frames/contour_processing");
            imwrite("debug_frames/contour_processing/contours_" + to_string(camera_idx) + ".jpg", contourVis);
        }

        return filteredContours;
    }

    cv::Mat visualizeContours(
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

    // TODO: determine if this is needed
    Mat createEnhancedContourMask(const Mat &inputMask, const ContourParams &params)
    {
        // Start with a copy of the input mask
        Mat contourMask = inputMask.clone();

        // Step 1: Dilate to connect broken segments
        Mat dilationKernel = getStructuringElement(
            params.dilationShape,
            Size(params.dilationKernelSize, params.dilationKernelSize));
        dilate(inputMask, contourMask, dilationKernel);

        // Step 2: Extract edges to highlight ring features
        Mat edges;
        Laplacian(contourMask, edges, CV_8U, params.laplacianKernelSize);
        threshold(edges, edges, params.edgeThreshold, 255, THRESH_BINARY);

        // Step 3: Combine original mask with edges
        contourMask = contourMask | edges;

        return contourMask;
    }

} // namespace contour_processing