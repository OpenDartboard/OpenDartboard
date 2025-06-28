#include <opencv2/imgproc.hpp>
#include <iostream>

#include "wire_processing.hpp"
#include "../../../utils/math.hpp"

using namespace cv;
using namespace std;

namespace wire_processing
{
    Mat detectMetalWires(const Mat &frame, const Mat &colorMask, const DartboardCalibration &calib)
    {
        Mat wireEnhanced;

        // STEP 1: Create ROI mask with 5% buffer around double outer ring
        Mat roiMask = Mat::zeros(frame.size(), CV_8UC1);
        RotatedRect bufferedRing = calib.doubleOuterRing;
        bufferedRing.size.width *= 1.05f;
        bufferedRing.size.height *= 1.05f;
        ellipse(roiMask, bufferedRing, Scalar(255), -1);

        // Apply ROI mask to source image
        Mat roiImage;
        frame.copyTo(roiImage, roiMask);

        // STEP 2: Convert to HSV for better color filtering
        Mat hsv;
        cvtColor(roiImage, hsv, COLOR_BGR2HSV);

        // STEP 3: Remove black pixels more aggressively (keep only bright pixels)
        Mat nonBlackMask;
        inRange(hsv, Scalar(0, 0, 100), Scalar(180, 255, 255), nonBlackMask);

        // STEP 4: Remove green pixels (HSV range for green)
        Mat nonGreenMask;
        inRange(hsv, Scalar(40, 50, 50), Scalar(80, 255, 255), nonGreenMask);
        bitwise_not(nonGreenMask, nonGreenMask);

        // STEP 5: Remove red pixels (HSV range for red - two ranges due to hue wrap)
        Mat redMask1, redMask2, nonRedMask;
        inRange(hsv, Scalar(0, 50, 50), Scalar(15, 255, 255), redMask1);
        inRange(hsv, Scalar(165, 50, 50), Scalar(180, 255, 255), redMask2);
        bitwise_or(redMask1, redMask2, nonRedMask);
        bitwise_not(nonRedMask, nonRedMask);

        // STEP 6: Combine all masks to keep only bright, non-green, non-red pixels
        bitwise_and(nonBlackMask, nonGreenMask, wireEnhanced);
        bitwise_and(wireEnhanced, nonRedMask, wireEnhanced);
        bitwise_and(wireEnhanced, roiMask, wireEnhanced);

        // STEP 7: Invert the mask so wire areas are white
        bitwise_not(wireEnhanced, wireEnhanced);
        bitwise_and(wireEnhanced, roiMask, wireEnhanced);

        // STEP 8: Simple morphological operations to clean up
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(wireEnhanced, wireEnhanced, MORPH_CLOSE, kernel);

        // Remove small noise
        Mat smallKernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        morphologyEx(wireEnhanced, wireEnhanced, MORPH_OPEN, smallKernel);

        // STEP 9: Extract green areas from colorMask and subtract them
        vector<Mat> colorChannels;
        split(colorMask, colorChannels);
        Mat greenChannel = colorChannels[1]; // Green channel from colorMask

        // Create green mask
        Mat greenMask;
        threshold(greenChannel, greenMask, 50, 255, THRESH_BINARY);

        // Subtract green areas from wireEnhanced (remove doubles/triples)
        subtract(wireEnhanced, greenMask, wireEnhanced);

        // STEP 10: Apply final tighter ROI mask (-5% from doubles ring) to clean up outer artifacts
        Mat finalRoiMask = Mat::zeros(frame.size(), CV_8UC1);
        RotatedRect tighterRing = calib.doubleOuterRing;
        tighterRing.size.width *= 0.95f; // -5% instead of +5%
        tighterRing.size.height *= 0.95f;
        ellipse(finalRoiMask, tighterRing, Scalar(255), -1);

        // Apply the final tighter mask
        bitwise_and(wireEnhanced, finalRoiMask, wireEnhanced);

        // STEP 11: Advanced noise cleanup - remove small isolated dots and artifacts
        vector<vector<Point>> contours;
        findContours(wireEnhanced, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Create clean mask by filtering out small contours
        Mat cleanMask = Mat::zeros(wireEnhanced.size(), CV_8UC1);
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i]);
            if (area > 100)
            { // Only keep contours larger than 100 pixels
                drawContours(cleanMask, contours, i, Scalar(255), -1);
            }
        }

        // Final morphological cleanup to smooth edges
        Mat finalKernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        morphologyEx(cleanMask, wireEnhanced, MORPH_CLOSE, finalKernel);

        return wireEnhanced;
    }

    // Function to find wire positions based on color transitions
    vector<Point2f> findWiresByColorTransitions(const Mat &mask, const Mat &colorMask, const DartboardCalibration &calib)
    {
        vector<Point2f> wirePoints;

        // Get the clean wedge mask
        Mat wireMask = detectMetalWires(mask, colorMask, calib);

        // Find all contours in the clean mask
        vector<vector<Point>> contours;
        findContours(wireMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // DEBUG: Save contour visualization
        Mat contourDebug = Mat::zeros(wireMask.size(), CV_8UC3);
        Mat wireDebug = Mat::zeros(wireMask.size(), CV_8UC3);

        // DEBUG: Create frame overlay visualization
        Mat frameOverlay = mask.clone();
        Mat wireMaskColor;
        cvtColor(wireMask, wireMaskColor, COLOR_GRAY2BGR);

        // Blend the frame with the mask (50% frame, 50% mask)
        Mat blended;
        addWeighted(frameOverlay, 0.7, wireMaskColor, 0.3, 0, blended);

        for (size_t i = 0; i < contours.size(); i++)
        {
            Scalar color(rand() % 256, rand() % 256, rand() % 256);
            drawContours(contourDebug, contours, i, color, 2);

            if (!contours[i].empty())
            {
                Moments moments = cv::moments(contours[i]);
                if (moments.m00 > 0)
                {
                    Point2f centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);
                    putText(contourDebug, to_string(i), centroid,
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
                }
            }
        }

        system("mkdir -p debug_frames/wire_processing");
        imwrite("debug_frames/wire_processing/contours_debug_" + to_string(calib.camera_index) + ".jpg", contourDebug);

        // Process each contour to find wedge boundaries (left and right edges)
        vector<pair<double, pair<float, float>>> wedgeData;

        for (const auto &contour : contours)
        {
            if (contour.size() < 5 || contourArea(contour) < 500)
                continue;

            vector<float> angles;
            Point2f center = Point2f(calib.center);

            for (const auto &point : contour)
            {
                Point2f direction = Point2f(point) - center;
                float angle = atan2(direction.y, direction.x);
                angles.push_back(angle);
            }

            sort(angles.begin(), angles.end());

            float leftAngle = angles.front();
            float rightAngle = angles.back();

            if (rightAngle - leftAngle > CV_PI)
            {
                float maxGap = 0;
                int gapIndex = 0;
                for (size_t i = 1; i < angles.size(); i++)
                {
                    float gap = angles[i] - angles[i - 1];
                    if (gap > maxGap)
                    {
                        maxGap = gap;
                        gapIndex = i;
                    }
                }
                leftAngle = angles[gapIndex];
                rightAngle = angles[gapIndex - 1];
            }

            double area = contourArea(contour);
            wedgeData.push_back({area, {leftAngle, rightAngle}});

            cout << "DEBUG: Wedge area=" << area << " leftAngle=" << (leftAngle * 180.0f / CV_PI)
                 << "° rightAngle=" << (rightAngle * 180.0f / CV_PI) << "°" << endl;
        }

        sort(wedgeData.begin(), wedgeData.end(),
             [](const auto &a, const auto &b)
             { return a.first > b.first; });

        // Store wire endpoints for consistent use across both debug visualizations
        vector<pair<Point2f, Point2f>> debugWirePairs;

        // Extract wire positions from wedge boundaries
        for (const auto &wedge : wedgeData)
        {
            float leftAngle = wedge.second.first;
            float rightAngle = wedge.second.second;

            // Calculate wire endpoints by intersecting with the actual doubles ring ellipse
            Point2f leftWire = math::intersectRayWithEllipse(Point2f(calib.center), leftAngle, calib.doubleOuterRing);
            Point2f rightWire = math::intersectRayWithEllipse(Point2f(calib.center), rightAngle, calib.doubleOuterRing);

            wirePoints.push_back(leftWire);
            wirePoints.push_back(rightWire);

            // Store for debug visualization consistency
            debugWirePairs.push_back({leftWire, rightWire});
        }

        // Draw debug visualizations using the SAME wire endpoints
        for (const auto &wirePair : debugWirePairs)
        {
            Point2f leftWire = wirePair.first;
            Point2f rightWire = wirePair.second;

            // Draw on wireDebug
            line(wireDebug, calib.center, leftWire, Scalar(0, 255, 0), 2);
            line(wireDebug, calib.center, rightWire, Scalar(0, 0, 255), 2);
            circle(wireDebug, leftWire, 3, Scalar(0, 255, 0), -1);
            circle(wireDebug, rightWire, 3, Scalar(0, 0, 255), -1);

            // Draw on frame overlay - SAME EXACT LINES
            line(blended, calib.center, leftWire, Scalar(0, 255, 0), 2);
            line(blended, calib.center, rightWire, Scalar(0, 0, 255), 2);
            circle(blended, leftWire, 5, Scalar(0, 255, 0), -1);
            circle(blended, rightWire, 5, Scalar(0, 0, 255), -1);
        }

        // Save both debug visualizations
        imwrite("debug_frames/wire_processing/wires_debug_" + to_string(calib.camera_index) + ".jpg", wireDebug);
        imwrite("debug_frames/wire_processing/frame_overlay_" + to_string(calib.camera_index) + ".jpg", blended);

        // Sort wire points by angle for consistent ordering
        sort(wirePoints.begin(), wirePoints.end(), [&calib](const Point2f &a, const Point2f &b)
             {
            Point2f dirA = a - Point2f(calib.center);
            Point2f dirB = b - Point2f(calib.center);
            float angleA = atan2(dirA.y, dirA.x);
            float angleB = atan2(dirB.y, dirB.x);
            return angleA < angleB; });

        cout << "DEBUG: WIRE_PROCESSING: Found " << wirePoints.size() << " wire boundaries from " << wedgeData.size() << " wedges" << endl;

        return wirePoints;
    }

    WireData processWires(const Mat &frame, const Mat &colorMask, const DartboardCalibration &calib, bool enableDebug)
    {
        WireData result;
        result.camera_index = calib.camera_index;

        if (!calib.hasDetectedEllipses || frame.empty() || colorMask.empty())
        {
            cout << "WIRE_PROCESSING: Invalid input data" << endl;
            return result;
        }

        cout << "WIRE_PROCESSING: Starting wire detection for camera " << calib.camera_index << endl;

        // Method 1: Use color transitions to find wires
        vector<Point2f> colorWires = findWiresByColorTransitions(frame, colorMask, calib);

        // Set up result
        result.wireEndpoints = colorWires;
        result.segmentNumbers = {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};
        result.isValid = (result.wireEndpoints.size() == 20);

        cout << "WIRE_PROCESSING: Found " << result.wireEndpoints.size() << " wires" << endl;

        // Handle debug output internally
        if (enableDebug)
        {
            cout << "WIRE_PROCESSING: Saving debug images to " << endl;

            // Create debug visualization using the FINAL PROCESSED wire endpoints
            Mat debug = frame.clone();

            // Draw detected wire endpoints - these are the FINAL results after sorting
            for (size_t i = 0; i < result.wireEndpoints.size(); i++)
            {
                Point2f wireEnd = result.wireEndpoints[i];

                // Draw wire line from center to endpoint
                line(debug, calib.center, wireEnd, Scalar(0, 255, 0), 2);

                // Draw endpoint circle
                circle(debug, wireEnd, 5, Scalar(0, 0, 255), -1);

                // Add wire index number (not segment number)
                putText(debug, to_string(i), Point(wireEnd.x + 10, wireEnd.y),
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
            }

            system("mkdir -p debug_frames/wire_processing");
            imwrite("debug_frames/wire_processing/wire_edges_" + to_string(calib.camera_index) + ".jpg", detectMetalWires(frame, colorMask, calib));
            imwrite("debug_frames/wire_processing/wire_result_" + to_string(calib.camera_index) + ".jpg", debug);
        }

        return result;
    }

} // namespace wire_processing