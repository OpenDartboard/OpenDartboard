#include "geometry_calibration.hpp"
#include "utils/dartboard_visualization.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

using namespace cv;
using namespace std;

DartboardCalibration::DartboardCalibration()
    : center(0, 0), radius(0), orientation(270.0),
      bullRadius(0), doubleRingInner(0), doubleRingOuter(0),
      tripleRingInner(0), tripleRingOuter(0), camera_index(-1)
{
}

// Method to create a standard calibration
bool DartboardCalibration::calibrateSingleCamera(const cv::Mat &frame, int cameraIdx, bool debugMode)
{
    cout << "DEBUG: Calibrating camera " << cameraIdx + 1 << "..." << endl;

    if (frame.empty())
    {
        cerr << "Empty frame from camera " << cameraIdx + 1 << endl;
        return false;
    }

    cv::Point center;
    double radius;
    double orientation;
    bool found = findDartboardCircle(frame, center, radius, orientation, cameraIdx, debugMode);

    if (found && radius > 25)
    {
        // Use helper function to create calibration with detected orientation
        *this = createStandardCalibration(center, radius, orientation, cameraIdx);
        return true;
    }
    else
    {
        cout << "FAIL: Failed to calibrate camera " << cameraIdx + 1 << endl;
        return false;
    }
}

// Simplified Color-Based Dartboard Detection
bool DartboardCalibration::findDartboardCircle(const cv::Mat &frame, cv::Point &center, double &radius, double &orientation, int camera_idx, bool debug_mode)
{
    // Basic validation
    if (frame.empty() || frame.channels() < 3)
        return false;

    cv::Mat colorFrame = frame.clone();
    cv::Point frameCenter(frame.cols / 2, frame.rows / 2);

    // MODIFICATION 1: Increase ROI radius by 10% to avoid cutting off dartboard edges
    cv::Mat roiMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    // Use 60% of minimum dimension instead of 50% (increase from half to 60% of frame)
    int initialRoiRadius = int(std::min(frame.cols, frame.rows) * 0.6);
    cv::circle(roiMask, frameCenter, initialRoiRadius, cv::Scalar(255), -1);

    // Create ROI-constrained color frame for all subsequent processing
    cv::Mat roiColorFrame;
    colorFrame.copyTo(roiColorFrame, roiMask);

    //---------- STEP 1: CREATE RED AND GREEN ONLY MASK ----------

    // Convert to HSV for better color separation - ONLY PROCESS WITHIN ROI
    cv::Mat hsvFrame;
    cv::cvtColor(roiColorFrame, hsvFrame, cv::COLOR_BGR2HSV);

    // UNIVERSAL approach - use wider ranges that work for ALL cameras
    cv::Mat redMask1, redMask2, redMask, greenMask, whiteMask;

    // Broad red detection that works universally (both primary and wrapped hue ranges)
    cv::inRange(hsvFrame, cv::Scalar(0, 40, 40), cv::Scalar(20, 255, 255), redMask1);
    cv::inRange(hsvFrame, cv::Scalar(160, 40, 40), cv::Scalar(180, 255, 255), redMask2);

    // Detect whitish areas for lighting variations
    cv::inRange(hsvFrame, cv::Scalar(0, 0, 150), cv::Scalar(180, 40, 255), whiteMask);

    // Green detection for triple ring with adaptive range
    cv::inRange(hsvFrame, cv::Scalar(30, 40, 40), cv::Scalar(90, 255, 255), greenMask);

    // Combine masks intelligently
    redMask = redMask1 | redMask2;
    cv::Mat redGreenMask = redMask | greenMask;

    // Add white areas if red/green detection is poor
    int colorCount = cv::countNonZero(redGreenMask);
    if (colorCount < (frame.rows * frame.cols * 0.05))
    { // If less than 5% detected
        redGreenMask = redGreenMask | whiteMask;
    }

    // Create visualization with ROI applied
    cv::Mat redGreenOnly = cv::Mat::zeros(frame.size(), CV_8UC3);
    for (int y = 0; y < redGreenMask.rows; y++)
    {
        for (int x = 0; x < redGreenMask.cols; x++)
        {
            if (redMask.at<uchar>(y, x) > 0)
            {
                redGreenOnly.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red
            }
            else if (greenMask.at<uchar>(y, x) > 0)
            {
                redGreenOnly.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // Green
            }
            else if (whiteMask.at<uchar>(y, x) > 0 && redGreenMask.at<uchar>(y, x) > 0)
            {
                redGreenOnly.at<cv::Vec3b>(y, x) = cv::Vec3b(200, 200, 200); // Light gray
            }
        }
    }

    // Save debug images
    if (debug_mode)
    {
        system("mkdir -p debug_frames");
        cv::imwrite("debug_frames/red_green_only_" + to_string(camera_idx) + ".jpg", redGreenOnly);
        cv::imwrite("debug_frames/red_green_mask_" + to_string(camera_idx) + ".jpg", redGreenMask);
    }

    //---------- STEP 2: ADAPTIVE MASK ENHANCEMENT ----------

    // Detect feature density to determine appropriate processing
    // Using ROI-masked versions for all calculations
    double featureDensity = cv::countNonZero(redGreenMask) / static_cast<double>(cv::countNonZero(roiMask));

    // Choose kernel size based on feature density - larger for sparse features
    int kernelSize = (featureDensity < 0.05) ? 7 : 5;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));

    // Apply more aggressive morphology for sparse features
    int iterations = (featureDensity < 0.05) ? 2 : 1;
    cv::morphologyEx(redGreenMask, redGreenMask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), iterations);

    // Create cleaner mask for circle detection
    cv::Mat circleMask = redGreenMask.clone();
    cv::morphologyEx(circleMask, circleMask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::morphologyEx(circleMask, circleMask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));

    // IMPROVED: Create enhanced contour mask to detect circular features better
    cv::Mat contourMask = redGreenMask.clone();

    // Simple ring enhancement - first dilate slightly to connect broken ring segments
    cv::dilate(redGreenMask, contourMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    // Then extract edges only - this will highlight the rings
    cv::Mat ringEdges;
    cv::Laplacian(contourMask, ringEdges, CV_8U, 3);
    cv::threshold(ringEdges, ringEdges, 10, 255, cv::THRESH_BINARY);

    // Combine original mask with edges to preserve all features while enhancing rings
    contourMask = contourMask | ringEdges;

    // Find contours using the enhanced mask within ROI
    vector<vector<cv::Point>> contours;
    cv::findContours(contourMask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Debug visualization with NO NEW images - use existing contours_*.jpg
    if (debug_mode)
    {
        cv::Mat contourVis = colorFrame.clone();
        // IMPROVED: Make contours much bolder (thickness 3 instead of 1)
        cv::drawContours(contourVis, contours, -1, cv::Scalar(0, 255, 255), 3);
        cv::imwrite("debug_frames/contours_" + to_string(camera_idx) + ".jpg", contourVis);
    }

    //---------- STEP 3: MULTI-STRATEGY DETECTION ----------

    cv::Point bestCenter = frameCenter;
    double bestScore = 0;
    double bestRadius = min(frame.cols, frame.rows) / 3.0;

    // Strategy A: Bull's eye detection
    cv::Mat bullsEye = redMask.clone();
    cv::morphologyEx(bullsEye, bullsEye, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    vector<vector<cv::Point>> bullContours;
    cv::findContours(bullsEye, bullContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Process bull contours
    for (const auto &contour : bullContours)
    {
        double area = cv::contourArea(contour);
        // Adaptive size filtering based on image size
        double minArea = 20;
        double maxArea = frame.total() * 0.005; // 0.5% of image max

        if (area < minArea || area > maxArea)
            continue;

        // Calculate circularity
        double perimeter = cv::arcLength(contour, true);
        double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        if (circularity < 0.5)
            continue;

        cv::Point2f center2f;
        float radius2f;
        cv::minEnclosingCircle(contour, center2f, radius2f);

        // Adaptive scoring based on distance from frame center
        double maxDist = cv::norm(cv::Point(0, 0) - cv::Point(frame.cols, frame.rows)) / 2.0;
        double distToCenter = cv::norm(cv::Point(center2f) - frameCenter);
        double centralityScore = 1.0 - (distToCenter / maxDist);

        // Combined score weighted by circularity and centrality
        double score = circularity * 0.6 + centralityScore * 0.4;

        if (score > bestScore)
        {
            bestScore = score;
            bestCenter = cv::Point(center2f);
            bestRadius = radius2f * 35; // Estimated board radius from bull
        }
    }

    // Strategy B: Circular pattern detection with adaptive parameters
    if (bestScore < 0.5)
    {
        // Adjust parameters based on feature density
        double dp = (featureDensity < 0.05) ? 1.0 : 1.2;
        int minDist = circleMask.rows / ((featureDensity < 0.05) ? 10 : 8);
        int cannyThreshold = (featureDensity < 0.05) ? 70 : 80;
        int accThreshold = (featureDensity < 0.05) ? 20 : 25;

        vector<cv::Vec3f> circles;
        cv::HoughCircles(circleMask, circles, cv::HOUGH_GRADIENT, dp, minDist,
                         cannyThreshold, accThreshold,
                         circleMask.rows / 12, circleMask.rows / 1.5);

        // Process detected circles
        if (!circles.empty())
        {
            double bestCircleScore = 0;
            int bestIdx = -1;

            for (size_t i = 0; i < circles.size(); i++)
            {
                cv::Point circleCenter(cvRound(circles[i][0]), cvRound(circles[i][1]));
                float circleRadius = circles[i][2];

                // Adaptive ROI size based on image dimensions
                int roiSize = max(5, min(20, frame.cols / 30));

                // Check for red content near center (potential bull)
                cv::Rect roi(
                    max(0, circleCenter.x - roiSize),
                    max(0, circleCenter.y - roiSize),
                    min(2 * roiSize, redMask.cols - circleCenter.x + roiSize),
                    min(2 * roiSize, redMask.rows - circleCenter.y + roiSize));

                // Avoid invalid ROIs
                if (roi.width <= 0 || roi.height <= 0)
                    continue;

                cv::Mat centerROI = redMask(roi);
                double centerRedness = cv::countNonZero(centerROI) / static_cast<double>(centerROI.total());

                // Adaptive centrality measure
                double maxDist = cv::norm(cv::Point(0, 0) - cv::Point(frame.cols, frame.rows)) / 2.0;
                double distToCenter = cv::norm(circleCenter - frameCenter);
                double centralityScore = 1.0 - (distToCenter / maxDist);

                // Balance between center proximity and redness
                double circleScore = centralityScore * 0.4 + centerRedness * 0.6;

                if (circleScore > bestCircleScore)
                {
                    bestCircleScore = circleScore;
                    bestIdx = i;
                }
            }

            // Update best detection if circle score is good
            if (bestIdx >= 0 && bestCircleScore > bestScore)
            {
                bestCenter = cv::Point(cvRound(circles[bestIdx][0]), cvRound(circles[bestIdx][1]));
                bestRadius = circles[bestIdx][2];
                bestScore = bestCircleScore;
            }
        }
    }

    // Strategy C: Color density map (universally applicable)
    if (bestScore < 0.4)
    {
        cv::Mat densityMap = cv::Mat::zeros(frame.size(), CV_32F);

        // Create density map with exponential distance weighting
        for (int y = 0; y < redGreenMask.rows; y++)
        {
            for (int x = 0; x < redGreenMask.cols; x++)
            {
                if (redGreenMask.at<uchar>(y, x) > 0)
                {
                    double dist = cv::norm(cv::Point(x, y) - frameCenter);
                    double weight = exp(-dist / (frame.cols / 2.0)) * 10.0;
                    densityMap.at<float>(y, x) = weight;
                }
            }
        }

        // Apply adaptive blur based on feature density
        int blurSize = (featureDensity < 0.05) ? 31 : 21;
        cv::GaussianBlur(densityMap, densityMap, cv::Size(blurSize, blurSize), 0);

        // Find density peak
        cv::Point maxLoc;
        double maxVal;
        cv::minMaxLoc(densityMap, nullptr, &maxVal, nullptr, &maxLoc);

        if (maxVal > 0)
        {
            // Normalized density score
            double densityScore = 0.3 + (maxVal / 20.0);

            if (densityScore > bestScore)
            {
                bestCenter = maxLoc;

                // Calculate radius from feature distribution
                cv::Mat nonZeroCoordinates;
                cv::findNonZero(redGreenMask, nonZeroCoordinates);

                // Compute average distance to colored pixels
                double sumDist = 0;
                int count = 0;
                for (int i = 0; i < nonZeroCoordinates.total(); i++)
                {
                    cv::Point pt = nonZeroCoordinates.at<cv::Point>(i);
                    double dist = cv::norm(pt - bestCenter);
                    if (dist < frame.cols / 2)
                    {
                        sumDist += dist;
                        count++;
                    }
                }

                // Calculate radius with safety check
                bestRadius = (count > 0) ? (sumDist / count) * 1.2 : (frame.cols / 5.0);
                bestScore = densityScore;
            }
        }
    }

    // Use the best center we found
    center = bestCenter;
    radius = bestRadius;

    // After we have detected the center and radius, find the orientation
    // Replace the hardcoded orientation with actual detection
    // Old code: orientation = 90.0; // Standard orientation

    // Use the detected masks to determine the orientation of segment 20
    orientation = detectOrientation(frame, center, radius, redMask, greenMask, debug_mode);

    // Final debug visualization
    if (debug_mode)
    {
        // Draw circular ROI on final visualization - make it PINK and BOLDER
        cv::Mat finalVis = colorFrame.clone();

        // MODIFICATION 2: Make ROI circle pink and thicker (3px instead of 1px)
        cv::circle(finalVis, frameCenter, initialRoiRadius, cv::Scalar(180, 105, 255), 3);

        // Draw frame center reference
        cv::circle(finalVis, frameCenter, 5, cv::Scalar(255, 0, 0), -1);
        cv::putText(finalVis, "Frame Center", frameCenter + cv::Point(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

        // Draw detected center and radius
        cv::circle(finalVis, center, 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(finalVis, center, radius, cv::Scalar(0, 255, 255), 2);
        cv::putText(finalVis, "Detected Center", center + cv::Point(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        // Add score info
        cv::putText(finalVis, "Score: " + to_string(bestScore).substr(0, 4),
                    cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Add orientation visualization
        cv::Point orientationPoint(
            center.x + radius * 1.1 * cos(orientation * CV_PI / 180.0),
            center.y + radius * 1.1 * sin(orientation * CV_PI / 180.0));

        cv::line(finalVis, center, orientationPoint, cv::Scalar(0, 255, 0), 2);
        cv::putText(finalVis, "20", orientationPoint + cv::Point(5, 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        cv::putText(finalVis, "Orientation: " + to_string(int(orientation)) + "°",
                    cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        cv::imwrite("debug_frames/final_detection_" + to_string(camera_idx) + ".jpg", finalVis);
    }

    return true;
}

// Create a standard calibration
DartboardCalibration DartboardCalibration::createStandardCalibration(cv::Point center, double radius, double orientation, int cameraIdx)
{
    DartboardCalibration calib;
    calib.center = center;
    calib.radius = radius;
    calib.orientation = orientation;
    calib.camera_index = cameraIdx;

    // Set standard ring proportions
    calib.bullRadius = radius * 0.07;
    calib.doubleRingInner = radius * 0.92;
    calib.doubleRingOuter = radius;
    calib.tripleRingInner = radius * 0.55;
    calib.tripleRingOuter = radius * 0.63;

    return calib;
}

// New method to detect dartboard orientation
double DartboardCalibration::detectOrientation(const cv::Mat &frame, const cv::Point &center,
                                               double radius, const cv::Mat &redMask,
                                               const cv::Mat &greenMask, bool debugMode)
{
    if (frame.empty() || radius <= 0)
        return 90.0; // Default orientation if invalid inputs

    // Parameters for sampling
    const int numSamples = 360;                  // 1-degree resolution
    const double samplingRadius = radius * 0.75; // Sample in between triple and double rings

    // Create vectors to store sampled values at each angle
    vector<ColorSample> samples(numSamples);

    // Sample points in a circle
    for (int i = 0; i < numSamples; i++)
    {
        // Calculate angle in radians (0 to 2π)
        double angle = i * (2 * CV_PI / numSamples);

        // Calculate sample point coordinates
        int x = center.x + samplingRadius * cos(angle);
        int y = center.y + samplingRadius * sin(angle);

        // Ensure point is within image bounds
        if (x < 0 || x >= frame.cols || y < 0 || y >= frame.rows)
            continue;

        // Get red and green values at this point
        double redValue = redMask.at<uchar>(y, x) > 0 ? 1.0 : 0.0;
        double greenValue = greenMask.at<uchar>(y, x) > 0 ? 1.0 : 0.0;

        // Store the sample
        samples[i].angle = angle;
        samples[i].redValue = redValue;
        samples[i].greenValue = greenValue;
        samples[i].totalValue = redValue + greenValue;
    }

    // Create smoothed profile by applying a moving average
    const int windowSize = 5;
    vector<double> smoothedProfile(numSamples);

    for (int i = 0; i < numSamples; i++)
    {
        double sum = 0.0;
        for (int j = -windowSize / 2; j <= windowSize / 2; j++)
        {
            int idx = (i + j + numSamples) % numSamples; // Wrap around for circular data
            sum += samples[idx].totalValue;
        }
        smoothedProfile[i] = sum / windowSize;
    }

    // Find pattern of 20 segments (each 18 degrees apart)
    // Strategy: Find pattern of peaks and valleys that match dartboard segmentation

    // First, normalize the profile
    double maxVal = *max_element(smoothedProfile.begin(), smoothedProfile.end());
    if (maxVal > 0)
    {
        for (auto &val : smoothedProfile)
            val /= maxVal;
    }

    // Calculate segment width in samples
    const int segmentWidth = numSamples / 20; // Each segment is 18 degrees (360/20)

    // Visual debug output
    if (debugMode)
    {
        // Create a visual representation of the color profile
        Mat profileVis = Mat::zeros(300, numSamples, CV_8UC3);

        for (int i = 0; i < numSamples; i++)
        {
            // Draw the profile value
            int height = int(smoothedProfile[i] * 250);
            line(profileVis, Point(i, profileVis.rows - 1),
                 Point(i, profileVis.rows - 1 - height), Scalar(0, 255, 255), 1);

            // Draw red/green samples
            if (samples[i].redValue > 0)
                circle(profileVis, Point(i, 280), 2, Scalar(0, 0, 255), -1);
            if (samples[i].greenValue > 0)
                circle(profileVis, Point(i, 290), 2, Scalar(0, 255, 0), -1);
        }

        // Draw horizontal grid lines
        for (int i = 0; i < 5; i++)
        {
            line(profileVis, Point(0, 50 + i * 50),
                 Point(profileVis.cols - 1, 50 + i * 50),
                 Scalar(50, 50, 50), 1);
        }

        // Draw segment boundaries (every 18 degrees)
        for (int i = 0; i < 20; i++)
        {
            line(profileVis, Point(i * segmentWidth, 0),
                 Point(i * segmentWidth, profileVis.rows - 1),
                 Scalar(70, 70, 70), 1);
        }

        imwrite("debug_frames/orientation_profile.jpg", profileVis);
    }

    // Find potential segment 20 locations
    // Strategy 1: Look for a pattern that matches the standard dartboard segment pattern
    // This is a simplified approach - in a real implementation, we would use a more
    // robust pattern matching algorithm

    // For now, use a heuristic: segment 20 is often at the top of the board
    // Try to find the best match near the top (90 degrees)

    // Start with a default orientation (top of image)
    double bestOrientation = 90.0;

    // Check if we have enough color information to make a reliable detection
    double totalColorSignal = accumulate(smoothedProfile.begin(), smoothedProfile.end(), 0.0);
    if (totalColorSignal > numSamples * 0.1)
    { // At least 10% of samples have color
        // Look for distinctive pattern (triple ring, double ring patterns)
        // For now, this is a simplified approach - locate the most prominent red/green pattern

        // Find the angle with the highest color concentration
        int maxIdx = max_element(smoothedProfile.begin(), smoothedProfile.end()) - smoothedProfile.begin();

        // Convert to degrees
        bestOrientation = (maxIdx * 360.0 / numSamples);

        // Adjust for standard dartboard orientation
        // The standard dartboard has segment 20 at the top (90 degrees in our coordinate system)
        // So we calculate how far our detected angle is from 90 degrees and rotate

        // Look for red/green/red pattern which indicates segment boundaries
        // This is a simplified approach - in a full implementation we would do more pattern analysis
    }

    // Normalize the orientation to 0-360 degrees
    while (bestOrientation < 0)
        bestOrientation += 360.0;
    while (bestOrientation >= 360.0)
        bestOrientation -= 360.0;

    return bestOrientation;
}

// New method to find dartboard as an ellipse
bool DartboardCalibration::findDartboardEllipse(const cv::Mat &frame, EllipseParams &params, int camera_idx, bool debugMode)
{
    // Basic validation
    if (frame.empty() || frame.channels() < 3)
        return false;

    cv::Mat colorFrame = frame.clone();
    cv::Point frameCenter(frame.cols / 2, frame.rows / 2);

    // Use 60% of minimum dimension for ROI
    int initialRoiRadius = int(std::min(frame.cols, frame.rows) * 0.6);
    cv::Mat roiMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::circle(roiMask, frameCenter, initialRoiRadius, cv::Scalar(255), -1);

    // Create ROI-constrained color frame
    cv::Mat roiColorFrame;
    colorFrame.copyTo(roiColorFrame, roiMask);

    // Process color masks similar to findDartboardCircle
    cv::Mat hsvFrame;
    cv::cvtColor(roiColorFrame, hsvFrame, cv::COLOR_BGR2HSV);

    cv::Mat redMask1, redMask2, redMask, greenMask, whiteMask;
    cv::inRange(hsvFrame, cv::Scalar(0, 40, 40), cv::Scalar(20, 255, 255), redMask1);
    cv::inRange(hsvFrame, cv::Scalar(160, 40, 40), cv::Scalar(180, 255, 255), redMask2);
    cv::inRange(hsvFrame, cv::Scalar(0, 0, 150), cv::Scalar(180, 40, 255), whiteMask);
    cv::inRange(hsvFrame, cv::Scalar(30, 40, 40), cv::Scalar(90, 255, 255), greenMask);

    redMask = redMask1 | redMask2;
    cv::Mat redGreenMask = redMask | greenMask;

    // Add white areas if needed
    int colorCount = cv::countNonZero(redGreenMask);
    if (colorCount < (frame.rows * frame.cols * 0.05))
    {
        redGreenMask = redGreenMask | whiteMask;
    }

    // Process the mask to find contours
    cv::Mat processedMask = redGreenMask.clone();
    int kernelSize = 5;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(processedMask, processedMask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(processedMask, processedMask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(processedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest contour that might be the dartboard
    double maxArea = 0;
    int bestContourIdx = -1;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            bestContourIdx = i;
        }
    }

    // If we found a suitable contour, fit an ellipse to it
    if (bestContourIdx >= 0 && contours[bestContourIdx].size() >= 5)
    {
        // Fit ellipse to the contour
        cv::RotatedRect ellipse = cv::fitEllipse(contours[bestContourIdx]);

        // Calculate equivalent circular radius (average of semi-major and semi-minor axes)
        double radius = (ellipse.size.width + ellipse.size.height) / 4.0;

        // Check if the ellipse is a reasonable size
        if (radius > 25)
        {
            params.ellipse = ellipse;
            params.center = ellipse.center;
            params.radius = radius;
            params.confidence = 0.9; // We could calculate a better confidence score

            // Debug visualization
            if (debugMode)
            {
                cv::Mat ellipseVis = colorFrame.clone();

                // Draw ROI
                cv::circle(ellipseVis, frameCenter, initialRoiRadius, cv::Scalar(180, 105, 255), 3);

                // Draw detected ellipse
                cv::ellipse(ellipseVis, ellipse, cv::Scalar(0, 255, 0), 2);

                // Draw ellipse center
                cv::circle(ellipseVis, ellipse.center, 5, cv::Scalar(0, 0, 255), -1);

                // Add info text
                cv::putText(ellipseVis, "Ellipse: " + std::to_string(int(ellipse.size.width)) + "x" + std::to_string(int(ellipse.size.height)), cv::Point(20, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                cv::putText(ellipseVis, "Angle: " + std::to_string(int(ellipse.angle)),
                            cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                cv::imwrite("debug_frames/ellipse_detection_" + std::to_string(camera_idx) + ".jpg", ellipseVis);
            }

            return true;
        }
    }

    // If ellipse detection failed, fall back to circle detection
    cv::Point center;
    double radius;
    double orientation = 90.0;
    bool circleFound = findDartboardCircle(frame, center, radius, orientation, camera_idx, debugMode);

    if (circleFound)
    {
        // Create a circular RotatedRect (angle 0)
        params.ellipse = cv::RotatedRect(center, cv::Size2f(radius * 2, radius * 2), 0);
        params.center = center;
        params.radius = radius;
        params.confidence = 0.7; // Lower confidence than ellipse detection
        return true;
    }

    return false;
}

// Method to correct perspective distortion
cv::Mat DartboardCalibration::correctPerspective(const cv::Mat &frame, const cv::Point &center,
                                                 double radius, const cv::RotatedRect &ellipse, bool debugMode)
{
    if (frame.empty())
        return frame;

    // Check if we have a severely distorted ellipse that needs correction
    double aspectRatio = std::max(ellipse.size.width, ellipse.size.height) /
                         std::min(ellipse.size.width, ellipse.size.height);

    // If the distortion is minor, return the original frame
    if (aspectRatio < 1.2)
    {
        return frame.clone();
    }

    // Calculate the transformation needed
    // Get the rotated rectangle vertices
    cv::Point2f vertices[4];
    ellipse.points(vertices);

    // Compute the target size for our corrected view (make it a square)
    int targetSize = static_cast<int>(std::max(ellipse.size.width, ellipse.size.height) * 1.2);

    // Define the destination points (a perfect square)
    cv::Point2f dstPoints[4];
    dstPoints[0] = cv::Point2f(0, 0);
    dstPoints[1] = cv::Point2f(targetSize, 0);
    dstPoints[2] = cv::Point2f(targetSize, targetSize);
    dstPoints[3] = cv::Point2f(0, targetSize);

    // Calculate perspective transform matrix
    cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(vertices, dstPoints);

    // Apply the transformation
    cv::Mat correctedFrame;
    cv::warpPerspective(frame, correctedFrame, perspectiveMatrix, cv::Size(targetSize, targetSize));

    // Debug visualization
    if (debugMode)
    {
        cv::Mat debugFrame = frame.clone();

        // Draw the original ellipse
        cv::ellipse(debugFrame, ellipse, cv::Scalar(0, 255, 0), 2);

        // Draw the ellipse vertices
        for (int i = 0; i < 4; i++)
        {
            cv::circle(debugFrame, vertices[i], 5, cv::Scalar(0, 0, 255), -1);
            cv::line(debugFrame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 255), 2);
        }

        // Add info text
        cv::putText(debugFrame, "Aspect ratio: " + std::to_string(aspectRatio).substr(0, 4),
                    cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Create side-by-side comparison
        int maxHeight = std::max(debugFrame.rows, correctedFrame.rows);
        int totalWidth = debugFrame.cols + correctedFrame.cols;
        cv::Mat comparison = cv::Mat::zeros(maxHeight, totalWidth, debugFrame.type());

        // Copy the images to the composite
        debugFrame.copyTo(comparison(cv::Rect(0, 0, debugFrame.cols, debugFrame.rows)));
        correctedFrame.copyTo(comparison(cv::Rect(debugFrame.cols, 0, correctedFrame.cols, correctedFrame.rows)));

        // Add labels
        cv::putText(comparison, "Original", cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
        cv::putText(comparison, "Corrected", cv::Point(debugFrame.cols + 20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);

        cv::imwrite("debug_frames/perspective_correction.jpg", comparison);
    }

    return correctedFrame;
}

// Multi-camera calibration orchestration
std::vector<DartboardCalibration> DartboardCalibration::calibrateMultipleCameras(const std::vector<cv::Mat> &frames, bool debugMode, int targetWidth, int targetHeight)
{
    std::vector<DartboardCalibration> calibrations;
    cout << "\n===== DARTBOARD CALIBRATION STARTED =====\n";

    if (frames.empty())
    {
        cerr << "No frames provided for calibration" << endl;
        return calibrations; // Return empty vector
    }

    for (size_t cam_idx = 0; cam_idx < frames.size(); cam_idx++)
    {
        if (frames[cam_idx].empty())
        {
            cerr << "WARN: Empty frame from camera " << cam_idx + 1 << endl;
            continue;
        }

        // Create and calibrate each camera
        DartboardCalibration calib;
        if (calib.calibrateSingleCamera(frames[cam_idx], cam_idx, debugMode))
        {
            calibrations.push_back(calib);
        }
        else
        {
            cout << "FAIL: Failed to calibrate camera " << cam_idx + 1 << endl;
        }
    }

    // Show multi-camera view with calibration overlay
    if (debugMode && !calibrations.empty())
    {
        dartboard_visualization::saveMultiCameraDebugView(
            frames,
            calibrations,
            {}, // No detections during calibration
            targetWidth,
            targetHeight,
            false); // Not using competition mode for debugging
    }

    cout << "===== DARTBOARD CALIBRATION COMPLETED =====\n";
    return calibrations;
}
