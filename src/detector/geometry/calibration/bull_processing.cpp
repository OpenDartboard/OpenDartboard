#include "bull_processing.hpp"
#include "color_processing.hpp"
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

namespace bull_processing
{
    Point processBull(
        const Mat &redGreenFrame,
        const Mat &mask,
        const Point &frameCenter,
        int camera_idx,
        bool debug_mode,
        const BullParams &params)
    {
        double confidence;

        // Extract red pixels from redGreenFrame
        Mat redMask = Mat::zeros(redGreenFrame.size(), CV_8UC1);
        for (int y = 0; y < redGreenFrame.rows; y++)
        {
            for (int x = 0; x < redGreenFrame.cols; x++)
            {
                Vec3b pixel = redGreenFrame.at<Vec3b>(y, x);
                if (pixel == Vec3b(0, 0, 255))
                { // Red pixel
                    redMask.at<uchar>(y, x) = 255;
                }
            }
        }

        // Calculate feature density for adaptive parameters
        double featureDensity = countNonZero(mask) / static_cast<double>(mask.total());

        Point bestCenter = frameCenter;
        double bestScore = 0;

        // Strategy A: Bull's eye detection
        Mat bullsEye = redMask.clone();
        morphologyEx(bullsEye, bullsEye, MORPH_CLOSE,
                     getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

        vector<vector<Point>> bullContours;
        findContours(bullsEye, bullContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        cout << "DEBUG: Strategy A - Found " << bullContours.size() << " bull contours" << endl;

        // Process bull contours
        for (const auto &contour : bullContours)
        {
            double area = contourArea(contour);
            double maxArea = redGreenFrame.total() * params.maxBullAreaPercent;

            if (area < params.minBullArea || area > maxArea)
                continue;

            // Calculate circularity
            double perimeter = arcLength(contour, true);
            double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
            if (circularity < params.minCircularity)
                continue;

            Point2f center2f;
            float radius2f;
            minEnclosingCircle(contour, center2f, radius2f);

            // Scoring based on distance from frame center
            double maxDist = norm(Point(0, 0) - Point(redGreenFrame.cols, redGreenFrame.rows)) / 2.0;
            double distToCenter = norm(Point(center2f) - frameCenter);
            double centralityScore = 1.0 - (distToCenter / maxDist);

            // Combined score weighted by circularity and centrality
            double score = circularity * params.circularityWeight + centralityScore * params.centralityWeight;

            if (score > bestScore)
            {
                bestScore = score;
                bestCenter = Point(center2f);
                cout << "DEBUG: Strategy A - New best score: " << score << " at (" << bestCenter.x << "," << bestCenter.y << ")" << endl;
            }
        }

        // Strategy B: Circular pattern detection with adaptive parameters
        if (bestScore < 0.5)
        {
            cout << "DEBUG: Strategy B - Trying Hough circles (bestScore=" << bestScore << ")" << endl;

            // Create circle detection mask
            Mat circleMask = mask.clone();
            morphologyEx(circleMask, circleMask, MORPH_OPEN,
                         getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
            morphologyEx(circleMask, circleMask, MORPH_CLOSE,
                         getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));

            // Adjust parameters based on feature density
            double dp = (featureDensity < params.lowFeatureDensityThreshold) ? params.houghDp1 : params.houghDp2;
            int minDist = circleMask.rows / ((featureDensity < params.lowFeatureDensityThreshold) ? params.houghMinDistDivisor1 : params.houghMinDistDivisor2);
            int cannyThreshold = (featureDensity < params.lowFeatureDensityThreshold) ? params.houghCannyThreshold1 : params.houghCannyThreshold2;
            int accThreshold = (featureDensity < params.lowFeatureDensityThreshold) ? params.houghAccThreshold1 : params.houghAccThreshold2;

            vector<Vec3f> circles;
            HoughCircles(circleMask, circles, HOUGH_GRADIENT, dp, minDist,
                         cannyThreshold, accThreshold,
                         circleMask.rows / params.houghMinRadiusDivisor,
                         circleMask.rows / params.houghMaxRadiusDivisor);

            cout << "DEBUG: Strategy B - Found " << circles.size() << " circles" << endl;

            // Process detected circles
            if (!circles.empty())
            {
                double bestCircleScore = 0;
                int bestIdx = -1;

                for (size_t i = 0; i < circles.size(); i++)
                {
                    Point circleCenter(cvRound(circles[i][0]), cvRound(circles[i][1]));

                    // Check for red content near center (potential bull)
                    int roiSize = max(5, min(20, redGreenFrame.cols / 30));
                    Rect roi(
                        max(0, circleCenter.x - roiSize),
                        max(0, circleCenter.y - roiSize),
                        min(2 * roiSize, redMask.cols - circleCenter.x + roiSize),
                        min(2 * roiSize, redMask.rows - circleCenter.y + roiSize));

                    // Avoid invalid ROIs
                    if (roi.width <= 0 || roi.height <= 0)
                        continue;

                    Mat centerROI = redMask(roi);
                    double centerRedness = countNonZero(centerROI) / static_cast<double>(centerROI.total());

                    // Centrality measure
                    double maxDist = norm(Point(0, 0) - Point(redGreenFrame.cols, redGreenFrame.rows)) / 2.0;
                    double distToCenter = norm(circleCenter - frameCenter);
                    double centralityScore = 1.0 - (distToCenter / maxDist);

                    // Combined score
                    double circleScore = centralityScore * params.centerProximityWeight + centerRedness * params.rednessWeight;

                    if (circleScore > bestCircleScore)
                    {
                        bestCircleScore = circleScore;
                        bestIdx = i;
                    }
                }

                // Update best detection if circle score is good
                if (bestIdx >= 0 && bestCircleScore > bestScore)
                {
                    bestCenter = Point(cvRound(circles[bestIdx][0]), cvRound(circles[bestIdx][1]));
                    bestScore = bestCircleScore;
                    cout << "DEBUG: Strategy B - New best score: " << bestScore << " at (" << bestCenter.x << "," << bestCenter.y << ")" << endl;
                }
            }
        }

        // Strategy C: Color density map (universally applicable)
        if (bestScore < params.minAcceptableScore)
        {
            cout << "DEBUG: Strategy C - Trying density map (bestScore=" << bestScore << ")" << endl;

            Mat densityMap = Mat::zeros(redGreenFrame.size(), CV_32F);

            // Create density map from mask
            for (int y = 0; y < mask.rows; y++)
            {
                for (int x = 0; x < mask.cols; x++)
                {
                    if (mask.at<uchar>(y, x) > 0)
                    {
                        double dist = norm(Point(x, y) - frameCenter);
                        double weight = exp(-dist / (redGreenFrame.cols / 2.0)) * 10.0;
                        densityMap.at<float>(y, x) = weight;
                    }
                }
            }

            // Apply adaptive blur based on feature density
            int blurSize = (featureDensity < params.lowFeatureDensityThreshold) ? 31 : 21;
            GaussianBlur(densityMap, densityMap, Size(blurSize, blurSize), 0);

            // Find density peak
            Point maxLoc;
            double maxVal;
            minMaxLoc(densityMap, nullptr, &maxVal, nullptr, &maxLoc);

            if (maxVal > 0)
            {
                // Normalized density score
                double densityScore = params.densityScoreBase + (maxVal / params.densityScoreDivisor);

                if (densityScore > bestScore)
                {
                    bestCenter = maxLoc;
                    bestScore = densityScore;
                    cout << "DEBUG: Strategy C - New best score: " << bestScore << " at (" << bestCenter.x << "," << bestCenter.y << ")" << endl;
                }
            }
        }

        // Set output confidence
        confidence = bestScore;

        cout << "DEBUG: Bull detection complete - center=(" << bestCenter.x << "," << bestCenter.y
             << "), confidence=" << confidence << endl;

        // Debug output
        if (debug_mode)
        {
            system("mkdir -p debug_frames/bull_processing");

            Mat bullDebug = redGreenFrame.clone();
            circle(bullDebug, bestCenter, 5, Scalar(255, 255, 0), -1); // Yellow center dot
            circle(bullDebug, bestCenter, 20, Scalar(0, 255, 255), 2); // Yellow circle (just for visualization)

            imwrite("debug_frames/bull_processing/bull_detection_" + to_string(camera_idx) + ".jpg", bullDebug);
        }

        return bestCenter;
    }

} // namespace bull_processing
