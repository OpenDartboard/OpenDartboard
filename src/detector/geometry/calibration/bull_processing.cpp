#include "bull_processing.hpp"
#include "color_processing.hpp"
#include "utils.hpp"
#include <cmath>

using namespace cv;
using namespace std;

namespace bull_processing
{
    Point processBull(const Mat &redGreenFrame, const Point &frameCenter, int camera_idx, bool debug_mode, const BullParams &params)
    {
        log_info("Bull detection camera " + log_string(camera_idx) + " starting...");
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

        Point bestCenter = frameCenter;
        double bestScore = 0;

        // Strategy A: Bull's eye detection
        Mat bullsEye = redMask.clone();
        morphologyEx(bullsEye, bullsEye, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
        vector<vector<Point>> bullContours;
        findContours(bullsEye, bullContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Strategy A: Process bull contours
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

            //  Bull must be very close to center
            double distToCenter = norm(Point(center2f) - frameCenter);
            double maxAllowedDistance = min(redGreenFrame.cols, redGreenFrame.rows) / 6.0; //  1/6

            // temporary removed to not clutter the output
            // cout << "DEBUG: Strategy A - Frame size: " << redGreenFrame.cols << "x" << redGreenFrame.rows
            //      << ", center: (" << frameCenter.x << "," << frameCenter.y << ")" << endl;
            // cout << "DEBUG: Strategy A - Contour center: (" << center2f.x << "," << center2f.y
            //      << "), distance: " << distToCenter << ", max allowed: " << maxAllowedDistance << endl;

            if (distToCenter > maxAllowedDistance)
            {
                // temporary removed to not clutter the output
                // cout << "DEBUG: Strategy A - REJECTED contour too far from center!" << endl;
                continue;
            }

            // Enhanced centrality scoring
            double maxDist = norm(Point(0, 0) - Point(redGreenFrame.cols, redGreenFrame.rows)) / 2.0;
            double centralityScore = 1.0 - (distToCenter / maxDist);
            double enhancedCentralityScore = centralityScore * centralityScore; // Square for stronger penalty

            // Size-based scoring - heavily favor smaller contours (bull should be small!)
            double idealArea = redGreenFrame.total() * params.idealBullAreaPercent;
            double sizeScore = 1.0;

            if (area > idealArea)
            {
                // Penalize larger areas exponentially
                double sizeFactor = area / idealArea;
                sizeScore = max(1.0 - params.maxSizePenalty, 1.0 / (sizeFactor * sizeFactor));
            }
            else
            {
                // Slightly favor areas closer to ideal
                sizeScore = 1.0 - (abs(area - idealArea) / idealArea) * 0.2;
            }

            // Combined score with size factor
            double score = (circularity * params.circularityWeight +
                            enhancedCentralityScore * params.centralityWeight +
                            sizeScore * params.sizeWeight) /
                           (params.circularityWeight + params.centralityWeight + params.sizeWeight);

            // temporary removed to not clutter the output
            // cout << "DEBUG: Strategy A - ACCEPTED contour - area: " << area << ", ideal: " << idealArea
            //      << ", sizeScore: " << sizeScore << ", centrality: " << enhancedCentralityScore
            //      << ", TOTAL: " << score << endl;

            if (score > bestScore)
            {
                bestScore = score;
                bestCenter = Point(center2f);
                log_debug("Strategy A - New best score: " + log_string(score) + " at (" + log_string(bestCenter.x) + "," + log_string(bestCenter.y) + ")");
            }
        }

        // Strategy B: Circular pattern detection with adaptive parameters
        if (bestScore < params.minAcceptableScore)
        {

            log_debug("Strategy B - Trying Hough circles (bestScore=" + log_string(bestScore) + ")");

            // Create circle detection mask
            // Calculate feature density for adaptive parameters

            Mat mask;
            cvtColor(redGreenFrame, mask, COLOR_BGR2GRAY); // Convert colored frame to grayscale
            threshold(mask, mask, 1, 255, THRESH_BINARY);  // Threshold: any non-black pixel becomes white

            double featureDensity = countNonZero(mask) / static_cast<double>(mask.total());
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

            log_debug("Strategy B - Found " + log_string(circles.size()) + " circles");

            // Process detected circles
            if (!circles.empty())
            {
                double bestCircleScore = 0;
                int bestIdx = -1;

                for (size_t i = 0; i < circles.size(); i++)
                {
                    Point circleCenter(cvRound(circles[i][0]), cvRound(circles[i][1]));

                    // MUCH MORE STRICT: Same stricter check for circles
                    double distToCenter = norm(circleCenter - frameCenter);
                    double maxAllowedDistance = min(redGreenFrame.cols, redGreenFrame.rows) / 6.0; // CHANGED: 1/6 instead of 1/4

                    log_debug("Strategy B - Circle center: (" + log_string(circleCenter.x) + "," + log_string(circleCenter.y) + "), distance: " + log_string(distToCenter) + ", max allowed: " + log_string(maxAllowedDistance));

                    if (distToCenter > maxAllowedDistance)
                    {
                        log_warning("Strategy B - REJECTED circle too far from center!");
                        continue;
                    }

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

                    // Centrality measure with enhanced penalty
                    double maxDist = norm(Point(0, 0) - Point(redGreenFrame.cols, redGreenFrame.rows)) / 2.0;
                    double centralityScore = 1.0 - (distToCenter / maxDist);
                    double enhancedCentralityScore = centralityScore * centralityScore; // Square for stronger penalty

                    // Combined score
                    double circleScore = enhancedCentralityScore * params.centerProximityWeight + centerRedness * params.rednessWeight;

                    log_debug("Strategy B - Circle at (" + log_string(circleCenter.x) + "," + log_string(circleCenter.y) + ") - dist=" + log_string(distToCenter) + ", score=" + log_string(circleScore));

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
                    log_debug("Strategy B - New best score: " + log_string(bestScore) + " at (" + log_string(bestCenter.x) + "," + log_string(bestCenter.y) + ")");
                }
            }
        }

        // Strategy C: Color density map (universally applicable)
        if (bestScore < params.minAcceptableScore)
        {
            log_debug("Strategy C - Trying density map (bestScore=" + log_string(bestScore) + ")");

            Mat densityMap = Mat::zeros(redGreenFrame.size(), CV_32F);
            Mat mask;
            cvtColor(redGreenFrame, mask, COLOR_BGR2GRAY); // Convert colored frame to grayscale
            threshold(mask, mask, 1, 255, THRESH_BINARY);  // Threshold: any non-black pixel becomes white
            double featureDensity = countNonZero(mask) / static_cast<double>(mask.total());

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
                    log_debug("Strategy C - New best score: " + log_string(bestScore) + " at (" + log_string(bestCenter.x) + "," + log_string(bestCenter.y) + ")");
                }
            }
        }

        // Set output confidence
        confidence = bestScore;

        log_info("Bull detection complete - center=(" + log_string(bestCenter.x) + "," + log_string(bestCenter.y) + "), confidence=" + log_string(confidence));

        // Debug output with PURPLE visualization
        if (debug_mode)
        {
            system("mkdir -p debug_frames/bull_processing");

            Mat bullDebug = redGreenFrame.clone();
            circle(bullDebug, bestCenter, 5, Scalar(255, 255, 255), -1); // Purple center dot
            circle(bullDebug, bestCenter, 20, Scalar(255, 255, 255), 3); // Purple circle

            imwrite("debug_frames/bull_processing/bull_detection_" + to_string(camera_idx) + ".jpg", bullDebug);
        }

        return bestCenter;
    }

} // namespace bull_processing
