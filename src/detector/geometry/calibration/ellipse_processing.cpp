#include "ellipse_processing.hpp"
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

namespace ellipse_processing
{
    // Rolling baseline ring width validation
    vector<Point> performDoubleRayTrace(const Mat &preprocessedMask, const Point &bullCenter, const EllipseParams &params,
                                        vector<Point> &allInnerPoints, vector<Point> &allOuterPoints, vector<bool> &rayValidFlags)
    {
        vector<double> ringWidths;
        allInnerPoints.clear();
        allOuterPoints.clear();
        rayValidFlags.clear();

        cout << "DEBUG: Starting double ray trace from (" << bullCenter.x << "," << bullCenter.y << ")" << endl;

        // PHASE 1: Cast all rays and measure inner/outer boundaries + ring widths
        for (double angle = 0; angle < 360; angle += params.angleStepDegrees)
        {
            double radians = angle * CV_PI / 180.0;
            Point2f direction(cos(radians), sin(radians));

            // Find INNER boundary (first white pixel)
            Point innerBoundary = bullCenter;
            bool foundInner = false;

            for (int distance = params.innerRayStartDistance; distance < params.maxRayDistance; distance++)
            {
                Point checkPoint = bullCenter + Point(direction.x * distance, direction.y * distance);

                if (checkPoint.x < 0 || checkPoint.x >= preprocessedMask.cols ||
                    checkPoint.y < 0 || checkPoint.y >= preprocessedMask.rows)
                    break;

                if (preprocessedMask.at<uchar>(checkPoint) > 0)
                {
                    innerBoundary = checkPoint;
                    foundInner = true;
                    break;
                }
            }

            // Find OUTER boundary (last white pixel before sustained black)
            Point outerBoundary = bullCenter;
            bool foundOuter = false;

            if (foundInner)
            {
                Point lastWhitePixel = innerBoundary;
                int consecutiveBlackCount = 0;
                const int minConsecutiveBlack = 5;

                for (int distance = norm(innerBoundary - bullCenter); distance < params.maxRayDistance; distance++)
                {
                    Point checkPoint = bullCenter + Point(direction.x * distance, direction.y * distance);

                    if (checkPoint.x < 0 || checkPoint.x >= preprocessedMask.cols ||
                        checkPoint.y < 0 || checkPoint.y >= preprocessedMask.rows)
                    {
                        if (lastWhitePixel != innerBoundary)
                        {
                            outerBoundary = lastWhitePixel;
                            foundOuter = true;
                        }
                        break;
                    }

                    bool isWhite = (preprocessedMask.at<uchar>(checkPoint) > 0);

                    if (isWhite)
                    {
                        lastWhitePixel = checkPoint;
                        consecutiveBlackCount = 0;
                    }
                    else
                    {
                        consecutiveBlackCount++;
                        if (consecutiveBlackCount >= minConsecutiveBlack && lastWhitePixel != innerBoundary)
                        {
                            outerBoundary = lastWhitePixel;
                            foundOuter = true;
                            break;
                        }
                    }
                }
            }

            // Calculate ring width
            bool validRay = foundInner && foundOuter;
            double ringWidth = validRay ? norm(outerBoundary - innerBoundary) : -1;

            // Store all data
            allInnerPoints.push_back(innerBoundary);
            allOuterPoints.push_back(outerBoundary);
            rayValidFlags.push_back(validRay);

            if (validRay)
            {
                ringWidths.push_back(ringWidth);
            }
            else
            {
                ringWidths.push_back(-1);
            }
        }

        cout << "DEBUG: Double ray trace completed. Found " << count(rayValidFlags.begin(), rayValidFlags.end(), true) << " valid rays" << endl;

        // PHASE 2: Rolling baseline validation
        vector<Point> validatedOuterPoints;
        vector<bool> finalValidFlags(rayValidFlags.size(), false);

        if (count(rayValidFlags.begin(), rayValidFlags.end(), true) >= params.minValidRingMeasurements)
        {

            // Create rolling baseline of ring widths
            vector<double> rollingBaseline;
            vector<double> validRingWidths;

            // Collect all valid ring widths
            for (size_t i = 0; i < rayValidFlags.size(); i++)
            {
                if (rayValidFlags[i] && ringWidths[i] > 0)
                {
                    validRingWidths.push_back(ringWidths[i]);
                }
            }

            if (validRingWidths.size() < params.minValidRingMeasurements)
            {
                cout << "DEBUG: Not enough valid ring widths for rolling baseline" << endl;
                return vector<Point>();
            }

            // Calculate global statistics for outlier detection
            sort(validRingWidths.begin(), validRingWidths.end());
            double globalMedian = validRingWidths[validRingWidths.size() / 2];

            // Calculate standard deviation
            double sum = 0;
            for (double width : validRingWidths)
            {
                sum += (width - globalMedian) * (width - globalMedian);
            }
            double stdDev = sqrt(sum / validRingWidths.size());

            cout << "DEBUG: Global ring width stats - median: " << globalMedian
                 << ", stdDev: " << stdDev << endl;

            // Rolling validation - go around the circle
            double rollingExpected = globalMedian; // Start with global median
            int validCount = 0;

            for (size_t i = 0; i < rayValidFlags.size(); i++)
            {
                if (!rayValidFlags[i] || ringWidths[i] <= 0)
                    continue;

                double currentWidth = ringWidths[i];
                double angle = i * params.angleStepDegrees;

                // Check against rolling expected value
                double deviation = abs(currentWidth - rollingExpected);
                double stdDeviation = abs(currentWidth - globalMedian) / stdDev;

                bool passesJumpTest = (deviation <= params.maxRingWidthJump);
                bool passesOutlierTest = (stdDeviation <= params.ringWidthOutlierThreshold);

                // temporary removed to not clutter the output
                // cout << "DEBUG: Ray " << angle << "° - width=" << currentWidth
                //      << ", expected=" << rollingExpected << ", deviation=" << deviation
                //      << ", stdDev=" << stdDeviation << " - ";

                if (passesJumpTest && passesOutlierTest)
                {
                    // ACCEPT this ray
                    validatedOuterPoints.push_back(allOuterPoints[i]);
                    finalValidFlags[i] = true;
                    validCount++;

                    // Update rolling expected (smooth transition)
                    double alpha = 0.3; // Smoothing factor
                    rollingExpected = alpha * currentWidth + (1.0 - alpha) * rollingExpected;

                    // temporary removed to not clutter the output
                    // cout << "ACCEPTED (newExpected=" << rollingExpected << ")" << endl;
                }
                else
                {
                    // temporary removed to not clutter the output
                    // cout << "REJECTED (jump=" << !passesJumpTest
                    //      << ", outlier=" << !passesOutlierTest << ")" << endl;
                }
            }

            // Update rayValidFlags to reflect final validation
            rayValidFlags = finalValidFlags;

            cout << "DEBUG: Rolling baseline validation result: " << validatedOuterPoints.size()
                 << " validated rays from " << allOuterPoints.size() << " total rays" << endl;

            return validatedOuterPoints;
        }

        cout << "DEBUG: Insufficient ring width measurements, using all valid rays" << endl;
        vector<Point> allValidOuterPoints;
        for (size_t i = 0; i < rayValidFlags.size(); i++)
        {
            if (rayValidFlags[i])
            {
                allValidOuterPoints.push_back(allOuterPoints[i]);
            }
        }
        return allValidOuterPoints;
    }

    // UPDATED: Main function uses MaskBundle and proper terminology
    EllipseBoundaryData processEllipse(
        const Mat &originalFrame,
        const mask_processing::MaskBundle &masks,
        const Point &bullCenter,
        const Point &frameCenter,
        int camera_idx,
        bool debug_mode,
        const EllipseParams &params)
    {
        cout << "DEBUG: Ellipse processing camera " << camera_idx << " starting..." << endl;
        EllipseBoundaryData result;

        // SECTION 1: RAY TRACING for DOUBLES (most accurate method)
        if (!masks.doublesMask.empty())
        {
            cout << "DEBUG: Processing doubles ring with ray tracing..." << endl;

            // No preprocessing needed - mask is already clean!
            int whitePixels = countNonZero(masks.doublesMask);
            cout << "DEBUG: Doubles mask has " << whitePixels << " white pixels" << endl;

            vector<Point> allInnerPoints, allOuterPoints;
            vector<bool> rayValidFlags;
            vector<Point> finalBoundaryPoints;

            if (whitePixels >= params.minWhitePixelsThreshold)
            {
                // Perform ray tracing on clean doubles mask
                finalBoundaryPoints = performDoubleRayTrace(masks.doublesMask, bullCenter, params,
                                                            allInnerPoints, allOuterPoints, rayValidFlags);

                if (finalBoundaryPoints.size() >= params.minValidRays)
                {
                    try
                    {
                        // Fit ellipse to validated outer boundary points
                        result.outerDoubleEllipse = fitEllipse(finalBoundaryPoints);
                        result.validOuterPoints = finalBoundaryPoints.size();
                        cout << "DEBUG: SUCCESS - Fitted outer double ellipse from " << result.validOuterPoints << " boundary points" << endl;

                        // Fit ellipse to inner boundary points (validated ones only)
                        vector<Point> validatedInnerPoints;
                        set<pair<int, int>> acceptedOuterPoints;
                        for (const Point &pt : finalBoundaryPoints)
                        {
                            acceptedOuterPoints.insert({pt.x, pt.y});
                        }

                        for (size_t i = 0; i < allOuterPoints.size(); i++)
                        {
                            if (acceptedOuterPoints.count({allOuterPoints[i].x, allOuterPoints[i].y}) > 0 &&
                                i < allInnerPoints.size())
                            {
                                validatedInnerPoints.push_back(allInnerPoints[i]);
                            }
                        }

                        if (validatedInnerPoints.size() >= 5)
                        {
                            result.innerDoubleEllipse = fitEllipse(validatedInnerPoints);
                            result.validInnerPoints = validatedInnerPoints.size();
                            cout << "DEBUG: SUCCESS - Fitted inner double ellipse from " << result.validInnerPoints << " inner points" << endl;
                        }
                        else
                        {
                            // Fallback: scale outer ellipse for inner boundary
                            result.innerDoubleEllipse = result.outerDoubleEllipse;
                            result.innerDoubleEllipse.size.width *= 0.92;
                            result.innerDoubleEllipse.size.height *= 0.92;
                            result.validInnerPoints = 0;
                            cout << "DEBUG: FALLBACK - Calculated inner double ellipse from outer" << endl;
                        }

                        result.hasValidDoubles = true;
                    }
                    catch (const cv::Exception &e)
                    {
                        cout << "DEBUG: Double ellipse fitting failed: " << e.what() << endl;
                        result.hasValidDoubles = false;
                    }
                }
                else
                {
                    cout << "DEBUG: Not enough boundary points for doubles: " << finalBoundaryPoints.size() << endl;
                    result.hasValidDoubles = false;
                }
            }
        }

        // SECTION 2: CONTOUR FITTING for TRIPLES (efficient method)
        if (!masks.triplesMask.empty())
        {
            cout << "DEBUG: Processing triples ring with contour fitting..." << endl;

            // Find ALL contours
            vector<vector<Point>> allTriplesContours;
            findContours(masks.triplesMask, allTriplesContours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            cout << "DEBUG: Found " << allTriplesContours.size() << " total contours" << endl;

            // Sort by area (largest first)
            sort(allTriplesContours.begin(), allTriplesContours.end(),
                 [](const vector<Point> &a, const vector<Point> &b)
                 {
                     return contourArea(a) > contourArea(b);
                 });

            // Debug contour areas
            for (size_t i = 0; i < allTriplesContours.size(); i++)
            {
                cout << "DEBUG: Contour " << i << " area: " << contourArea(allTriplesContours[i]) << endl;
            }

            try
            {
                // Fit outer ellipse to largest contour
                if (allTriplesContours.size() >= 1 && allTriplesContours[0].size() >= 5)
                {
                    result.outerTripleEllipse = fitEllipse(allTriplesContours[0]);
                    cout << "DEBUG: SUCCESS - Fitted outer triple ellipse from largest contour (area: " << contourArea(allTriplesContours[0]) << ")" << endl;
                }

                // Find a DIFFERENT contour for inner (skip the one we just used)
                bool foundInner = false;
                for (size_t i = 1; i < allTriplesContours.size(); i++)
                {
                    if (allTriplesContours[i].size() >= 5)
                    {
                        double areaRatio = contourArea(allTriplesContours[i]) / contourArea(allTriplesContours[0]);
                        cout << "DEBUG: Checking contour " << i << " - area ratio: " << areaRatio << endl;

                        // Only use if it's significantly different in size (not the same contour)
                        if (areaRatio < 0.9) // At least 10% smaller
                        {
                            result.innerTripleEllipse = fitEllipse(allTriplesContours[i]);
                            cout << "DEBUG: SUCCESS - Fitted inner triple ellipse from contour " << i << " (area: " << contourArea(allTriplesContours[i]) << ")" << endl;
                            foundInner = true;
                            break;
                        }
                    }
                }

                if (!foundInner)
                {
                    cout << "DEBUG: FAILED - No suitable inner contour found, all contours too similar" << endl;
                }

                result.hasValidTriples = (result.outerTripleEllipse.size.area() > 0 && result.innerTripleEllipse.size.area() > 0);
            }
            catch (const cv::Exception &e)
            {
                cout << "DEBUG: FAIL - Triple ellipse fitting failed: " << e.what() << endl;
                result.hasValidTriples = false;
            }
        }

        // SECTION 3: CONTOUR FITTING for BULL RINGS (simple method)
        if (!masks.outerBullMask.empty())
        {
            cout << "DEBUG: Processing outer bull (25-point) with contour fitting..." << endl;

            vector<vector<Point>> outerBullContours;
            findContours(masks.outerBullMask, outerBullContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            if (!outerBullContours.empty())
            {
                try
                {
                    // Sort by area and fit to largest contour
                    sort(outerBullContours.begin(), outerBullContours.end(),
                         [](const vector<Point> &a, const vector<Point> &b)
                         {
                             return contourArea(a) > contourArea(b);
                         });

                    if (outerBullContours[0].size() >= 5)
                    {
                        result.outerBullEllipse = fitEllipse(outerBullContours[0]);
                        cout << "DEBUG: SUCCESS - Fitted outer bull ellipse from contour" << endl;
                    }
                }
                catch (const cv::Exception &e)
                {
                    cout << "DEBUG: Outer bull ellipse fitting failed: " << e.what() << endl;
                }
            }
        }

        // SECTION 3.1: CONTOUR FITTING for INNER BULL RING (50-point)
        if (!masks.bullMask.empty())
        {
            cout << "DEBUG: Processing bullseye (50-point) with contour fitting..." << endl;

            vector<vector<Point>> bullContours;
            findContours(masks.bullMask, bullContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            if (!bullContours.empty())
            {
                try
                {
                    // Sort by area and fit to largest contour
                    sort(bullContours.begin(), bullContours.end(),
                         [](const vector<Point> &a, const vector<Point> &b)
                         {
                             return contourArea(a) > contourArea(b);
                         });

                    if (bullContours[0].size() >= 5)
                    {
                        result.innerBullEllipse = fitEllipse(bullContours[0]);
                        cout << "DEBUG: SUCCESS - Fitted bullseye ellipse from contour" << endl;
                    }
                }
                catch (const cv::Exception &e)
                {
                    cout << "DEBUG: Bullseye ellipse fitting failed: " << e.what() << endl;
                }
            }
        }

        // Set bull validation flag
        result.hasValidBulls = (result.outerBullEllipse.size.area() > 0 || result.innerBullEllipse.size.area() > 0);

        // SECTION 4: PERSPECTIVE ANALYSIS (using doubles as reference)
        if (result.hasValidDoubles)
        {
            Point ellipseCenter(result.outerDoubleEllipse.center);
            result.offsetX = ellipseCenter.x - bullCenter.x;
            result.offsetY = ellipseCenter.y - bullCenter.y;
            result.offsetMagnitude = norm(Point2f(result.offsetX, result.offsetY));
            result.offsetAngle = atan2(result.offsetY, result.offsetX) * 180.0 / CV_PI;

            cout << "DEBUG: Perspective offset - X=" << result.offsetX << ", Y=" << result.offsetY
                 << ", magnitude=" << result.offsetMagnitude << "px" << endl;
        }

        // SECTION 5: DEBUG VISUALIZATION
        if (debug_mode)
        {
            Mat ellipseVis = originalFrame.clone();

            // Draw all detected ellipses with different colors
            if (result.hasValidDoubles)
            {
                ellipse(ellipseVis, result.outerDoubleEllipse, Scalar(255, 0, 255), 3);
                ellipse(ellipseVis, result.innerDoubleEllipse, Scalar(255, 255, 0), 2);
            }

            if (result.hasValidTriples)
            {
                ellipse(ellipseVis, result.outerTripleEllipse, Scalar(255, 0, 255), 2);
                ellipse(ellipseVis, result.innerTripleEllipse, Scalar(255, 255, 0), 2);
            }

            if (result.hasValidBulls)
            {
                ellipse(ellipseVis, result.outerBullEllipse, Scalar(255, 0, 255), 2);
                ellipse(ellipseVis, result.innerBullEllipse, Scalar(255, 255, 0), 2);
            }

            // // Draw bull center
            // circle(ellipseVis, bullCenter, 8, Scalar(0, 0, 0), -1);
            // circle(ellipseVis, bullCenter, 10, Scalar(255, 255, 255), 2);

            system("mkdir -p debug_frames/ellipse_processing");
            imwrite("debug_frames/ellipse_processing/ellipse_result_" + to_string(camera_idx) + ".jpg", ellipseVis);

            cout << "DEBUG: Saved ellipse visualization with all ring types" << endl;
        }

        return result;
    }

} // namespace ellipse_processing