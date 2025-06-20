#include "color_processing.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

namespace color_processing
{
    Mat processColors(
        const Mat &roiFrame,
        int camera_idx,
        bool debug_mode,
        const ColorParams &params)
    {

        Mat redMask, greenMask;

        // ===== SECTION 1: PREPROCESSING =====
        Mat filteredFrame;
        bilateralFilter(roiFrame, filteredFrame, params.bilateralD, params.bilateralSigmaColor, params.bilateralSigmaSpace);

        Mat hsvFrame;
        cvtColor(filteredFrame, hsvFrame, COLOR_BGR2HSV);

        // ===== SECTION 2: COLOR DETECTION =====
        // This handles the color variation from bottom to top of dartboard
        Mat adaptiveRedMask = Mat::zeros(hsvFrame.size(), CV_8UC1);

        for (int y = 0; y < hsvFrame.rows; y++)
        {
            // Calculate vertical position factor (0 at bottom, 1 at top)
            float verticalFactor = static_cast<float>(hsvFrame.rows - y) / hsvFrame.rows;

            // Apply a non-linear transform to make upper areas more sensitive
            if (verticalFactor > 0.5)
            {
                verticalFactor = 0.5 + pow(verticalFactor - 0.5, 0.8) * 0.5;
            }

            // Calculate adaptive parameters based on vertical position
            int hueShift = cvRound(params.topRedHueShift * verticalFactor);
            int satShift = cvRound(params.topRedSatShift * verticalFactor);
            int valShift = cvRound(params.topRedValShift * verticalFactor);

            // Apply different thresholds for each row based on vertical position
            Mat rowMask1, rowMask2;

            // First red range with adaptive parameters
            inRange(hsvFrame.row(y),
                    Scalar(params.redLowHue1 + hueShift,
                           max(0, params.redLowSat1 - satShift),
                           max(0, params.redLowVal1 - valShift)),
                    Scalar(params.redHighHue1 + hueShift,
                           params.redHighSat1,
                           params.redHighVal1),
                    rowMask1);

            // Second red range with adaptive parameters
            inRange(hsvFrame.row(y),
                    Scalar(params.redLowHue2,
                           params.redLowSat2 - satShift,
                           params.redLowVal2 - valShift),
                    Scalar(params.redHighHue2,
                           params.redHighSat2,
                           params.redHighVal2),
                    rowMask2);

            // Combine results
            rowMask1 = rowMask1 | rowMask2;
            rowMask1.copyTo(adaptiveRedMask.row(y));
        }

        // ===== SECTION 3: STANDARD COLOR DETECTION =====
        // Standard detection as fallback
        Mat redMask1, redMask2;
        inRange(hsvFrame,
                Scalar(params.redLowHue1, params.redLowSat1, params.redLowVal1),
                Scalar(params.redHighHue1, params.redHighSat1, params.redHighVal1), redMask1);
        inRange(hsvFrame,
                Scalar(params.redLowHue2, params.redLowSat2, params.redLowVal2),
                Scalar(params.redHighHue2, params.redHighSat2, params.redHighVal2), redMask2);

        // Combine standard detection with adaptive detection
        redMask = redMask1 | redMask2 | adaptiveRedMask;

        // Detect green color
        inRange(hsvFrame,
                Scalar(params.greenLowHue, params.greenLowSat, params.greenLowVal),
                Scalar(params.greenHighHue, params.greenHighSat, params.greenHighVal), greenMask);

        // ===== SECTION 4: BASIC MORPHOLOGY =====
        // Apply small closing to connect nearby segments
        morphologyEx(redMask, redMask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(params.basicCloseKernelSize, params.basicCloseKernelSize)));
        morphologyEx(greenMask, greenMask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(params.basicCloseKernelSize, params.basicCloseKernelSize)));

        // ===== SECTION 5: BULL'S EYE ENHANCEMENT =====
        // After detecting red and green
        Mat bullsEyeMask = redMask.clone();
        Rect centerRegion(
            roiFrame.cols / 2 - roiFrame.cols / params.bullsEyeRegionSize,
            roiFrame.rows / 2 - roiFrame.rows / params.bullsEyeRegionSize,
            roiFrame.cols / (params.bullsEyeRegionSize / 2),
            roiFrame.rows / (params.bullsEyeRegionSize / 2));
        Mat centerMask = Mat::zeros(bullsEyeMask.size(), CV_8UC1);
        rectangle(centerMask, centerRegion, Scalar(255), -1);
        bullsEyeMask = bullsEyeMask & centerMask;
        dilate(bullsEyeMask, bullsEyeMask, getStructuringElement(MORPH_ELLIPSE, Size(params.bullsEyeDilateKernel, params.bullsEyeDilateKernel)));
        redMask = redMask | bullsEyeMask;

        // ===== SECTION 6: COMBINE COLORS & INITIAL CLEANING =====
        Mat redGreenMask = redMask | greenMask;

        // Basic cleaning of the mask
        Mat cleanMask;
        morphologyEx(redGreenMask, cleanMask, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(params.cleanOpenKernelSize, params.cleanOpenKernelSize)));
        morphologyEx(cleanMask, cleanMask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(params.cleanCloseKernelSize, params.cleanCloseKernelSize)));

        Mat enhancedMask = cleanMask.clone();

        // ===== SECTION 6: MULTI-SCALE MORPHOLOGY =====
        for (int size = 3; size <= params.maxMorphologyKernelSize; size += 2)
        {
            morphologyEx(enhancedMask, enhancedMask, MORPH_CLOSE,
                         getStructuringElement(MORPH_ELLIPSE, Size(size, size)));
        }

        // ===== SECTION 6.5: RING CONNECTION ENHANCEMENT =====
        {
            // Apply stronger closing to top half where rings are often broken
            int topHeight = enhancedMask.rows / 2;
            Rect topRegion(0, 0, enhancedMask.cols, topHeight);
            Mat topMask = enhancedMask(topRegion).clone();

            // More aggressive closing for top half
            morphologyEx(topMask, topMask, MORPH_CLOSE,
                         getStructuringElement(MORPH_ELLIPSE, Size(params.topRegionCloseKernel, params.topRegionCloseKernel)));

            // Copy back to enhanced mask
            topMask.copyTo(enhancedMask(topRegion));
        }

        // ===== SECTION 7: FINAL TARGETED COMPONENT FILTERING =====
        // Go back to original approach but add SPECIFIC text blob removal
        Mat labels, stats, centroids;
        int nLabels = connectedComponentsWithStats(enhancedMask, labels, stats, centroids);

        int largestIdx = 0;
        int largestArea = 0;
        for (int i = 1; i < nLabels; i++)
        {
            int area = stats.at<int>(i, CC_STAT_AREA);
            if (area > largestArea)
            {
                largestArea = area;
                largestIdx = i;
            }
        }

        Mat filteredMask = Mat::zeros(enhancedMask.size(), CV_8UC1);
        Point2f imageCenter(enhancedMask.cols / 2.0f, enhancedMask.rows / 2.0f);

        for (int i = 1; i < nLabels; i++)
        {
            int area = stats.at<int>(i, CC_STAT_AREA);
            Point2f componentCenter(centroids.at<double>(i, 0), centroids.at<double>(i, 1));

            bool isSizeOK = (area > max(params.minLargeComponentSize, largestArea / params.largestAreaDivisor));
            double distToCenter = norm(componentCenter - imageCenter);
            bool isCentral = (distToCenter < enhancedMask.cols * params.centralityThreshold);
            bool isBullsEyeArea = (distToCenter < enhancedMask.cols * params.bullsEyeThreshold);

            // Enhanced text filter - specifically target edge text blobs
            int left = stats.at<int>(i, CC_STAT_LEFT);
            int top = stats.at<int>(i, CC_STAT_TOP);
            int width = stats.at<int>(i, CC_STAT_WIDTH);
            int height = stats.at<int>(i, CC_STAT_HEIGHT);
            double aspectRatio = (double)width / height;
            bool isLikelyText = (aspectRatio > params.textAspectRatioMax || aspectRatio < params.textAspectRatioMin) && area < params.textMaxArea;

            // Specific edge-based text removal
            bool isEdgeText = false;
            double edgeDistance = min({left, top, enhancedMask.cols - (left + width), enhancedMask.rows - (top + height)});
            if (edgeDistance < enhancedMask.cols * params.edgeTextThreshold && area < params.edgeTextMaxArea)
            {
                isEdgeText = true;
            }

            // Position-based text filtering (target known problem areas)
            bool isBottomLeftText = (left < enhancedMask.cols * params.bottomLeftTextX && (top + height) > enhancedMask.rows * params.bottomLeftTextY);
            bool isTopRightText = ((left + width) > enhancedMask.cols * params.topRightTextX && top < enhancedMask.rows * params.topRightTextY);
            bool isPositionalText = (isBottomLeftText || isTopRightText) && area < params.positionalTextMaxArea;

            bool isTooSmall = (area < params.minBlobArea);
            bool isTooFarFromCenter = (distToCenter > (enhancedMask.cols * params.maxDistanceFromCenter / 2));

            // Connectivity check (keep this - it helps with inner rings)
            bool isConnected = false;
            if (area > params.minConnectedArea)
            {
                for (int j = 1; j < nLabels; j++)
                {
                    if (i != j && stats.at<int>(j, CC_STAT_AREA) > params.minConnectedNeighborArea)
                    {
                        Point2f otherCenter(centroids.at<double>(j, 0), centroids.at<double>(j, 1));
                        double dist = norm(componentCenter - otherCenter);
                        if (dist < enhancedMask.cols * params.connectivityThreshold)
                        {
                            isConnected = true;
                            break;
                        }
                    }
                }
            }

            // KEEP COMPONENT if it's good dartboard stuff, REJECT if it's obvious text
            if (!isEdgeText && !isPositionalText &&
                (i == largestIdx ||
                 (area > largestArea / params.largestAreaRatio && isCentral && !isLikelyText && !isTooSmall) ||
                 (isSizeOK && isConnected && !isTooFarFromCenter) ||
                 isBullsEyeArea))
            {
                // Copy component to filtered mask
                for (int y = top; y < top + height; y++)
                {
                    for (int x = left; x < left + width; x++)
                    {
                        if (y >= 0 && y < labels.rows && x >= 0 && x < labels.cols &&
                            labels.at<int>(y, x) == i)
                        {
                            filteredMask.at<uchar>(y, x) = 255;
                        }
                    }
                }
            }
        }

        // ===== FINAL SECTION: CREATE COLORED OUTPUT =====
        Mat redGreenFrame = Mat::zeros(roiFrame.size(), CV_8UC3);

        for (int y = 0; y < filteredMask.rows; y++)
        {
            for (int x = 0; x < filteredMask.cols; x++)
            {
                if (filteredMask.at<uchar>(y, x) > 0)
                {
                    if (redMask.at<uchar>(y, x) > 0)
                    {
                        redGreenFrame.at<Vec3b>(y, x) = Vec3b(0, 0, 255); // Red
                    }
                    else if (greenMask.at<uchar>(y, x) > 0)
                    {
                        redGreenFrame.at<Vec3b>(y, x) = Vec3b(0, 255, 0); // Green
                    }
                    else
                    {
                        // Gray for structural pixels
                        bool nearbyColor = false;
                        int checkDistance = params.nearbyColorCheckDistance;

                        for (int ny = max(0, y - checkDistance); ny <= min(filteredMask.rows - 1, y + checkDistance) && !nearbyColor; ny++)
                        {
                            for (int nx = max(0, x - checkDistance); nx <= min(filteredMask.cols - 1, x + checkDistance) && !nearbyColor; nx++)
                            {
                                if (redMask.at<uchar>(ny, nx) > 0 || greenMask.at<uchar>(ny, nx) > 0)
                                {
                                    nearbyColor = true;
                                    break;
                                }
                            }
                        }

                        if (nearbyColor)
                        {
                            redGreenFrame.at<Vec3b>(y, x) = Vec3b(80, 80, 80); // Gray structural pixels
                        }
                    }
                }
            }
        }

        // Save debug images
        if (debug_mode)
        {
            system("mkdir -p debug_frames/color_processing");
            imwrite("debug_frames/color_processing/red_green_frame_" + to_string(camera_idx) + ".jpg", redGreenFrame);
        }

        return redGreenFrame; // CLEAN: Return colored frame directly!
    }

    // Optional helper for when individual masks are needed
    void getIndividualColorMasks(
        const Mat &roiFrame,
        Mat &redMask,
        Mat &greenMask,
        const ColorParams &params)
    {
        // Simplified version of color detection that just returns the masks
        Mat hsvFrame;
        cvtColor(roiFrame, hsvFrame, COLOR_BGR2HSV);

        // Basic red detection
        Mat redMask1, redMask2;
        inRange(hsvFrame,
                Scalar(params.redLowHue1, params.redLowSat1, params.redLowVal1),
                Scalar(params.redHighHue1, params.redHighSat1, params.redHighVal1), redMask1);
        inRange(hsvFrame,
                Scalar(params.redLowHue2, params.redLowSat2, params.redLowVal2),
                Scalar(params.redHighHue2, params.redHighSat2, params.redHighVal2), redMask2);
        redMask = redMask1 | redMask2;

        // Basic green detection
        inRange(hsvFrame,
                Scalar(params.greenLowHue, params.greenLowSat, params.greenLowVal),
                Scalar(params.greenHighHue, params.greenHighSat, params.greenHighVal), greenMask);
    }

} // namespace color_detection
