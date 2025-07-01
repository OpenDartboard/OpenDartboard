#include <opencv2/imgproc.hpp>
#include <iostream>

#include "wire_processing.hpp"
#include "geometry_calibration.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

namespace wire_processing
{
    Mat detectMetalWires(const Mat &frame, const Mat &colorMask, const DartboardCalibration &calib)
    {
        Mat wireEnhanced;

        // STEP 1: Create ROI mask with 5% buffer around double outer ring
        Mat roiMask = Mat::zeros(frame.size(), CV_8UC1);
        RotatedRect bufferedRing = calib.ellipses.outerDoubleEllipse;
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
        RotatedRect tighterRing = calib.ellipses.outerDoubleEllipse;
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

        // Apply final mask to wireEnhanced
        Mat bullMask = Mat::zeros(wireEnhanced.size(), CV_8UC1);
        ellipse(bullMask, calib.ellipses.outerBullEllipse, Scalar(255), -1);

        // Remove bull area from wire mask
        wireEnhanced = wireEnhanced & ~bullMask;

        return wireEnhanced;
    }

    // Hough line-based wire detection
    vector<Point2f> findWiresByHoughLines(const Mat &mask, const Mat &colorMask, const DartboardCalibration &calib, bool debug_mode, const WireDetectionConfig &config)
    {
        vector<Point2f> wirePoints;

        log_debug("STARTING HOUGH LINE WIRE DETECTION for camera " + log_string(calib.camera_index));

        // Get the wire mask
        Mat wireMask = detectMetalWires(mask, colorMask, calib);

        // Apply edge detection
        Mat edges;
        Canny(wireMask, edges, config.cannyLow, config.cannyHigh);

        // Detect lines using Hough transform
        vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1, CV_PI / 180, config.houghThreshold, config.houghMinLineLength, config.houghMaxLineGap);

        log_debug("Found " + log_string(lines.size()) + " Hough lines");

        // Filter lines that are radial (pass near bull center)
        vector<Vec4i> radialLines;
        Point2f bullCenter = Point2f(calib.bullCenter);

        for (const Vec4i &line : lines)
        {
            Point2f p1(line[0], line[1]);
            Point2f p2(line[2], line[3]);

            // Calculate distance from line to bull center
            Point2f lineVec = p2 - p1;
            Point2f centerVec = bullCenter - p1;

            // Distance from point to line formula
            float lineLength = norm(lineVec);
            if (lineLength < 10)
                continue; // Skip very short lines

            Point2f normalizedLine = lineVec / lineLength;
            float distanceToCenter = abs(centerVec.x * normalizedLine.y - centerVec.y * normalizedLine.x);

            if (distanceToCenter < config.houghDistanceTolerance)
            {
                radialLines.push_back(line);
            }
        }

        log_debug("Filtered to " + log_string(radialLines.size()) + " radial lines");

        // Convert lines to wire endpoints
        for (const Vec4i &line : radialLines)
        {
            Point2f p1(line[0], line[1]);
            Point2f p2(line[2], line[3]);

            // Determine which point is farther from center (that's our wire endpoint)
            float dist1 = norm(p1 - bullCenter);
            float dist2 = norm(p2 - bullCenter);

            Point2f farPoint = (dist1 > dist2) ? p1 : p2;
            Point2f direction = farPoint - bullCenter;
            direction = direction / norm(direction); // Normalize

            // Calculate angle and extend to ellipse boundary
            float angle = atan2(direction.y, direction.x);
            Point2f wireEnd = math::intersectRayWithEllipse(bullCenter, angle, calib.ellipses.outerDoubleEllipse);

            wirePoints.push_back(wireEnd);
        }

        // Sort by angle
        sort(wirePoints.begin(), wirePoints.end(), [&calib](const Point2f &a, const Point2f &b)
             {
            Point2f dirA = a - Point2f(calib.bullCenter);
            Point2f dirB = b - Point2f(calib.bullCenter);
            float angleA = atan2(dirA.y, dirA.x);
            float angleB = atan2(dirB.y, dirB.x);
            return angleA < angleB; });

        // Debug visualization
        if (debug_mode)
        {
            system("mkdir -p debug_frames/wire_processing");

            // Show detected lines
            Mat linesDebug = mask.clone();

            // Draw all detected lines in gray
            for (const Vec4i &line : lines)
            {
                cv::line(linesDebug, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(128, 128, 128), 1);
            }

            // Draw radial lines in cyan
            for (const Vec4i &line : radialLines)
            {
                cv::line(linesDebug, Point(line[0], line[1]), Point(line[2], line[3]), Scalar(255, 255, 0), 2);
            }

            // Draw final wire lines in yellow
            for (size_t i = 0; i < wirePoints.size(); i++)
            {
                cv::line(linesDebug, calib.bullCenter, wirePoints[i], Scalar(0, 255, 255), 2);
                circle(linesDebug, wirePoints[i], 5, Scalar(0, 255, 255), -1);
                putText(linesDebug, to_string(i), Point(wirePoints[i].x + 5, wirePoints[i].y),
                        FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
            }

            imwrite("debug_frames/wire_processing/hough_lines_result_" + to_string(calib.camera_index) + ".jpg", linesDebug);
            imwrite("debug_frames/wire_processing/hough_edges_" + to_string(calib.camera_index) + ".jpg", edges);
        }

        log_debug("HOUGH LINE DETECTION COMPLETE: Found " + log_string(wirePoints.size()) + " wire points");

        return wirePoints;
    }

    // wire detection - no filtering, trusting the mask
    vector<Point2f> findWiresByColorTransitions(const Mat &mask, const Mat &colorMask, const DartboardCalibration &calib, bool debug_mode)
    {
        vector<Point2f> wirePoints;

        log_info("STARTING WIRE DETECTION for camera " + log_string(calib.camera_index));

        // Get the wire mask (areas between dartboard segments)
        Mat wireMask = detectMetalWires(mask, colorMask, calib);

        // Find contours (each should represent a dartboard segment)
        vector<vector<Point>> contours;
        findContours(wireMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        log_debug("Found " + log_string(contours.size()) + " raw contours");

        // Just extract angles from ALL contours the mask gives us
        for (size_t i = 0; i < contours.size(); i++)
        {
            const vector<Point> &segment = contours[i];

            // Only skip truly empty contours
            if (segment.empty())
            {
                continue;
            }

            log_debug("Processing contour " + log_string(i) + " with " + log_string(segment.size()) + " points");

            // Find min/max angles of this segment
            vector<float> segmentAngles;
            for (const Point &pt : segment)
            {
                Point2f dir = Point2f(pt) - Point2f(calib.bullCenter);
                float angle = atan2(dir.y, dir.x);
                segmentAngles.push_back(angle);
            }

            sort(segmentAngles.begin(), segmentAngles.end());

            float leftAngle = segmentAngles.front();
            float rightAngle = segmentAngles.back();

            // Handle angle wraparound (when segment crosses 0°)
            if (rightAngle - leftAngle > CV_PI)
            {
                // Find the largest gap - that's where the wraparound is
                float maxGap = 0;
                size_t gapIdx = 0;
                for (size_t j = 1; j < segmentAngles.size(); j++)
                {
                    float gap = segmentAngles[j] - segmentAngles[j - 1];
                    if (gap > maxGap)
                    {
                        maxGap = gap;
                        gapIdx = j;
                    }
                }
                leftAngle = segmentAngles[gapIdx];
                rightAngle = segmentAngles[gapIdx - 1];
            }

            // Create wire endpoints by intersecting with ellipse
            Point2f leftWire = math::intersectRayWithEllipse(
                Point2f(calib.bullCenter), leftAngle, calib.ellipses.outerDoubleEllipse);
            Point2f rightWire = math::intersectRayWithEllipse(
                Point2f(calib.bullCenter), rightAngle, calib.ellipses.outerDoubleEllipse);

            wirePoints.push_back(leftWire);
            wirePoints.push_back(rightWire);

            log_debug("Segment " + log_string(i) + " -> wires at " +
                      log_string(leftAngle * 180.0f / CV_PI) + "° and " +
                      log_string(rightAngle * 180.0f / CV_PI) + "°");
        }

        // Sort all wire points by angle for consistent ordering
        sort(wirePoints.begin(), wirePoints.end(), [&calib](const Point2f &a, const Point2f &b)
             {
            Point2f dirA = a - Point2f(calib.bullCenter);
            Point2f dirB = b - Point2f(calib.bullCenter);
            float angleA = atan2(dirA.y, dirA.x);
            float angleB = atan2(dirB.y, dirB.x);
            return angleA < angleB; });

        // debuging code
        if (debug_mode)
        {

            // Create debug visualizations
            system("mkdir -p debug_frames/wire_processing");

            // Existing debug: Show all contours
            Mat contoursImg = Mat::zeros(mask.size(), CV_8UC3);
            for (size_t i = 0; i < contours.size(); i++)
            {
                drawContours(contoursImg, contours, i, {255, 255, 255}, 1);

                if (!contours[i].empty())
                {
                    Moments m = moments(contours[i]);
                    if (m.m00 > 0)
                    {
                        Point center(m.m10 / m.m00, m.m01 / m.m00);
                        putText(contoursImg, to_string(i), center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
                    }
                }
            }
            imwrite("debug_frames/wire_processing/all_contours_" + to_string(calib.camera_index) + ".jpg", contoursImg);

            // DEBUG CODE: START HERE
            // Layered visualization on blue background
            Mat layeredDebug = Mat::zeros(mask.size(), CV_8UC3);

            // Layer 1: Blue mask (50% transparent)
            for (int y = 0; y < wireMask.rows; y++)
            {
                for (int x = 0; x < wireMask.cols; x++)
                {
                    if (wireMask.at<uchar>(y, x) > 0)
                    {
                        layeredDebug.at<Vec3b>(y, x) = Vec3b(255, 100, 0); // Blue with some intensity
                    }
                }
            }

            // Layer 2: Cyan contours (3px, some transparency)
            for (size_t i = 0; i < contours.size(); i++)
            {
                drawContours(layeredDebug, contours, i, Scalar(255, 255, 0), 3); // Cyan - good on blue
            }

            // Layer 3: Yellow/White wire lines (2px)
            for (size_t i = 0; i < wirePoints.size(); i++)
            {
                Scalar lineColor = (i % 2 == 0) ? Scalar(0, 255, 255) : Scalar(255, 255, 255); // Yellow/White - good on blue+cyan
                line(layeredDebug, calib.bullCenter, wirePoints[i], lineColor, 2);
                circle(layeredDebug, wirePoints[i], 4, lineColor, -1);
            }

            imwrite("debug_frames/wire_processing/wire_processing_result_" + to_string(calib.camera_index) + ".jpg", layeredDebug);

            // Same layers but on actual frame
            Mat frameLayered = mask.clone();

            // Layer 1: Blue mask overlay on frame (50% blend)
            Mat blueMask = Mat::zeros(mask.size(), CV_8UC3);
            for (int y = 0; y < wireMask.rows; y++)
            {
                for (int x = 0; x < wireMask.cols; x++)
                {
                    if (wireMask.at<uchar>(y, x) > 0)
                    {
                        blueMask.at<Vec3b>(y, x) = Vec3b(255, 0, 0); // Pure blue
                    }
                }
            }
            addWeighted(frameLayered, 0.7, blueMask, 0.3, 0, frameLayered);

            // Layer 2: Cyan contours (3px)
            for (size_t i = 0; i < contours.size(); i++)
            {
                drawContours(frameLayered, contours, i, Scalar(255, 255, 0), 3); // Cyan
            }

            // Layer 3: Yellow/White wire lines (2px)
            for (size_t i = 0; i < wirePoints.size(); i++)
            {
                Scalar lineColor = (i % 2 == 0) ? Scalar(0, 255, 255) : Scalar(255, 255, 255); // Yellow/White
                line(frameLayered, calib.bullCenter, wirePoints[i], lineColor, 2);
                circle(frameLayered, wirePoints[i], 4, lineColor, -1);
            }

            imwrite("debug_frames/wire_processing/wire_process_frame_result_" + to_string(calib.camera_index) + ".jpg", frameLayered);
            // DEBUG CODE: START HERE
        }

        log_debug("WIRE DETECTION COMPLETE: Found " + log_string(wirePoints.size()) + " wire points from " +
                  log_string(contours.size()) + " segments (NO FILTERING)");

        return wirePoints;
    }

    // Smooth contour boundaries and find clean edge angles
    pair<float, float> extractCleanAnglesFromContour(const vector<Point> &contour, Point2f bullCenter)
    {
        // Method 1: Smooth the contour boundary using Gaussian blur approach
        vector<Point2f> smoothedContour;

        // Convert to floating point for smoothing
        for (const auto &pt : contour)
        {
            smoothedContour.push_back(Point2f(pt.x, pt.y));
        }

        // Apply boundary smoothing - average each point with its neighbors
        vector<Point2f> smoothed = smoothedContour;
        int smoothingRadius = 3; // Number of neighbors to average

        for (int iter = 0; iter < 2; iter++)
        { // Multiple iterations for better smoothing
            for (size_t i = 0; i < smoothedContour.size(); i++)
            {
                Point2f sum(0, 0);
                int count = 0;

                for (int j = -smoothingRadius; j <= smoothingRadius; j++)
                {
                    int idx = (i + j + smoothedContour.size()) % smoothedContour.size();
                    sum += smoothedContour[idx];
                    count++;
                }

                smoothed[i] = sum / count;
            }
            smoothedContour = smoothed;
        }

        // Method 2: Find the extreme angular positions from smoothed contour
        vector<float> angles;
        for (const auto &point : smoothedContour)
        {
            Point2f direction = point - bullCenter;
            float angle = atan2(direction.y, direction.x);
            angles.push_back(angle);
        }

        if (angles.empty())
        {
            // Fallback to original contour
            for (const auto &point : contour)
            {
                Point2f direction = Point2f(point) - bullCenter;
                float angle = atan2(direction.y, direction.x);
                angles.push_back(angle);
            }
        }

        sort(angles.begin(), angles.end());

        float leftAngle = angles.front();
        float rightAngle = angles.back();

        // Handle angle wraparound
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

        return {leftAngle, rightAngle};
    }

    // Group wires by angular proximity
    vector<vector<Point2f>> groupWiresByAngle(const vector<Point2f> &wires, Point2f center, float angleTolerance)
    {
        vector<vector<Point2f>> groups;
        vector<bool> used(wires.size(), false);

        for (size_t i = 0; i < wires.size(); i++)
        {
            if (used[i])
                continue;

            vector<Point2f> group;
            Point2f dir1 = wires[i] - center;
            float angle1 = atan2(dir1.y, dir1.x) * 180.0f / CV_PI;

            group.push_back(wires[i]);
            used[i] = true;

            // Find all wires within angular tolerance
            for (size_t j = i + 1; j < wires.size(); j++)
            {
                if (used[j])
                    continue;

                Point2f dir2 = wires[j] - center;
                float angle2 = atan2(dir2.y, dir2.x) * 180.0f / CV_PI;

                float angleDiff = abs(angle1 - angle2);
                if (angleDiff > 180.0f)
                    angleDiff = 360.0f - angleDiff;

                if (angleDiff <= angleTolerance)
                {
                    group.push_back(wires[j]);
                    used[j] = true;
                }
            }

            groups.push_back(group);
        }

        return groups;
    }

    // Score a wire based on line quality (your brilliant scoring idea!)
    float scoreWire(Point2f wire, Point2f center, const Mat &wireMask)
    {
        // Cast ray from center through wire point
        Point2f direction = wire - center;
        direction = direction / norm(direction);

        float score = 0.0f;
        int samples = 0;

        // Sample along the line from center to wire
        for (float dist = 10.0f; dist < norm(wire - center); dist += 2.0f)
        {
            Point2f samplePoint = center + direction * dist;

            if (samplePoint.x < 2 || samplePoint.y < 2 ||
                samplePoint.x >= wireMask.cols - 2 || samplePoint.y >= wireMask.rows - 2)
            {
                continue;
            }

            // Check 2-pixel padding on both sides of the line
            Point2f perpendicular(-direction.y, direction.x);

            // Sample left and right of the line
            Point2f leftPoint = samplePoint + perpendicular * 2.0f;
            Point2f rightPoint = samplePoint - perpendicular * 2.0f;

            if (leftPoint.x >= 0 && leftPoint.y >= 0 && leftPoint.x < wireMask.cols && leftPoint.y < wireMask.rows &&
                rightPoint.x >= 0 && rightPoint.y >= 0 && rightPoint.x < wireMask.cols && rightPoint.y < wireMask.rows)
            {

                // Check for white pixels (wire areas) on both sides
                int leftWhite = (wireMask.at<uchar>(leftPoint.y, leftPoint.x) > 128) ? 1 : 0;
                int rightWhite = (wireMask.at<uchar>(rightPoint.y, rightPoint.x) > 128) ? 1 : 0;

                // Score higher if we have white on both sides (wire boundary)
                score += leftWhite + rightWhite;
                samples++;
            }
        }

        return (samples > 0) ? score / samples : 0.0f;
    }

    // Select average wire position from a group (simpler than scoring!)
    Point2f selectAverageWireFromGroup(const vector<Point2f> &group, Point2f center)
    {
        if (group.empty())
            return Point2f(0, 0);
        if (group.size() == 1)
            return group[0];

        // Calculate average position
        Point2f avgPosition(0, 0);
        for (const Point2f &wire : group)
        {
            avgPosition += wire;
        }
        avgPosition = avgPosition / (float)group.size();

        // Convert to angle and extend to ellipse boundary for consistency
        Point2f direction = avgPosition - center;
        float avgAngle = atan2(direction.y, direction.x);

        log_debug("Average wire from group of " + log_string(group.size()) + " at angle: " +
                  log_string(avgAngle * 180.0f / CV_PI) + "°");

        return avgPosition;
    }

    // Ensemble method combining both approaches - AVERAGE VERSION
    vector<Point2f> findWiresByEnsemble(const Mat &mask, const Mat &colorMask, const DartboardCalibration &calib, bool debug_mode, const WireDetectionConfig &config)
    {
        log_debug("STARTING ENSEMBLE WIRE DETECTION (AVERAGE) for camera " + log_string(calib.camera_index));

        // Get wires from both methods
        vector<Point2f> contourWires = findWiresByColorTransitions(mask, colorMask, calib, false);
        vector<Point2f> houghWires = findWiresByHoughLines(mask, colorMask, calib, false, config);

        log_debug("Contour method found " + log_string(contourWires.size()) + " wires");
        log_debug("Hough method found " + log_string(houghWires.size()) + " wires");

        // Combine all wire candidates
        vector<Point2f> allWires = contourWires;
        allWires.insert(allWires.end(), houghWires.begin(), houghWires.end());

        log_debug("Combined total: " + log_string(allWires.size()) + " wire candidates");

        // Group wires by angular proximity (±9° tolerance for 18° dartboard segments)
        vector<vector<Point2f>> wireGroups = groupWiresByAngle(allWires, Point2f(calib.bullCenter), 9.0f);

        log_debug("Grouped into " + log_string(wireGroups.size()) + " angular groups");

        // Select AVERAGE wire from each group (no complex scoring!)
        vector<Point2f> finalWires;
        for (size_t i = 0; i < wireGroups.size(); i++)
        {
            Point2f avgWire = selectAverageWireFromGroup(wireGroups[i], Point2f(calib.bullCenter));
            finalWires.push_back(avgWire);

            log_debug("Group " + log_string(i) + " has " + log_string(wireGroups[i].size()) + " candidates, selected average");
        }

        // Sort by angle
        sort(finalWires.begin(), finalWires.end(), [&calib](const Point2f &a, const Point2f &b)
             {
            Point2f dirA = a - Point2f(calib.bullCenter);
            Point2f dirB = b - Point2f(calib.bullCenter);
            float angleA = atan2(dirA.y, dirA.x);
            float angleB = atan2(dirB.y, dirB.x);
            return angleA < angleB; });

        // Debug visualization
        if (debug_mode)
        {
            system("mkdir -p debug_frames/wire_processing");

            Mat ensembleDebug = mask.clone();

            // Draw grouped candidates in different colors
            vector<Scalar> colors = {
                Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255),
                Scalar(255, 255, 0), Scalar(255, 0, 255), Scalar(0, 255, 255)};

            // Draw all candidates in gray
            for (const Point2f &wire : allWires)
            {
                line(ensembleDebug, calib.bullCenter, wire, Scalar(255, 255, 0), 1);
                circle(ensembleDebug, wire, 3, Scalar(128, 128, 128), 1);
            }

            // Draw each group in different colors
            for (size_t i = 0; i < wireGroups.size(); i++)
            {
                Scalar color = colors[i % colors.size()];
                for (const Point2f &wire : wireGroups[i])
                {
                    circle(ensembleDebug, wire, 6, color, 2);
                }
            }

            // Draw final averaged wires in bright yellow (thicker)
            for (size_t i = 0; i < finalWires.size(); i++)
            {
                line(ensembleDebug, calib.bullCenter, finalWires[i], Scalar(0, 255, 255), 2);
                circle(ensembleDebug, finalWires[i], 6, Scalar(0, 255, 255), 1);
                putText(ensembleDebug, to_string(i), Point(finalWires[i].x + 10, finalWires[i].y),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
            }

            imwrite("debug_frames/wire_processing/ensemble_average_result_" + to_string(calib.camera_index) + ".jpg", ensembleDebug);
        }

        log_debug("ENSEMBLE AVERAGE DETECTION COMPLETE: Selected " + log_string(finalWires.size()) + " averaged wires from " + log_string(allWires.size()) + " candidates");

        return finalWires;
    }

    WireData processWires(const Mat &frame, const Mat &colorMask, const DartboardCalibration &calib, bool enableDebug, const WireDetectionConfig &config)
    {
        WireData result;
        result.camera_index = calib.camera_index;

        if (!calib.ellipses.hasValidDoubles || frame.empty() || colorMask.empty())
        {
            log_error("Invalid input data");
            return result;
        }

        log_debug("Starting wire detection for camera " + log_string(calib.camera_index));
        log_debug("Frame size: " + log_string(frame.cols) + "x" + log_string(frame.rows));

        // Choose detection method based on config
        vector<Point2f> colorWires;

        if (config.useHoughLinesDetection)
        {
            log_debug("Using HOUGH LINE detection method");
            result.detectionMethod = "hough_lines";
            colorWires = findWiresByHoughLines(frame, colorMask, calib, enableDebug, config);
        }
        else
        {
            log_debug("Using ENSEMBLE detection method (Contour + Hough + Scoring)");
            result.detectionMethod = "ensemble";
            colorWires = findWiresByEnsemble(frame, colorMask, calib, enableDebug, config);
        }

        // Set up result
        result.wireEndpoints = colorWires;
        result.segmentNumbers = {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};
        result.isValid = (result.wireEndpoints.size() >= 16); // Allow some tolerance

        log_debug("Found " + log_string(result.wireEndpoints.size()) + " wire boundaries using " + result.detectionMethod);
        log_debug("Wire detection completed successfully");

        // Handle debug output internally
        if (enableDebug)
        {
            log_debug("Saving debug images");

            // Create debug visualization using the FINAL PROCESSED wire endpoints
            Mat debug = frame.clone();

            // Draw detected wire endpoints - these are the FINAL results after sorting
            for (size_t i = 0; i < result.wireEndpoints.size(); i++)
            {
                Point2f wireEnd = result.wireEndpoints[i];

                // Draw wire line from center to endpoint
                line(debug, calib.bullCenter, wireEnd, Scalar(0, 0, 255), 2);

                // Draw endpoint circle
                circle(debug, wireEnd, 5, Scalar(0, 255, 255), -1);

                // Add wire index number (not segment number)
                putText(debug, to_string(i), Point(wireEnd.x + 10, wireEnd.y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
            }

            system("mkdir -p debug_frames/wire_processing");
            imwrite("debug_frames/wire_processing/wire_edges_" + to_string(calib.camera_index) + ".jpg", detectMetalWires(frame, colorMask, calib));
            imwrite("debug_frames/wire_processing/wire_result_" + to_string(calib.camera_index) + ".jpg", debug);
        }

        return result;
    }

} // namespace wire_processing