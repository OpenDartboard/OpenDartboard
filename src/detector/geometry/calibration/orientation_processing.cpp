#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <tesseract/baseapi.h>

#include "orientation_processing.hpp"
#include "perspective_processing.hpp"
#include "geometry_calibration.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

namespace orientation_processing
{
    // Create perspective-aware number region mask
    static Mat createNumberRegionMask(const Mat &frame, const DartboardCalibration &calib, bool enableDebug, const OrientationParams &params)
    {
        log_debug("Creating number region mask for camera " + log_string(calib.camera_index));

        // Get the outer doubles ellipse (our reference)
        RotatedRect outerDoubles = calib.ellipses.outerDoubleEllipse;

        // Create dartboard boundary - scale up from outer doubles
        RotatedRect dartboardBoundary = outerDoubles;
        dartboardBoundary.size.width *= params.boundryScaleFactor;
        dartboardBoundary.size.height *= params.boundryScaleFactor;

        // Apply perspective correction using the existing offset data
        Point2f perspectiveOffset(calib.ellipses.offsetX, calib.ellipses.offsetY);
        dartboardBoundary.center = outerDoubles.center + perspectiveOffset * (params.boundryScaleFactor - 1) * 2;

        log_debug("Perspective offset: (" + log_string(perspectiveOffset.x) + "," + log_string(perspectiveOffset.y) + ")");
        log_debug("Dartboard outer boundary center: (" + log_string(dartboardBoundary.center.x) + "," + log_string(dartboardBoundary.center.y) + ")");

        RotatedRect dartboardInnerBoundary = outerDoubles;
        dartboardInnerBoundary.size.width *= params.innerBoundryScaleFactor;
        dartboardInnerBoundary.size.height *= params.innerBoundryScaleFactor;

        // Apply perspective correction using the existing offset data
        Point2f perspectiveOffsetInner(calib.ellipses.offsetX, calib.ellipses.offsetY);
        dartboardInnerBoundary.center = outerDoubles.center + perspectiveOffsetInner * (params.innerBoundryScaleFactor - 1) * 2;

        log_debug("Perspective offset: (" + log_string(perspectiveOffsetInner.x) + "," + log_string(perspectiveOffsetInner.y) + ")");
        log_debug("Dartboard inner boundary center: (" + log_string(dartboardInnerBoundary.center.x) + "," + log_string(dartboardInnerBoundary.center.y) + ")");

        // Create the two masks
        Mat outerMask = Mat::zeros(frame.size(), CV_8UC1);
        Mat innerMask = Mat::zeros(frame.size(), CV_8UC1);

        // Draw the ellipses
        ellipse(outerMask, dartboardBoundary, Scalar(255), -1);      // Outer boundary (white)
        ellipse(innerMask, dartboardInnerBoundary, Scalar(255), -1); // Inner boundary (white)

        // Create ring mask (outer - inner = ring between them)
        Mat numberRegionMask = outerMask - innerMask;

        log_debug("Number region mask created successfully");
        log_debug("Preprocessing text region for camera " + log_string(calib.camera_index));

        // Extract the number region from frame using the mask
        Mat numberRegion = Mat::zeros(frame.size(), CV_8UC3);
        frame.copyTo(numberRegion, numberRegionMask);

        // Convert to grayscale
        Mat grayRegion;
        cvtColor(numberRegion, grayRegion, COLOR_BGR2GRAY);

        // Remove black pixels AND dark grays - numbers are on white/colored background
        Mat brightMask;
        threshold(grayRegion, brightMask, params.brightnessThreshold, 255, THRESH_BINARY); // Remove dark areas

        // Apply bright mask to keep only bright areas where numbers can be
        Mat brightRegion;
        grayRegion.copyTo(brightRegion, brightMask);

        // Enhance contrast for better text detection
        Mat enhancedGray;
        Ptr<cv::CLAHE> claheProcessor = cv::createCLAHE(3.0, Size(8, 8));
        claheProcessor->apply(brightRegion, enhancedGray);

        // Binary threshold to get white text on black background
        Mat binaryTextMask;
        threshold(enhancedGray, binaryTextMask, 0, 255, THRESH_BINARY + THRESH_OTSU);

        log_debug("Text preprocessing completed");

        /// TODO so spider check and then get star camera index

        // Debug output
        if (enableDebug)
        {
            imwrite("debug_frames/orientation_processing/number_region_" + to_string(calib.camera_index) + ".jpg", numberRegion);
            imwrite("debug_frames/orientation_processing/bright_region_" + to_string(calib.camera_index) + ".jpg", brightRegion);
            imwrite("debug_frames/orientation_processing/binary_text_" + to_string(calib.camera_index) + ".jpg", binaryTextMask);
        }

        return binaryTextMask; // Return perspective-corrected masks
    }

    // Find the 4 clip wires by extending wires outward and checking for dartboard collision
    static vector<pair<Point2f, Point2f>> findClipWires(const Mat &frame, const Mat &mask, const DartboardCalibration &calib, bool enableDebug, const OrientationParams &params)
    {
        log_debug("Finding clip wires for camera " + log_string(calib.camera_index));

        vector<pair<Point2f, Point2f>> clipWires;

        // Get dartboard center from ellipse
        Point2f center = calib.bullCenter;

        // Loop through all detected wire endpoints
        for (const auto &wireEndpoint : calib.wires.wireEndpoints)
        {
            // Calculate direction vector from center to wire endpoint
            Point2f direction = wireEndpoint - center;
            float length = norm(direction);

            if (length > 0)
            {
                // Normalize direction
                direction = direction / length;

                // Extend wire outward by 1000+ pixels from the endpoint
                Point2f extendedEnd = wireEndpoint + direction * params.wireExtensionDistance;

                // Check if extended line hits dartboard (sample points along extension)
                bool hitsEmptySpace = true;
                int sampleCount = params.spiderSampleCount; // Sample 50 points along the extension

                for (int i = 1; i <= sampleCount; i++)
                {
                    float t = (float)i / sampleCount;
                    Point2f samplePoint = wireEndpoint + direction * (params.wireExtensionDistance * t);

                    // Check bounds
                    if (samplePoint.x >= 0 && samplePoint.x < mask.cols &&
                        samplePoint.y >= 0 && samplePoint.y < mask.rows)
                    {
                        // If we hit white (dartboard), this is not a clip wire
                        if (mask.at<uchar>(samplePoint) > 128)
                        {
                            hitsEmptySpace = false;
                            break;
                        }
                    }
                }

                // If wire extension stays in empty space, it's a clip wire
                if (hitsEmptySpace)
                {
                    clipWires.push_back(make_pair(wireEndpoint, extendedEnd));
                    log_debug("Found clip wire at (" + log_string(wireEndpoint.x) + "," + log_string(wireEndpoint.y) + ")");
                }
            }
        }

        if (enableDebug)
        {
            Mat clipWireDebug = frame.clone();
            for (const auto &wire : clipWires)
            {
                line(clipWireDebug, wire.first, wire.second, Scalar(0, 255, 0), 2);
                circle(clipWireDebug, wire.first, 5, Scalar(255, 0, 0), -1);
            }
            imwrite("debug_frames/orientation_processing/clip_wires_" + to_string(calib.camera_index) + ".jpg", clipWireDebug);
        }

        if (clipWires.empty())
        {
            LOG_ERROR("No clip wires found for camera " + log_string(calib.camera_index));
        }
        else
        {
            log_debug("Found " + log_string(clipWires.size()) + " clip wires");
        }
        return clipWires;
    }

    // Determine camera position and calculate comprehensive orientation data
    static OrientationData determineCameraPosition(const vector<pair<Point2f, Point2f>> &clipWires, const DartboardCalibration &calib)
    {
        log_info("=== DETERMINING CAMERA POSITION FOR CAMERA " + log_string(calib.camera_index) + " ===");

        OrientationData result;
        result.camera_index = calib.camera_index;

        Point2f center = calib.bullCenter;

        // STEP 1: Check if this is the star camera (using clip wire angle analysis)
        if (clipWires.size() == 4)
        {
            // Calculate angles of all 4 clip wires relative to south
            vector<float> angles;
            for (const auto &wire : clipWires)
            {
                Point2f wireDirection = wire.first - center;
                float length = norm(wireDirection);
                if (length > 0)
                {
                    wireDirection = wireDirection / length;
                    float angle = atan2(wireDirection.x, wireDirection.y) * 180.0f / CV_PI;
                    if (angle < 0)
                        angle += 360;
                    angles.push_back(angle);
                }
            }

            if (angles.size() == 4)
            {
                sort(angles.begin(), angles.end());

                // Calculate angle differences and standard deviation
                vector<float> angleDiffs;
                for (int i = 0; i < 4; i++)
                {
                    float diff = angles[(i + 1) % 4] - angles[i];
                    if (i == 3)
                        diff = (angles[0] + 360) - angles[3];
                    if (diff < 0)
                        diff += 360;
                    angleDiffs.push_back(diff);
                }

                float avgDiff = 0;
                for (float diff : angleDiffs)
                    avgDiff += diff;
                avgDiff /= 4;

                float variance = 0;
                for (float diff : angleDiffs)
                {
                    variance += (diff - avgDiff) * (diff - avgDiff);
                }
                variance /= 4;
                float stdDev = sqrt(variance);

                // Star camera has irregular spacing (high std dev)
                result.isStarCamera = (abs(avgDiff - 90.0f) < 15.0f) && (stdDev > 15.0f);
            }
        }

        // STEP 2: Find south wire index
        float smallestAngle = 360.0f;

        for (int i = 0; i < calib.wires.wireEndpoints.size(); i++)
        {
            Point2f wireDirection = calib.wires.wireEndpoints[i] - center;
            float length = norm(wireDirection);
            if (length > 0)
            {
                wireDirection = wireDirection / length;
                float angle = atan2(wireDirection.x, wireDirection.y) * 180.0f / CV_PI;
                if (angle < 0)
                    angle += 360;

                if (angle >= 0 && angle < smallestAngle)
                {
                    smallestAngle = angle;
                    result.southWireIndex = i;
                    result.angleOffsetFromSouth = angle; // Store the exact angle offset
                }
            }
        }

        // STEP 3: Determine camera position and calculate all orientation data
        if (result.isStarCamera)
        {
            result.cameraPosition = CameraPosition::MIDDLE;
            result.wedgeNumber = 6;
            // For star camera: wedge 20 is 5 steps back from south wire (wedge 6)
            result.wedge20WireIndex = result.southWireIndex - 5;
            if (result.wedge20WireIndex < 0)
                result.wedge20WireIndex += calib.wires.wireEndpoints.size();
        }
        else
        {
            // For non-star cameras: analyze CLIP WIRES to determine camera position
            if (clipWires.size() == 4)
            {
                Point2f southDirection(0, 1); // Pointing down
                int validClipWires = 0;

                // Calculate average cross product of all clip wires relative to south line
                for (const auto &clipWire : clipWires)
                {
                    Point2f wireDirection = clipWire.first - center;
                    float length = norm(wireDirection);
                    if (length > 0)
                    {
                        wireDirection = wireDirection / length;

                        // Calculate cross product to determine which side of south line the clip wire is on
                        float crossProduct = southDirection.x * wireDirection.y - southDirection.y * wireDirection.x;
                        result.avgClipWireCrossProduct += crossProduct;
                        validClipWires++;
                    }
                }

                if (validClipWires > 0)
                {
                    result.avgClipWireCrossProduct /= validClipWires;

                    if (result.avgClipWireCrossProduct < 0)
                    {
                        // Clip wires are predominantly to the LEFT of south line → TOP camera → wedge 12
                        result.cameraPosition = CameraPosition::TOP;
                        result.wedgeNumber = 12;
                        // For top camera: wedge 20 is 17 steps back from south wire (wedge 12)
                        result.wedge20WireIndex = result.southWireIndex - 18;
                        if (result.wedge20WireIndex < 0)
                            result.wedge20WireIndex += calib.wires.wireEndpoints.size();
                    }
                    else
                    {
                        // Clip wires are predominantly to the RIGHT of south line → BOTTOM camera → wedge 7
                        result.cameraPosition = CameraPosition::BOTTOM;
                        result.wedgeNumber = 7;
                        // For bottom camera: wedge 20 is 12 steps back from south wire (wedge 7)
                        result.wedge20WireIndex = result.southWireIndex - 12;
                        if (result.wedge20WireIndex < 0)
                            result.wedge20WireIndex += calib.wires.wireEndpoints.size();
                    }
                }
            }
        }

        // STEP 4: Debug output
        log_info("Camera " + log_string(calib.camera_index) + " Position: " + cameraPositionToString(result.cameraPosition));
        log_info("Camera " + log_string(calib.camera_index) + " Wedge Number: " + log_string(result.wedgeNumber));
        log_info("Camera " + log_string(calib.camera_index) + " South Wire Index: " + log_string(result.southWireIndex));
        log_info("Camera " + log_string(calib.camera_index) + " Wedge 20 Wire Index: " + log_string(result.wedge20WireIndex));
        log_info("Camera " + log_string(calib.camera_index) + " Angle Offset: " + log_string(result.angleOffsetFromSouth) + "°");
        log_info("=== END CAMERA POSITION DETERMINATION ===");

        return result;
    }

    OrientationData processOrientation(
        const Mat &frame,
        const Mat &colorMask,
        const DartboardCalibration &calib,
        bool enableDebug,
        const OrientationParams &params)
    {
        log_info("STARTING ORIENTATION DETECTION for camera " + log_string(calib.camera_index));

        // Safety checks
        if (frame.empty())
        {
            log_error("Invalid input data for orientation detection");
            OrientationData result;
            result.camera_index = calib.camera_index;
            return result;
        }

        log_debug("Available wire count: " + log_string(calib.wires.wireEndpoints.size()));

        // Create debug visualization framework
        Mat debugFrame;
        if (enableDebug)
        {
            debugFrame = frame.clone();
            system("mkdir -p debug_frames/orientation_processing");
        }

        // STEP 1: Create the number region mask and apply preprocessing
        Mat binaryTextMask = createNumberRegionMask(frame, calib, enableDebug, params);

        // STEP 2: Find clip wires using the dartboard mask
        vector<pair<Point2f, Point2f>> clipWires = findClipWires(frame, binaryTextMask, calib, enableDebug, params);

        // STEP 3: Determine comprehensive orientation data
        OrientationData result = determineCameraPosition(clipWires, calib);

        // STEP 4: Comprehensive debug visualization
        if (enableDebug)
        {
            Mat orientationDebug = frame.clone();
            Point2f center = calib.bullCenter;

            // Draw clip wires in green
            for (const auto &wire : clipWires)
            {
                line(orientationDebug, wire.first, wire.second, Scalar(0, 255, 0), 2);
                circle(orientationDebug, wire.first, 5, Scalar(0, 255, 0), -1);
            }

            // Draw south wire in blue (if found)
            if (result.southWireIndex >= 0 && result.southWireIndex < calib.wires.wireEndpoints.size())
            {
                Point2f southWire = calib.wires.wireEndpoints[result.southWireIndex];
                line(orientationDebug, center, southWire, Scalar(255, 0, 0), 3);
                circle(orientationDebug, southWire, 8, Scalar(255, 0, 0), -1);
                putText(orientationDebug, "SOUTH", southWire + Point2f(10, 10),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
            }

            // Draw wedge 20 wire (for all cameras)
            if (result.wedge20WireIndex >= 0 && result.wedge20WireIndex < calib.wires.wireEndpoints.size())
            {
                Point2f wedge20Wire = calib.wires.wireEndpoints[result.wedge20WireIndex];
                line(orientationDebug, center, wedge20Wire, Scalar(0, 0, 255), 4);
                circle(orientationDebug, wedge20Wire, 10, Scalar(0, 0, 255), -1);
                putText(orientationDebug, "WEDGE 20", wedge20Wire + Point2f(10, -10),
                        FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
            }

            // Draw south reference line
            Point2f southLineEnd = center + Point2f(0, 100);
            line(orientationDebug, center, southLineEnd, Scalar(128, 128, 128), 2);
            putText(orientationDebug, "SOUTH REF", southLineEnd + Point2f(5, 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(128, 128, 128), 1);

            // Add comprehensive camera status text
            string statusText = "CAM " + to_string(calib.camera_index) + ": " +
                                (result.isStarCamera ? "STAR" : "NON-STAR") + " | " +
                                cameraPositionToString(result.cameraPosition) + " | WEDGE " + to_string(result.wedgeNumber) +
                                " | S:" + to_string(result.southWireIndex) + " | W20:" + to_string(result.wedge20WireIndex);
            putText(orientationDebug, statusText, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);

            imwrite("debug_frames/orientation_processing/orientation_result_" + to_string(calib.camera_index) + ".jpg", orientationDebug);
        }

        return result;
    }

} // namespace orientation_processing