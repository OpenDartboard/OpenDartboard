#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

// Include the full definition
#include "../geometry_detector.hpp" // For DartboardCalibration
#include "math_utils.hpp"           // For math utilities

namespace dartboard_visualization
{

    // Define standard dartboard segment values (clockwise order)
    static const int standardSegments[20] =
        {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};

    // Main overlay drawing function - used for debugging and visualization
    inline void drawCalibrationOverlay(cv::Mat &frame,
                                       const DartboardCalibration &calib,
                                       bool showDetails)
    {
        // Safety check
        if (frame.empty() || calib.radius <= 0)
        {
            return;
        }

        // Draw center point with contrasting colors for better visibility
        cv::circle(frame, calib.center, 3, cv::Scalar(0, 0, 0), -1);    // Black center
        cv::circle(frame, calib.center, 5, cv::Scalar(0, 255, 255), 2); // Yellow outline

        // Always draw as ellipses
        // Create rotated rect for each ring
        cv::RotatedRect outerRing(calib.center, calib.axes, calib.angle);

        // Scale for inner rings based on standard proportions
        cv::Size2f doubleRingSize(calib.axes.width * 0.92, calib.axes.height * 0.92);
        cv::RotatedRect doubleRingInner(calib.center, doubleRingSize, calib.angle);

        cv::Size2f tripleOuterSize(calib.axes.width * 0.63, calib.axes.height * 0.63);
        cv::RotatedRect tripleRingOuter(calib.center, tripleOuterSize, calib.angle);

        cv::Size2f tripleInnerSize(calib.axes.width * 0.55, calib.axes.height * 0.55);
        cv::RotatedRect tripleRingInner(calib.center, tripleInnerSize, calib.angle);

        // Draw rings as ellipses
        cv::ellipse(frame, outerRing, cv::Scalar(255, 255, 255), 2);
        cv::ellipse(frame, doubleRingInner, cv::Scalar(0, 0, 255), 2);
        cv::ellipse(frame, tripleRingOuter, cv::Scalar(0, 255, 0), 2);
        cv::ellipse(frame, tripleRingInner, cv::Scalar(0, 255, 0), 2);

        // Draw bull
        cv::circle(frame, calib.center, calib.bullRadius, cv::Scalar(0, 0, 200), 2);
        cv::circle(frame, calib.center, calib.bullRadius / 2, cv::Scalar(0, 0, 200), -1);

        // CHANGED: Use the orientation from calibration instead of hardcoding to 270
        double orientationRad = calib.orientation * CV_PI / 180.0;

        // Calculate segment 20 position (top)
        cv::Point segment20(
            calib.center.x + calib.radius * 1.1 * cos(orientationRad),
            calib.center.y + calib.radius * 1.1 * sin(orientationRad));

        // Draw segment 20 indicator
        cv::line(frame, calib.center, segment20, cv::Scalar(0, 0, 255), 3);
        cv::putText(frame, "20", segment20 + cv::Point(0, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 5); // Black outline
        cv::putText(frame, "20", segment20 + cv::Point(0, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2); // Yellow text

        // Draw segment lines consistently
        for (int i = 0; i < 20; i++)
        {
            // Calculate the angle for this segment line
            // Start from segment 20 (orientation) and move clockwise
            double segmentAngle = orientationRad + i * (2.0 * CV_PI / 20.0);
            cv::Point lineEnd(
                calib.center.x + calib.radius * cos(segmentAngle),
                calib.center.y + calib.radius * sin(segmentAngle));
            cv::line(frame, calib.center, lineEnd, cv::Scalar(200, 200, 200), 1);

            // Add segment numbers with better visibility if detailed view requested
            if (showDetails)
            {
                // Draw the segment label at the midpoint of the segment
                double labelAngle = orientationRad + (i + 0.5) * (2.0 * CV_PI / 20.0);
                cv::Point labelPos(
                    calib.center.x + calib.radius * 0.75 * cos(labelAngle),
                    calib.center.y + calib.radius * 0.75 * sin(labelAngle));

                // Use the standardSegments array for correct numbering
                int segValue = standardSegments[i];

                // Draw text with background for visibility
                cv::putText(frame, std::to_string(segValue), labelPos,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3); // Black outline
                cv::putText(frame, std::to_string(segValue), labelPos,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1); // White text
            }
        }

        // Add improved calibration info overlay
        if (showDetails)
        {
            // Create dark semi-transparent background for better text visibility
            cv::Mat overlay;
            frame.copyTo(overlay);
            // Make the overlay box larger to accommodate more debug info
            cv::rectangle(overlay, cv::Point(10, 10), cv::Point(250, 160),
                          cv::Scalar(0, 0, 0), -1);
            cv::addWeighted(overlay, 0.7, frame, 0.3, 0, frame);

            // Camera info
            cv::putText(frame,
                        "Camera " + std::to_string(calib.camera_index + 1),
                        cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Center coordinates (more prominent)
            cv::putText(frame,
                        "Center: (" + std::to_string(calib.center.x) + "," +
                            std::to_string(calib.center.y) + ")",
                        cv::Point(15, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Radius info
            cv::putText(frame,
                        "Radius: " + std::to_string(int(calib.radius)),
                        cv::Point(15, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Image dimensions (helps with scaling/aspect ratio understanding)
            cv::putText(frame,
                        "Frame: " + std::to_string(frame.cols) + "x" + std::to_string(frame.rows),
                        cv::Point(15, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Aspect ratio
            float aspectRatio = frame.cols / static_cast<float>(frame.rows);
            cv::putText(frame,
                        "Aspect: " + std::to_string(aspectRatio).substr(0, 4) + ":1",
                        cv::Point(15, 110), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Offset from frame center
            int centerOffsetX = calib.center.x - (frame.cols / 2);
            int centerOffsetY = calib.center.y - (frame.rows / 2);
            cv::putText(frame,
                        "Offset: (" + std::to_string(centerOffsetX) + "," +
                            std::to_string(centerOffsetY) + ")",
                        cv::Point(15, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Estimated camera skew/angle based on center offset
            float angleFromCenter = atan2(centerOffsetY, centerOffsetX) * 180.0 / CV_PI;
            cv::putText(frame,
                        "Angle: " + std::to_string(int(angleFromCenter)) + "°",
                        cv::Point(15, 150), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Add reference grid for alignment (only with detailed view)
            // Draw crosshairs through frame center
            cv::Point frameCenter(frame.cols / 2, frame.rows / 2);
            cv::line(frame,
                     cv::Point(frameCenter.x, 0),
                     cv::Point(frameCenter.x, frame.rows),
                     cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
            cv::line(frame,
                     cv::Point(0, frameCenter.y),
                     cv::Point(frame.cols, frameCenter.y),
                     cv::Scalar(100, 100, 100), 1, cv::LINE_AA);

            // Draw line from frame center to dart center to visualize offset
            cv::line(frame, frameCenter, calib.center,
                     cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

            // Indicate frame center with small circle
            cv::circle(frame, frameCenter, 5, cv::Scalar(70, 70, 70), 1);
            cv::circle(frame, frameCenter, 2, cv::Scalar(70, 70, 70), -1);
        }
    }

    // Cleaner "competition mode" overlay without debug info
    inline void drawCompetitionOverlay(
        cv::Mat &frame,
        const DartboardCalibration &calib)
    {
        // Safety check
        if (frame.empty() || calib.radius <= 0)
            return;

        // Draw clean rings using elliptical parameters for better accuracy
        // Create rotated rect for each ring
        cv::RotatedRect outerRing(calib.center, calib.axes, calib.angle);

        // Scale for inner rings based on standard proportions
        cv::Size2f doubleRingSize(calib.axes.width * 0.92, calib.axes.height * 0.92);
        cv::RotatedRect doubleRingInner(calib.center, doubleRingSize, calib.angle);

        cv::Size2f tripleOuterSize(calib.axes.width * 0.63, calib.axes.height * 0.63);
        cv::RotatedRect tripleRingOuter(calib.center, tripleOuterSize, calib.angle);

        cv::Size2f tripleInnerSize(calib.axes.width * 0.55, calib.axes.height * 0.55);
        cv::RotatedRect tripleRingInner(calib.center, tripleInnerSize, calib.angle);

        // Draw rings as ellipses
        cv::ellipse(frame, outerRing, cv::Scalar(255, 255, 255), 2);
        cv::ellipse(frame, doubleRingInner, cv::Scalar(0, 0, 200), 2);
        cv::ellipse(frame, tripleRingOuter, cv::Scalar(0, 200, 0), 2);
        cv::ellipse(frame, tripleRingInner, cv::Scalar(0, 200, 0), 2);

        // Draw bull with clean red/green colors
        cv::circle(frame, calib.center, calib.bullRadius, cv::Scalar(0, 0, 200), 2);
        cv::circle(frame, calib.center, calib.bullRadius / 2, cv::Scalar(0, 0, 200), -1);

        // Add segment dividers (20 segments) with clean lines
        double orientationRad = calib.orientation * CV_PI / 180.0;
        for (int i = 0; i < 20; i++)
        {
            double segmentAngle = orientationRad + i * (2.0 * CV_PI / 20.0);

            // Calculate end points on ellipse boundary using parametric equation
            double a = calib.axes.width / 2;  // semi-major axis
            double b = calib.axes.height / 2; // semi-minor axis

            // Rotate angle by the ellipse angle to account for ellipse rotation
            double rotatedAngle = segmentAngle - (calib.angle * CV_PI / 180.0);

            // Parametric equation of ellipse
            double x = a * cos(rotatedAngle);
            double y = b * sin(rotatedAngle);

            // Rotate back to original coordinate system
            double cosAngle = cos(calib.angle * CV_PI / 180.0);
            double sinAngle = sin(calib.angle * CV_PI / 180.0);
            double xRot = x * cosAngle - y * sinAngle;
            double yRot = x * sinAngle + y * cosAngle;

            // Create end point and draw line
            cv::Point lineEnd(calib.center.x + xRot, calib.center.y + yRot);
            cv::line(frame, calib.center, lineEnd, cv::Scalar(255, 255, 255), 1);
        }
    }

    // Utility to scale calibration parameters for visualization
    inline DartboardCalibration scaleCalibrationForDisplay(
        const DartboardCalibration &calib,
        const cv::Mat &originalFrame,
        int targetWidth, int targetHeight,
        cv::Point &outOffset)
    {
        // Create a copy to modify
        DartboardCalibration scaledCalib = calib;

        // Calculate aspect ratio-preserving dimensions
        double aspectRatio = originalFrame.cols / (double)originalFrame.rows;
        int scaledWidth = targetWidth;
        int scaledHeight = static_cast<int>(scaledWidth / aspectRatio);

        // If height exceeds target, scale down proportionally
        if (scaledHeight > targetHeight)
        {
            scaledHeight = targetHeight;
            scaledWidth = static_cast<int>(scaledHeight * aspectRatio);
        }

        // Calculate offset to center frame on black background
        outOffset.x = (targetWidth - scaledWidth) / 2;
        outOffset.y = (targetHeight - scaledHeight) / 2;

        // Calculate scale factors
        double scaleX = scaledWidth / (double)originalFrame.cols;
        double scaleY = scaledHeight / (double)originalFrame.rows;

        // Apply scale and offset to center point
        scaledCalib.center.x = static_cast<int>(scaledCalib.center.x * scaleX) + outOffset.x;
        scaledCalib.center.y = static_cast<int>(scaledCalib.center.y * scaleY) + outOffset.y;

        // Use the smaller scale factor to avoid oval shapes
        double scaleFactor = std::min(scaleX, scaleY);
        scaledCalib.radius *= scaleFactor;

        // IMPORTANT: Scale the elliptical parameters too
        scaledCalib.axes.width *= scaleFactor;
        scaledCalib.axes.height *= scaleFactor;

        // Update all ring proportions
        // After scaling the radius, we can derive the other ring sizes
        scaledCalib.bullRadius = scaledCalib.radius * 0.07;
        scaledCalib.doubleRingInner = scaledCalib.radius * 0.92;
        scaledCalib.doubleRingOuter = scaledCalib.radius;
        scaledCalib.tripleRingInner = scaledCalib.radius * 0.55;
        scaledCalib.tripleRingOuter = scaledCalib.radius * 0.63;

        return scaledCalib;
    }

    // Function to save a multi-camera debug view
    inline void saveMultiCameraDebugView(
        const std::vector<cv::Mat> &frames,
        const std::vector<DartboardCalibration> &calibrations,
        const std::vector<DartDetection> &detections,
        int target_width, int target_height,
        bool competition_style = false)
    {
        if (frames.empty())
            return;

        // Create resized frames
        std::vector<cv::Mat> resized_frames;
        std::vector<cv::Point> offsets; // Store offsets for each frame
        int max_width = 0, total_height = 0;

        for (const auto &frame : frames)
        {
            cv::Mat resized;
            cv::Point offset;

            if (frame.empty())
            {
                // Use placeholder for missing frames
                resized = cv::Mat(target_height, target_width, CV_8UC3, cv::Scalar(30, 30, 30));
                cv::putText(resized, "Camera disconnected", cv::Point(180, 240),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                offset = cv::Point(0, 0); // No offset for placeholders
            }
            else
            {
                // Create black background image of standard size
                resized = cv::Mat(target_height, target_width, CV_8UC3, cv::Scalar(0, 0, 0));

                // Calculate aspect-ratio preserving dimensions and place the frame
                int targetWidth = target_width;
                int targetHeight = static_cast<int>(targetWidth / (frame.cols / (double)frame.rows));

                if (targetHeight > target_height)
                {
                    targetHeight = target_height;
                    targetWidth = static_cast<int>(targetHeight * (frame.cols / (double)frame.rows));
                }

                offset.x = (target_width - targetWidth) / 2;
                offset.y = (target_height - targetHeight) / 2;

                // Resize the actual frame
                cv::Mat properlyResized;
                cv::resize(frame, properlyResized, cv::Size(targetWidth, targetHeight));

                // Create ROI and copy the resized frame
                cv::Mat roi = resized(cv::Rect(offset.x, offset.y, targetWidth, targetHeight));
                properlyResized.copyTo(roi);
            }

            resized_frames.push_back(resized);
            offsets.push_back(offset);
            max_width = std::max(max_width, resized.cols);
            total_height += resized.rows;
        }

        // Create combined image
        cv::Mat combined = cv::Mat::zeros(total_height, max_width, CV_8UC3);
        int y_offset = 0;

        // Add title to the multi-view image
        cv::putText(combined, competition_style ? "Dartboard Game View" : "Dartboard Multi-Camera View",
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                    0.8, cv::Scalar(255, 255, 255), 2);

        // Add each camera view
        for (size_t i = 0; i < resized_frames.size(); i++)
        {
            // Get region for this camera
            cv::Mat region = combined(cv::Rect(0, y_offset,
                                               resized_frames[i].cols,
                                               resized_frames[i].rows));

            // Copy frame
            resized_frames[i].copyTo(region);

            // Draw calibration overlay if available for this camera
            bool found_calibration = false;
            for (size_t calib_idx = 0; calib_idx < calibrations.size(); calib_idx++)
            {
                // Use the stored camera_index to find the right calibration
                if (calibrations[calib_idx].camera_index == i && !frames[i].empty())
                {
                    // Use our utility method to get correctly scaled calibration
                    cv::Point unused_offset; // We already have the offset
                    DartboardCalibration displayCalib = scaleCalibrationForDisplay(
                        calibrations[calib_idx],
                        frames[i],
                        target_width,
                        target_height,
                        unused_offset);

                    // Apply the stored offset
                    displayCalib.center.x = (displayCalib.center.x - unused_offset.x) + offsets[i].x;
                    displayCalib.center.y = (displayCalib.center.y - unused_offset.y) + offsets[i].y;

                    std::cout << "DEBUG: Cam " << i + 1
                              << " center=(" << displayCalib.center.x << ","
                              << displayCalib.center.y << "), center offset=("
                              << (displayCalib.center.x - (region.cols / 2)) << ","
                              << (displayCalib.center.y - (region.rows / 2)) << ")"
                              << ", radius=(" << displayCalib.radius << ")"
                              << ", orientation=" << displayCalib.orientation
                              << std::endl;

                    // Use either competition style or debug style based on setting
                    if (competition_style)
                    {
                        drawCompetitionOverlay(region, displayCalib);

                        // Add camera number in small, unobtrusive text for competition mode
                        cv::putText(region, "CAM " + std::to_string(i + 1),
                                    cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                    cv::Scalar(200, 200, 200), 1);
                    }
                    else
                    {
                        drawCalibrationOverlay(region, displayCalib, true);
                    }

                    found_calibration = true;
                    break;
                }
            }

            // If no calibration found for this camera, just add the camera label
            if (!found_calibration)
            {
                std::cout << "DEBUG-MULTIVIEW: No calibration for camera " << i + 1 << std::endl;
            }

            y_offset += region.rows;
        }

        // Save the debug view
        std::string dir = "debug_frames";
        system(("mkdir -p " + dir).c_str());
        cv::imwrite(dir + "/dartboard_view.jpg", combined);
    }

    // Function to save a single camera debug view
    inline void saveSingleCameraDebugView(
        const cv::Mat &frame,
        const DartboardCalibration &calibration,
        const std::string &outputPath,
        int targetWidth,
        int targetHeight,
        const std::string &label = "")
    {
        // Create debug view and scale calibration
        cv::Mat debugVis = cv::Mat(targetHeight, targetWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Point offset;

        // Scale calibration for display - reuse existing function
        DartboardCalibration displayCalib = scaleCalibrationForDisplay(
            calibration, frame, targetWidth, targetHeight, offset);

        // Resize the frame and place it on the background
        cv::Mat properlyResized;
        int resizedWidth = targetWidth - 2 * offset.x;
        int resizedHeight = targetHeight - 2 * offset.y;
        cv::resize(frame, properlyResized, cv::Size(resizedWidth, resizedHeight));
        cv::Mat roi = debugVis(cv::Rect(offset.x, offset.y, resizedWidth, resizedHeight));
        properlyResized.copyTo(roi);

        // Draw calibration overlay using existing function
        drawCalibrationOverlay(debugVis, displayCalib, true);

        // Add camera identifier if label is provided
        if (!label.empty())
        {
            cv::rectangle(debugVis, cv::Point(targetWidth - 160, 10),
                          cv::Point(targetWidth - 10, 40), cv::Scalar(0, 0, 0), -1);
            cv::putText(debugVis, label, cv::Point(targetWidth - 150, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
        }

        // Save debug image
        cv::imwrite(outputPath, debugVis);
    }

} // end namespace
