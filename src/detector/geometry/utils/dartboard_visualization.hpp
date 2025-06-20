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
            cv::rectangle(overlay, cv::Point(10, 10), cv::Point(250, 180),
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

            // Ellipse axes info (new)
            cv::putText(frame,
                        "Axes: " + std::to_string(int(calib.axes.width / 2)) + "x" +
                            std::to_string(int(calib.axes.height / 2)),
                        cv::Point(15, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Ellipse angle info (new)
            cv::putText(frame,
                        "Ellipse Angle: " + std::to_string(int(calib.angle)) + "°",
                        cv::Point(15, 110), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Image dimensions (helps with scaling/aspect ratio understanding)
            cv::putText(frame,
                        "Frame: " + std::to_string(frame.cols) + "x" + std::to_string(frame.rows),
                        cv::Point(15, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Aspect ratio
            float aspectRatio = frame.cols / static_cast<float>(frame.rows);
            cv::putText(frame,
                        "Aspect: " + std::to_string(aspectRatio).substr(0, 4) + ":1",
                        cv::Point(15, 150), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 1);

            // Offset from frame center
            int centerOffsetX = calib.center.x - (frame.cols / 2);
            int centerOffsetY = calib.center.y - (frame.rows / 2);
            cv::putText(frame,
                        "Offset: (" + std::to_string(centerOffsetX) + "," +
                            std::to_string(centerOffsetY) + ")",
                        cv::Point(15, 170), cv::FONT_HERSHEY_SIMPLEX, 0.6,
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
