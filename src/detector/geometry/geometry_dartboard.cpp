#include "geometry_dartboard.hpp"
#include "geometry_detector.hpp" // For DartboardCalibration
#include <iostream>
#include <cmath>

// Define standard dartboard segment values (clockwise order)
const int GeometryDartboard::standardSegments[20] =
    {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};

void GeometryDartboard::drawCalibrationOverlay(
    cv::Mat &frame,
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

    // Draw outer boundary
    cv::circle(frame, calib.center, calib.radius, cv::Scalar(255, 255, 255), 2);

    // Draw rings with distinctive colors
    cv::circle(frame, calib.center, calib.doubleRingInner, cv::Scalar(0, 0, 255), 2); // Red
    cv::circle(frame, calib.center, calib.doubleRingOuter, cv::Scalar(0, 0, 255), 2); // Red
    cv::circle(frame, calib.center, calib.tripleRingInner, cv::Scalar(0, 255, 0), 2); // Green
    cv::circle(frame, calib.center, calib.tripleRingOuter, cv::Scalar(0, 255, 0), 2); // Green

    // Draw bull
    cv::circle(frame, calib.center, calib.bullRadius, cv::Scalar(0, 255, 255), 2);    // Yellow
    cv::circle(frame, calib.center, calib.bullRadius / 2, cv::Scalar(255, 0, 0), -1); // Red center

    // ALWAYS USE 270 DEGREES FOR SEGMENT 20 (top) - FIXED ORIENTATION
    double orientationRad = 270.0 * CV_PI / 180.0;

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
        cv::rectangle(overlay, cv::Point(10, 10), cv::Point(250, 100),
                      cv::Scalar(0, 0, 0), -1);
        cv::addWeighted(overlay, 0.7, frame, 0.3, 0, frame);

        // UPDATED: Show camera number and coordinates instead of angle
        cv::putText(frame,
                    "Camera " + std::to_string(calib.camera_index + 1),
                    cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 1);

        // Show X,Y coordinates of center
        cv::putText(frame,
                    "Center: (" + std::to_string(calib.center.x) + "," +
                        std::to_string(calib.center.y) + ")",
                    cv::Point(15, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 1);

        // Show radius (still useful for debugging)
        cv::putText(frame,
                    "Radius: " + std::to_string(int(calib.radius)),
                    cv::Point(15, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 1);

        // REMOVED: No longer show the "CALIBRATED" text as it's redundant
    }
}

std::string GeometryDartboard::calculateScore(
    const cv::Point &dartPosition,
    const DartboardCalibration &calib)
{
    // Calculate distance from center
    double distance = distanceToPoint(dartPosition, calib.center);

    // Miss if outside dartboard
    if (distance > calib.radius * 1.1)
    {
        return "MISS";
    }

    // Calculate angle in radians from center to dart position
    double angleRad = atan2(dartPosition.y - calib.center.y,
                            dartPosition.x - calib.center.x);

    // Convert to degrees
    double angleDeg = angleRad * 180.0 / CV_PI;

    // Normalize to positive angles (0-360)
    while (angleDeg < 0)
    {
        angleDeg += 360.0;
    }

    // ALWAYS USE 270 DEGREES FOR SEGMENT 20 - FIXED ORIENTATION
    double adjustedAngle = angleDeg - 270.0;

    // Normalize again after adjustment
    while (adjustedAngle < 0)
        adjustedAngle += 360.0;
    while (adjustedAngle >= 360.0)
        adjustedAngle -= 360.0;

    // Calculate segment index (20 segments of 18 degrees each)
    int segmentIndex = static_cast<int>(adjustedAngle / 18.0) % 20;
    int segmentValue = standardSegments[segmentIndex];

    // Check for bullseye and scoring regions
    bool isBull = distance <= calib.bullRadius;
    bool isDoubleBull = distance <= calib.bullRadius / 2.0;
    bool isDouble = distance >= calib.doubleRingInner && distance <= calib.doubleRingOuter;
    bool isTriple = distance >= calib.tripleRingInner && distance <= calib.tripleRingOuter;

    // Return the appropriate score
    if (isBull)
    {
        return isDoubleBull ? "BULL" : "S-BULL";
    }

    return formatScore(segmentValue, isDouble, isTriple, false);
}

std::string GeometryDartboard::formatScore(
    int value, bool isDouble, bool isTriple, bool isBull)
{
    if (isBull)
    {
        return "BULL";
    }
    if (isDouble)
    {
        return "D" + std::to_string(value);
    }
    if (isTriple)
    {
        return "T" + std::to_string(value);
    }
    return "S" + std::to_string(value);
}

double GeometryDartboard::distanceToPoint(
    const cv::Point &p1, const cv::Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) +
                     std::pow(p1.y - p2.y, 2));
}
