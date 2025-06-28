#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

// Include the calibration struct
#include "../calibration/geometry_calibration.hpp"

namespace dartboard_visualization
{
    inline cv::Mat drawCalibrationOverlay(const cv::Mat &frame, const DartboardCalibration &calib, bool showDetails = true)
    {
        cv::Mat visFrame = frame.clone();

        // Safety check
        if (frame.empty())
        {
            return visFrame;
        }

        // Only draw if we have valid calibration
        if (!calib.hasDetectedEllipses)
        {
            // Draw basic info for failed calibration
            cv::putText(visFrame, "CALIBRATION FAILED", cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
            cv::circle(visFrame, calib.center, 10, cv::Scalar(0, 0, 255), 3);
            return visFrame;
        }

        // ===== DRAW ALL DETECTED ELLIPSES =====

        // 1. Doubles ring (outermost) - THICK CYAN
        cv::ellipse(visFrame, calib.doubleOuterRing, cv::Scalar(255, 255, 0), 4); // Cyan outer boundary
        cv::ellipse(visFrame, calib.doubleInnerRing, cv::Scalar(255, 255, 0), 3); // Cyan inner boundary

        // 2. Triples ring - THICK GREEN
        cv::ellipse(visFrame, calib.tripleOuterRing, cv::Scalar(0, 255, 0), 3); // Green outer
        cv::ellipse(visFrame, calib.tripleInnerRing, cv::Scalar(0, 255, 0), 3); // Green inner

        // 3. Bull rings - THICK RED
        cv::ellipse(visFrame, calib.bullOuterRing, cv::Scalar(0, 0, 255), 3); // Red outer bull (25-point)
        cv::ellipse(visFrame, calib.bullInnerRing, cv::Scalar(0, 0, 255), 3); // Red inner bull (50-point)

        // ===== DRAW DARTBOARD SEGMENTS =====

        std::vector<int> segments = {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};
        float anglePerSegment = 360.0f / 20.0f;

        // EXPERIMENTAL: Add small orientation offset for fine-tuning alignment
        float orientationOffset = 0.5f; // Try values like -10, -5, 0, +5, +10 degrees
        float startAngle = calib.orientation + orientationOffset;

        for (int i = 0; i < 20; i++)
        {
            float segmentAngle = startAngle + (i * anglePerSegment);

            // EXPERIMENTAL: Try different perspective corrections
            cv::RotatedRect outerRing = calib.doubleOuterRing;
            float cx = outerRing.center.x;
            float cy = outerRing.center.y;
            float a = outerRing.size.width / 2.0f;
            float b = outerRing.size.height / 2.0f;
            float angle_rad = outerRing.angle * CV_PI / 180.0f;

            // Try perspective distortion compensation
            float radians = segmentAngle * CV_PI / 180.0f;
            cv::Point2f testDir(cos(radians), sin(radians));

            // Calculate how much this direction is affected by perspective
            float cos_a = cos(-angle_rad);
            float sin_a = sin(-angle_rad);
            float dx_rot = testDir.x * cos_a - testDir.y * sin_a;
            float dy_rot = testDir.x * sin_a + testDir.y * cos_a;

            // Estimate perspective distortion at this angle
            float distortionX = abs(dx_rot) * (a / b - 1.0f);
            float distortionY = abs(dy_rot) * (b / a - 1.0f);
            float totalDistortion = distortionX + distortionY;

            // Apply correction - experiment with different factors
            float correctionFactor = totalDistortion * -3.0f;
            float correctedAngle = segmentAngle + correctionFactor;

            float correctedRadians = correctedAngle * CV_PI / 180.0f;
            cv::Point2f direction(cos(correctedRadians), sin(correctedRadians));

            // Debug output for first few segments
            if (i < 3)
            {
                std::cout << "Segment " << i << ": original=" << segmentAngle
                          << " distortion=" << totalDistortion
                          << " corrected=" << correctedAngle << std::endl;
            }

            // Calculate intersection with doubles outer ring (REUSE existing variables!)
            float cos_a2 = cos(-angle_rad);
            float sin_a2 = sin(-angle_rad);
            float dx_rot2 = direction.x * cos_a2 - direction.y * sin_a2;
            float dy_rot2 = direction.x * sin_a2 + direction.y * cos_a2;

            float A = (dx_rot2 * dx_rot2) / (a * a) + (dy_rot2 * dy_rot2) / (b * b);
            float t = sqrt(1.0f / A);

            cv::Point ellipseEnd = calib.center + cv::Point(direction.x * t * a, direction.y * t * b);
            cv::Point2f relative(ellipseEnd.x - cx, ellipseEnd.y - cy);
            cv::Point2f rotated(
                relative.x * cos(angle_rad) - relative.y * sin(angle_rad),
                relative.x * sin(angle_rad) + relative.y * cos(angle_rad));
            cv::Point finalEnd(cx + rotated.x, cy + rotated.y);

            // Calculate ACTUAL ellipse intersection distance for THIS specific segment
            cv::Point2f rayStart = cv::Point2f(calib.center.x, calib.center.y);

            // Use the CORRECT ray direction from the original math (finalEnd direction)
            cv::Point2f correctRayDir = cv::Point2f(finalEnd - calib.center);
            correctRayDir = correctRayDir / cv::norm(correctRayDir); // Normalize

            // Find where this ray actually intersects the ellipse boundary
            float actualEllipseDistance = 0;
            for (float dist = 1.0f; dist < 500.0f; dist += 0.5f)
            {
                cv::Point2f testPoint = rayStart + correctRayDir * dist;
                cv::Point2f relative = testPoint - cv::Point2f(cx, cy);
                cv::Point2f rotated(
                    relative.x * cos(-angle_rad) - relative.y * sin(-angle_rad),
                    relative.x * sin(-angle_rad) + relative.y * cos(-angle_rad));

                float ellipseValue = (rotated.x * rotated.x) / (a * a) + (rotated.y * rotated.y) / (b * b);
                if (ellipseValue >= 1.0f)
                {
                    actualEllipseDistance = dist;
                    break;
                }
            }

            // CLAMP to the actual ellipse distance for this segment with 10% padding
            cv::Point2f lineVector = cv::Point2f(finalEnd - calib.center);
            float originalDistance = cv::norm(lineVector);

            if (originalDistance > actualEllipseDistance && actualEllipseDistance > 0)
            {
                // Add 10% padding to the clamped distance
                float paddedDistance = actualEllipseDistance * 1.10f;
                cv::Point2f normalizedDirection = lineVector / originalDistance;
                finalEnd = calib.center + cv::Point(normalizedDirection.x * paddedDistance,
                                                    normalizedDirection.y * paddedDistance);
            }

            // Draw segment lines - BRIGHT YELLOW
            cv::line(visFrame, calib.center, finalEnd, cv::Scalar(0, 255, 255), 2);

            // Position numbers at the END of segment lines with perpendicular offset
            cv::Point2f lineDirection = cv::Point2f(finalEnd - calib.center);
            lineDirection = lineDirection / cv::norm(lineDirection); // Normalize

            // Calculate perpendicular vector (rotate 90 degrees)
            cv::Point2f perpendicular(-lineDirection.y, lineDirection.x);

            // Position at end of line + small outward offset + LARGER perpendicular offset
            cv::Point2f labelBasePos = cv::Point2f(calib.center) + lineDirection * (cv::norm(cv::Point2f(finalEnd - calib.center)) + 15.0f);
            cv::Point2f labelPos = labelBasePos + perpendicular * 45.0f; // Increased from 20 to 35

            // Black outline for better visibility
            cv::putText(visFrame, std::to_string(segments[i]), cv::Point(labelPos.x - 8, labelPos.y + 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 3); // Black outline
            cv::putText(visFrame, std::to_string(segments[i]), cv::Point(labelPos.x - 8, labelPos.y + 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2); // Yellow text
        }

        // ===== CALIBRATION STATUS & INFO =====

        if (showDetails)
        {
            // Status banner - GREEN for success
            cv::putText(visFrame, "CALIBRATION SUCCESS", cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 3);

            // Camera info
            cv::putText(visFrame, "Camera " + std::to_string(calib.camera_index), cv::Point(20, 70),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            // Bull center coordinates
            cv::putText(visFrame, "Bull: (" + std::to_string(calib.center.x) + "," + std::to_string(calib.center.y) + ")",
                        cv::Point(20, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            // Perspective info (if there's offset)
            if (calib.perspectiveOffsetMagnitude > 5)
            {
                std::string offsetInfo = "Offset: " + std::to_string(int(calib.perspectiveOffsetMagnitude)) + "px";
                cv::Scalar offsetColor = cv::Scalar(0, 255, 255); // Yellow for moderate offset

                if (calib.perspectiveOffsetMagnitude > 50)
                {
                    offsetColor = cv::Scalar(0, 165, 255); // Orange for high offset
                    if (abs(calib.perspectiveOffsetX) > abs(calib.perspectiveOffsetY))
                    {
                        offsetInfo += (calib.perspectiveOffsetX > 0) ? " (Camera LEFT)" : " (Camera RIGHT)";
                    }
                    else
                    {
                        offsetInfo += (calib.perspectiveOffsetY > 0) ? " (Camera HIGH)" : " (Camera LOW)";
                    }
                }

                cv::putText(visFrame, offsetInfo, cv::Point(20, 130),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, offsetColor, 2);
            }

            // Ring detection status
            std::string ringStatus = "Rings: ";
            ringStatus += "Doubles✓ ";
            ringStatus += calib.tripleOuterRing.size.area() > 0 ? "Triples✓ " : "Triples✗ ";
            ringStatus += calib.bullOuterRing.size.area() > 0 ? "Bulls✓" : "Bulls✗";

            cv::putText(visFrame, ringStatus, cv::Point(20, frame.rows - 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            // Legend in bottom-right corner
            int legendY = frame.rows - 120;
            cv::putText(visFrame, "Legend:", cv::Point(frame.cols - 200, legendY),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            cv::line(visFrame, cv::Point(frame.cols - 190, legendY + 20), cv::Point(frame.cols - 170, legendY + 20), cv::Scalar(255, 255, 0), 3);
            cv::putText(visFrame, "Doubles", cv::Point(frame.cols - 165, legendY + 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            cv::line(visFrame, cv::Point(frame.cols - 190, legendY + 40), cv::Point(frame.cols - 170, legendY + 40), cv::Scalar(0, 255, 0), 3);
            cv::putText(visFrame, "Triples", cv::Point(frame.cols - 165, legendY + 45),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            cv::line(visFrame, cv::Point(frame.cols - 190, legendY + 60), cv::Point(frame.cols - 170, legendY + 60), cv::Scalar(0, 0, 255), 3);
            cv::putText(visFrame, "Bulls", cv::Point(frame.cols - 165, legendY + 65),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        }

        return visFrame;
    }

} // namespace dartboard_visualization