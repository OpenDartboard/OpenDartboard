#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

// Include the calibration struct
#include "geometry_calibration.hpp"
#include "utils.hpp"

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
        if (!calib.ellipses.hasValidDoubles)
        {
            // Draw basic info for failed calibration
            cv::putText(visFrame, "CALIBRATION FAILED", cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
            cv::circle(visFrame, calib.bullCenter, 10, cv::Scalar(0, 0, 255), 3);
            return visFrame;
        }

        // ===== DRAW DETECTED ELLIPSES =====
        // Doubles ring (outermost) - THICK CYAN
        cv::ellipse(visFrame, calib.ellipses.outerDoubleEllipse, cv::Scalar(255, 255, 0), 4);
        cv::ellipse(visFrame, calib.ellipses.innerDoubleEllipse, cv::Scalar(255, 255, 0), 3);

        // Triples ring - THICK GREEN
        cv::ellipse(visFrame, calib.ellipses.outerTripleEllipse, cv::Scalar(0, 255, 0), 3);
        cv::ellipse(visFrame, calib.ellipses.innerTripleEllipse, cv::Scalar(0, 255, 0), 3);

        // Bull rings - THICK RED
        cv::ellipse(visFrame, calib.ellipses.outerBullEllipse, cv::Scalar(0, 0, 255), 3);
        cv::ellipse(visFrame, calib.ellipses.innerBullEllipse, cv::Scalar(0, 0, 255), 3);

        // ===== DRAW ACTUAL DETECTED WIRES =====
        if (calib.wires.isValid && !calib.wires.wireEndpoints.empty())
        {
            log_debug("Drawing " + log_string(calib.wires.wireEndpoints.size()) + " detected wires");

            // Draw each detected wire
            for (size_t i = 0; i < calib.wires.wireEndpoints.size(); i++)
            {
                cv::Point2f wireEnd = calib.wires.wireEndpoints[i];

                // Check if this is the wedge 20 wire and draw it thicker
                if (calib.orientation.wedge20WireIndex >= 0 && i == calib.orientation.wedge20WireIndex)
                {
                    // Draw wedge 20 wire as thick red line
                    cv::line(visFrame, calib.bullCenter, wireEnd, cv::Scalar(0, 0, 255), 5);
                    cv::circle(visFrame, wireEnd, 6, cv::Scalar(0, 0, 255), -1);
                }
                else
                {
                    // Draw regular wire line from bull center to detected endpoint - BRIGHT YELLOW
                    cv::line(visFrame, calib.bullCenter, wireEnd, cv::Scalar(0, 255, 255), 2);
                    cv::circle(visFrame, wireEnd, 3, cv::Scalar(0, 255, 255), -1);
                }
            }

            // Draw dartboard segment numbers using orientation data
            if (calib.orientation.wedge20WireIndex >= 0 && calib.wires.wireEndpoints.size() >= 20)
            {
                // Standard dartboard sequence starting from 20
                std::vector<int> dartboardSequence = {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};

                // Draw filled wedge 20 sector first (so it's behind other elements)
                int wedge20SegmentIndex = 0; // First in sequence
                int currentWireIndex = (calib.orientation.wedge20WireIndex + wedge20SegmentIndex) % calib.wires.wireEndpoints.size();
                int nextWireIndex = (currentWireIndex + 1) % calib.wires.wireEndpoints.size();

                cv::Point2f currentWire = calib.wires.wireEndpoints[currentWireIndex];
                cv::Point2f nextWire = calib.wires.wireEndpoints[nextWireIndex];

                // Create filled triangle for wedge 20
                std::vector<cv::Point> wedge20Points = {
                    calib.bullCenter,
                    cv::Point(currentWire.x, currentWire.y),
                    cv::Point(nextWire.x, nextWire.y)};

                // Draw filled wedge with transparency
                cv::Mat overlay = visFrame.clone();
                cv::fillPoly(overlay, wedge20Points, cv::Scalar(0, 0, 255)); // Red fill
                cv::addWeighted(visFrame, 0.7, overlay, 0.3, 0, visFrame);   // 30% transparency

                for (int i = 0; i < 20 && i < calib.wires.wireEndpoints.size(); i++)
                {
                    // Calculate which wire index corresponds to this dartboard segment
                    int wireIndex = (calib.orientation.wedge20WireIndex + i) % calib.wires.wireEndpoints.size();
                    int nextWireIndex = (wireIndex + 1) % calib.wires.wireEndpoints.size();

                    cv::Point2f currentWire = calib.wires.wireEndpoints[wireIndex];
                    cv::Point2f nextWire = calib.wires.wireEndpoints[nextWireIndex];

                    cv::Point2f currentDir = currentWire - cv::Point2f(calib.bullCenter);
                    cv::Point2f nextDir = nextWire - cv::Point2f(calib.bullCenter);

                    float currentAngle = atan2(currentDir.y, currentDir.x);
                    float nextAngle = atan2(nextDir.y, nextDir.x);

                    // Handle angle wraparound
                    if (nextAngle < currentAngle)
                    {
                        nextAngle += 2 * CV_PI;
                    }

                    // Calculate middle angle between current and next wire
                    float midAngle = (currentAngle + nextAngle) / 2.0f;
                    cv::Point2f midDirection(cos(midAngle), sin(midAngle));

                    // Position number at distance from center
                    float labelDistance = cv::norm(currentDir) * 1.15f; // 15% beyond wire endpoint
                    cv::Point2f labelPos = cv::Point2f(calib.bullCenter) + midDirection * labelDistance;

                    // Get the correct dartboard number for this segment
                    int segmentNumber = dartboardSequence[i];

                    // Highlight wedge 20 with different color
                    cv::Scalar textColor = (segmentNumber == 20) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 255);
                    cv::Scalar outlineColor = cv::Scalar(0, 0, 0);

                    // Draw segment number with outline for visibility
                    cv::putText(visFrame, std::to_string(segmentNumber),
                                cv::Point(labelPos.x - 8, labelPos.y + 4),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, outlineColor, 3); // Black outline
                    cv::putText(visFrame, std::to_string(segmentNumber),
                                cv::Point(labelPos.x - 8, labelPos.y + 4),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, textColor, 2); // Colored text
                }
            }
        }
        else
        {
            log_warning("No valid wire data to draw");
            cv::putText(visFrame, "NO WIRE DATA", cv::Point(20, frame.rows - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
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
            cv::putText(visFrame, "Bull: (" + std::to_string(calib.bullCenter.x) + "," + std::to_string(calib.bullCenter.y) + ")",
                        cv::Point(20, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            // Wire detection info
            std::string wireInfo = "Wires: " + std::to_string(calib.wires.wireEndpoints.size()) + "/??";
            cv::Scalar wireColor = calib.wires.isValid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            cv::putText(visFrame, wireInfo, cv::Point(20, 130),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, wireColor, 2);

            // Perspective info
            if (calib.ellipses.offsetMagnitude > 5)
            {
                std::string offsetInfo = "Offset: " + std::to_string(int(calib.ellipses.offsetMagnitude)) + "px";
                cv::Scalar offsetColor = cv::Scalar(0, 255, 255); // Yellow for moderate offset

                if (calib.ellipses.offsetMagnitude > 50)
                {
                    offsetColor = cv::Scalar(0, 165, 255); // Orange for high offset
                    if (abs(calib.ellipses.offsetX) > abs(calib.ellipses.offsetY))
                    {
                        offsetInfo += (calib.ellipses.offsetX > 0) ? " (Camera LEFT)" : " (Camera RIGHT)";
                    }
                    else
                    {
                        offsetInfo += (calib.ellipses.offsetY > 0) ? " (Camera HIGH)" : " (Camera LOW)";
                    }
                }

                cv::putText(visFrame, offsetInfo, cv::Point(20, 160),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, offsetColor, 2);
            }

            // Ring detection status
            std::string ringStatus = "Rings: ";
            ringStatus += calib.ellipses.hasValidDoubles ? "Doubles (YES) " : "Doubles(NO) ";
            ringStatus += calib.ellipses.hasValidTriples ? "Triples(YES) " : "Triples(NO) ";
            ringStatus += calib.ellipses.hasValidBulls ? "Bulls(YES)" : "Bulls(NO)";

            cv::putText(visFrame, ringStatus, cv::Point(20, frame.rows - 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            // Simple legend in bottom-right corner
            int legendY = frame.rows - 100;
            cv::putText(visFrame, "Legend:", cv::Point(frame.cols - 180, legendY),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            // Doubles
            cv::line(visFrame, cv::Point(frame.cols - 170, legendY + 20), cv::Point(frame.cols - 150, legendY + 20), cv::Scalar(255, 255, 0), 3);
            cv::putText(visFrame, "Doubles", cv::Point(frame.cols - 145, legendY + 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            // Triples
            cv::line(visFrame, cv::Point(frame.cols - 170, legendY + 40), cv::Point(frame.cols - 150, legendY + 40), cv::Scalar(0, 255, 0), 3);
            cv::putText(visFrame, "Triples", cv::Point(frame.cols - 145, legendY + 45),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            // Bulls
            cv::line(visFrame, cv::Point(frame.cols - 170, legendY + 60), cv::Point(frame.cols - 150, legendY + 60), cv::Scalar(0, 0, 255), 3);
            cv::putText(visFrame, "Bulls", cv::Point(frame.cols - 145, legendY + 65),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

            // Wires
            cv::line(visFrame, cv::Point(frame.cols - 170, legendY + 80), cv::Point(frame.cols - 150, legendY + 80), cv::Scalar(0, 255, 255), 2);
            cv::putText(visFrame, "Wires", cv::Point(frame.cols - 145, legendY + 85),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        }

        return visFrame;
    }

} // namespace dartboard_visualization