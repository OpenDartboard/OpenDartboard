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

        // Step 1: Create blur mask
        Mat blurredFrame;
        GaussianBlur(redGreenFrame, blurredFrame, Size(7, 7), 2.0);

        // Step 2: Convert to black/white - anything not black becomes white
        Mat grayMask;
        cvtColor(blurredFrame, grayMask, COLOR_BGR2GRAY);
        Mat binaryMask;
        threshold(grayMask, binaryMask, 1, 255, THRESH_BINARY); // Any non-zero pixel becomes white

        // Debug: Save masks
        if (debug_mode)
        {
            system("mkdir -p debug_frames/bull_processing");
            imwrite("debug_frames/bull_processing/blurred_frame_" + to_string(camera_idx) + ".jpg", blurredFrame);
            imwrite("debug_frames/bull_processing/binary_mask_" + to_string(camera_idx) + ".jpg", binaryMask);
        }

        // Step 3: Find contours
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(binaryMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        log_debug("Found " + log_string(contours.size()) + " contours total");

        if (contours.empty())
        {
            log_warning("No contours found");
            return frameCenter;
        }

        // Step 4: Analyze all contours with detailed info
        Point bestBullCenter = frameCenter;
        double bestScore = 0;
        int bestContourIndex = -1;

        for (size_t i = 0; i < contours.size(); i++)
        {
            const auto &contour = contours[i];
            double area = contourArea(contour);

            // Skip tiny contours - increased threshold
            if (area < 200)
                continue;

            // Calculate circularity
            double perimeter = arcLength(contour, true);
            if (perimeter == 0)
                continue;

            double circularity = (4 * CV_PI * area) / (perimeter * perimeter);

            // Get contour center
            Point2f center2f;
            float radius;
            minEnclosingCircle(contour, center2f, radius);
            Point center(center2f);

            // Check if this is inner or outer contour
            bool isInner = (hierarchy[i][2] != -1); // Has children (inner contour)
            bool isOuter = (hierarchy[i][3] == -1); // No parent (outer contour)

            // Simple scoring for bull detection
            double score = circularity * 0.8 + (1.0 / (1.0 + area / 200.0)) * 0.2;

            log_debug("Contour " + log_string(i) +
                      ": area=" + log_string((int)area) +
                      ", circ=" + log_string(circularity).substr(0, 4) +
                      ", center=(" + log_string(center.x) + "," + log_string(center.y) + ")" +
                      ", inner=" + log_string(isInner) +
                      ", outer=" + log_string(isOuter) +
                      ", score=" + log_string(score).substr(0, 4));

            if (score > bestScore && circularity > 0.3)
            {
                bestScore = score;
                bestBullCenter = center;
                bestContourIndex = i;
            }
        }

        if (bestContourIndex == -1)
        {
            log_warning("No good contours found, using frame center");
            bestBullCenter = frameCenter;
        }

        log_info("Bull detection complete - center=(" + log_string(bestBullCenter.x) +
                 "," + log_string(bestBullCenter.y) + "), score=" + log_string(bestScore));

        // Enhanced debug visualization
        if (debug_mode)
        {
            Mat bullDebug = redGreenFrame.clone();

            // Dim the background to 20% to make outlines pop
            bullDebug = bullDebug * 0.2;

            // Collect contour info for top display
            vector<string> contourInfo;
            vector<Scalar> contourColors;
            int validContourCount = 0;

            // Draw ALL contours with new color scheme and collect info
            for (size_t i = 0; i < contours.size(); i++)
            {
                double area = contourArea(contours[i]);
                if (area < 200)
                    continue; // Skip tiny ones in visualization

                double perimeter = arcLength(contours[i], true);
                double circularity = (perimeter > 0) ? (4 * CV_PI * area) / (perimeter * perimeter) : 0;

                Point2f center2f;
                float radius;
                minEnclosingCircle(contours[i], center2f, radius);
                Point center(center2f);

                bool isInner = (hierarchy[i][2] != -1);
                bool isOuter = (hierarchy[i][3] == -1);

                // New color scheme - avoiding green/red
                Scalar color;
                string status;
                if (i == bestContourIndex)
                {
                    color = Scalar(255, 255, 0); // Bright CYAN for best
                    status = "BEST";
                }
                else if (isInner)
                {
                    color = Scalar(0, 255, 255); // YELLOW for inner
                    status = "INNER";
                }
                else if (isOuter)
                {
                    color = Scalar(255, 0, 255); // MAGENTA for outer
                    status = "OUTER";
                }
                else
                {
                    color = Scalar(255, 255, 255); // WHITE for poor
                    status = "POOR";
                }

                drawContours(bullDebug, contours, i, color, 3);

                // Find leftmost point of the contour
                Point leftmostPoint = contours[i][0];
                for (const Point &pt : contours[i])
                {
                    if (pt.x < leftmostPoint.x)
                    {
                        leftmostPoint = pt;
                    }
                }

                // Small number overlay at leftmost point (white text with black outline)
                string numberLabel = to_string(validContourCount);
                Point labelPos = leftmostPoint + Point(-15, 5); // Slightly left of leftmost point

                putText(bullDebug, numberLabel, labelPos,
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 3); // Black outline
                putText(bullDebug, numberLabel, labelPos,
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2); // White text

                // Collect info for top display with color
                string info = "Contour " + to_string(validContourCount) + ": A=" + to_string(int(area)) +
                              " C=" + to_string(circularity).substr(0, 4) + " (" + status + ")";
                contourInfo.push_back(info);
                contourColors.push_back(color);

                validContourCount++;
            }

            // Draw final bull center with bright white
            circle(bullDebug, bestBullCenter, 4, Scalar(255, 255, 255), -1);

            // Display contour info at top with colored backgrounds
            int yPos = 30;
            for (size_t i = 0; i < contourInfo.size(); i++)
            {
                const string &info = contourInfo[i];
                Scalar bgColor = contourColors[i];

                // Get text size for background rectangle
                Size textSize = getTextSize(info, FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);

                // Draw colored background (dimmed version of contour color)
                Scalar dimmedColor = bgColor * 0.3; // 30% of original color
                rectangle(bullDebug, Point(10, yPos - 20), Point(15 + textSize.width, yPos + 5),
                          dimmedColor, -1);

                // Draw text in white for readability
                putText(bullDebug, info, Point(15, yPos),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
                yPos += 25;
            }

            imwrite("debug_frames/bull_processing/bull_detection_" + to_string(camera_idx) + ".jpg", bullDebug);
        }

        return bestBullCenter;
    }

} // namespace bull_processing