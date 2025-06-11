#include "geometry_detector.hpp"
#include "geometry_dartboard.hpp"
#include <iostream>

using namespace cv;
using namespace std;

// IMPROVED: More robust color-based dartboard detection that starts from the center
bool GeometryDetector::findDartboardCircle(const cv::Mat &frame, cv::Point &center, double &radius, int camera_idx)
{
    // Add debug logging with camera index
    std::cout << "DEBUG-FIND: Starting circle detection for camera " << camera_idx + 1 << std::endl;

    // We need a color image for this approach
    if (frame.empty())
        return false;

    // Must be a color image
    cv::Mat colorFrame;
    if (frame.channels() == 3)
        colorFrame = frame.clone();
    else
    {
        cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
        std::cerr << "Warning: Using grayscale image for color-based detection" << std::endl;
    }

    // Create debug visualization
    cv::Mat debugVis;
    if (debug_mode)
        debugVis = colorFrame.clone();

    // STEP 1: Start with the assumption that bull's eye is near center of frame
    cv::Point frameCenter(frame.cols / 2, frame.rows / 2);

    // Define initial search area around center (use 40% of frame dimension)
    int searchRadius = std::min(frame.cols, frame.rows) * 0.4;
    cv::Rect centerRegion(
        std::max(0, frameCenter.x - searchRadius),
        std::max(0, frameCenter.y - searchRadius),
        std::min(frame.cols - 1, searchRadius * 2),
        std::min(frame.rows - 1, searchRadius * 2));

    // Convert to HSV for better color segmentation
    cv::Mat hsvFrame;
    cv::cvtColor(colorFrame, hsvFrame, cv::COLOR_BGR2HSV);

    // STEP 2: Create improved color masks with more precise thresholds
    cv::Mat redMask1, redMask2, redMask, greenMask;

    // Red comes in two ranges in HSV space
    cv::inRange(hsvFrame, cv::Scalar(0, 100, 70), cv::Scalar(8, 255, 255), redMask1);
    cv::inRange(hsvFrame, cv::Scalar(172, 100, 70), cv::Scalar(180, 255, 255), redMask2);
    redMask = redMask1 | redMask2;

    // Green ring detection
    cv::inRange(hsvFrame, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), greenMask);

    // Combined color mask
    cv::Mat colorMask = redMask | greenMask;

    // Apply morphology to clean noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_CLOSE, kernel);

    if (debug_mode)
    {
        // Create visualization showing red and green parts separately
        cv::Mat colorMaskVis = cv::Mat::zeros(colorFrame.size(), CV_8UC3);

        for (int y = 0; y < colorFrame.rows; y++)
        {
            for (int x = 0; x < colorFrame.cols; x++)
            {
                if (redMask.at<uchar>(y, x) > 0)
                    colorMaskVis.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red
                else if (greenMask.at<uchar>(y, x) > 0)
                    colorMaskVis.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // Green
            }
        }

        // Draw frame center and search region
        cv::circle(colorMaskVis, frameCenter, 10, cv::Scalar(255, 255, 255), 2);
        cv::rectangle(colorMaskVis, centerRegion, cv::Scalar(255, 255, 255), 2);

        cv::imwrite("debug_frames/circles/color_mask.jpg", colorMaskVis);
    }

    // STEP 3: FIRST look for the bull's eye (red circle) near center of frame
    cv::Mat centerRedMask = redMask(centerRegion);

    // Find contours in the center region red mask
    std::vector<std::vector<cv::Point>> bullContours;
    cv::findContours(centerRedMask, bullContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Filter bull contours by size and circularity
    cv::Point bullCenter(-1, -1);
    double bullRadius = 0;
    double bestBullScore = 0;

    for (const auto &contour : bullContours)
    {
        double area = cv::contourArea(contour);
        if (area < 20)
            continue; // Skip tiny contours

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        // Convert back to absolute coordinates
        center.x += centerRegion.x;
        center.y += centerRegion.y;

        // Score based on circularity and proximity to frame center
        double circularity = area / (CV_PI * radius * radius);
        double centerDistance = cv::norm(cv::Point2f(frameCenter) - center);
        double centralityScore = 1.0 - (centerDistance / (searchRadius * 1.5));
        double score = circularity * 0.6 + centralityScore * 0.4;

        if (score > bestBullScore)
        {
            bestBullScore = score;
            bullCenter = cv::Point(center.x, center.y);
            bullRadius = radius;
        }
    }

    // If we found a good bull's eye, use it as our center
    bool foundBull = (bullCenter.x > 0 && bestBullScore > 0.5);
    if (foundBull)
    {
        center = bullCenter;

        if (debug_mode)
        {
            cv::circle(debugVis, bullCenter, bullRadius, cv::Scalar(255, 255, 0), 2);
            cv::putText(debugVis, "BULL", bullCenter + cv::Point(-20, -20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        }
    }
    else
    {
        // Fall back to frame center if bull not detected
        center = frameCenter;
    }

    // Add debug logging at key points
    std::cout << "DEBUG-FIND: Starting circle detection" << std::endl;

    // STEP 4: Now look for the dartboard outer circle using the full color mask
    // Look for contours in wider area around detected bull center
    int outerSearchRadius = std::min(frame.cols, frame.rows) * 0.6;
    cv::Rect dartboardRegion(
        std::max(0, center.x - outerSearchRadius),
        std::max(0, center.y - outerSearchRadius),
        std::min(frame.cols - 1, outerSearchRadius * 2),
        std::min(frame.rows - 1, outerSearchRadius * 2));

    // We need to adjust the mask ROI
    cv::Mat dartboardMask;
    if (dartboardRegion.x >= 0 && dartboardRegion.y >= 0 &&
        dartboardRegion.x + dartboardRegion.width <= colorMask.cols &&
        dartboardRegion.y + dartboardRegion.height <= colorMask.rows)
    {

        dartboardMask = colorMask(dartboardRegion);
    }
    else
    {
        dartboardMask = colorMask;
    }

    std::vector<std::vector<cv::Point>> dartboardContours;
    cv::findContours(dartboardMask, dartboardContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Use circle fitting to find the outer ring
    radius = 0;
    float bestCircleScore = 0;

    for (const auto &contour : dartboardContours)
    {
        // Convert contour points to absolute coordinates
        std::vector<cv::Point> absContour;
        for (const auto &p : contour)
        {
            absContour.push_back(cv::Point(p.x + dartboardRegion.x, p.y + dartboardRegion.y));
        }

        // Skip small contours
        if (absContour.size() < 10)
            continue;

        // Try to fit a circle to this contour
        cv::Point2f contourCenter;
        float contourRadius;
        cv::minEnclosingCircle(absContour, contourCenter, contourRadius);

        // Calculate distance from bull center to this contour center
        double centerDistance = cv::norm(cv::Point2f(center) - contourCenter);

        // Score: prioritize circles close to our detected bull center
        double centralityScore = 1.0 - (centerDistance / (outerSearchRadius / 2.0));
        centralityScore = std::max(0.0, std::min(1.0, centralityScore));

        // Score: prioritize larger circles (more likely to be the dartboard)
        double sizeScore = contourRadius / (outerSearchRadius / 2.0);
        sizeScore = std::min(1.0, sizeScore);

        // Calculate overall score
        double score = centralityScore * 0.7 + sizeScore * 0.3;

        if (score > bestCircleScore && contourRadius > 20)
        {
            bestCircleScore = score;
            radius = contourRadius;
            // Keep the bull center we found, just update radius
        }
    }

    // If we couldn't find a good radius from the color detection
    if (radius <= 20)
    {
        // Estimate radius from image dimensions
        radius = std::min(frame.cols, frame.rows) / 3.0;
    }

    // STEP 5: Create debug visualization
    if (debug_mode)
    {
        // Create a temporary calibration object for visualization
        DartboardCalibration tempCalib;
        tempCalib.center = center;
        tempCalib.radius = radius;
        tempCalib.orientation = 270.0; // Fixed - segment 20 at top
        tempCalib.camera_index = camera_idx;

        // Set standard ring proportions
        tempCalib.bullRadius = radius * 0.07;
        tempCalib.doubleRingInner = radius * 0.92;
        tempCalib.doubleRingOuter = radius;
        tempCalib.tripleRingInner = radius * 0.55;
        tempCalib.tripleRingOuter = radius * 0.63;

        // Create debug view and scale calibration using shared utility method
        cv::Mat debugVis = cv::Mat(target_height, target_width, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Point offset;

        // Scale calibration for display - use our utility method
        DartboardCalibration displayCalib = scaleCalibrationForDisplay(
            tempCalib, colorFrame, target_width, target_height, offset);

        // Resize the frame and place it on the background
        cv::Mat properlyResized;
        int targetWidth = target_width - 2 * offset.x;
        int targetHeight = target_height - 2 * offset.y;
        cv::resize(colorFrame, properlyResized, cv::Size(targetWidth, targetHeight));
        cv::Mat roi = debugVis(cv::Rect(offset.x, offset.y, targetWidth, targetHeight));
        properlyResized.copyTo(roi);

        // Use the renderer with our scaled calibration
        GeometryDartboard::drawCalibrationOverlay(debugVis, displayCalib, true);

        // Add camera identifier
        cv::rectangle(debugVis, cv::Point(target_width - 160, 10), cv::Point(target_width - 10, 40),
                      cv::Scalar(0, 0, 0), -1);
        cv::putText(debugVis, "Camera " + std::to_string(camera_idx + 1) + " Overlay",
                    cv::Point(target_width - 150, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 255), 1);

        // Save debug image
        std::string dir = "debug_frames/circles";
        system(("mkdir -p " + dir).c_str());
        cv::imwrite(dir + "/camera" + std::to_string(camera_idx + 1) + "_overlay.jpg", debugVis);
    }

    // Final coordinates
    std::cout << "DEBUG-FIND: Final center for camera " << camera_idx + 1 << "=("
              << center.x << "," << center.y << "), radius=" << radius << std::endl;

    return true;
}

// SIMPLIFIED: Calibrate dartboard - just detect circle and use fixed orientation
bool GeometryDetector::calibrateDartboard(const std::vector<cv::Mat> &frames)
{
    calibrations.clear();
    std::cout << "\n===== DARTBOARD CALIBRATION STARTED =====\n";

    if (frames.empty())
    {
        std::cerr << "No frames provided for calibration" << std::endl;
        return false;
    }

    for (size_t cam_idx = 0; cam_idx < frames.size(); cam_idx++)
    {
        std::cout << "Calibrating camera " << cam_idx + 1 << "..." << std::endl;

        if (frames[cam_idx].empty())
        {
            std::cerr << "Empty frame from camera " << cam_idx + 1 << std::endl;
            continue;
        }

        // CHANGED: Skip preprocessing - use the original color frame directly
        // This preserves color information for finding red/green rings
        cv::Point center;
        double radius;
        bool found = findDartboardCircle(frames[cam_idx], center, radius, cam_idx);

        // Debug logging for calibration
        std::cout << "DEBUG-CALIB: Processing camera " << cam_idx + 1 << std::endl;

        // CHANGED: Lower minimum radius threshold from 40 to 25 to accommodate smaller dartboards
        if (found && radius > 25) // Was: radius > 40
        {
            // Always use fixed orientation - standard dartboard
            DartboardCalibration calib;
            calib.center = center;
            calib.radius = radius;
            calib.orientation = 270.0;    // Fixed - segment 20 at top
            calib.camera_index = cam_idx; // CRITICAL: This stores the actual camera index

            // Set standard ring proportions
            calib.bullRadius = radius * 0.07;
            calib.doubleRingInner = radius * 0.92;
            calib.doubleRingOuter = radius;
            calib.tripleRingInner = radius * 0.55;
            calib.tripleRingOuter = radius * 0.63;

            // Log before adding to calibrations
            std::cout << "DEBUG-CALIB: Storing camera " << cam_idx + 1
                      << " center=(" << center.x << "," << center.y << ")" << std::endl;

            calibrations.push_back(calib);
            std::cout << "Camera " << cam_idx + 1 << " calibrated: center=("
                      << center.x << "," << center.y << "), radius=" << radius << std::endl;
        }
        else
        {
            std::cout << "Failed to calibrate camera " << cam_idx + 1 << std::endl;
        }
    }

    // Before saving multi-camera view, log all calibrations
    if (!calibrations.empty())
    {
        std::cout << "DEBUG-CALIB: Final calibration summary:" << std::endl;
        for (size_t i = 0; i < calibrations.size(); i++)
        {
            std::cout << "DEBUG-CALIB:   Camera " << i + 1
                      << " center=(" << calibrations[i].center.x << "," << calibrations[i].center.y
                      << "), radius=" << calibrations[i].radius << std::endl;
        }
    }

    // Show multi-camera view with calibration overlay
    if (debug_mode && !calibrations.empty())
    {
        saveMultiCameraDebugView(frames, {});
    }

    std::cout << "===== DARTBOARD CALIBRATION COMPLETED =====\n";
    return !calibrations.empty();
}

// Simple method to extract color profile - needed for other methods
std::vector<cv::Vec3b> GeometryDetector::extractColorProfile(const cv::Mat &frame,
                                                             const cv::Point &center,
                                                             double radius,
                                                             int samples)
{
    std::vector<cv::Vec3b> profile;
    if (frame.empty() || frame.channels() < 3 || radius <= 0)
        return profile;

    for (int i = 0; i < samples; i++)
    {
        double angle = i * 2.0 * CV_PI / samples;
        int x = center.x + radius * cos(angle);
        int y = center.y + radius * sin(angle);

        if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows)
        {
            profile.push_back(frame.at<cv::Vec3b>(y, x));
        }
        else
        {
            profile.push_back(cv::Vec3b(0, 0, 0));
        }
    }

    return profile;
}

// Helper method for color comparison
double GeometryDetector::colorDifference(const cv::Vec3b &color1, const cv::Vec3b &color2)
{
    return std::sqrt(
        std::pow(color1[0] - color2[0], 2) +
        std::pow(color1[1] - color2[1], 2) +
        std::pow(color1[2] - color2[2], 2));
}
