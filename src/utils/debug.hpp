#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include "logging.hpp"

namespace debug
{

    // Print application startup banner
    inline void printStartup(const std::string &appName, const std::string &version)
    {
        std::cout << "=====================================\n";
        std::cout << "  " << appName << " v" << version << " starting...\n";
        std::cout << "=====================================\n";
    }

    // Print configuration details
    inline void printConfig(int width, int height, int fps,
                            const std::string &model,
                            const std::vector<std::string> &cams)
    {
        std::cout << "Configuration:\n";
        std::cout << "  - Resolution: " << width << "x" << height << "\n";
        std::cout << "  - FPS: " << fps << "\n";
        std::cout << "  - Model: " << model << "\n";
        std::cout << "  - Cameras (" << cams.size() << "):\n";
        for (size_t i = 0; i < cams.size(); i++)
        {
            std::cout << "      " + std::to_string(i + 1) + ": " + cams[i] << std::endl;
        }
        std::cout << "-------------------------------------" << std::endl;
    }

    // Print version information and exit
    inline void printVersionAndExit(const std::string &version)
    {
        std::cout << "OpenDartboard runtime version: " << version << std::endl;
        exit(0);
    }

    // Print help message and exit
    inline void printHelpAndExit()
    {
        std::cout << "Usage: opendartboard [options]\n";
        std::cout << "Options:\n";
        std::cout << "  --model <path>       Path to the AI model file (default: /usr/local/share/opendartboard/models/dart.param)\n";
        std::cout << "  --cams <cameras>     Comma-separated list of camera devices (default: /dev/video0,/dev/video1,/dev/video2)\n";
        std::cout << "  --width <width>      Frame width (default: 640)\n";
        std::cout << "  --height <height>    Frame height (default: 360)\n";
        std::cout << "  --fps <fps>          Frames per second (default: 15)\n";
        std::cout << "  --detector <type>    Detector type: geometry, ai, custom (default: geometry)\n";
        std::cout << "  --debug, -d          Enable debug mode (saves frames to debug_frames/ directory)\n";
        std::cout << "  --quiet, -q          Quiet mode (only show errors)\n";
        std::cout << "  --version            Show version information\n";
        std::cout << "  --help               Show this help message\n";
        exit(0);
    }

    // Create combined calibration visualization showing all processing steps
    inline cv::Mat createCombinedCalibrationVisualization(int numCameras, const std::string &baseDir = "debug_frames")
    {
        std::vector<std::string> stepNames = {
            "1. ROI Processing",
            "2. Color Processing",
            "3. Bull Processing",
            "4. Mask Processing",
            // temporarily disabled contour processing
            // "5. Contour Processing",
            "6. Ellipse Processing",
            "7. Wire Processing",
            "8. Orientation Processing",
            "9. Final Calibration"};

        std::vector<std::string> stepPaths = {
            "roi_processing/roi_frame_",
            "color_processing/red_green_frame_",
            "bull_processing/bull_detection_",
            "mask_processing/mask_grid_",
            // temporarily disabled contour processing
            // "contour_processing/contours_",
            "ellipse_processing/ellipse_result_",
            "wire_processing/ensemble_average_result_",
            "orientation_processing/orientation_result_",
            "geometry_calibration/calibration_camera_"};

        // Load first image to get dimensions
        cv::Mat firstImage = cv::imread(baseDir + "/" + stepPaths[0] + "0.jpg");
        if (firstImage.empty())
        {
            log_error("Could not load debug images for combined visualization");
            return cv::Mat::zeros(480, 640, CV_8UC3);
        }

        int imgWidth = firstImage.cols;
        int imgHeight = firstImage.rows;
        int labelHeight = 40;
        int padding = 10;

        // Calculate combined image dimensions
        int combinedWidth = (imgWidth + padding) * numCameras + padding;
        int combinedHeight = (imgHeight + labelHeight + padding) * stepNames.size() + labelHeight + padding;

        cv::Mat combinedImage = cv::Mat::zeros(combinedHeight, combinedWidth, CV_8UC3);
        combinedImage.setTo(cv::Scalar(50, 50, 50)); // Dark gray background

        // Add title
        cv::putText(combinedImage, "OpenDartboard Calibration Pipeline - All Cameras",
                    cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

        // Add camera headers
        for (int cam = 0; cam < numCameras; cam++)
        {
            int x = padding + cam * (imgWidth + padding);
            int y = labelHeight + 20;
            cv::putText(combinedImage, "Camera " + std::to_string(cam),
                        cv::Point(x + imgWidth / 2 - 50, y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        }

        // Process each step
        for (size_t step = 0; step < stepNames.size(); step++)
        {
            int stepY = labelHeight + padding + step * (imgHeight + labelHeight + padding);

            // Add step label
            cv::putText(combinedImage, stepNames[step],
                        cv::Point(20, stepY + labelHeight - 5), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            // Add images for each camera
            for (int cam = 0; cam < numCameras; cam++)
            {
                std::string imagePath = baseDir + "/" + stepPaths[step] + std::to_string(cam) + ".jpg";
                cv::Mat stepImage = cv::imread(imagePath);

                if (!stepImage.empty())
                {
                    int x = padding + cam * (imgWidth + padding);
                    int y = stepY + labelHeight;

                    // Ensure the image fits in the allocated space
                    if (stepImage.cols != imgWidth || stepImage.rows != imgHeight)
                    {
                        cv::resize(stepImage, stepImage, cv::Size(imgWidth, imgHeight));
                    }

                    cv::Rect roi(x, y, imgWidth, imgHeight);
                    if (roi.x + roi.width <= combinedImage.cols && roi.y + roi.height <= combinedImage.rows)
                    {
                        stepImage.copyTo(combinedImage(roi));
                    }
                }
                else
                {
                    // Draw placeholder for missing image
                    int x = padding + cam * (imgWidth + padding);
                    int y = stepY + labelHeight;
                    cv::Rect roi(x, y, imgWidth, imgHeight);
                    cv::rectangle(combinedImage, roi, cv::Scalar(100, 100, 100), -1);
                    cv::putText(combinedImage, "Missing Image",
                                cv::Point(x + 20, y + imgHeight / 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

                    log_warning("Missing image: " + imagePath);
                }
            }
        }

        return combinedImage;
    }

    // Helper function to create combined frame with label
    inline cv::Mat createCombinedFrame(const vector<Mat> &frames, const string &label)
    {
        if (frames.empty())
            return cv::Mat();

        int numCams = min(3, (int)frames.size());
        int frameWidth = frames[0].cols;
        int frameHeight = frames[0].rows;

        // Horizontal layout: [Cam0][Cam1][Cam2]
        Mat combined(frameHeight, frameWidth * numCams, frames[0].type());

        for (int i = 0; i < numCams; i++)
        {
            if (!frames[i].empty())
            {
                Rect roi(i * frameWidth, 0, frameWidth, frameHeight);
                frames[i].copyTo(combined(roi));

                // Add camera and stream labels
                putText(combined, "Camera " + to_string(i),
                        Point(i * frameWidth + 10, 30),
                        FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
                putText(combined, label,
                        Point(i * frameWidth + 10, 60),
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
            }
        }

        return combined;
    }

} // namespace debug
