#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

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
            std::cout << "      " << (i + 1) << ": " << cams[i] << "\n";
        }
        std::cout << "-------------------------------------\n";
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
        std::cout << "  --model <path>     Path to the model file (default: /usr/local/share/opendartboard/models/dart.param)\n";
        std::cout << "  --cams <cameras>   Comma-separated list of camera devices (default: /dev/video0,/dev/video1,/dev/video2)\n";
        std::cout << "  --width <width>    Frame width (default: 640)\n";
        std::cout << "  --height <height>  Frame height (default: 360)\n";
        std::cout << "  --fps <fps>        Frames per second (default: 15)\n";
        std::cout << "  --debug, -d        Enable debug mode (saves frames to debug_frames/ directory)\n";
        std::cout << "  --version          Show version information\n";
        std::cout << "  --help             Show this help message\n";
        exit(0);
    }

    // Save frames to disk with subdirectory option - SIMPLIFIED to reduce disk I/O
    inline void saveFrames(const std::vector<cv::Mat> &frames, const std::string &subdir = "", int fps = 15)
    {
        // Only save frames if we're in an important directory
        if (subdir != "calibration" && subdir != "multicam")
        {
            return;
        }

        // Create base debug directory
        static std::map<std::string, bool> dirs_created;
        std::string dir_path = "debug_frames";

        // Create subdirectory if specified
        if (!subdir.empty())
        {
            dir_path += "/" + subdir;
        }

        // Create directory if not already done
        if (!dirs_created[dir_path])
        {
            system(("mkdir -p " + dir_path).c_str());
            dirs_created[dir_path] = true;
            std::cout << "Debug mode enabled - saving frames to " << dir_path << "/ directory (1 FPS)" << std::endl;
        }

        // Track time to save at ~1 FPS regardless of actual camera FPS
        static std::map<std::string, std::chrono::steady_clock::time_point> last_saves;
        auto now = std::chrono::steady_clock::now();

        if (last_saves.find(subdir) == last_saves.end())
        {
            last_saves[subdir] = now - std::chrono::seconds(2); // Initialize to save first frame
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_saves[subdir])
                           .count();

        // Save even less frequently (every 2 seconds) to reduce disk I/O
        if (elapsed < 2000)
        {
            return;
        }

        // Reset timer
        last_saves[subdir] = now;

        // Save each frame
        static std::map<std::string, int> frame_counts;
        for (size_t i = 0; i < frames.size(); i++)
        {
            std::string filename = dir_path + "/cam" + std::to_string(i + 1) +
                                   "_frame" + std::to_string(frame_counts[subdir]) + ".jpg";
            cv::imwrite(filename, frames[i]);
        }

        // Increment frame counter, reset after 10 to avoid too many files
        frame_counts[subdir] = (frame_counts[subdir] + 1) % 10;
    }

    // Convert a frame to grayscale
    inline cv::Mat convertToGrayscale(const cv::Mat &frame)
    {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }

    // Apply threshold to motion difference between two frames
    inline cv::Mat applyMotionThreshold(const cv::Mat &frame1, const cv::Mat &frame2)
    {
        // Check if frames have the same size and type
        if (frame1.size() != frame2.size() || frame1.type() != frame2.type())
        {
            std::cerr << "Frame size or type mismatch: "
                      << frame1.size() << " vs " << frame2.size()
                      << " (types: " << frame1.type() << " vs " << frame2.type() << ")" << std::endl;

            // Return an empty matrix in case of mismatch
            return cv::Mat();
        }

        // Calculate absolute difference
        cv::Mat diff;
        cv::absdiff(frame1, frame2, diff);

        // Apply threshold
        cv::Mat thresh;
        cv::threshold(diff, thresh, 25, 255, cv::THRESH_BINARY);

        // Apply morphological operations to reduce noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE, kernel);

        return thresh;
    }

} // namespace debug
