#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <thread>
#include <chrono>
#include "utils.hpp"

using namespace cv;
using namespace std;

namespace camera
{
    // Determine if a given path is a video file based on its extension
    inline bool isVideoFile(const string &path)
    {
        string lower_path = path;
        transform(lower_path.begin(), lower_path.end(), lower_path.begin(), ::tolower);

        const vector<string> video_extensions = {".mp4", ".avi", ".mkv", ".mov", ".wmv"};
        for (const auto &ext : video_extensions)
        {
            if (lower_path.length() >= ext.length() &&
                lower_path.substr(lower_path.length() - ext.length()) == ext)
            {
                return true;
            }
        }
        return false;
    }

    // Initialize cameras based on provided sources
    inline bool initializeCameras(vector<VideoCapture> &cameras, const vector<string> &camera_sources, int width, int height, int fps)
    {
        cameras.clear();
        log_info("Initializing " + log_string(camera_sources.size()) + " cameras...");

        for (size_t i = 0; i < camera_sources.size(); i++)
        {
            VideoCapture cap;
            log_debug("Opening camera " + log_string(i + 1) + ": " + camera_sources[i]);

            if (isVideoFile(camera_sources[i]))
            {
                log_debug("Detected video file: " + camera_sources[i]);
                cap.open(camera_sources[i]);

#ifdef DEBUG_SEEK_VIDEO
                double seek_seconds = 3 - (i * 0.18); // Example: seek 4 seconds for first video, 3 for second, etc.
                if (seek_seconds > 0 && cap.isOpened())
                {
                    double video_fps = cap.get(CAP_PROP_FPS);
                    if (video_fps > 0)
                    {
                        int target_frame = static_cast<int>(video_fps * seek_seconds);
                        cap.set(CAP_PROP_POS_FRAMES, target_frame);
                        log_debug("Seeked video " + log_string(i + 1) + " forward by " + log_string(seek_seconds) + " seconds (frame " + log_string(target_frame) + ")");
                    }
                    else
                    {
                        log_warning("Could not determine FPS for video " + camera_sources[i] + ", skipping seek");
                    }
                }
#endif
            }
            else
            {
                cap.open(camera_sources[i]);
                cap.set(CAP_PROP_FRAME_WIDTH, width);
                cap.set(CAP_PROP_FRAME_HEIGHT, height);
                cap.set(CAP_PROP_FPS, fps);
            }

            if (!cap.isOpened())
            {
                log_error("Failed to open camera/video " + camera_sources[i]);
                return false;
            }

            cameras.push_back(cap);
            log_info("Camera/video " + log_string(i + 1) + " initialized successfully");
        }

        return cameras.size() > 0;
    }

    // Capture frames from all cameras with resizing if needed
    inline vector<Mat> captureFrames(vector<VideoCapture> &cameras)
    {
        vector<Mat> frames;
        frames.reserve(cameras.size());

        for (size_t i = 0; i < cameras.size(); i++)
        {
            Mat frame;
            bool success = cameras[i].read(frame);

            if (success && !frame.empty())
            {
                // capture frame
                frames.push_back(frame);
            }
            else
            {
                log_error("Failed to capture frame from camera " + log_string(i + 1));
                continue;
            }
        }

#ifdef DEBUG_VIA_VIDEO_INPUT
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / 60)));
#endif

        return frames;
    }

    // Capture and average multiple frames for better quality
    inline vector<Mat> captureAndAverageFrames(vector<VideoCapture> &cameras, int numFrames)
    {
        vector<Mat> averagedFrames;
        vector<vector<Mat>> allFrameSets;

        for (int i = 0; i < numFrames; i++)
        {
            vector<Mat> frameSet = captureFrames(cameras);
            if (!frameSet.empty())
            {
                allFrameSets.push_back(frameSet);
            }
        }

        if (allFrameSets.empty())
            return vector<Mat>();

        // Average frames for each camera
        for (size_t cam = 0; cam < allFrameSets[0].size(); cam++)
        {
            if (allFrameSets.size() == 1)
            {
                averagedFrames.push_back(allFrameSets[0][cam]);
                continue;
            }

            Mat averaged;
            allFrameSets[0][cam].convertTo(averaged, CV_32F);

            for (size_t frameSet = 1; frameSet < allFrameSets.size(); frameSet++)
            {
                if (cam < allFrameSets[frameSet].size())
                {
                    Mat temp;
                    allFrameSets[frameSet][cam].convertTo(temp, CV_32F);
                    averaged += temp;
                }
            }

            averaged /= static_cast<float>(allFrameSets.size());

            Mat result;
            averaged.convertTo(result, CV_8U);
            averagedFrames.push_back(result);
        }

        return averagedFrames;
    }

    // Helper function for motion detection - convert frame to grayscale
    inline Mat convertToGrayscale(const Mat &frame)
    {
        Mat gray;
        if (frame.channels() == 3)
        {
            cvtColor(frame, gray, COLOR_BGR2GRAY);
        }
        else
        {
            gray = frame.clone();
        }
        return gray;
    }

    // Helper function for motion detection - apply threshold and morphology
    inline Mat applyMotionThreshold(const Mat &frame1, const Mat &frame2)
    {
        // Check if frames have the same size and type
        if (frame1.size() != frame2.size() || frame1.type() != frame2.type())
        {
            log_error("Frame size or type mismatch in motion detection: " + to_string(frame1.cols) + "x" + to_string(frame1.rows) + " vs " + to_string(frame2.cols) + "x" + to_string(frame2.rows));
            return Mat(); // Return empty matrix
        }

        // Calculate absolute difference
        Mat diff;
        absdiff(frame1, frame2, diff);

        // Apply threshold
        Mat thresh;
        threshold(diff, thresh, 25, 255, THRESH_BINARY);

        // Apply morphological operations to reduce noise
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(thresh, thresh, MORPH_CLOSE, kernel);

        return thresh;
    }

    // Detect motion between current and previous frames using proper threshold calculation
    inline bool detectMotion(const vector<Mat> &current_frames, const vector<Mat> &previous_frames, double threshold_ratio = 0.05)
    {
        if (current_frames.size() != previous_frames.size() || current_frames.empty())
        {
            return false;
        }

        for (size_t i = 0; i < current_frames.size(); i++)
        {
            if (current_frames[i].empty() || previous_frames[i].empty())
            {
                continue;
            }

            // Convert to grayscale
            Mat prev_gray = convertToGrayscale(previous_frames[i]);
            Mat curr_gray = convertToGrayscale(current_frames[i]);

            // Get motion mask
            Mat motion_mask = applyMotionThreshold(prev_gray, curr_gray);

            // Skip if motion_mask is empty (error occurred)
            if (motion_mask.empty())
            {
                continue;
            }

            // Count motion pixels and calculate ratio
            int motion_pixels = countNonZero(motion_mask);
            int total_pixels = motion_mask.rows * motion_mask.cols;
            double motion_ratio = (double)motion_pixels / total_pixels;

            if (motion_ratio > threshold_ratio)
            {
                return true; // Motion detected on this camera
            }
        }

        return false; // No motion detected on any camera
    }
}
