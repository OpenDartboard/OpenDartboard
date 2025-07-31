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

    // Simple function to decote fourcc code to a human-readable string
    inline string decodeFourCC(int fourcc)
    {
        char code[5];
        code[0] = (fourcc & 0xFF);
        code[1] = (fourcc >> 8) & 0xFF;
        code[2] = (fourcc >> 16) & 0xFF;
        code[3] = (fourcc >> 24) & 0xFF;
        code[4] = '\0';
        return string(code);
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
                cap.open(camera_sources[i], CAP_V4L2);
                cap.set(CAP_PROP_FRAME_WIDTH, width);
                cap.set(CAP_PROP_FRAME_HEIGHT, height);
                cap.set(CAP_PROP_FPS, fps);
                // Set MJPEG codec for better performance
                int fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G'); // MJPEG codec
                cap.set(CAP_PROP_FOURCC, fourcc);                     // Set MJPEG codec

                log_debug("Opened camera " + log_string(i + 1) + " at " + log_string(width) + "x" + log_string(height) + " @ " + log_string(fps) + " FPS" +
                          " (FOURCC: " + log_string_src(decodeFourCC(fourcc)) + ")");
            }

            if (!cap.isOpened())
            {
                log_error("Failed to open camera/video " + camera_sources[i]);
                return false;
            }

            // Verify camera properties after opening
            if (!isVideoFile(camera_sources[i]))
            {
                double actual_width = cap.get(CAP_PROP_FRAME_WIDTH);
                double actual_height = cap.get(CAP_PROP_FRAME_HEIGHT);
                double actual_fps = cap.get(CAP_PROP_FPS);
                double fourcc = cap.get(CAP_PROP_FOURCC);

                log_debug("Camera " + log_string(i + 1) + " verification:");
                log_debug("  Resolution: " + log_string((int)actual_width) + "x" + log_string((int)actual_height) + " (expected: " + log_string(width) + "x" + log_string(height) + ")");
                log_debug("  FPS: " + log_string((int)actual_fps) + " (expected: " + log_string(fps) + ")");
                log_debug("  FOURCC: " + log_string_src(decodeFourCC(fourcc)) + " (expected: " + log_string_src(decodeFourCC(VideoWriter::fourcc('M', 'J', 'P', 'G'))) + ")");
                log_debug("  Backend: " + log_string_src(cap.getBackendName()) + " (expected: " + log_string_src((string) "V4L2") + ")");
            }

            cameras.push_back(cap);
            log_info("Camera/video " + log_string(i + 1) + " initialized successfully");
        }

        return cameras.size() > 0;
    }

    // Capture frames from all cameras
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

}