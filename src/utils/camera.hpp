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
            log_info("Opening camera " + log_string(i + 1) + ": " + camera_sources[i]);

            if (isVideoFile(camera_sources[i]))
            {
                log_debug("Detected video file: " + camera_sources[i]);
                cap.open(camera_sources[i]);

#ifdef DEBUG_SEEK_VIDEO
                int seek_seconds = 5;
                if (seek_seconds > 0 && cap.isOpened())
                {
                    double video_fps = cap.get(CAP_PROP_FPS);
                    if (video_fps > 0)
                    {
                        int target_frame = static_cast<int>(video_fps * seek_seconds);
                        cap.set(CAP_PROP_POS_FRAMES, target_frame);
                        log_info("Seeked video " + log_string(i + 1) + " forward by " + log_string(seek_seconds) + " seconds (frame " + log_string(target_frame) + ")");
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
            }

            if (!cap.isOpened())
            {
                log_error("Failed to open camera/video " + camera_sources[i]);
                return false;
            }

            cap.set(CAP_PROP_FRAME_WIDTH, width);
            cap.set(CAP_PROP_FRAME_HEIGHT, height);
            cap.set(CAP_PROP_FPS, fps);

            cameras.push_back(cap);
            log_info("Camera/video " + log_string(i + 1) + " initialized successfully");
        }

        return cameras.size() > 0;
    }

    // Capture frames from all cameras with resizing if needed
    inline vector<Mat> captureFrames(vector<VideoCapture> &cameras, int target_width, int target_height)
    {
        vector<Mat> frames;

        for (size_t i = 0; i < cameras.size(); i++)
        {
            Mat frame;
            bool success = cameras[i].read(frame);

            if (!success || frame.empty())
            {
                log_error("Failed to capture frame from camera " + log_string(i + 1));
                continue;
            }

            if (!frame.empty() && (frame.cols != target_width || frame.rows != target_height))
            {
                Mat resized;
                resize(frame, resized, Size(target_width, target_height));
                frames.push_back(resized);
            }
            else
            {
                frames.push_back(frame);
            }
        }

        return frames;
    }

    // Capture and average multiple frames for better quality
    inline vector<Mat> captureAndAverageFrames(vector<VideoCapture> &cameras, int target_width, int target_height, int fps, int numFrames)
    {
        vector<Mat> averagedFrames;
        vector<vector<Mat>> allFrameSets;

        for (int i = 0; i < numFrames; i++)
        {
            vector<Mat> frameSet = captureFrames(cameras, target_width, target_height);
            if (!frameSet.empty())
            {
                allFrameSets.push_back(frameSet);
            }

            int frameIntervalMs = 1000 / fps;
            this_thread::sleep_for(chrono::milliseconds(frameIntervalMs));
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
