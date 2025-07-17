#include "motion_processing.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

namespace motion_processing
{
    // Static storage for previous frames (maintains state between calls)
    static vector<Mat> previous_frames;
    static bool initialized = false;

    // Session management state
    static bool is_moving = false;
    static vector<bool> are_moving;
    static vector<bool> was_moving_in_session;
    static chrono::steady_clock::time_point motion_session_start_time;
    static chrono::steady_clock::time_point last_session_end_time;

    vector<bool> detectMotion(const vector<Mat> &current_frames, bool debug_mode, const MotionParams &params)
    {
        // Initialize previous frames on first run
        if (!initialized || previous_frames.size() != current_frames.size())
        {
            previous_frames.clear();

            // check if we have any frames to initialize
            if (current_frames.size() != 3)
            {
                log_warning("Motion processing initialized with " + to_string(current_frames.size()) + " cameras, but expected 3 cameras. Skipping initialization.");
                return vector<bool>(current_frames.size(), false);
            }

            for (const auto &frame : current_frames)
            {
                previous_frames.push_back(frame.clone());
            }

            initialized = true;
            log_debug("Motion processing initialized with " + to_string(current_frames.size()) + " cameras");

            // Return no motion on first frame
            return vector<bool>(current_frames.size(), false);
        }

        // Detect current motion using our processing
        vector<bool> current_motions(current_frames.size(), false);

        // Motion detection for each camera
        for (size_t i = 0; i < current_frames.size() && i < previous_frames.size(); i++)
        {
            if (current_frames[i].empty() || previous_frames[i].empty())
                continue;

            // Convert to grayscale
            Mat prev_gray, curr_gray;
            cvtColor(previous_frames[i], prev_gray, COLOR_BGR2GRAY);
            cvtColor(current_frames[i], curr_gray, COLOR_BGR2GRAY);

            // Calculate absolute difference
            Mat diff;
            absdiff(prev_gray, curr_gray, diff);

            // Apply threshold
            Mat thresh;
            threshold(diff, thresh, params.binary_threshold, 255, THRESH_BINARY);

            // Apply morphological operations to reduce noise
            Mat kernel = getStructuringElement(params.morph_type, Size(params.morph_kernel_size, params.morph_kernel_size));
            morphologyEx(thresh, thresh, MORPH_CLOSE, kernel);

            // Debug: Save motion detection images
            if (debug_mode)
            {
                system("mkdir -p debug_frames/motion_processing");
                imwrite("debug_frames/motion_processing/diff_cam_" + to_string(i) + ".jpg", diff);
                imwrite("debug_frames/motion_processing/thresh_cam_" + to_string(i) + ".jpg", thresh);
            }

            // Count motion pixels and calculate ratio
            int motion_pixels = countNonZero(thresh);
            int total_pixels = thresh.rows * thresh.cols;
            double motion_ratio = (double)motion_pixels / total_pixels;

            if (motion_ratio > params.threshold_ratio)
            {
                current_motions[i] = true;
            }
        }

        // Update previous frames for next iteration
        for (size_t i = 0; i < current_frames.size(); i++)
        {
            previous_frames[i] = current_frames[i].clone();
        }

        return current_motions;
    }

    MotionResult processMotion(const vector<Mat> &current_frames, bool debug_mode, const MotionParams &params)
    {
        MotionResult result;

        // Initialize session state vectors if needed
        if (are_moving.size() != current_frames.size())
        {
            are_moving.resize(current_frames.size(), false);
            was_moving_in_session.resize(current_frames.size(), false);
        }

        // Get motion detection results
        vector<bool> motions = detectMotion(current_frames, debug_mode, params);
        auto now = chrono::steady_clock::now();

        // Check if any camera detects motion this frame
        bool any_motion_this_frame = count(motions.begin(), motions.end(), true) > 0;

        // Check if we're in cooldown period
        auto cooldown_elapsed = chrono::duration_cast<chrono::milliseconds>(now - last_session_end_time).count();
        bool in_cooldown = cooldown_elapsed < params.cooldown_period_ms;

        if (any_motion_this_frame && !is_moving && !in_cooldown)
        {
            // Start new motion session
            is_moving = true;
            motion_session_start_time = now;
            fill(are_moving.begin(), are_moving.end(), false);
            fill(was_moving_in_session.begin(), was_moving_in_session.end(), false);
            log_debug("Motion session started");
        }
        else if (any_motion_this_frame && !is_moving && in_cooldown)
        {
            if (debug_mode)
            {
                cout << "DEBUG: Motion detected but in cooldown period (" << cooldown_elapsed << "ms/" << params.cooldown_period_ms << "ms)" << endl;
            }
        }

        if (is_moving)
        {
            // Update camera states during active session
            for (size_t i = 0; i < motions.size(); i++)
            {
                if (motions[i])
                {
                    are_moving[i] = true;
                    was_moving_in_session[i] = true;
                }
                else
                {
                    are_moving[i] = false;
                }
            }

            // Check session end conditions
            auto session_duration = chrono::duration_cast<chrono::milliseconds>(now - motion_session_start_time).count();
            int any_left_moving = count(are_moving.begin(), are_moving.end(), true);
            int cameras_that_moved = count(was_moving_in_session.begin(), was_moving_in_session.end(), true);
            int total_cameras = motions.size();

            // Debug output
            if (debug_mode)
            {
                cout << "DEBUG: duration=" << session_duration << "ms";
                cout << " | Currently moving: ";
                for (size_t i = 0; i < are_moving.size(); i++)
                {
                    if (are_moving[i])
                        cout << "cam" << i << " ";
                }
                cout << " | Participated: ";
                for (size_t i = 0; i < was_moving_in_session.size(); i++)
                {
                    if (was_moving_in_session[i])
                        cout << "cam" << i << " ";
                }
                cout << " | Total: " << cameras_that_moved << "/" << total_cameras;
            }

            // Check success condition FIRST (before timeout)
            if (any_left_moving == 0 && cameras_that_moved == total_cameras)
            {
                if (debug_mode)
                    cout << " -> ENDING SESSION (success)" << endl;
                log_debug("Motion session ended (all cameras participated and stopped) - duration: " + to_string(session_duration) + "ms");

                result.session_ended = true;
                result.end_reason = "success";
                result.session_duration_ms = session_duration;
                result.cameras_participated = cameras_that_moved;
                result.total_cameras = total_cameras;

                is_moving = false;
                last_session_end_time = now;
            }
            else if (session_duration >= params.max_session_duration_ms)
            {
                if (debug_mode)
                    cout << " -> ENDING SESSION (timeout)" << endl;
                log_debug("Motion session ended (timeout) - duration: " + to_string(session_duration) + "ms");

                result.session_ended = true;
                result.end_reason = "timeout";
                result.session_duration_ms = session_duration;
                result.cameras_participated = cameras_that_moved;
                result.total_cameras = total_cameras;

                is_moving = false;
                last_session_end_time = now;
            }
            else
            {
                if (debug_mode)
                    cout << " -> continuing..." << endl;
            }

            // Set current session state
            result.session_active = is_moving;
            result.currently_moving = are_moving;
            result.participated_cameras = was_moving_in_session;
        }

        result.session_active = is_moving;
        return result;
    }

} // namespace motion_processing
