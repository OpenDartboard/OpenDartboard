#include "dart_processing.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

namespace dart_processing
{
    // Static state tracking
    static vector<DartBoardState> previous_states = {DartBoardState::CLEAN};
    static DartBoardState best_previous_state = DartBoardState::CLEAN;
    static int stability_frame_count = 0;
    static bool initialized = false;

    // Frame averaging state machine - OPTIMIZED
    static bool collecting_frames = false;
    static vector<Mat> accumulated_frames; // Pre-computed sum per camera (CV_32F)
    static int frames_collected = 0;

    DartStateResult processDartState(const vector<Mat> &current_frames, const vector<Mat> &background_frames,
                                     bool movement_finished, bool debug_mode, const DartParams &params)
    {
        DartStateResult result;

        if (!initialized)
        {
            accumulated_frames.resize(current_frames.size());
            initialized = true;
        }

        // Frame collection state machine
        if (movement_finished && !collecting_frames)
        {
            collecting_frames = true;
            frames_collected = 0;

            // Initialize accumulated frames to zero
            for (size_t i = 0; i < current_frames.size(); i++)
            {
                if (!current_frames[i].empty())
                {
                    Mat gray;
                    cvtColor(current_frames[i], gray, COLOR_BGR2GRAY);
                    accumulated_frames[i] = Mat::zeros(gray.size(), CV_32F);
                }
            }
            // log_info("Motion finished - starting frame collection");
        }

        if (collecting_frames)
        {
            // Add current frames to accumulation (spreads the cost across frames)
            for (size_t i = 0; i < current_frames.size(); i++)
            {
                if (!current_frames[i].empty())
                {
                    Mat current_gray, float_frame;
                    cvtColor(current_frames[i], current_gray, COLOR_BGR2GRAY);
                    current_gray.convertTo(float_frame, CV_32F);
                    accumulated_frames[i] += float_frame; // Accumulate sum
                }
            }
            frames_collected++;

            if (frames_collected < params.stability_frames)
            {
                return result; // Still collecting, return empty result
            }

            // We have enough frames, process once then stop collecting
            collecting_frames = false;
            // log_info("Frame collection complete - processing");
        }
        else if (!movement_finished)
        {
            return result; // Not collecting and no movement finished, return empty
        }

        // initialise variables
        result.camera_results.resize(current_frames.size());

        // Process all cameras - use pre-computed averages
        vector<DartBoardState> camera_states;

        for (size_t i = 0; i < current_frames.size(); i++)
        {

            Mat background_gray;
            cvtColor(background_frames[i], background_gray, COLOR_BGR2GRAY);

            Mat averaged_frame;
            accumulated_frames[i].convertTo(averaged_frame, CV_8U, 1.0 / frames_collected);

            // Calculate difference from background - NO FILTERING
            Mat diff;
            absdiff(averaged_frame, background_gray, diff);

            // clean up the difference image
            // GaussianBlur(diff, diff, Size(5, 5), 1.0); // Softer blending of dart edges
            medianBlur(diff, diff, params.blur_kernel_size);                    // Smooth out noise in grayscale diff
            dilate(diff, diff, Mat(), Point(-1, -1), params.dilate_iterations); // Strengthen dart signals
            erode(diff, diff, Mat(), Point(-1, -1), params.erode_iterations);   // Remove small noise

            // Threshold the difference image
            Mat thresh;
            threshold(diff, thresh, params.background_diff_threshold, 255, THRESH_BINARY);

            // Apply morphological operations to clean up the thresholded image
            Mat morph_kernel = getStructuringElement(MORPH_RECT, Size(params.morph_kernel_size, params.morph_kernel_size));
            Mat morph_kernel2 = getStructuringElement(MORPH_RECT, Size(params.morph_kernel_size / 2, params.morph_kernel_size / 2));
            morphologyEx(thresh, thresh, MORPH_CLOSE, morph_kernel);
            morphologyEx(thresh, thresh, MORPH_OPEN, morph_kernel);
            morphologyEx(thresh, thresh, MORPH_CLOSE, morph_kernel2);
            morphologyEx(thresh, thresh, MORPH_OPEN, morph_kernel2);

            // Count total changed pixels instead of contour analysis
            int total_changed_pixels = countNonZero(thresh);
            int total_pixels = thresh.rows * thresh.cols;
            float change_ratio = ((double)total_changed_pixels / (double)total_pixels) * 100.0f; // Percentage of changed pixels

            // Debug output per camera
            if (debug_mode)
            {
                // Save debug images
                system("mkdir -p debug_frames/dart_processing");
                imwrite("debug_frames/dart_processing/diff_cam_" + to_string(i) + ".jpg", diff);
                imwrite("debug_frames/dart_processing/thresh_cam_" + to_string(i) + ".jpg", thresh);
                imwrite("debug_frames/dart_processing/averaged_cam_" + to_string(i) + ".jpg", averaged_frame);
            }

            // set camera result
            result.camera_results[i].total_contours = total_changed_pixels;
            result.camera_results[i].change_ratio = change_ratio;
            result.camera_results[i].total_pixels = total_pixels;

            auto candidate_state = DartBoardState::CLEAN;

            if (change_ratio >= 0.1) // Threshold for detecting a dart
            {
                if (previous_states[i] == DartBoardState::CLEAN)
                {
                    candidate_state = DartBoardState::DART_1;
                }
                else if (previous_states[i] == DartBoardState::DART_1)
                {
                    candidate_state = DartBoardState::DART_2;
                }
                else if (previous_states[i] == DartBoardState::DART_2)
                {
                    candidate_state = DartBoardState::DART_3;
                }
                else if (previous_states[i] == DartBoardState::DART_3)
                {
                    candidate_state = DartBoardState::DART_3; // Stay in DART_3
                }
            }
            else // Threshold for no dart
            {
                if (previous_states[i] == DartBoardState::DART_1 ||
                    previous_states[i] == DartBoardState::DART_2 ||
                    previous_states[i] == DartBoardState::DART_3 ||
                    previous_states[i] == DartBoardState::CLEAN)
                {
                    candidate_state = DartBoardState::CLEAN;
                }
            }

            // store previous state so we can compare to global variable
            result.camera_results[i].previous_state = previous_states[i];
            previous_states[i] = candidate_state;
            result.camera_results[i].detected_state = candidate_state;

            // debug output
            printf("Camera %zu: State=%d, Change Ratio=%.2f, Total Pixels=%d, Total Changed Pixels=%d\n", i, static_cast<int>(candidate_state), change_ratio, total_pixels, total_changed_pixels);
        }

        // Now we have all camera states, determine best states with 2 out of 3 majority rule
        int state_counts[4] = {0}; // CLEAN, DART_1, DART_2, DART_3
        for (const auto &camera_result : result.camera_results)
        {
            state_counts[static_cast<int>(camera_result.detected_state)]++;
        }

        // Determine the best state based on majority voting
        DartBoardState best_state = DartBoardState::CLEAN;
        int max_count = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (state_counts[i] > max_count)
            {
                max_count = state_counts[i];
                best_state = static_cast<DartBoardState>(i);
            }
        }

        // Set the best state as the current confirmed state
        result.current_state = best_state;
        result.previous_state = best_previous_state;
        best_previous_state = best_state;

        return result;
    }

} // namespace dart_processing
