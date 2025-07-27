#include "dart_processing.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

namespace dart_processing
{
    // Static state tracking
    static vector<DartBoardState> previous_states = {
        DartBoardState::CLEAN,
        DartBoardState::CLEAN,
        DartBoardState::CLEAN};
    static DartBoardState best_previous_state = DartBoardState::CLEAN;
    static int stability_frame_count = 0;
    static bool initialized = false;

    // Frame averaging state machine - OPTIMIZED
    static bool collecting_frames = false;
    static vector<Mat> accumulated_frames; // Pre-computed sum per camera (CV_32F)
    static int frames_collected = 0;

    // Working backgrounds - one per camera
    static vector<Mat> working_backgrounds;

    // get name of ENUM
    string getDartBoardStateName(DartBoardState state)
    {
        switch (state)
        {
        case DartBoardState::CLEAN:
            return "CLEAN";
        case DartBoardState::DART_1:
            return "DART_1";
        case DartBoardState::DART_2:
            return "DART_2";
        case DartBoardState::DART_3:
            return "DART_3";
        default:
            return "UNKNOWN";
        }
    }

    pair<Point2f, Point2f> detectTipAndCenter(const Mat &binary_thresh, bool debug_mode, int camera_id)
    {
        Point2f tip_position(0, 0);
        Point2f center_position(0, 0);

        if (binary_thresh.empty())
        {
            return make_pair(tip_position, center_position);
        }

        // Find ALL contours
        vector<vector<Point>> contours;
        findContours(binary_thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            return make_pair(tip_position, center_position);
        }

        vector<vector<Point>> dart_pieces;
        for (const auto &contour : contours)
        {
            double area = contourArea(contour);
            if (area > 400 && area < 20000) // More inclusive range for dart pieces
            {
                dart_pieces.push_back(contour);
            }
        }

        if (dart_pieces.empty())
        {
            return make_pair(tip_position, center_position);
        }

        // Sort by area (largest first)
        sort(dart_pieces.begin(), dart_pieces.end(), [](const vector<Point> &a, const vector<Point> &b)
             { return contourArea(a) > contourArea(b); });

        // Combine ALL dart pieces into one big point cloud
        vector<Point> all_points;
        for (const auto &piece : dart_pieces)
        {
            all_points.insert(all_points.end(), piece.begin(), piece.end());
        }

        if (all_points.empty())
        {
            return make_pair(tip_position, center_position);
        }

        // Get Hull
        vector<Point> hull;
        convexHull(all_points, hull);

        if (hull.size() < 3)
        {
            return make_pair(tip_position, center_position);
        }

        // Get center of the biggest shape
        vector<Point> biggest_shape = dart_pieces[0];
        Moments m = moments(biggest_shape);
        if (m.m00 == 0)
        {
            return make_pair(tip_position, center_position);
        }

        Point2f biggest_shape_center(m.m10 / m.m00, m.m01 / m.m00);
        center_position = biggest_shape_center;

        // Find the HULL point furthest from the biggest shape's center
        double max_distance = 0;
        Point furthest_hull_point;

        for (const Point &hull_point : hull)
        {
            double distance = norm(Point2f(hull_point) - biggest_shape_center);
            if (distance > max_distance)
            {
                max_distance = distance;
                furthest_hull_point = hull_point;
            }
        }

        if (max_distance > 10) // Minimum distance threshold
        {
            tip_position = Point2f(furthest_hull_point);
        }
        else
        {
            log_debug("No tip found - max hull distance too small: " + to_string(max_distance));
        }

        // Debug visualization
        if (debug_mode)
        {
            Mat debug_img;
            cvtColor(binary_thresh, debug_img, COLOR_GRAY2BGR);

            // Draw convex hull
            if (!hull.empty())
            {
                vector<vector<Point>> hull_vec = {hull};
                drawContours(debug_img, hull_vec, -1, Scalar(255, 0, 0), 2);
            }

            // Draw ALL dart pieces
            for (size_t i = 0; i < dart_pieces.size(); i++)
            {
                Scalar color;
                if (i == 0)
                    color = Scalar(0, 255, 0); // Green for largest
                else if (i == 1)
                    color = Scalar(0, 128, 0); // Less green for second largest
                else if (i == 2)
                    color = Scalar(0, 255, 255); // Yellow for third
                else
                    color = Scalar(255, 0, 255); // Magenta for others

                // Draw filled contour with orange color
                drawContours(debug_img, vector<vector<Point>>{dart_pieces[i]}, -1, color, -1, 8);
                drawContours(debug_img, vector<vector<Point>>{dart_pieces[i]}, -1, Scalar(255, 165, 255), 1);

                // Draw centroid and area
                Moments m = moments(dart_pieces[i]);
                if (m.m00 > 0)
                {
                    Point2f center(m.m10 / m.m00, m.m01 / m.m00);
                    circle(debug_img, center, 4, Scalar(0, 0, 0), -1);

                    int area = (int)contourArea(dart_pieces[i]);
                    putText(debug_img, to_string(area), center + Point2f(8, 0),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
                }
            }

            // Draw center of biggest shape
            circle(debug_img, biggest_shape_center, 6, Scalar(0, 255, 255), -1);
            putText(debug_img, "CENTER", biggest_shape_center + Point2f(10, 0), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 255), 1);

            // Draw line from center to tip
            if (norm(tip_position) > 0)
            {
                line(debug_img, biggest_shape_center, tip_position, Scalar(0, 0, 255), 2);

                // Draw the final tip (RED)
                circle(debug_img, tip_position, 8, Scalar(0, 0, 255), -1);
                circle(debug_img, tip_position, 8, Scalar(255, 255, 255), 2);
                putText(debug_img, "TIP", tip_position + Point2f(15, -10),
                        FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);

                // Add distance info
                putText(debug_img, to_string((int)max_distance) + "px", tip_position + Point2f(15, 10),
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
            }

            // Add detailed summary
            string summary = "Pieces: " + to_string(dart_pieces.size()) +
                             " | Hull: " + to_string(hull.size()) + "pts" +
                             " | Biggest: " + to_string((int)contourArea(biggest_shape)) + "px" +
                             (norm(tip_position) > 0 ? " | TIP FOUND" : " | NO TIP");
            putText(debug_img, summary, Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

            if (norm(tip_position) > 0)
            {
                string coords = "TIP: (" + to_string((int)tip_position.x) + "," + to_string((int)tip_position.y) +
                                ") | Distance: " + to_string((int)max_distance) + "px";
                putText(debug_img, coords, Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
            }

            // Save debug image
            system("mkdir -p debug_frames/dart_processing");
            imwrite("debug_frames/dart_processing/tip_detection_cam_" + to_string(camera_id) + ".jpg", debug_img);
        }

        return make_pair(tip_position, center_position);
    }

    DartStateResult processDartState(const vector<Mat> &current_frames, const vector<Mat> &background_frames,
                                     bool movement_finished, bool debug_mode, const DartParams &params)
    {
        DartStateResult result;

        if (!initialized)
        {
            accumulated_frames.resize(current_frames.size());
            working_backgrounds.resize(current_frames.size()); // Initialize working backgrounds
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

            // Calculate difference from background
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
            morphologyEx(thresh, thresh, MORPH_CLOSE, morph_kernel);  // Close small gaps in darts
            morphologyEx(thresh, thresh, MORPH_OPEN, morph_kernel);   // Open small noise
            morphologyEx(thresh, thresh, MORPH_CLOSE, morph_kernel2); // Close smaller gaps in darts
            morphologyEx(thresh, thresh, MORPH_OPEN, morph_kernel2);  // Open smaller noise

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
            result.camera_results[i].total_changed_pixels = total_changed_pixels;
            result.camera_results[i].change_ratio = change_ratio;
            result.camera_results[i].total_pixels = total_pixels;

            // Crate a working background for this camera
            Mat single_thresh;

            // Determine candidate state based on change ratio and previous state
            auto candidate_state = DartBoardState::CLEAN;
            if (change_ratio >= 0.22) // Threshold for detecting a dart
            {

                // CHECK FROM CLEAN AND OR UNKNOW STATES TOO (MAYBE NOT DEFINED YET)
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

                if (!working_backgrounds[i].empty())
                {
                    Mat diff_working;
                    absdiff(averaged_frame, working_backgrounds[i], diff_working);
                    medianBlur(diff_working, diff_working, params.blur_kernel_size);                    // Smooth out noise in grayscale diff
                    dilate(diff_working, diff_working, Mat(), Point(-1, -1), params.dilate_iterations); // Strengthen dart signals
                    erode(diff_working, diff_working, Mat(), Point(-1, -1), params.erode_iterations);   // Remove small noise
                    threshold(diff_working, single_thresh, params.background_diff_threshold, 255, THRESH_BINARY);
                    morphologyEx(single_thresh, single_thresh, MORPH_CLOSE, morph_kernel);  // Close small gaps in darts
                    morphologyEx(single_thresh, single_thresh, MORPH_OPEN, morph_kernel);   // Open small noise
                    morphologyEx(single_thresh, single_thresh, MORPH_CLOSE, morph_kernel2); // Close smaller gaps in darts
                    morphologyEx(single_thresh, single_thresh, MORPH_OPEN, morph_kernel2);  // Open smaller noise

                    if (debug_mode)
                    {
                        imwrite("debug_frames/dart_processing/diff_working_cam_" + to_string(i) + ".jpg", diff_working);
                    }
                }
                else
                {
                    // clone the one above
                    single_thresh = thresh.clone();
                }

                working_backgrounds[i] = averaged_frame.clone();

                // Use smart tip detection
                auto tip_and_center = detectTipAndCenter(single_thresh, debug_mode, static_cast<int>(i));
                Point2f tip_pos = tip_and_center.first;
                Point2f center_pos = tip_and_center.second;

                // If tip position is valid, update the camera result
                if (norm(tip_pos) > 0)
                {
                    result.camera_results[i].tip_position = tip_pos;
                    result.camera_results[i].center_position = tip_and_center.second;
                    result.camera_results[i].tip_found = true;
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
                    // Reset working background when going to CLEAN (safety reset)
                    working_backgrounds[0] = Mat();
                    working_backgrounds[1] = Mat();
                    working_backgrounds[2] = Mat();

                    // clone the diff to working diff
                    // we know the latest is good!
                    single_thresh = thresh.clone();

                    if (debug_mode)
                    {
                        // no tip found or none tip at all just write a black image to overwrite the tip_detection_cam_
                        Mat black_image = Mat::zeros(single_thresh.size(), CV_8UC1);
                        imwrite("debug_frames/dart_processing/tip_detection_cam_" + to_string(i) + ".jpg", black_image);
                    }
                }
            }

            if (debug_mode)
            {
                imwrite("debug_frames/dart_processing/diff_thresh_cam_" + to_string(i) + ".jpg", single_thresh);

                // debug output
                string a = getDartBoardStateName(previous_states[i]);
                string b = getDartBoardStateName(candidate_state);
                log_info("STATE GUESS: From: " + a + " -> " + b);
            }

            // store previous state so we can compare to global variable
            previous_states[i] = candidate_state;
            // add to result
            result.camera_results[i].detected_state = candidate_state;
        }

        // Now we should have the results for all cameras, we need to see what global state we should be at.
        // What can happen is for example:
        // Camera 0: DART_1, Camera 1: DART_1, Camera 2: DART_2
        // OR Camera 0: DART_3, Camera 1: CLEAN, Camera 2: DART_3
        // OR Camera 0: CLEAN, Camera 1: DART_1, Camera 2: DART_2
        // OR any other combination
        // ---
        // We need to determine the best state based on majority rule
        // And also update the ones that are not in the best state to the best state
        int moves_up = 0;
        int goes_clean = 0;
        int stays_same = 0;

        // Loop through all cameras once
        for (size_t i = 0; i < result.camera_results.size(); i++)
        {
            if (result.camera_results[i].detected_state == DartBoardState::CLEAN)
            {
                goes_clean++;
            }
            else if (result.camera_results[i].detected_state > best_previous_state)
            {
                moves_up++;
            }
            else
            {
                stays_same++;
            }
        }

        // Pick the winner
        DartBoardState final_state;
        if (goes_clean >= 2)
        {
            final_state = DartBoardState::CLEAN; // Rule 3: 2+ think CLEAN
        }
        else if (moves_up >= 2)
        {
            final_state = static_cast<DartBoardState>(static_cast<int>(best_previous_state) + 1); // Rule 1: 2+ move up
        }
        else
        {
            final_state = best_previous_state; // Rule 2: Stay put
        }

        // log
        string a = getDartBoardStateName(best_previous_state);
        string b = getDartBoardStateName(final_state);
        log_debug("FINAL State: From: " + a + " -> " + b);

        // Set ALL cameras to the final state
        for (size_t i = 0; i < previous_states.size(); i++)
        {
            previous_states[i] = final_state;
        }

        // set new best previous state
        result.current_state = final_state;
        result.previous_state = best_previous_state;
        best_previous_state = final_state;

        log_info(""); // empty line for readability

        return result;
    }
}