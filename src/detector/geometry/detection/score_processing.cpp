#include "score_processing.hpp"
#include "utils.hpp"
#include "utils/streamer.hpp"
#include "../calibration/geometry_calibration.hpp"

using namespace cv;
using namespace std;

namespace score_processing
{

    static bool initialized = false;
    static unique_ptr<streamer> point_on_screen_streamer;

    // Helper: Check if point is inside ellipse (pure math)
    bool isPointInEllipse(Point2f point, const RotatedRect &ellipse)
    {
        Point2f center = ellipse.center;
        Point2f relative = point - center;

        // Rotate point to ellipse coordinate system
        float angle_rad = -ellipse.angle * CV_PI / 180.0f;
        float cos_a = cos(angle_rad);
        float sin_a = sin(angle_rad);

        Point2f rotated(
            relative.x * cos_a - relative.y * sin_a,
            relative.x * sin_a + relative.y * cos_a);

        // Ellipse equation: (x/a)² + (y/b)² <= 1
        float a = ellipse.size.width / 2.0f;
        float b = ellipse.size.height / 2.0f;

        return (rotated.x * rotated.x) / (a * a) + (rotated.y * rotated.y) / (b * b) <= 1.0f;
    }

    // Clean, angle-based scoring function
    string getScoreAtPoint(Point2f pixel, const DartboardCalibration &calib)
    {
        // Validation check
        if (!calib.ellipses.hasValidDoubles || calib.wires.wireEndpoints.size() < 20)
        {
            log_debug("SCORE: Invalid calibration data");
            return "MISS";
        }

        Point2f center = Point2f(calib.bullCenter);

        // 1. RING DETECTION - Check from inside out
        bool in_inner_bull = isPointInEllipse(pixel, calib.ellipses.innerBullEllipse);
        if (in_inner_bull)
        {
            log_debug("SCORE: Point in INNER BULL");
            return "BULL";
        }

        bool in_outer_bull = isPointInEllipse(pixel, calib.ellipses.outerBullEllipse);
        if (in_outer_bull)
        {
            log_debug("SCORE: Point in OUTER BULL");
            return "OUTER";
        }

        bool in_inner_triple = isPointInEllipse(pixel, calib.ellipses.innerTripleEllipse);
        bool in_outer_triple = isPointInEllipse(pixel, calib.ellipses.outerTripleEllipse);
        bool in_inner_double = isPointInEllipse(pixel, calib.ellipses.innerDoubleEllipse);
        bool in_outer_double = isPointInEllipse(pixel, calib.ellipses.outerDoubleEllipse);

        log_debug("SCORE: Ellipse tests - Inner_T:" + log_string(in_inner_triple) +
                  " Outer_T:" + log_string(in_outer_triple) +
                  " Inner_D:" + log_string(in_inner_double) +
                  " Outer_D:" + log_string(in_outer_double));

        // Determine ring type with clear logic
        string ring_prefix;
        if (in_outer_double && !in_inner_double)
        {
            ring_prefix = "D"; // In the double ring (narrow band)
            log_debug("SCORE: Ring type = DOUBLE");
        }
        else if (in_outer_triple && !in_inner_triple)
        {
            ring_prefix = "T"; // In the triple ring (narrow band)
            log_debug("SCORE: Ring type = TRIPLE");
        }
        else if (in_outer_double)
        {
            ring_prefix = "S"; // Anywhere else inside the dartboard
            log_debug("SCORE: Ring type = SINGLE");
        }
        else
        {
            log_debug("SCORE: Point outside dartboard");
            return "MISS";
        }

        // 2. WEDGE DETECTION using angles
        if (!calib.orientation.isStarCamera || calib.orientation.wedge20WireIndex < 0)
        {
            log_debug("SCORE: No orientation data, defaulting to 20");
            return ring_prefix + "20";
        }

        // Calculate angle from center to point
        Point2f direction = pixel - center;
        float point_angle = atan2(direction.y, direction.x);
        if (point_angle < 0)
            point_angle += 2 * CV_PI; // Normalize to 0-2π

        log_debug("SCORE: Point angle = " + log_string(point_angle * 180.0f / CV_PI) + " degrees");

        // Standard dartboard sequence starting from 20
        vector<int> dartboard_numbers = {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};
        int wire20_index = calib.orientation.wedge20WireIndex;

        // Check each wedge by calculating its angular boundaries
        for (int i = 0; i < 20; i++)
        {
            int wire1_index = (wire20_index + i) % calib.wires.wireEndpoints.size();
            int wire2_index = (wire20_index + i + 1) % calib.wires.wireEndpoints.size();

            Point2f wire1 = calib.wires.wireEndpoints[wire1_index];
            Point2f wire2 = calib.wires.wireEndpoints[wire2_index];

            // Calculate angles for both wire boundaries
            Point2f dir1 = wire1 - center;
            Point2f dir2 = wire2 - center;
            float angle1 = atan2(dir1.y, dir1.x);
            float angle2 = atan2(dir2.y, dir2.x);

            if (angle1 < 0)
                angle1 += 2 * CV_PI;
            if (angle2 < 0)
                angle2 += 2 * CV_PI;

            // Ensure angle1 < angle2 (handle wraparound)
            if (angle2 < angle1)
                angle2 += 2 * CV_PI;

            // Check if point angle is between the two wire angles
            float test_angle = point_angle;
            if (test_angle < angle1)
                test_angle += 2 * CV_PI;

            if (test_angle >= angle1 && test_angle <= angle2)
            {
                int number = dartboard_numbers[i];
                log_debug("SCORE: Found wedge " + log_string(number) +
                          " (angle1=" + log_string(angle1 * 180.0f / CV_PI) +
                          ", angle2=" + log_string(angle2 * 180.0f / CV_PI) + ")");
                return ring_prefix + to_string(number);
            }
        }

        log_debug("SCORE: No wedge found - this shouldn't happen");
        return "MISS";
    }

    ScoreResult processScore(const vector<Mat> &background_frames, const dart_processing::DartStateResult &dart_result, const vector<DartboardCalibration> &calibrations, bool debug_mode)
    {

        if (!initialized)
        {
            log_debug("SCORE: Initializing score processing");
            point_on_screen_streamer = make_unique<streamer>(8088, 1);
            initialized = true;
        }

        ScoreResult result;

        if (dart_result.previous_state == dart_result.current_state)
        {
            // No state change, return invalid result
            result.valid = false;
            return result;
        }

        // State changed - now process the scoring
        switch (dart_result.current_state)
        {
        case dart_processing::DartBoardState::CLEAN:
            result.score = "END";
            result.confidence = 1.0f;
            result.camera_index = -1;
            result.valid = true;
            break;

        case dart_processing::DartBoardState::DART_1:
        case dart_processing::DartBoardState::DART_2:
        case dart_processing::DartBoardState::DART_3:
            // Collect scores from all cameras with detected tips
            vector<pair<string, int>> camera_scores; // (score, camera_index)
            vector<Mat> points_on_screen;            // For debug images

            for (size_t i = 0; i < dart_result.camera_results.size(); i++)
            {
                log_debug("-------");
                string score_test = getScoreAtPoint(dart_result.camera_results[i].tip_position, calibrations[i]);
                log_debug("-------");

                // print image
                if (debug_mode)
                {
                    log_warning("Camera " + to_string(i) + " score: " + score_test);

                    // just draw the point on the screen
                    Mat some_mat = background_frames[i].clone();
                    circle(some_mat, dart_result.camera_results[i].tip_position, 5, Scalar(0, 255, 0), -1);
                    putText(some_mat, score_test, dart_result.camera_results[i].tip_position + Point2f(10, 10),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
                    system("mkdir -p debug_frames/score_processing");
                    imwrite("debug_frames/score_processing/point_on_screen" + to_string(i) + ".jpg", some_mat);

                    points_on_screen.push_back(some_mat);
                }

                if (dart_result.camera_results[i].tip_found && score_test != "MISS")
                {
                    camera_scores.push_back({score_test, static_cast<int>(i)});
                }
            }

            if (debug_mode)
            {
                try
                {
                    point_on_screen_streamer->push(debug::createCombinedFrame(points_on_screen, "points_on_screen"));
                }
                catch (const std::exception &e)
                {
                    log_error("Failed to push points_on_screen frame: " + string(e.what()));
                }
            }

            if (!camera_scores.empty())
            {
                // Consensus scoring logic
                string final_score;
                int best_camera = -1;

                // Count occurrences of each score
                map<string, vector<int>> score_cameras;
                for (const auto &[score, camera_idx] : camera_scores)
                {
                    score_cameras[score].push_back(camera_idx);
                }

                // Look for consensus (2+ cameras agreeing)
                string consensus_score;
                int max_consensus = 0;
                for (const auto &[score, cameras] : score_cameras)
                {
                    if (cameras.size() >= 2 && cameras.size() > max_consensus)
                    {
                        consensus_score = score;
                        max_consensus = cameras.size();
                    }
                }

                if (!consensus_score.empty())
                {
                    // Use consensus score, pick first camera from the group
                    final_score = consensus_score;
                    best_camera = score_cameras[consensus_score][0];
                    log_info("Consensus score: " + final_score + " from " + to_string(max_consensus) + " cameras");
                }
                else
                {
                    // No consensus, use first available score
                    final_score = camera_scores[0].first;
                    best_camera = camera_scores[0].second;
                    log_info("No consensus, using single camera score: " + final_score + " from camera " + to_string(best_camera));
                }

                result.score = final_score;
                result.pixel_position = dart_result.camera_results[best_camera].tip_position;
                result.center_position = dart_result.camera_results[best_camera].center_position;
                result.dartboard_position = dart_result.camera_results[best_camera].tip_position; // TODO: Convert to dartboard coordinates
                result.confidence = consensus_score.empty() ? 0.7f : 0.9f;
                result.camera_index = best_camera;
                result.valid = true;
            }
            else
            {
                // State changed but no valid scores found
                result.score = "MISS";
                result.confidence = 0.5f;
                result.camera_index = -1;
                result.valid = true;

                if (debug_mode)
                {
                    log_warning("State changed but no valid scores found!");
                }
            }
            break;
        }

        return result;
    }

} // namespace score_processing
