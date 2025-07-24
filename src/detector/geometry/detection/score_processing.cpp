#include "score_processing.hpp"
#include "utils.hpp"
#include "../calibration/geometry_calibration.hpp"

using namespace cv;
using namespace std;

namespace score_processing
{
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
            // Find the best camera with a detected tip
            int best_camera = -1;
            Point2f best_tip_position(-1, -1);
            Point2f best_center_position(-1, -1);

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
                }

                if (dart_result.camera_results[i].tip_found)
                {
                    best_camera = static_cast<int>(i);
                    best_tip_position = dart_result.camera_results[i].tip_position;
                    best_center_position = dart_result.camera_results[i].center_position;
                    // break; // Take the first camera with a tip
                }
            }

            if (best_camera >= 0)
            {

                // WE HAVE A DART TIP! Mock scoring for now 🎯
                vector<string> mock_scores = {
                    "S20", "S1", "S18", "S4", "S13", "S6", "S10", "S15", "S2", "S17",
                    "S3", "S19", "S7", "S16", "S8", "S11", "S14", "S9", "S12", "S5",
                    "D20", "D1", "D18", "D4", "D13", "D6", "D10", "D15", "D2", "D17",
                    "D3", "D19", "D7", "D16", "D8", "D11", "D14", "D9", "D12", "D5",
                    "T20", "T1", "T18", "T4", "T13", "T6", "T10", "T15", "T2", "T17",
                    "T3", "T19", "T7", "T16", "T8", "T11", "T14", "T9", "T12", "T5",
                    "BULL", "OUTER"};

                result.score = mock_scores[rand() % mock_scores.size()];
                result.pixel_position = best_tip_position;
                result.center_position = best_center_position;
                result.dartboard_position = best_tip_position; // TODO: Convert to dartboard coordinates
                result.confidence = 0.9f;
                result.camera_index = best_camera;
                result.valid = true;
            }
            else
            {
                // State changed but no tip found
                result.score = "MISS";
                result.confidence = 0.5f;
                result.camera_index = -1;
                result.valid = true;

                if (debug_mode)
                {
                    log_warning("State changed but no tip found! State: ");
                }
            }
            break;
        }

        return result;
    }

} // namespace score_processing
