#include "perspective_processing.hpp"
#include "geometry_calibration.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

namespace perspective_processing
{
    // Find intersection of line with ellipse - returns the farther point from center
    Point2f findEllipseLineIntersection(const RotatedRect &ellipse, Point2f lineStart, Point2f lineEnd)
    {
        // Use existing math utility if available, otherwise implement basic intersection
        Point2f direction = lineEnd - lineStart;
        direction = direction / norm(direction);
        float angle = atan2(direction.y, direction.x);

        // Use utility function if it exists
        return math::intersectRayWithEllipse(lineStart, angle, ellipse);
    }

    // Find all ring-wire intersections for PnP calibration
    RingIntersections findAllRingWireIntersections(const DartboardCalibration &calib, const DartboardSpec &spec)
    {
        RingIntersections result;

        if (!calib.ellipses.hasValidDoubles || calib.wires.wireEndpoints.size() < 16)
        {
            log_error("Insufficient calibration data for intersection calculation");
            return result;
        }

        // Available rings (only use what we have detected)
        vector<RotatedRect> rings;
        vector<float> ringRadii;

        if (calib.ellipses.hasValidDoubles)
        {
            rings.push_back(calib.ellipses.outerDoubleEllipse);
            // rings.push_back(calib.ellipses.innerDoubleEllipse);
            ringRadii.push_back(spec.outerDoubleRadius);
            // ringRadii.push_back(spec.innerDoubleRadius);
        }

        log_debug("Processing " + log_string(rings.size()) + " rings with " + log_string(calib.wires.wireEndpoints.size()) + " wires");

        result.intersections.resize(calib.wires.wireEndpoints.size());
        result.modelPoints.resize(calib.wires.wireEndpoints.size());

        // For each wire, find intersections with all rings
        for (size_t wireIdx = 0; wireIdx < calib.wires.wireEndpoints.size(); wireIdx++)
        {
            Point2f wireEnd = calib.wires.wireEndpoints[wireIdx];
            Point2f bullCenter = Point2f(calib.bullCenter);

            // Calculate angle for this wire (for model points)
            Point2f direction = wireEnd - bullCenter;
            float wireAngle = atan2(direction.y, direction.x);

            result.intersections[wireIdx].resize(rings.size());
            result.modelPoints[wireIdx].resize(rings.size());

            // Find intersection with each ring
            for (size_t ringIdx = 0; ringIdx < rings.size(); ringIdx++)
            {
                Point2f intersection = findEllipseLineIntersection(rings[ringIdx], bullCenter, wireEnd);
                result.intersections[wireIdx][ringIdx] = intersection;

                // Create corresponding model point
                float radius = ringRadii[ringIdx];
                Point3f modelPoint(
                    radius * cos(wireAngle),
                    radius * sin(wireAngle),
                    0.0f // Z = 0 (planar board)
                );
                result.modelPoints[wireIdx][ringIdx] = modelPoint;
            }
        }

        result.isValid = true;
        log_debug("Successfully computed " + log_string(calib.wires.wireEndpoints.size() * rings.size()) + " intersection points");

        return result;
    }

    // Rectify board using PnP pose estimation
    Mat rectifyBoard(const Mat &image, const Mat &rvec, const Mat &tvec, const Mat &cameraMatrix, const DartboardSpec &spec)
    {
        // INCREASE CORRECTION STRENGTH - use larger output area
        float correctionMultiplier = 1.5f; // Increase this to make correction stronger
        float boardSize = spec.outerDoubleRadius * correctionMultiplier;

        vector<Point3f> outputCorners3D = {
            Point3f(-boardSize, -boardSize, 0),
            Point3f(boardSize, -boardSize, 0),
            Point3f(boardSize, boardSize, 0),
            Point3f(-boardSize, boardSize, 0)};

        // Project to image space using estimated pose
        vector<Point2f> outputCornersImage;
        projectPoints(outputCorners3D, rvec, tvec, cameraMatrix, Mat(), outputCornersImage);

        // Define destination corners (square output image)
        vector<Point2f> destinationCorners = {
            Point2f(0, 0),
            Point2f(spec.outputSize, 0),
            Point2f(spec.outputSize, spec.outputSize),
            Point2f(0, spec.outputSize)};

        // Compute perspective transform
        Mat perspectiveMatrix = getPerspectiveTransform(outputCornersImage, destinationCorners);

        // Apply transformation
        Mat rectified;
        warpPerspective(image, rectified, perspectiveMatrix, Size(spec.outputSize, spec.outputSize), INTER_LINEAR, BORDER_CONSTANT, Scalar(0));

        return rectified;
    }

    Mat processPerspective(
        const Mat &rawImage,
        const DartboardCalibration &calib,
        bool enableDebug,
        const DartboardSpec &spec)
    {
        log_debug("Processing perspective correction for camera " + log_string(calib.camera_index));

        if (rawImage.empty())
        {
            log_error("Empty input image");
            return rawImage.clone();
        }

        // Step 1: Find all ring-wire intersections
        RingIntersections intersections = findAllRingWireIntersections(calib, spec);
        if (!intersections.isValid)
        {
            log_error("Failed to compute ring-wire intersections");
            return rawImage.clone();
        }

        // Step 2: Flatten correspondences for PnP
        vector<Point3f> objectPoints;
        vector<Point2f> imagePoints;

        for (size_t wireIdx = 0; wireIdx < intersections.intersections.size(); wireIdx++)
        {
            for (size_t ringIdx = 0; ringIdx < intersections.intersections[wireIdx].size(); ringIdx++)
            {
                objectPoints.push_back(intersections.modelPoints[wireIdx][ringIdx]);
                imagePoints.push_back(intersections.intersections[wireIdx][ringIdx]);
            }
        }

        log_debug("Using " + log_string(objectPoints.size()) + " point correspondences for PnP");

        // Step 3: Set up camera matrix (approximation)
        double fx = (rawImage.cols / 2.0) / tan(120.0 * CV_PI / 360.0) * (calib.ellipses.outerDoubleEllipse.size.width * 0.5) / 170.0; // scale factor
        double fy = fx;
        double cx = rawImage.cols * 0.5;
        double cy = rawImage.rows * 0.5;
        // Mat cameraMatrix = (Mat_<double>(3, 3) << fx, 0, cx, 0, fx, cy, 0, 1);
        Mat cameraMatrix = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        // Mat cameraMatrix = (Mat_<double>(3, 3) << rawImage.cols, 0, rawImage.cols * 0.5,
        //                     0, rawImage.cols, rawImage.rows * 0.5, // Assume square pixels
        //                     0, 0, 1);

        // Step 4: Solve PnP
        Mat rvec, tvec;
        bool pnpSuccess = solvePnP(objectPoints, imagePoints, cameraMatrix, Mat(), rvec, tvec);

        if (!pnpSuccess)
        {
            log_error("PnP calibration failed");
            return rawImage.clone();
        }

        log_debug("PnP calibration successful");

        // Step 5: Rectify the board
        Mat rectified = rectifyBoard(rawImage, rvec, tvec, cameraMatrix, spec);

        // Debug output with correspondence verification
        if (enableDebug)
        {
            system("mkdir -p debug_frames/perspective_processing");

            // Save rectified image
            imwrite("debug_frames/perspective_processing/rectified_" + to_string(calib.camera_index) + ".jpg", rectified);

            // RECOMPUTE reprojection for debug (since variables are out of scope)
            vector<Point2f> reprojectedDebug;
            projectPoints(objectPoints, rvec, tvec, cameraMatrix, Mat(), reprojectedDebug);

            double totalErrorDebug = 0.0;
            for (size_t i = 0; i < imagePoints.size(); i++)
            {
                double error = norm(reprojectedDebug[i] - imagePoints[i]);
                totalErrorDebug += error;
            }
            double avgErrorDebug = totalErrorDebug / imagePoints.size();

            // Create DETAILED debug visualization showing intersections AND reprojection errors
            Mat debugImg = rawImage.clone();

            // Draw the outer doubles ellipse for reference
            ellipse(debugImg, calib.ellipses.outerDoubleEllipse, Scalar(0, 255, 0), 2);

            // Draw ALL wire lines from bull center
            for (size_t wireIdx = 0; wireIdx < calib.wires.wireEndpoints.size(); wireIdx++)
            {
                Point2f wireEnd = calib.wires.wireEndpoints[wireIdx];
                line(debugImg, calib.bullCenter, wireEnd, Scalar(255, 0, 0), 1);
            }

            // Draw ring-wire intersections with detailed info
            for (size_t wireIdx = 0; wireIdx < intersections.intersections.size(); wireIdx++)
            {
                for (size_t ringIdx = 0; ringIdx < intersections.intersections[wireIdx].size(); ringIdx++)
                {
                    Point2f intersection = intersections.intersections[wireIdx][ringIdx];

                    size_t reprojIdx = wireIdx * intersections.intersections[wireIdx].size() + ringIdx;
                    Point2f reproj = reprojectedDebug[reprojIdx];

                    // Original intersection in YELLOW (large circle)
                    circle(debugImg, intersection, 6, Scalar(0, 255, 255), -1);

                    // Reprojected point in CYAN (smaller circle)
                    circle(debugImg, reproj, 3, Scalar(255, 255, 0), -1);

                    // Red error line connecting them
                    line(debugImg, intersection, reproj, Scalar(0, 0, 255), 1);

                    // Wire number next to intersection point
                    putText(debugImg, "W" + to_string(wireIdx),
                            Point(intersection.x + 8, intersection.y - 5),
                            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);

                    // Show individual error for this point
                    double error = norm(intersection - reproj);
                    if (error > 5.0)
                    { // Only show high errors
                        putText(debugImg, to_string((int)error) + "px",
                                Point(intersection.x + 8, intersection.y + 10),
                                FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 0, 255), 1);
                    }
                }
            }

            // Add detailed information overlay
            putText(debugImg, "Yellow=Detected, Cyan=Expected, Red=Error",
                    Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
            putText(debugImg, "Avg Error: " + to_string(avgErrorDebug) + "px",
                    Point(20, 60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
            putText(debugImg, "Wire Count: " + to_string(calib.wires.wireEndpoints.size()),
                    Point(20, 90), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

            // Show wire ordering - print first 10 correspondences
            int textY = 120;
            for (size_t wireIdx = 0; wireIdx < min(size_t(10), intersections.intersections.size()); wireIdx++)
            {
                if (!intersections.intersections[wireIdx].empty())
                {
                    Point2f intersection = intersections.intersections[wireIdx][0]; // Just show ring 0
                    Point3f model = intersections.modelPoints[wireIdx][0];

                    string info = "W" + to_string(wireIdx) + ": img(" +
                                  to_string((int)intersection.x) + "," + to_string((int)intersection.y) +
                                  ") <-> model(" + to_string((int)model.x) + "," + to_string((int)model.y) + ")";

                    putText(debugImg, info, Point(20, textY),
                            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(200, 200, 200), 1);
                    textY += 15;
                }
            }

            imwrite("debug_frames/perspective_processing/intersections_" + to_string(calib.camera_index) + ".jpg", debugImg);
        }

        log_debug("Perspective correction completed");
        return rectified;
    }

} // namespace perspective_processing
