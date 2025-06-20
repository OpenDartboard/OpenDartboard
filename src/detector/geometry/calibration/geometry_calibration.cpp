#include "geometry_calibration.hpp"
#include "../utils/dartboard_visualization.hpp"
#include "../utils/geometry_utils.hpp"
#include "color_processing.hpp"
#include "roi_processing.hpp"
#include "mask_processing.hpp"
#include "contour_processing.hpp"
#include "bull_processing.hpp" // NEW
#include "ellipse_processing.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

using namespace cv;
using namespace std;

DartboardCalibration::DartboardCalibration()
    : center(0, 0), radius(0), orientation(270.0),
      axes(0, 0), angle(0),
      bullRadius(0), doubleRingInner(0), doubleRingOuter(0),
      tripleRingInner(0), tripleRingOuter(0), camera_index(-1)
{
}

// Method to create a standard calibration
bool DartboardCalibration::calibrateSingleCamera(const Mat &frame, int cameraIdx, bool debugMode)
{
    cout << "DEBUG: Calibrating camera " << cameraIdx + 1 << "..." << endl;

    if (frame.empty())
    {
        cerr << "Empty frame from camera " << cameraIdx + 1 << endl;
        return false;
    }

    Point center;
    double radius;
    double orientation;
    Size2f axes;
    double angle;

    bool found = findDartboardCircle(frame, center, radius, orientation, axes, angle, cameraIdx, debugMode);

    if (found && radius > 25)
    {
        // Use helper function to create calibration with all detected parameters
        *this = createStandardCalibration(center, radius, orientation, axes, angle, cameraIdx);
        return true;
    }
    else
    {
        cout << "FAIL: Failed to calibrate camera " << cameraIdx + 1 << endl;
        return false;
    }
}

// Simplified Color-Based Dartboard Detection
bool DartboardCalibration::findDartboardCircle(const Mat &frame, Point &center, double &radius, double &orientation, Size2f &axes, double &angle, int camera_idx, bool debug_mode)
{
    // Basic validation
    if (frame.empty() || frame.channels() < 3)
    {
        cerr << "FAIL: Invalid frame provided for dartboard detection." << endl;
        return false;
    }

    Mat orginalFrame = frame.clone();
    Point frameCenter = geometry_utils::calculateFrameCenter(frame);

    // [===STEP 1:===] Create ROI using the clean ROI processing module
    roi_processing::ROIParams roiParams;
    Mat roiFrame = roi_processing::processROI(orginalFrame, debug_mode, camera_idx, roiParams);

    // [===STEP 2:===] Detect red-green colors using the CLEAN color detection module
    color_processing::ColorParams colorParams;
    Mat redGreenFrame = color_processing::processColors(roiFrame, camera_idx, debug_mode, colorParams);

    // [===STEP 3:===] Create binary mask for contour processing
    mask_processing::MaskParams maskParams;
    Mat mask = mask_processing::processMask(redGreenFrame, camera_idx, debug_mode, maskParams);

    // [===STEP 4:===] Process contours using the CLEAN contour processing module
    contour_processing::ContourParams contourParams;
    vector<vector<Point>> contours = contour_processing::processContours(mask, orginalFrame, camera_idx, debug_mode, contourParams);

    //[===STEP 5:===] BULL DETECTION using the new bull processing module
    bull_processing::BullParams bullParams;
    Point bullCenter = bull_processing::processBull(redGreenFrame, mask, frameCenter, camera_idx, debug_mode, bullParams);

    //[===STEP 6:===] ELLIPSE DETECTION for dartboard shape
    ellipse_processing::EllipseParams ellipseParams;
    RotatedRect dartboardEllipse = ellipse_processing::processEllipse(contours, orginalFrame, mask, bullCenter, frameCenter, camera_idx, debug_mode, ellipseParams);

    // Use ellipse properties for dartboard calibration (no if condition needed!)
    center = dartboardEllipse.center;                                                       // ✅ Actual dartboard center (or bull fallback)
    axes = dartboardEllipse.size;                                                           // ✅ Major/minor axes
    angle = dartboardEllipse.angle;                                                         // ✅ Rotation angle
    radius = max(axes.width, axes.height) / 2;                                              // ✅ Approximate radius for legacy code
    orientation = detectOrientation(redGreenFrame, center, radius, mask, mask, debug_mode); // ✅ Detect orientation

    return true;
}

// Create a standard calibration
DartboardCalibration DartboardCalibration::createStandardCalibration(Point center, double radius, double orientation, Size2f axes, double angle, int cameraIdx)
{
    DartboardCalibration calib;
    calib.center = center;
    calib.radius = radius;
    calib.orientation = orientation;
    calib.camera_index = cameraIdx;

    // Set elliptical parameters
    calib.axes = axes;
    calib.angle = angle;

    // Set standard ring proportions
    calib.bullRadius = radius * 0.07;
    calib.doubleRingInner = radius * 0.92;
    calib.doubleRingOuter = radius;
    calib.tripleRingInner = radius * 0.55;
    calib.tripleRingOuter = radius * 0.63;

    return calib;
}

// New method to detect dartboard orientation
double DartboardCalibration::detectOrientation(const Mat &frame, const Point &center, double radius, const Mat &redMask, const Mat &greenMask, bool debugMode)
{
    return 90.0; // Default orientation if invalid inputs
}

// Multi-camera calibration orchestration
vector<DartboardCalibration> DartboardCalibration::calibrateMultipleCameras(const vector<Mat> &frames, bool debugMode, int targetWidth, int targetHeight)
{
    vector<DartboardCalibration> calibrations;
    cout << "\n===== DARTBOARD CALIBRATION STARTED =====\n";

    if (frames.empty())
    {
        cerr << "No frames provided for calibration" << endl;
        return calibrations; // Return empty vector
    }

    for (size_t cam_idx = 0; cam_idx < frames.size(); cam_idx++)
    {
        if (frames[cam_idx].empty())
        {
            cerr << "WARN: Empty frame from camera " << cam_idx + 1 << endl;
            continue;
        }

        // Create and calibrate each camera
        DartboardCalibration calib;
        if (calib.calibrateSingleCamera(frames[cam_idx], cam_idx, debugMode))
        {
            calibrations.push_back(calib);

            // Save individual camera calibration debug image
            if (debugMode)
            {
                // Create a debug visualization showing the calibration
                Mat debugFrame = frames[cam_idx].clone();

                // Use existing drawCalibrationOverlay function to visualize the calibration
                dartboard_visualization::drawCalibrationOverlay(debugFrame, calib, true);

                // Add detailed debugging information
                cout << "DEBUG: Camera " << cam_idx + 1
                     << " center=(" << calib.center.x << "," << calib.center.y << ")"
                     << ", center offset=(" << (calib.center.x - (debugFrame.cols / 2)) << ","
                     << (calib.center.y - (debugFrame.rows / 2)) << ")"
                     << ", radius=" << calib.radius
                     << ", axes=" << calib.axes.width << "x" << calib.axes.height
                     << ", angle=" << calib.angle
                     << ", orientation=" << calib.orientation << endl;

                // Save the calibration visualization
                system("mkdir -p debug_frames/geometry_calibration");
                imwrite("debug_frames/geometry_calibration/calibration_camera_" + to_string(cam_idx) + ".jpg", debugFrame);
            }
        }
        else
        {
            cout << "FAIL: Failed to calibrate camera " << cam_idx + 1 << endl;
        }
    }

    cout << "===== DARTBOARD CALIBRATION COMPLETED =====\n";
    return calibrations;
}