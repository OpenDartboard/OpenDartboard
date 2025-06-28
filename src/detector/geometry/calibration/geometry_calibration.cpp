#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

#include "geometry_calibration.hpp"
#include "color_processing.hpp"
#include "roi_processing.hpp"
#include "mask_processing.hpp"
#include "contour_processing.hpp"
#include "bull_processing.hpp"
#include "ellipse_processing.hpp"
#include "wire_processing.hpp"
#include "dartboard_visualization.hpp"
#include "../../../utils/debug.hpp"
#include "../../../utils/math.hpp"

using namespace cv;
using namespace std;

namespace geometry_calibration
{
    // CLEAN: Just run the pipeline and return calibration - like a JS function
    DartboardCalibration calibrateSingleCamera(const Mat &frame, int cameraIdx, bool debugMode)
    {
        cout << "DEBUG: Calibrating camera " << cameraIdx + 1 << "..." << endl;

        if (frame.empty())
        {
            cerr << "Empty frame from camera " << cameraIdx + 1 << endl;
            return DartboardCalibration(); // Return empty calibration
        }

        Mat orginalFrame = frame.clone();
        Point frameCenter = math::calculateFrameCenter(frame);

        // [===STEP 1:===] Create ROI using the clean ROI processing module
        roi_processing::ROIParams roiParams;
        Mat roiFrame = roi_processing::processROI(orginalFrame, debugMode, cameraIdx, roiParams);

        // [===STEP 2:===] Detect red-green colors using the CLEAN color detection module
        color_processing::ColorParams colorParams;
        Mat redGreenFrame = color_processing::processColors(roiFrame, cameraIdx, debugMode, colorParams);

        // [===STEP 3:===] contour DETECTION using the new contour processing module
        // Note: This step is currently commented out as it is not used in the new pipeline
        // Uncomment if contour processing is needed in the future, for now leave it here for reference
        // contour_processing::ContourParams contourParams;
        // vector<vector<Point>> contours = contour_processing::processContours(redGreenFrame, orginalFrame, cameraIdx, debugMode, contourParams);

        // [===STEP 4:===] BULL dectection using the new bull processing module
        bull_processing::BullParams bullParams;
        Point bullCenter = bull_processing::processBull(redGreenFrame, frameCenter, cameraIdx, debugMode, bullParams);

        // [===STEP 5:===] Create binary mask for contour processing
        mask_processing::MaskParams maskParams;
        mask_processing::MaskBundle masks = mask_processing::processMask(redGreenFrame, bullCenter, cameraIdx, debugMode, maskParams);

        // [===STEP 6:===] ELLIPSE DETECTION for dartboard shape - NOW USES ALL MASKS
        ellipse_processing::EllipseParams ellipseParams;
        ellipse_processing::EllipseBoundaryData ellipseData = ellipse_processing::processEllipse(orginalFrame, masks, bullCenter, frameCenter, cameraIdx, debugMode, ellipseParams);

        // [===STEP 7:===] BUILD CALIBRATION DIRECTLY FROM DETECTED DATA
        DartboardCalibration calibration;

        // Basic info
        calibration.camera_index = cameraIdx;
        calibration.center = bullCenter;

        calibration.doubleOuterRing = ellipseData.outerDoubleEllipse;
        calibration.doubleInnerRing = ellipseData.innerDoubleEllipse;

        calibration.tripleOuterRing = ellipseData.outerTripleEllipse;
        calibration.tripleInnerRing = ellipseData.innerTripleEllipse;

        calibration.bullOuterRing = ellipseData.outerBullEllipse;
        calibration.bullInnerRing = ellipseData.innerBullEllipse;

        // perspective analysis from ellipse data
        calibration.perspectiveOffsetX = ellipseData.offsetX;
        calibration.perspectiveOffsetY = ellipseData.offsetY;
        calibration.perspectiveOffsetMagnitude = ellipseData.offsetMagnitude;

        calibration.hasDetectedEllipses = ellipseData.hasValidDoubles && ellipseData.hasValidTriples && ellipseData.hasValidBulls;

        // [===STEP 8:===] Extract actual wire positions for segment alignment
        wire_processing::WireData wireData = wire_processing::processWires(orginalFrame, redGreenFrame, calibration, debugMode);

        // TODO: Use wireData to refine dartboard orientation and segment alignment
        // For now, we'll use the default orientation from the ellipse processing

        return calibration;
    }

    // Multi-camera calibration orchestration
    vector<DartboardCalibration> calibrateMultipleCameras(const vector<Mat> &frames, bool debugMode, int targetWidth, int targetHeight)
    {
        vector<DartboardCalibration> calibrations;
        cout << "\n===== DARTBOARD CALIBRATION STARTED =====\n";

        if (frames.empty())
        {
            cerr << "No frames provided for calibration" << endl;
            return calibrations;
        }

        for (size_t cam_idx = 0; cam_idx < frames.size(); cam_idx++)
        {
            if (frames[cam_idx].empty())
            {
                cerr << "WARN: Empty frame from camera " << cam_idx + 1 << endl;
                continue;
            }

            // CLEAN: Just call the static function
            DartboardCalibration calibration = calibrateSingleCamera(frames[cam_idx], cam_idx, debugMode);
            calibrations.push_back(calibration);

            if (debugMode)
            {
                // Create a debug visualization showing the calibration
                cv::Mat visFrame = dartboard_visualization::drawCalibrationOverlay(frames[cam_idx], calibration, true);
                system("mkdir -p debug_frames/geometry_calibration");
                imwrite("debug_frames/geometry_calibration/calibration_camera_" + to_string(cam_idx) + ".jpg", visFrame);
            }
        }

        if (debugMode && !calibrations.empty())
        {
            // Create combined visualization using debug utility
            cout << "DEBUG: Creating combined calibration visualization..." << endl;
            Mat combinedViz = debug::createCombinedCalibrationVisualization(frames.size());

            if (!combinedViz.empty())
            {
                system("mkdir -p debug_frames");
                string filename = "debug_frames/calibration_summary_all_cameras.jpg";
                imwrite(filename, combinedViz);
                cout << "DEBUG: Saved combined calibration visualization: " << filename << endl;
            }
        }

        cout << "===== DARTBOARD CALIBRATION COMPLETED =====\n";
        return calibrations;
    }

} // namespace geometry_calibration