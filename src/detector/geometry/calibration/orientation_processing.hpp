#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

// Forward declaration to avoid circular dependency
struct DartboardCalibration;

namespace orientation_processing
{

    // Configuration for orientation detection methods
    struct OrientationParams
    {
        // Scale to dartboard edge (standard ratio 225mm/170mm ≈ 1.32)
        float boundryScaleFactor = 1.37f;      // Scale factor for dartboard boundary
        float innerBoundryScaleFactor = 1.10f; // Scale factor for inner boundary
        float carvingWireThickness = 75.0f;    // Thickness of wire lines for segment separation

        // Image preprocessing parameters
        int brightnessThreshold = 100; // Threshold for removing dark pixels

        // spider detection parameters
        int wireExtensionDistance = 200; // Distance to extend wires for collision detection
        int spiderSampleCount = 50;      // Number of samples to check along wire extension
    };

    // Camera position enumeration for fixed-size serialization
    enum class CameraPosition : int
    {
        UNKNOWN = 0,
        TOP = 1,
        MIDDLE = 2,
        BOTTOM = 3
    };

    // Result structure for orientation detection
    struct OrientationData
    {
        int camera_index = -1;                                   // Which camera this is for
        bool isStarCamera = false;                               // Whether the orientation was successfully detected
        Point2f orientation;                                     // Detected orientation vector (e.g., "20" segment position)
        int southWireIndex = -1;                                 // Index of the south wire for this camera
        int wedge20WireIndex = -1;                               // Index of the "20" segment wire
        float angleOffsetFromSouth = 0.0f;                       // Angle offset from the south
        CameraPosition cameraPosition = CameraPosition::UNKNOWN; // Camera position (enum instead of string)
        int wedgeNumber = -1;                                    // Wedge number (6, 7, or 12)
        float avgClipWireCrossProduct = 0.0f;                    // Average
    };

    // Helper function to convert enum to string for display
    inline string cameraPositionToString(CameraPosition pos)
    {
        switch (pos)
        {
        case CameraPosition::TOP:
            return "TOP";
        case CameraPosition::MIDDLE:
            return "MIDDLE";
        case CameraPosition::BOTTOM:
            return "BOTTOM";
        default:
            return "UNKNOWN";
        }
    }

    // Main processing function
    OrientationData processOrientation(
        const Mat &frame,
        const Mat &colorMask,
        const DartboardCalibration &calib,
        bool enableDebug = false,
        const OrientationParams &params = OrientationParams());

} // namespace orientation_processing
