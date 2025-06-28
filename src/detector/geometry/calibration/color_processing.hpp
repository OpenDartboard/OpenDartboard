#pragma once

#include <opencv2/opencv.hpp>
#include <string>

using namespace cv;
using namespace std;

namespace color_processing
{
    // Structure for color detection parameters - allows for easy configuration
    struct ColorParams
    {
        // Red color HSV ranges for dartboard detection
        int redLowHue1 = 0;    // First red range - low hue (wraps around HSV circle)
        int redLowSat1 = 60;   // First red range - minimum saturation
        int redLowVal1 = 60;   // First red range - minimum value/brightness
        int redHighHue1 = 20;  // First red range - high hue
        int redHighSat1 = 255; // First red range - maximum saturation
        int redHighVal1 = 255; // First red range - maximum value/brightness

        // Second red range (for hue wrapping around 180°)
        int redLowHue2 = 160;  // Second red range - low hue (handles HSV wraparound)
        int redLowSat2 = 60;   // Second red range - minimum saturation
        int redLowVal2 = 60;   // Second red range - minimum value/brightness
        int redHighHue2 = 180; // Second red range - high hue (HSV maximum)
        int redHighSat2 = 255; // Second red range - maximum saturation
        int redHighVal2 = 255; // Second red range - maximum value/brightness

        // Adaptive red detection for varying lighting (top vs bottom of dartboard)
        int topRedHueShift = 5;  // Hue adjustment for top portion of dartboard
        int topRedSatShift = 30; // Saturation adjustment for top portion of dartboard
        int topRedValShift = 20; // Value/brightness adjustment for top portion of dartboard

        // Green color HSV range for dartboard detection
        int greenLowHue = 38;   // Green range - low hue (green section of HSV)
        int greenLowSat = 60;   // Green range - minimum saturation
        int greenLowVal = 60;   // Green range - minimum value/brightness
        int greenHighHue = 88;  // Green range - high hue
        int greenHighSat = 255; // Green range - maximum saturation
        int greenHighVal = 255; // Green range - maximum value/brightness

        // White/bright area detection (for potential future use)
        int whiteLowVal = 210; // White detection - minimum brightness threshold
        int whiteHighSat = 15; // White detection - maximum saturation (low = white)

        // Image preprocessing parameters
        int bilateralD = 9;           // Bilateral filter diameter (noise reduction)
        int bilateralSigmaColor = 75; // Bilateral filter color variance (edge preservation)
        int bilateralSigmaSpace = 75; // Bilateral filter spatial variance (smoothing)

        // Basic morphological operations
        int basicCloseKernelSize = 3;    // Small closing to connect nearby segments
        int cleanOpenKernelSize = 3;     // Opening to remove small noise
        int cleanCloseKernelSize = 5;    // Closing to fill small gaps
        int maxMorphologyKernelSize = 7; // Maximum kernel size for multi-scale morphology

        // Bull's eye enhancement
        int bullsEyeRegionSize = 8;   // Bull's eye region divisor (image_width/8)
        int bullsEyeDilateKernel = 5; // Dilation kernel for bull's eye enhancement

        // Ring connection enhancement
        int topRegionCloseKernel = 9; // Aggressive closing kernel for top region

        // Component size filtering
        int minLargeComponentSize = 100;    // Minimum area for large dartboard components
        int minBlobArea = 80;               // Minimum area for any blob to be considered
        double maxDistanceFromCenter = 0.6; // Maximum distance from center as ratio of image width

        // Text detection and filtering
        double textAspectRatioMin = 0.4; // Minimum aspect ratio for text detection
        double textAspectRatioMax = 2.5; // Maximum aspect ratio for text detection
        int textMaxArea = 500;           // Maximum area for components classified as text

        // Component geometric filtering
        double centralityThreshold = 0.25;    // Distance threshold for central components (25% of image)
        double bullsEyeThreshold = 0.1;       // Distance threshold for bull's eye area (10% of image)
        double connectivityThreshold = 0.075; // Distance threshold for connected components
        int minConnectedArea = 80;            // Minimum area to check for connectivity
        int minConnectedNeighborArea = 100;   // Minimum neighbor area for connectivity
        int largestAreaDivisor = 10;          // Divisor for largest area comparison (area > largest/10)
        int largestAreaRatio = 3;             // Ratio for large area comparison (area > largest/3)

        // Specific text filtering (targeting known problem areas)
        double edgeTextThreshold = 0.1;   // Distance from edge to classify as edge text (10% of image)
        int edgeTextMaxArea = 2000;       // Maximum area for edge text classification
        double bottomLeftTextX = 0.3;     // X threshold for bottom-left text detection (30% of width)
        double bottomLeftTextY = 0.7;     // Y threshold for bottom-left text detection (70% of height)
        double topRightTextX = 0.7;       // X threshold for top-right text detection (70% of width)
        double topRightTextY = 0.3;       // Y threshold for top-right text detection (30% of height)
        int positionalTextMaxArea = 3000; // Maximum area for positional text classification

        // Visualization parameters
        int nearbyColorCheckDistance = 2; // Pixel distance to check for nearby colors in visualization
    };

    // Input: roiFrame → Output: redGreenFrame (with red=bull, green=rings)
    Mat processColors(const Mat &roiFrame, int camera_idx = 0, bool debug_mode = false, const ColorParams &params = ColorParams());

    // Helper for when individual masks are needed
    pair<Mat, Mat> getIndividualColorMasks(const Mat &roiFrame, const ColorParams &params = ColorParams());
}
