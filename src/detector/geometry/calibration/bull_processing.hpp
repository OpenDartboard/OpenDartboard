#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace bull_processing
{
    // Simplified parameters for bull detection only
    struct BullParams
    {
        // Strategy A: Bull's eye detection parameters
        double minBullArea = 20.0;          // Minimum area for bull contour
        double maxBullAreaPercent = 0.0008; // Maximum area as percentage of total image (0.5%)
        double minCircularity = 0.5;        // Minimum circularity for bull detection

        // Size-based scoring parameters
        double idealBullAreaPercent = 0.0008; // Ideal bull area as percentage of image (0.08%)
        double sizeWeight = 0.3;              // Weight for size scoring in bull detection
        double maxSizePenalty = 0.5;          // Maximum penalty for oversized contours

        // Strategy B: Hough circle detection parameters
        double lowFeatureDensityThreshold = 0.05; // Threshold for low feature density
        double houghDp1 = 1.0;                    // DP for low density
        double houghDp2 = 1.2;                    // DP for high density
        int houghMinDistDivisor1 = 10;            // Min distance divisor for low density
        int houghMinDistDivisor2 = 8;             // Min distance divisor for high density
        int houghCannyThreshold1 = 70;            // Canny threshold for low density
        int houghCannyThreshold2 = 80;            // Canny threshold for high density
        int houghAccThreshold1 = 20;              // Accumulator threshold for low density
        int houghAccThreshold2 = 25;              // Accumulator threshold for high density
        int houghMinRadiusDivisor = 12;           // Minimum radius divisor
        double houghMaxRadiusDivisor = 1.5;       // Maximum radius divisor

        // Strategy C: Density map parameters
        double densityScoreBase = 0.3;     // Base score for density strategy
        double densityScoreDivisor = 20.0; // Divisor for density score calculation

        // Scoring parameters
        double minAcceptableScore = 0.5;    // Minimum score to accept result
        double circularityWeight = 0.4;     // Weight for circularity in bull detection (REDUCED to make room for size)
        double centralityWeight = 0.3;      // Weight for centrality in bull detection (REDUCED to make room for size)
        double centerProximityWeight = 0.4; // Weight for center proximity in circle detection
        double rednessWeight = 0.6;         // Weight for redness in circle detection
    };

    // Just find the bull center
    Point processBull(
        const Mat &redGreenFrame,
        const Point &frameCenter,
        int camera_idx = 0,
        bool debug_mode = false,
        const BullParams &params = BullParams());

} // namespace bull_processing
