#include "geometry_dartboard.hpp"
#include "geometry_detector.hpp" // For DartboardCalibration
#include <iostream>
#include <cmath>

// Define standard dartboard segment values (clockwise order)
const int GeometryDartboard::standardSegments[20] =
    {20, 1, 18, 4, 13, 6, 10, 15, 2, 17, 3, 19, 7, 16, 8, 11, 14, 9, 12, 5};

// Remove the drawCalibrationOverlay method - now in dartboard_visualization.hpp

std::string GeometryDartboard::calculateScore(
    const cv::Point &dartPosition,
    const DartboardCalibration &calib)
{

    // Dont currenlty work
    return "MISS";

    // -----

    // TODO: the fueture of this code is to calculate the score of a dart throw
    // Check if dart position is valid
}

std::string GeometryDartboard::formatScore(
    int value, bool isDouble, bool isTriple, bool isBull)
{
    if (isBull)
    {
        return "BULL";
    }
    if (isDouble)
    {
        return "D" + std::to_string(value);
    }
    if (isTriple)
    {
        return "T" + std::to_string(value);
    }
    return "S" + std::to_string(value);
}
