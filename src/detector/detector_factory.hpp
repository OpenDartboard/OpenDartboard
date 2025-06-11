#pragma once
#include "dart_detector.hpp"
#include "geometry/geometry_detector.hpp"
#include <memory>
#include <iostream>

class DetectorFactory
{
public:
    static std::unique_ptr<DartDetector> createDetector(bool use_ai, bool debug_mode = false,
                                                        int target_width = 640, int target_height = 480)
    {
        if (use_ai)
        {
            std::cout << "Loading AI-based dart detector" << std::endl;
            // Placeholder for AI detector - will add later
            std::cerr << "AI detection not yet implemented, using geometry detector" << std::endl;
            return std::make_unique<GeometryDetector>(debug_mode, target_width, target_height);
        }
        else
        {
            std::cout << "Loading geometry-based dart detector" << std::endl;
            return std::make_unique<GeometryDetector>(debug_mode, target_width, target_height);
        }
    }
};
