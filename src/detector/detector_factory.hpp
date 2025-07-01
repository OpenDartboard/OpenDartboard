#pragma once
#include "dart_detector.hpp"
#include "geometry/geometry_detector.hpp"
#include "utils.hpp"
#include <memory>

class DetectorFactory
{
public:
    static std::unique_ptr<DartDetector> createDetector(bool use_ai, bool debug_mode = false,
                                                        int target_width = 640, int target_height = 480)
    {
        if (use_ai)
        {
            log_info("Loading AI-based dart detector");
            // Placeholder for AI detector - will add later
            log_warning("AI detection not yet implemented, using geometry detector");
            return std::make_unique<GeometryDetector>(debug_mode, target_width, target_height);
        }
        else
        {
            log_info("Loading geometry-based dart detector");
            return std::make_unique<GeometryDetector>(debug_mode, target_width, target_height);
        }
    }
};
