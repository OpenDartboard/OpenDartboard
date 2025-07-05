#pragma once
#include "detector_interface.hpp"
#include "geometry/geometry_detector.hpp"
#include "utils.hpp"
#include <memory>

class DetectorFactory
{
public:
    static std::unique_ptr<DetectorInterface> createDetector(bool use_ai, bool debug_mode, int target_width, int target_height, int target_fps)
    {
        if (use_ai)
        {
            log_info("Loading AI-based dart detector");
            // Placeholder for AI detector - will add later
            log_warning("AI detection not yet implemented, using geometry detector");
            return std::make_unique<GeometryDetector>(debug_mode, target_width, target_height, target_fps);
        }
        else
        {
            log_info("Loading geometry-based dart detector");
            return std::make_unique<GeometryDetector>(debug_mode, target_width, target_height, target_fps);
        }
    }
};
