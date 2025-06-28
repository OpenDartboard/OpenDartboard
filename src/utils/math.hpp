#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;
using namespace cv;

namespace math
{
    // Calculates Euclidean distance between two points
    // Returns the distance as a double
    inline double distanceToPoint(const Point &p1, const Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    // Calculates intersection point of a ray with an ellipse
    // Returns the intersection point on the ellipse boundary
    // If no intersection exists, returns a fallback point using the larger ellipse axis
    inline Point2f intersectRayWithEllipse(Point2f rayOrigin, float rayAngle, RotatedRect ellipse)
    {
        // Convert angle to direction vector
        Point2f direction(cos(rayAngle), sin(rayAngle));

        // Get ellipse parameters
        Point2f ellipseCenter = ellipse.center;
        float a = ellipse.size.width / 2.0f;                 // Semi-major axis
        float b = ellipse.size.height / 2.0f;                // Semi-minor axis
        float ellipseAngle = ellipse.angle * CV_PI / 180.0f; // Convert to radians

        // Transform ray to ellipse coordinate system
        Point2f relativeOrigin = rayOrigin - ellipseCenter;

        // Rotate to align with ellipse axes
        float cos_theta = cos(-ellipseAngle);
        float sin_theta = sin(-ellipseAngle);

        float x0 = relativeOrigin.x * cos_theta - relativeOrigin.y * sin_theta;
        float y0 = relativeOrigin.x * sin_theta + relativeOrigin.y * cos_theta;

        float dx = direction.x * cos_theta - direction.y * sin_theta;
        float dy = direction.x * sin_theta + direction.y * cos_theta;

        // Solve quadratic equation for ray-ellipse intersection
        // Ray: (x0 + t*dx, y0 + t*dy)
        // Ellipse: (x/a)² + (y/b)² = 1
        float A = (dx * dx) / (a * a) + (dy * dy) / (b * b);
        float B = 2.0f * ((x0 * dx) / (a * a) + (y0 * dy) / (b * b));
        float C = (x0 * x0) / (a * a) + (y0 * y0) / (b * b) - 1.0f;

        float discriminant = B * B - 4 * A * C;

        if (discriminant < 0)
        {
            // No intersection, fallback to fixed radius
            float radius = std::max(a, b) * 0.9f;
            return rayOrigin + direction * radius;
        }

        // Take the positive intersection (going outward from origin)
        float t1 = (-B + sqrt(discriminant)) / (2 * A);
        float t2 = (-B - sqrt(discriminant)) / (2 * A);
        float t = (t1 > 0) ? t1 : t2;

        if (t <= 0)
        {
            // Intersection behind origin, use fallback
            float radius = std::max(a, b) * 0.9f;
            return rayOrigin + direction * radius;
        }

        // Calculate intersection point in ellipse coordinates
        float x_intersect = x0 + t * dx;
        float y_intersect = y0 + t * dy;

        // Transform back to original coordinate system
        float x_final = x_intersect * cos_theta + y_intersect * sin_theta + ellipseCenter.x;
        float y_final = -x_intersect * sin_theta + y_intersect * cos_theta + ellipseCenter.y;

        return Point2f(x_final, y_final);
    }

    // Calculate the center point of a frame/image
    inline Point calculateFrameCenter(const Mat &frame)
    {
        return Point(frame.cols / 2, frame.rows / 2);
    }

    // Additional math utilities can be added here in the future
    // For example:
    // - angle between points
    // - vector operations
    // - geometric transformations
}