#pragma once
#include <vector>
#include <optional>
#include <cmath>

// ---------------------------------------------------------------------------
// Conventions
//   Units:           all lengths in inches, all angles in radians.
//   Field frame:     origin at the center of the field; the walls sit at about +/-70.2 in.
//                    +x points right and +y points up (toward the 0 degree wall) in the
//                    audience-view field drawings.
//   Pose.theta:      0 points along +x, positive is counter-clockwise.
//   Robot frame:     origin at the bot center; a = forward, b = left.
//   SensorPos.alpha: the sensor's facing angle relative to the robot's heading
//                    (counter-clockwise, 0 = straight ahead).
//   SensorPos.h:     height of the sensor above the floor.
// ---------------------------------------------------------------------------

// inches, radians, since std::cos and std::sin radians.
struct Pose {
    double x, y, theta;
    Pose operator+(const Pose& other) const {   // adds x and y coords; takes theta from the right-hand side (other)
        return {x + other.x, y + other.y, other.theta};
    }
};

// using coordinate segments to construct the field.
struct Segment {
    double x1, y1, x2, y2;
};

// sensor offsets, measured from bot center.
struct SensorPos {
    double a, b, alpha, h;
};

// in order to input corners for the polygon function.
struct Point {
    double x, y;
};

struct SensorModel {
    SensorPos pos;
    std::vector<Segment> map;  // a particular sensor's internal map of the field at their height.
};

// helper functions, to construct segments for the map:

// helper function to construct a polygon from its corners.
// will be helpful in constructing maps of the goals.
// corners must be listed in order around the shape (either direction)
// returns empty list if fewer than three corners are given
std::vector<Segment> constructPolygon(const std::vector<Point>& corners);

// function that returns a list of segments. That list of segments is the field map.
std::vector<Segment> constructFieldMap(double sensorHeight);

// EKF only needs p(z|x), where z is the sensor's noisy reading and x is a position.
// This computes h(x), the theoretic reading at position x.
// We will later fit a noisy distribution around h(x) to compute p(z|x)
//   inside: return std::nullopt;  when nothing is in range
//           return best;          when there's a hit
std::optional<double> expectedDistance(const Pose& p, const SensorModel& sensor);

//trig helper methods so there's less clutter
inline const double cosine(const double theta) {
    return std::cos(theta);
}
inline const double sine(const double theta) {
    return std::sin(theta);
}