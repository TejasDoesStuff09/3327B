#include "lift.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// anonymous namespace: these constants are private to this file
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double degToRad = pi / 180.0;
constexpr double radToDeg = 180.0 / pi;

// TODO: placeholders until the lift is built, measure these
// length of one arm segment, in inches
constexpr double armLength = 12.0;
// how many arm segments are stacked, each one adds to the height (a plain arm is 1)
constexpr double stages = 2.0;
// degrees the arm turns per degree the motor turns (output teeth / input teeth, inverted)
// example: a 12 tooth gear driving a 36 tooth gear gives 12 / 36
constexpr double motorToArmRatio = 1.0;
// arm angle above horizontal when the motor reads zero, in degrees
constexpr double startingAngle = 30.0;

} // namespace

Lift::Lift(pros::MotorGroup* motors) : motors(motors) {}

void Lift::tare() { motors->tare_position_all(); }

void Lift::moveInches(double inches, int velocity) {
    // the motors report degrees, average them in case the two sides are slightly different
    std::vector<double> positions = motors->get_position_all();
    if (positions.empty()) return; // no motors connected
    const double motorDegrees = std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();

    // motor degrees -> arm angle -> height above the starting height
    const double currentAngle = startingAngle + motorDegrees * motorToArmRatio;
    const double currentHeight = stages * armLength * std::sin(currentAngle * degToRad);

    // add the requested change and go back the other way: height -> arm angle -> motor degrees
    // sin can only be -1 to 1, so clamp to stop asking for a height the arm can't reach
    const double targetSin = std::clamp((currentHeight + inches) / (stages * armLength), -1.0, 1.0);
    const double targetAngle = std::asin(targetSin) * radToDeg;
    const double targetMotorDegrees = (targetAngle - startingAngle) / motorToArmRatio;

    motors->move_absolute(targetMotorDegrees, velocity);
}
