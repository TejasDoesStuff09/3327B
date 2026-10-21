#include "lift.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

// anonymous namespace: these constants are private to this file
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double degToRad = pi / 180.0;
constexpr double radToDeg = 180.0 / pi;

} // namespace

Lift::Lift(pros::MotorGroup* motors, LiftSettings settings) : motors(motors), settings(settings) {}

void Lift::tare() { motors->tare_position_all(); }

void Lift::moveInches(double inches, int velocity) {
    // the motors report degrees, average them in case the two sides are slightly different
    std::vector<double> positions = motors->get_position_all();
    if (positions.empty()) return; // no motors connected
    const double motorDegrees = std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();

    // a DR4B has two stages of arms that both lift, so the claw rises 2 * armLength * sin(angle)
    const double reach = 2.0 * settings.armLength;

    // motor degrees -> arm angle -> height
    const double currentAngle = settings.startingAngle + motorDegrees * settings.gearRatio;
    const double currentHeight = reach * std::sin(currentAngle * degToRad);

    // add the requested change and go back the other way: height -> arm angle -> motor degrees
    // sin can only be -1 to 1, so clamp to stop asking for a height the arm can't reach
    const double targetSin = std::clamp((currentHeight + inches) / reach, -1.0, 1.0);
    const double targetAngle = std::asin(targetSin) * radToDeg;
    const double targetMotorDegrees = (targetAngle - settings.startingAngle) / settings.gearRatio;

    motors->move_absolute(targetMotorDegrees, velocity);
}
