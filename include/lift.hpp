#pragma once
#include "pros/motor_group.hpp"

/**
 * @brief Measurements that describe a double reverse four-bar (DR4B) lift
 *
 * These change from robot to robot, so they live in a struct instead of being hardcoded.
 */
struct LiftSettings {
    // length of one arm, pivot to pivot, in inches
    double armLength;
    // degrees the arm turns per degree the motor turns (output teeth / input teeth, so 12 -> 36 is 12 / 36)
    double gearRatio;
    // arm angle above horizontal when the motor reads zero, in degrees
    double startingAngle;
};

class Lift {
    public:
        /**
         * @brief Lift constructor
         *
         * @param settings the measurements of this lift, see LiftSettings
         * @param motors pointer to the motor group that drives the lift. The group is owned by the
         * caller (usually a global in main.cpp), the Lift only borrows it, just like lemlib::Drivetrain
         *
         * @b Example
         * @code {.cpp}
         * pros::MotorGroup liftMotors({5, -6}, pros::MotorGears::red);
         * Lift lift(&liftMotors, {12, 12.0 / 36.0, 30}); // armLength, gearRatio, startingAngle
         * @endcode
         */
        Lift(pros::MotorGroup* motors, LiftSettings settings);

        /**
         * @brief Set the current lift position as zero (the lowest position)
         *
         * Call this in initialize() with the lift resting at the bottom.
         */
        void tare();

        /**
         * @brief Move the lift up or down by a number of inches
         *
         * @param inches distance to move, positive is up and negative is down
         * @param velocity max motor velocity in RPM, 0 to 200 for a red cartridge
         *
         * @b Example
         * @code {.cpp}
         * lift.moveInches(6);  // up 6 inches
         * lift.moveInches(-6); // back down
         * @endcode
         */
        void moveInches(double inches, int velocity = 100);

    private:
        // pointer, not a copy: the motors belong to main.cpp and we only borrow them
        pros::MotorGroup* motors;
        // our own copy, it is small and nobody else needs to change it
        LiftSettings settings;
};
