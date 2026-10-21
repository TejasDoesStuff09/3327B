#pragma once
#include "pros/motor_group.hpp"

class Lift {
    public:
        /**
         * @brief Lift constructor
         *
         * @param motors pointer to the motor group that drives the lift. The group is owned by the
         * caller (usually a global in main.cpp), the Lift only borrows it, just like lemlib::Drivetrain
         *
         * @b Example
         * @code {.cpp}
         * pros::MotorGroup liftMotors({5, -6}, pros::MotorGears::red);
         * Lift lift(&liftMotors);
         * @endcode
         */
        Lift(pros::MotorGroup* motors);

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
};
