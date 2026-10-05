#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// TODO: fill in ports, dimensions and PID values below.
// Negative port numbers reverse that motor.

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup left_motor_group({}, pros::MotorGears::green);  // TODO: left ports
pros::MotorGroup right_motor_group({}, pros::MotorGears::green); // TODO: right ports

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,
                              &right_motor_group,
                              0,                      // TODO: track width (inches)
                              lemlib::Omniwheel::NEW_4, // TODO: wheel type
                              0,                      // TODO: drivetrain rpm
                              2                       // horizontal drift (LemLib default; tune later)
);

// inertial sensor
pros::Imu imu(0); // TODO: IMU port

// odometry settings (no tracking wheels yet: drive motors + IMU only)
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu);

// lateral PID controller (LemLib example values, not tuned)
lemlib::ControllerSettings lateral_controller(10,  // kP
                                              0,   // kI
                                              3,   // kD
                                              3,   // anti windup
                                              1,   // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20   // maximum acceleration (slew)
);

// angular PID controller (LemLib example values, not tuned)
lemlib::ControllerSettings angular_controller(2,   // kP
                                              0,   // kI
                                              10,  // kD
                                              3,   // anti windup
                                              1,   // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * All other competition modes are blocked by initialize.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate sensors

	// print robot pose to the brain screen
	pros::Task screen_task([&]() {
		while (true) {
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			pros::delay(20);
		}
	});
}

/**
 * Runs while the robot is disabled, either from the Field Management System
 * or the VEX Competition Switch, following either autonomous or opcontrol.
 */
void disabled() {}

/**
 * Runs after initialize() when connected to the Field Management System or
 * the VEX Competition Switch. Exits when the robot is enabled.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code.
 */
void autonomous() {
	// TODO: autonomous routines
}

/**
 * Runs the operator control code (driver control).
 */
void opcontrol() {
	while (true) {
		// get joystick positions
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		// arcade drive
		chassis.arcade(leftY, rightX);

		// delay to save resources
		pros::delay(20);
	}
}
