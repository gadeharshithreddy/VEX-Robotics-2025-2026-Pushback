#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cstddef>

// Brain Devices

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivetrain Motors
pros::MotorGroup left_motors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({20, 10, 9}, pros::MotorGearset::blue);

// Intake Motors
pros::Motor top_intake_motor(8);
pros::Motor bottom_intake_motor(7);

// Optical Sensor
// pros::Optical color_sensor(19);

// Pneumatics
pros::adi::Pneumatics middle_goal_mech('A', true);
pros::adi::Pneumatics loader_mech('B', false);

// IMU + Rotation Sensors
pros::Imu imu(16);
pros::Rotation horizontal_sensor(17);
pros::Rotation vertical_sensor(-15);

// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

// Lemlib Drivetrain Setup
lemlib::Drivetrain drivetrain(
	&left_motors,
	&right_motors,
	14,
	lemlib::Omniwheel::NEW_325,
	450,
	8
);

// Lemlib Tracking Wheel Setup
lemlib::TrackingWheel horizontal_tracking_wheel(
	&horizontal_sensor,
	lemlib::Omniwheel::NEW_2,
	-6.375
);
lemlib::TrackingWheel vertical_tracking_wheel(
	&vertical_sensor,
	lemlib::Omniwheel::NEW_2,
	1.75
);

// Lemlib Odometry Configuration
// Wrap drivetrain motor groups as tracking wheels (uses motor encoders)
// lemlib::TrackingWheel left_tracking_wheel(
//     &left_motors,
//     lemlib::Omniwheel::NEW_325, // wheel diameter — adjust if your wheels differ
//     -12.08 / 2.0,               // distance from tracking center (negative for left side)
//     450                         // drivetrain rpm — match drivetrain rpm in your Drivetrain
// );
// lemlib::TrackingWheel right_tracking_wheel(
//     &right_motors,
//     lemlib::Omniwheel::NEW_325,
//     12.08 / 2.0,                // positive for right side
//     450
// );

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr,
    &horizontal_tracking_wheel,
    nullptr,
    &imu
);

// Lemlib PID Tuning
lemlib::ControllerSettings lateral_controller(
	16.5, // proportional gain (kP)
	0, // integral gain (kI)
	40, // derivative gain (kD)
	0, // anti windup
	1, // small error range, in inches
	100, // small error range timeout, in milliseconds
	1.5, // large error range, in inches
	300, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	8, // proportional gain (kP)
    0, // integral gain (kI)
    40, // derivative gain (kD)
	0, // anti windup
	1, // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3, // large error range, in degrees
	300, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

// Controller Input Changing
lemlib::ExpoDriveCurve throttle_curve(
	3, // joystick deadband out of 127
	10, // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
	3, // joystick deadband out of 127
	10, // minimum output where drivetrain will move out of 127
	1.04 // expo curve gain
);

// Lemlib Chassis
lemlib::Chassis chassis(
	drivetrain,
	lateral_controller,
	angular_controller,
	sensors,
	&throttle_curve,
	&steer_curve
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    chassis.calibrate();
	pros::delay(2000);
    
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void scoreLoaderBlocks() {
	top_intake_motor.move(127);
	bottom_intake_motor.move(127);
	pros::delay(3500);
	top_intake_motor.move(0);
	bottom_intake_motor.move(0);
}

void autonomous() {
	// chassis.setPose(0, 0, 0);
	// bottom_intake_motor.move(127);
	// chassis.moveToPoint(0, 28, 2000, {.maxSpeed=60});
	// pros::delay(2000);
	// bottom_intake_motor.move(0);
	// chassis.turnToHeading(180, 500, {.maxSpeed=60});
	// chassis.turnToHeading(0, 500, {.maxSpeed=60});
	// chassis.turnToHeading(90, 3000);

	// Day 1 Testing. Able to pick up some blocks, but not all four
	// bottom_intake_motor.move(127);
	// chassis.moveToPoint(1, 23.25, 1000);
	// chassis.turnToHeading(-31.06, 500, {.maxSpeed=127});
	// chassis.moveToPose(-2.68, 31.41, -23.11, 7000, {.maxSpeed=40});
	// pros::delay(2000);
	// bottom_intake_motor.move(0);

	// Option 2 Code:
	chassis.setPose(0, 0, 0);
	// Part 1:
	// Going to first loader:
	// Could try .25 for x, 49.85 for x, theta: -0.46, moving forward for first loader
	chassis.moveToPoint(0, 50, 2000);
	// Turns to face first loader
	chassis.turnToHeading(90, 500); // 2.03, 52.07, 90.14
	
	// Collecting First loader blocks ----------------
	bottom_intake_motor.move(127);
	loader_mech.extend();
	// Rams into first loader
	chassis.moveToPose(9.25, 51.73, 90, 2000);
	pros::delay(2000);
	bottom_intake_motor.move(0);
	
	// Going back to retract loader mech
	chassis.moveToPoint(0, 50, 2000);
	loader_mech.retract();

	// Going to First Long Goal
	chassis.turnToHeading(0, 500);
	chassis.moveToPoint(1.4, 65, 500); // try 59.8 for y
	chassis.turnToHeading(-90, 500);
	// Moving straight
	chassis.moveToPoint(-92.5, 67, 4000); // -92 heading

	// Aligning with long goal
	chassis.moveToPose(-84.76, 51.67, -90, 2000);

	// Scoring blocks
	scoreLoaderBlocks();
	
	// // Part 2:
	// // Right before the second loader
	// chassis.moveToPoint(102, 52.46, 1000); // theta: -90

	// // Collecting blocks from the loader ------------------------
	// bottom_intake_motor.move(127);
	// loader_mech.extend();

	// chassis.moveToPose(-110.33, 52.52, -90, 2000);
	// bottom_intake_motor.move(0);
	
	// // Going back to retract loader mech
	// chassis.moveToPose(-84.34, 52.8, -90, 2000); // Theta: -88.2
	// loader_mech.retract();

	// // Scoring blocks
	// scoreLoaderBlocks();

	// // Getting ready to move to the third Loader
	// chassis.moveToPoint(-93.84, 55.76, 4000); // theta -180
	// chassis.turnToHeading(-180, 500);

	// // Moving + Turning to face loader
	// chassis.moveToPoint(-97.54, -44.79, 4000); // theta -177.7
	// chassis.turnToHeading(-90, 500); // -98.06, -49.13, -88.29

	// // Ramming into third loader to collect blocks --------------------------------
	// bottom_intake_motor.move(127);
	// loader_mech.extend();
	// // Rams into third loader
	// chassis.moveToPose(-115.32, -48.37, -90, 2000); // theta: -85.98
	// pros::delay(2000);
	// bottom_intake_motor.move(0);
	
	// // Going back to retract loader mech
	// chassis.moveToPoint(-103.51, -48.98, 2000);
	// loader_mech.retract();

	// // Part 3:
	// // Going to the second long goal
	// // Turning left
	// chassis.turnToHeading(-180, 500); // -103.87, -47.83, -176.84
	// // moving straight
	// chassis.moveToPoint(-105.14, -59.79, 1000); // theta -178.82
	// // turning left
	// chassis.turnToHeading(-270, 500); // -106.91, -61.64, -265.51
	// // Going straight to the other side
	// chassis.moveToPoint(-7.57, -68.92, 4000); // theta: -265.8
	// // Turning left
	// chassis.turnToHeading(0, 500); // -5.76, -57.89, -353.38
	// // moving forward
	// chassis.moveToPoint(-5.76, -57.89, 1000);
	// // Turning right ^ for x and y coordinates
	// chassis.turnToHeading(90, 500);

	// // Going to score the blocks in the second long goal
	// chassis.moveToPose(-20.85, -54.58, 90, 1000); // theta: -264.7

	// // Scoring the blocks
	// scoreLoaderBlocks();

	// // Right before loader
	// chassis.moveToPoint(0.73, -55.79, 1000); // theta -264.69

	// // Ramming into fourth loader to collect blocks --------------------
	// bottom_intake_motor.move(127);
	// loader_mech.extend();
	// // Rams into fourth loader
	// chassis.moveToPose(4.72, -56.52, 90, 2000); // theta: -264.65
	// pros::delay(2000);
	// bottom_intake_motor.move(0);
	
	// // Going back to retract loader mech
	// chassis.moveToPoint(-20.85, -54.58, 2000); // Could be right before loader as well
	// loader_mech.retract();

	// // Going to score the blocks in the second long goal for the second time
	// chassis.moveToPose(-20.85, -54.58, 90, 1000); // theta: -264.7

	// // Scoring the blocks
	// scoreLoaderBlocks();

	
	// // Traveling to Park the Robot, take the robot to location right before park
	// chassis.moveToPoint(-4.55, -57.35, 1000); // theta -263.22
	// chassis.turnToHeading(0, 500); // -4.03, -58.78, -353.62
	// chassis.moveToPoint(-1.11, -35.99, 1000); // theta -353.47
	// chassis.turnToHeading(90, 500); // -1.12, -36.92, -262.53
	// chassis.moveToPoint(10.61, -36.8, 1000); // theta -264.03
	// chassis.turnToHeading(0, 500); // 12.1, -38.84, -352.52

	// // Parking // Need to Figure it out.
	// bottom_intake_motor.move(127);
	// top_intake_motor.move(127);
	// chassis.moveToPoint(17, -16.76, 5000, {.maxSpeed=70}); // 15.97, -16.78, -350.94
	// chassis.turnToHeading(0, 500);
	// pros::delay(2000);
	// bottom_intake_motor.move(0);
	// top_intake_motor.move(0);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void showMovementTask() {
    char buffer[32];
    while (true) {
        auto pose = chassis.getPose();
        pros::lcd::print(0, "X: %.2f", pose.x);
        pros::lcd::print(1, "Y: %.2f", pose.y);
        pros::lcd::print(2, "Theta: %.2f", pose.theta);
        pros::delay(50);
    }
}

void opcontrol() {
	static pros::Task show_movement(showMovementTask);

	// autonomous();

	// Variables
	bool loader_mech_extended = false;
	bool middle_goal_mech_extended = true;
	bool spin_top_intake = false;
	bool spin_bottom_intake = false;
	bool spin_bottom_intake_backward = false;

	// loop forever
    while (true) {
        

		// Autonomous Testing
		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
		// 	chassis.setPose(0, 0, 0);
		// 	chassis.moveToPoint(0, 24, 5000);
		// }
		
		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
		// 	chassis.setPose(0, 0, 0);
		// 	chassis.turnToHeading(90, 2000);
		// }


		// Variable Control
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			// To spin the bottom intake
			spin_bottom_intake = !spin_bottom_intake;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
			// To spin both at the same time
			spin_top_intake = !spin_top_intake;
			spin_bottom_intake = spin_top_intake;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			// Loader Mechanism
			loader_mech_extended = !loader_mech_extended;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			// Middle Goal Mechanism
			middle_goal_mech_extended = !middle_goal_mech_extended;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			spin_bottom_intake_backward = !spin_bottom_intake_backward;
			spin_bottom_intake = false;
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			autonomous();
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			// chassis.setPose(0, 0, 0);
			// chassis.turnToHeading(90, 500);
			autonomous();
		}

		// Setting the variable values
		if (spin_top_intake) {
			top_intake_motor.move(127);
		} else {
			top_intake_motor.move(0);
		}

		if (spin_bottom_intake) {
			bottom_intake_motor.move(127);
		} else if (spin_bottom_intake_backward) {
			bottom_intake_motor.move(-127);
		} else {
			bottom_intake_motor.move(0);
		}

		if (middle_goal_mech_extended) {
			middle_goal_mech.extend();
		} else {
			middle_goal_mech.retract();
		}

		if (loader_mech_extended) {
			loader_mech.extend();
		} else {
			loader_mech.retract();
		}

		// get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
		// Gives more forward power in cost of turning speed
        chassis.arcade(leftY, rightX, true, 0.25);

        // delay to save resources
        pros::delay(25);
    }
}
