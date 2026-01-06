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
	0, // small error range, in inches
	0, // small error range timeout, in milliseconds
	0, // large error range, in inches
	0, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	8, // proportional gain (kP)
    0, // integral gain (kI)
    40, // derivative gain (kD)
	0, // anti windup
	0, // small error range, in degrees
	0, // small error range timeout, in milliseconds
	0, // large error range, in degrees
	0, // large error range timeout, in milliseconds
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
void autonomous() {
	chassis.setPose(0, 0, 0);
	// bottom_intake_motor.move(127);
	// chassis.moveToPoint(0, 28, 2000, {.maxSpeed=60});
	// pros::delay(2000);
	// bottom_intake_motor.move(0);
	// chassis.turnToHeading(180, 500, {.maxSpeed=60});
	// chassis.turnToHeading(0, 500, {.maxSpeed=60});
	chassis.turnToHeading(90, 3000);
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
