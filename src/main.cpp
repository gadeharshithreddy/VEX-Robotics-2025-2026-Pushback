#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"

// Brain Devices

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivetrain Motors
pros::MotorGroup left_motors({-11, -1, -2}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({7, 8, 20}, pros::MotorGearset::blue);

// Intake Motors
pros::Motor top_intake_motor(-3);
pros::Motor bottom_intake_motor(10);

// Optical Sensor
pros::Optical color_sensor(19);

// Pneumatics
pros::adi::Pneumatics middle_goal_mech('A', false);
pros::adi::Pneumatics loader_mech('B', true);

// IMU + Rotation Sensors
pros::Imu imu(5);
pros::Rotation horizontal_sensor(6);
pros::Rotation vertical_sensor(9);

// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

// Lemlib Drivetrain Setup
lemlib::Drivetrain drivetrain(
	&left_motors,
	&right_motors,
	12.08,
	lemlib::Omniwheel::OLD_325,
	450,
	8
);

// Lemlib Tracking Wheel Setup
lemlib::TrackingWheel horizontal_tracking_wheel(
	&horizontal_sensor,
	lemlib::Omniwheel::NEW_275,
	0
);
lemlib::TrackingWheel vertical_tracking_wheel(
	&vertical_sensor,
	lemlib::Omniwheel::NEW_275,
	0
);

// Lemlib Odometry Configuration
lemlib::OdomSensors sensors(
	&vertical_tracking_wheel,
	nullptr,
	&horizontal_tracking_wheel,
	nullptr,
	&imu	
);

// Lemlib PID Tuning
lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
	0, // integral gain (kI)
	3, // derivative gain (kD)
	0, // anti windup
	0, // small error range, in inches
	0, // small error range timeout, in milliseconds
	0, // large error range, in inches
	0, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
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
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
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
void autonomous() {}

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
void opcontrol() {
	// loop forever
    while (true) {
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