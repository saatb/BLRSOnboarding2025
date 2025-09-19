#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-1});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor intake(16);
pros::adi::AnalogIn line_tracker('A');
pros::Imu imu(10);
pros::Distance dist(9);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	line_tracker.calibrate();

	pros::lcd::register_btn1_cb(on_center_button);
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
	bool turnLeft = true;
	int defaultSpeed = 20;
	int turnSpeed = 20;
	bool reversed = false;
	bool lineTracking = true;

	unsigned long startTime = pros::millis();
	
	left_mg.move(80);
	right_mg.move(80);
	pros::delay(500);
	while (true) {
		master.print(0,0, ": %.2d", dist.get_distance());

		// --- line-tracking runs every loop ---
		if (lineTracking){
		if (line_tracker.get_value() < 2750) {
			left_mg.move(80);
			right_mg.move(80);
		} else {
			if (turnLeft) {
				left_mg.move(turnSpeed);
				right_mg.move(-turnSpeed);
			} else {
				left_mg.move(-turnSpeed);
				right_mg.move(turnSpeed);
			}

			if (imu.get_heading() <= 325 && imu.get_heading() >= 180) {
				turnLeft = false;
			} else if (imu.get_heading() >= 35 && imu.get_heading() < 180) {
				turnLeft = true;
			}
		}
	}
		// timed amount - avoid false positives on the legs of the goal
		if (pros::millis() - startTime >= 20000 && dist.get_distance() < 200) {
			lineTracking = false;
			while (dist.get_distance() > 20){
				intake.move_velocity(-150);
				left_mg.move(80);
				right_mg.move(80);
				pros::delay(10);
			}

			left_mg.move(0);
			right_mg.move(0);
			intake.move_velocity(-50); //keep ball in intake
			startTime = pros::millis();
		}

		if (dist.get_distance() < 20) { //if we have a ball, begin coming back
			if (!reversed) {
				while ((imu.get_heading() < 170) || (imu.get_heading() > 190)) { //if not turned around, turn around!
					left_mg.move(turnSpeed);
					right_mg.move(-turnSpeed);
				}
				reversed = true;
				intake.move_velocity(0);
				left_mg.move(127);
				right_mg.move(127);
				pros::delay(1500);
				intake.move_velocity(200);
			}
		}
		pros::delay(10);
	}
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
void opcontrol() {
	while (true) {
		master.print(0,0, ": %.8d", line_tracker.get_value());

		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update

		if (master.get_digital(DIGITAL_A)) {
			intake.move(-100);
		} else if (master.get_digital(DIGITAL_B)) {
			intake.move(100);
		} else {
			intake.move(0);
		}
	}
}