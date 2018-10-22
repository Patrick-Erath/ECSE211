package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {

	private Odometer odometer;
	private static double position[]; // array containing x, y and theta coordinates
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;

	private double xPos; // current x position of robot
	private double yPos; // current y position of robot
	private double thetaPos; // angle robot is currently positioned at
	private double dx;
	private double dy;

	public static final double WHEEL_RAD = 2.27;
	public static final double TRACK = 11.00;
	private static final int MOTOR_STRAIGHT = 170; // forward moving speed
	private static final int MOTOR_ROTATE = 100; // rotating speed
	private static final double TILE_SIZE = 30.48;

	private boolean navigating = true;

	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	public void run() {
		// map1
		// travelTo(0, 2);
		// travelTo(1, 1);
		// travelTo(2, 2);
		// travelTo(2, 1);
		// travelTo(1, 0);

		// map2
		// travelTo(1, 1);
		// travelTo(0, 2);
		// travelTo(2, 2);
		// travelTo(2, 1);
		// travelTo(1, 0);

		// map3
		 travelTo(1, 0);
		 travelTo(2, 1);
		 travelTo(2, 2);
		 travelTo(0, 2);
		 travelTo(1, 1);

		// map4
		//travelTo(0, 1);
	//	travelTo(1, 2);
		//travelTo(1, 0);
		//travelTo(2, 1);
		//travelTo(2, 2);
	}

	/**
	 * travelTo method causes robot to travel to a specified set of coordinates
	 * calls turnTo method
	 * 
	 * @param x
	 * @param y
	 * @return void
	 * 
	 */
	void travelTo(double x, double y) {
		navigating = true;

		position = odometer.getXYT(); // polls odometer for information
		xPos = position[0];
		yPos = position[1];
		thetaPos = position[2];

		dx = (x * TILE_SIZE) - xPos; // required change in x
		dy = (y * TILE_SIZE) - yPos; // required change in y

		thetaPos = thetaPos * Math.PI / 180; // converting angle to radians
		double thetaMin = Math.atan2(dx, dy) - thetaPos; // calculating angle
		double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2)); // calculating overall distance to travel

		turnTo(thetaMin); // calling turnTo method that specifies the minimum angle

		leftMotor.setSpeed(MOTOR_STRAIGHT);
		rightMotor.setSpeed(MOTOR_STRAIGHT);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

		leftMotor.stop(true);
		rightMotor.stop(true);
		navigating = false; // robot no longer navigating

	}

	/**
	 * turnTo method chooses the minimum angle and turns the robot to this angle
	 * called by travelTo method
	 * 
	 * @param theta
	 * @return void
	 */
	void turnTo(double theta) {

		// choosing minimum angle
		if (theta > Math.PI) {
			theta -= 2 * Math.PI; // ex: 200 - 360 = -160; angle of 160 degrees is smaller than an angle of 200
									// degrees
		} else if (theta < -Math.PI) {
			theta += 2 * Math.PI; // ex: -200 + 360 = 160
		}

		leftMotor.setSpeed(MOTOR_ROTATE); // setting rotating speed
		rightMotor.setSpeed(MOTOR_ROTATE); // setting rotating speed

		if (theta < 0) { // case 1: rotate theta degrees left
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, -(theta * 180) / Math.PI), false);

		} else { // case 2: rotate theta degrees right
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, (theta * 180) / Math.PI), false);
		}
	}

	/**
	 * isNavigating method returns true if travelTo or turnTo methods have been
	 * called by another thread
	 * 
	 * @return boolean
	 * @throws OdometerExceptions
	 */

	boolean isNavigating() throws OdometerExceptions {
		return navigating;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance. called by travelTo method
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
