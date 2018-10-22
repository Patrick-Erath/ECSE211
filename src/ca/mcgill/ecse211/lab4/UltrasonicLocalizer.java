package ca.mcgill.ecse211.lab4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;

	private int distance = 0;
	private int filterControl = 0;

	public static final int FILTER_OUT = 30;
	public static final int WALL_DISTANCE = 40;
	public static final int WALL_DISTANCE_RISING = 30;
	public static final int MOTOR_ROTATE = 100;
	public static final double WHEEL_RAD = 2.27;
	public static final double TRACK = 10.50;

	/**
	 * UltrasonicLocalizer constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 * @throws OdometerExceptions
	 */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) throws OdometerExceptions {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		this.usSensor = ultrasonicSensor.getMode("Distance");
		this.usData = new float[usSensor.sampleSize()];
	}

	/**
	 * run method calls either falling or rising edge
	 */
	public void run() {
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}

		if (Lab4.wall) { // when wall = true, call falling edge method
			fallingedge();
		} else if (!Lab4.wall) { // when wall = false, call falling edge method
			risingedge();
		}

	}

	/**
	 * falling edge method called by run method positions robot at angle of 0
	 * degrees
	 * falling edge method called when the robot is originally not facing a wall
	 */
	public void fallingedge() {
		double theta;
		sensorValue();

		if (this.distance < 100) {
			// rotates counterclockwise until distance from wall is bigger than 100
			while (this.distance < 100) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			// rotates counterclockwise until distance from wall is smaller than 40
			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			odometer.setTheta(0); // sets the value of the angle to 0

			// rotates clockwise until distance is greater than 100
			while (this.distance < 100) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			// rotates clockwise until distancce small than 40
			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			theta = odometer.getXYT()[2]; // record angle reading
		}

		else { // distance >= 100; sensor far from wall
				// rotates counterclockwise until distance smaller than 40
			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			odometer.setTheta(0); // sets the value of the angle to 0

			// rotates clockwise until distance greater than 100
			while (this.distance < 100) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			// rotates clockwise until distance smaller than 40
			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			theta = odometer.getXYT()[2]; // record angle reading
		}

		// turns to adjust position; results in robot positioned at
		// angle = 0 degrees
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 49 + theta / 2.0), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 49 + theta / 2.0), false);
		odometer.setTheta(0);
		leftMotor.stop();
		rightMotor.stop();
	}

	/**
	 * rising edge method called by run method positions robot at angle of 0 degrees
	 * rising edge method is called when the robot is originally facing a wall
	 */
	public void risingedge() {
		double theta;
		sensorValue();
		if (this.distance > WALL_DISTANCE_RISING) {

			// rotates counterclowise until distance smaller than 30
			while (this.distance > WALL_DISTANCE_RISING) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			// rotates counterclowise until distance greater than 40
			while (this.distance < WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			odometer.setTheta(0); // sets angle to 0

			// rotates clockwise until distance smaller than 30
			while (this.distance > WALL_DISTANCE_RISING) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			// rotates clockwise until distance greater than 40
			while (this.distance < WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			theta = odometer.getXYT()[2]; // record angle reading

		} else {
			// rotates counterclockwise until distance greater than 40
			while (this.distance < WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.backward();
				rightMotor.forward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();
			odometer.setTheta(0); // set angle to 0

			// rotates clockwise until distance smaller than 30
			while (this.distance > WALL_DISTANCE_RISING) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			// rotates clockwise until distance greater than 40
			while (this.distance < WALL_DISTANCE) {
				leftMotor.setSpeed(MOTOR_ROTATE);
				rightMotor.setSpeed(MOTOR_ROTATE);
				leftMotor.forward();
				rightMotor.backward();
				sensorValue();
			}
			leftMotor.stop();
			rightMotor.stop();

			theta = odometer.getXYT()[2]; // record angle reading
		}
		// turns to adjust position; results in robot positioned at angle = 0
		// degrees
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta / 2.0 - 49), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta / 2.0 - 49), false);
		odometer.setTheta(0);
		leftMotor.stop();
		rightMotor.stop();
	}

	private void sensorValue() {
		usSensor.fetchSample(usData, 0);
		int dist = (int) Math.abs(usData[0] * 100.0);
		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (dist >= 255 && filterControl < FILTER_OUT) {
			filterControl++;
		} else if (dist >= 255) {
			this.distance = dist;
		} else {
			filterControl = 0;
			this.distance = dist;
		}
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance. called by leftTurn, rightTurn methods
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
