package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class LightLocalizer implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private SampleProvider color;
	private float[] lsData;
	private float colourIntensity;

	private static final int travelDistance = 13;
	private double TRACK;
	private double WHEEL_RAD;

	/**
	 * LightLocalizer constructor
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

		EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
		color = lightSensor.getMode("Red");
		lsData = new float[lightSensor.sampleSize()];
	}

	/** 
	 * run method
	 * calls localization method
	 */
	public void run() {
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}
		localization();
	}
	
/**
 * localization method replaces the robot at the origin
 * @return void
 */
	public void localization() {
		sensorValue();
		// color.fetchSample(lsData, 0);
		// this.colourIntensity = lsData[0];

		while (colourIntensity > 0.3) { // advance forward as long as sensor is on the wooden tiles
			leftMotor.forward();
			rightMotor.forward();
			sensorValue();
			// color.fetchSample(lsData, 0);
			// this.colourIntensity = lsData[0];
		}
		// sensor has detected black line
		leftMotor.stop(true);
		rightMotor.stop(false);

		// turn 90 degrees clockwise
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
		sensorValue();
		// color.fetchSample(lsData, 0);
		// this.colourIntensity = lsData[0];

		while (colourIntensity > 0.3) { // advance forward as long as sensor is on the wooden tiles
			leftMotor.forward();
			rightMotor.forward();
			sensorValue();
			// color.fetchSample(lsData, 0);
			// this.colourIntensity = lsData[0];
		}
		// sensor has detected black line
		leftMotor.stop(true);
		rightMotor.stop(false);

		// move 13cm backwards
		leftMotor.rotate(convertDistance(WHEEL_RAD, -travelDistance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -travelDistance), false);
		leftMotor.stop(true);
		rightMotor.stop(false);

		// turn 90 degrees counterclockwise
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);

		leftMotor.stop(true);
		rightMotor.stop(false);

		// move 13cm backwards
		leftMotor.rotate(convertDistance(WHEEL_RAD, -travelDistance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -travelDistance), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * method gets intensity of reflected light measured by color sensor black
	 * @return void
	 */
	public void sensorValue() {
		color.fetchSample(lsData, 0);
		this.colourIntensity = lsData[0];
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
