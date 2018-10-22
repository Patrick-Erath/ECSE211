package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

/**
 * This class implements the BangBangController for our Wall Follower.
 * 
 * @author Caspar Cedro & Patrick Erath
 */
public class BangBangController implements UltrasonicController {
  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  private int filterControl = 0;
  private static final int FILTER_OUT = 20;

  /**
   * This constructor creates an instance of a BangBangController that our Wall Follower uses to
   * navigate.
   * 
   * @param bandCenter the distance from the wall that should be kept.
   * @param bandWidth the "deadband" distance that our Wall Follower can keep about the bandCenter.
   * @param motorLow the speed that the slower motor on the Wall Follower should spin at.
   * @param motorHigh the speed that the faster motor on the Wall Follower should spin at.
   */
  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  /**
   * This method takes and uses the latest distance value obtained from our ultrasonic sensor to
   * control the motors on the Wall Follower.
   * 
   * @param distance the distance from an obstruction that the Wall Follower is currently at.
   */
  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)

    distance = (int) (distance * Math.sqrt(2) / 2); // taking into account that ultrasonic sensor is
                                                    // at a 45 degree

    if (distance < this.bandCenter - bandwidth - 2) { // too CLOSE to the wall
      // turn RIGHT
      WallFollowingLab.leftMotor.setSpeed(this.motorHigh + 100);
      WallFollowingLab.rightMotor.setSpeed(this.motorLow + 10);
      // Making the left wheel go forward and the right wheel backwards
      // This makes for faster turning right and allows for better avoiding of the wall
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.backward();
    } else if (distance > this.bandCenter + bandwidth) { // too FAR from the wall
      // turn left
      // Making the right motor turn faster than the left motor
      WallFollowingLab.leftMotor.setSpeed(this.motorLow);
      WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
    } else {
      // ceteris paribus go straight
      WallFollowingLab.leftMotor.setSpeed(this.motorHigh);
      WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.forward();
    }
  }

  /**
   * This method returns the current distance that our Wall Follower is at from an obstruction.
   * 
   * @param distance the current distance that our Wall Follower is at from an obstruction.
   */
  @Override
  public int readUSDistance() {
    return this.distance;
  }

  /**
   * This method returns the average of a number of distances obtained from our ultrasonic sensor
   * over a period of time (currently unused).
   * 
   * @param distances an array of distance values that are obtained over a period of time.
   */
  @Override
  public int getAverageDistance(float[] distances) {
    // TODO Auto-generated method stub
    return 0;
  }
}
