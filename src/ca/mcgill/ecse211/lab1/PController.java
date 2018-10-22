package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements the PController for our Wall Follower.
 * 
 * @author Caspar Cedro & Patrick Erath
 */
public class PController implements UltrasonicController {
  /* Constants */
  private static final int MOTOR_SPEED = 150;
  private static final int FILTER_OUT = 10;
  private static final int GAIN = 15;

  private final int bandCenter;
  private final int bandWidth;
  private int distance = 0;
  private int filterControl;

  /**
   * This constructor creates an instance of a PController that our Wall Follower uses to navigate.
   * 
   * @param bandCenter the distance from the wall that should be kept.
   * @param bandWidth the "deadband" distance that our Wall Follower can keep about the bandCenter.
   */
  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
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
    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    distance = (int) (distance * Math.sqrt(2) / 2); // first calculate our adjacent distance
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    int distErr = bandCenter - distance;
    int diff = Math.abs(7 * distErr); // Must be absolute otherwise reverse turns occur.
    if (diff > MOTOR_SPEED) {
      diff = (int) (MOTOR_SPEED / 3.5f); // max motor speed.
    }

    if (distance < 15) {
      // we're too close to the wall - turn right.
      WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
      WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
      WallFollowingLab.leftMotor.forward();
      WallFollowingLab.rightMotor.backward();
    } else {
      if (Math.abs(distErr) <= bandWidth - 1) {
        // case 1: Error in bounds, no correction, go forward
        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
      } else if (distErr > 0) {
        // case 2: positive error, move away from wall
        int motorHigh = MOTOR_SPEED + diff;
        int motorLow = MOTOR_SPEED - diff;
        WallFollowingLab.leftMotor.setSpeed(motorHigh);
        WallFollowingLab.rightMotor.setSpeed(motorLow);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
      } else if (distErr < 0) {
        // case 3: negative error, move towards wall
        int motorHigh = MOTOR_SPEED + diff;
        int motorLow = MOTOR_SPEED - diff;
        WallFollowingLab.leftMotor.setSpeed(motorLow);
        WallFollowingLab.rightMotor.setSpeed(motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
      }
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
    float averageDistance = 0.0f;
    for (int i = 0; i < distances.length; i++) {
      averageDistance += distances[i];
    }
    return (int) (averageDistance /= distances.length);
  }
}
