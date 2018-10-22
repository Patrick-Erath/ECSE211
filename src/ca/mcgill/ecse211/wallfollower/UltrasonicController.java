package ca.mcgill.ecse211.wallfollower;

/**
 * This interface contains definitions common to the BangBangController and PController for our Wall
 * Follower.
 * 
 * @author Caspar Cedro & Patrick Erath
 */
public interface UltrasonicController {

  /**
   * This method takes and uses the latest distance value obtained from our ultrasonic sensor to
   * control the motors on the Wall Follower.
   * 
   * @param distance the distance from an obstruction that the Wall Follower is currently at.
   */
  public void processUSData(int distance);

  /**
   * This method returns the current distance that our Wall Follower is at from an obstruction.
   * 
   * @param distance the current distance that our Wall Follower is at from an obstruction.
   */
  public int readUSDistance();

  /**
   * This method returns the average of a number of distances obtained from our ultrasonic sensor
   * over a period of time (currently unused).
   * 
   * @param distances an array of distance values that are obtained over a period of time.
   */
  public int getAverageDistance(float distances[]);

}
