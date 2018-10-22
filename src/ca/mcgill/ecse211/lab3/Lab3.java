package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.wallfollower.PController;
import ca.mcgill.ecse211.wallfollower.UltrasonicController;
import ca.mcgill.ecse211.wallfollower.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * This class implements the main starting point for the Navigation and Obstacle Avoidance lab
 * 
 * @author Caspar Cedro & Patrick Erath
 */
public class Lab3 {
  /**
   * This variable stores the index of the track we currently want to navigate
   */
  public static int index = 0;

  /**
   * Motor object instance that allows control of the left motor connected to port B
   */
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * Motor object instance that allows control of the right motor connected to port A
   */
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * TextLCD object instance that allows text to be shown on the EV3 brick's display
   */
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();

  /**
   * A Port object instance that accesses the light sensor connected to Port S2
   */
  // public static final Port lightSensorPort = LocalEV3.get().getPort("S2");

  /**
   * A Port object instance that accesses the ultrasonic sensor connected to Port S1
   */
  public static final Port usPort = LocalEV3.get().getPort("S1");

  /**
   * An UltrasonicPoller object instance that polls our ultrasonic sensor
   */
  public static UltrasonicPoller usPoller;

  /**
   * A PController object instance that is currently unused
   */
  public static PController pController;

  /**
   * A Navigation object instance that allows our robot to travel over a predefined map
   */
  public static Navigation nav;

  /**
   * An Odometer object instance that keeps track of wheel rotations
   */
  public static Odometer odometer;

  /**
   * A SensorModes object instance that controls what mode our ultrasonic sensor is currently in
   */
  public static SensorModes usSensor;

  /**
   * An array of arrays which contain (x,y) coordinates for maps 1-4 as defined in Lab 3's PDF
   * Description
   */
  public static final int[][][] waypoints =
      {{{0, 2}, {1, 1}, {2, 2}, {2, 1}, {1, 0}}, {{1, 1}, {0, 2}, {2, 2}, {2, 1}, {1, 0}},
          {{1, 0}, {2, 1}, {2, 2}, {0, 2}, {1, 1}}, {{0, 1}, {1, 2}, {1, 0}, {2, 1}, {2, 2}}};

  /**
   * This variable denotes the radius of our wheels in cm.
   */
  public static final double WHEEL_RAD = 2.1;

  /**
   * This variable denotes the track distance between the center of the wheels in cm (measured and
   * adjusted based on trial and error).
   */
  // 11.6
  public static final double WB = 12;

  private static final int bandCenter = 10; // Offset from the wall (cm)
  private static final int bandWidth = 3; // Width of dead band (cm)

  /**
   * This method is our main entry point - instantiate objects used and set up sensor.
   * 
   * @param args an array of arguments that can be passed in via commandline or otherwise.
   */
  public static void main(String[] args) throws OdometerExceptions {
    // Light sensor related objects
    // SensorModes lightSensorMode = new EV3ColorSensor(lightSensorPort);
    // SampleProvider lightSensor = lightSensorMode.getMode("Red");
    // SampleProvider lightSensorMean = new MeanFilter(lightSensor, 5);

    // PController
    pController = new PController(bandCenter, bandWidth);

    // Ultrasonic sensor related objects
    usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // this instance
    float[] usData = new float[usDistance.sampleSize()];

    usPoller = new UltrasonicPoller(usSensor, usData, pController);

    // Odometer related objects
    // Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, WB, WHEEL_RAD);

    // Odometer correction...
    // OdometryCorrection odometryCorrection = new OdometryCorrection(lightSensorMean);

    odometer = new Odometer(leftMotor, rightMotor);
    odometer.start();
    Display odometryDisplay = new Display(lcd);
    usPoller.start();

    nav = new Navigation(leftMotor, rightMotor, lcd, odometer, usPoller, pController);
    nav.start();

    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();

    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      if (Button.waitForAnyPress() == Button.ID_LEFT) {
        doCourse();
      } else if (Button.waitForAnyPress() == Button.ID_RIGHT) {
        odometer.setX(0);
        odometer.setY(0);
        odometer.setAng(90);
        if (index < waypoints.length - 1) {
          index++;
        } else {
          index = 0;
        }
      }
    }
    System.exit(0);
  }

  /**
   * This method makes our robot travel a predefined map of (x,y) coordinates
   */
  public static void doCourse() {
    for (int[] point : waypoints[index]) {
      nav.travelTo((double) point[0] * 30.48d, (double) point[1] * 30.48d, true);
      while (nav.isTravelling()) {
        try {
          Thread.sleep(500);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }
}
