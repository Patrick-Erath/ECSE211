/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class implements correction for the odometry on our robot using a light sensor.
 * 
 * @author Caspar Cedro & Patrick Erath
 */
public class OdometryCorrection implements Runnable {
  private SampleProvider colorSensor;
  private float[] lightData;
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   * @param colorSensor a SampleProvider instance that provides sample readings from the light
   *        sensor defined in Lab2.java
   */
  public OdometryCorrection(SampleProvider colorSensor) throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.colorSensor = colorSensor;
    this.lightData = new float[colorSensor.sampleSize()];
  }

  /**
   * Here is where the odometer correction code should be run. When we run a black line we want to
   * know whether we should reset our x and y coordinates.
   * 
   * @throws OdometerExceptions
   */
  public void run() {
    long correctionStart, correctionEnd;
    int numberOfLines = 0;
    boolean aboveLine = false;

    while (true) {
      correctionStart = System.currentTimeMillis();
      colorSensor.fetchSample(lightData, 0);

      // TODO Trigger correction (When do I have information to correct?)
      // MeanFilter only provides 1 sample - not need to average (based on println results).
      /*
       * float avg = 0.0f; //System.out.println(lightData.length); for(int i = 0; i <
       * lightData.length; i++) { avg += lightData[i]; } avg /= lightData.length;
       */
      // System.out.println(avg);

      if (lightData[0] < 0.25 && !aboveLine) {
        Sound.beep();
        numberOfLines++;
        aboveLine = true;

        // We only reset relative to the 0,0 position on the grid.
        // Everything else accumulates over time.
        switch (numberOfLines) {
          case 1:
            odometer.setY(0);
            break;
          case 4:
            odometer.setX(0);
            break;
          case 9:
            odometer.setY(0);
            break;
          case 12:
            odometer.setX(0);
            break;
          default:
            break;
        }

        // TODO Update odometer with new calculated (and more accurate) vales
        // odometer.setXYT(X, Y, Theta);
      } else if (lightData[0] >= 0.25) {
        aboveLine = false;
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
