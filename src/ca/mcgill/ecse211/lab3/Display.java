package ca.mcgill.ecse211.lab3;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.wallfollower.PController;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {
  private Odometer odo;
  private TextLCD lcd;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * This is the class constructor
   * 
   * @param lcd A TextLCD object instance to control
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd) throws OdometerExceptions {
    odo = Lab3.odometer;
    this.lcd = lcd;
  }

  /**
   * This is the overloaded class constructor
   * 
   * @param lcd A TextLCD object instance to control
   * @param timeout A duration of time to update the display for
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
    odo = Lab3.odometer;
    this.timeout = timeout;
    this.lcd = lcd;
  }

  /**
   * This method is called when the Display thread is started.
   */
  public void run() {
    lcd.clear();

    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("Waypoint Index: " + Lab3.index, 0, 0);
      lcd.drawString("X: " + numberFormat.format(odo.getX()), 0, 1);
      lcd.drawString("Y: " + numberFormat.format(odo.getY()), 0, 2);
      lcd.drawString("T: " + numberFormat.format(odo.getAng()), 0, 3);
      lcd.drawString("US Dist: " + Lab3.pController.readUSDistance(), 0, 4);
      // lcd.drawString("Light: "+Lab3., 0, 5);

      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);
  }
}
