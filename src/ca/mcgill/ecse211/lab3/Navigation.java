package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.wallfollower.PController;
import ca.mcgill.ecse211.wallfollower.UltrasonicPoller;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * 
 * The Navigator class extends the functionality of the Navigation class.
 * It offers an alternative travelTo() method which uses a state machine
 * to implement obstacle avoidance.
 * 
 * The Navigator class does not override any of the methods in Navigation.
 * All methods with the same name are overloaded i.e. the Navigator version
 * takes different parameters than the Navigation version.
 * 
 * This is useful if, for instance, you want to force travel without obstacle
 * detection over small distances. One place where you might want to do this
 * is in the ObstacleAvoidance class. Another place is methods that implement 
 * specific features for future milestones such as retrieving an object.
 * 
 * 
 */
public class Navigation extends Thread {

	enum State {
		INIT, TURNING, TRAVELLING, EMERGENCY
	};

	State state;

	private boolean isNavigating = false;

	private double destx, desty;

	final static int SLEEP_TIME = 50;

	PController pCont;

	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;
    final static double DEG_ERR = 3.0d, CM_ERR = 0.3d;
    Odometer odometer;
    UltrasonicPoller up;
    private static EV3LargeRegulatedMotor leftMotor;
    private static EV3LargeRegulatedMotor rightMotor;
    private static TextLCD lcd;

    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
	public Navigation(EV3LargeRegulatedMotor left, EV3LargeRegulatedMotor right, TextLCD lcd, Odometer odo, UltrasonicPoller upoller, PController pController) {
	  this.odometer = odo;

      // set acceleration
	  this.leftMotor = left;
	  this.rightMotor = right;
	  this.lcd = lcd;
      this.leftMotor.setAcceleration(ACCELERATION);
      this.rightMotor.setAcceleration(ACCELERATION);
      this.up = upoller;
      this.pCont = pController;
	}

	/**
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading
	 * 
	 * When avoid=true, the nav thread will handle traveling. If you want to
	 * travel without avoidance, this is also possible. In this case,
	 * the method in the Navigation class is used.
	 * 
	 */
	public void travelTo(double x, double y, boolean avoid) {
		if (avoid) {
			destx = x;
			desty = y;
			isNavigating = true;
		} else {
		  double minAng;
	        while (!checkIfDone(x,y)) {
	            minAng = getDestAngle(x,y);
	            this.turnTo(minAng, false);
	            this.setSpeeds(FAST, FAST);
	        }
	        this.setSpeeds(0, 0);
	    }

	    
	}
	
	/**
	   * This method is where the logic for the odometer will run. Use the methods provided from the
	   * OdometerData class to implement the odometer.
	   */
	public void turnTo(double angle, boolean stop) {

      double error = angle - this.odometer.getAng();

      while (Math.abs(error) > DEG_ERR) {

          error = angle - this.odometer.getAng();

          if (error < -180.0) {
              this.setSpeeds(-SLOW, SLOW);
          } else if (error < 0.0) {
              this.setSpeeds(SLOW, -SLOW);
          } else if (error > 180.0) {
              this.setSpeeds(SLOW, -SLOW);
          } else {
              this.setSpeeds(-SLOW, SLOW);
          }
      }

      if (stop) {
          this.setSpeeds(0, 0);
      }
  }

	 
	protected boolean checkIfDone(double x, double y) {
	 if(Math.abs(x - this.odometer.getX()) < CM_ERR
              && Math.abs(y - this.odometer.getY()) < CM_ERR) {
	  this.odometer.setX(x);
	  this.odometer.setY(y);
	  //this.odometer.setAng();
	  return true;
	  } else {
      return false;
	  }
  }

  protected boolean facingDest(double angle) {
      return Math.abs(angle - this.odometer.getAng()) < DEG_ERR;
  }

  protected double getDestAngle(double x, double y) {
      double minAng = (Math.atan2(y - this.odometer.getY(), x - this.odometer.getX()))
              * (180.0 / Math.PI);
      if (minAng < 0) {
          minAng += 360.0;
      }
      return minAng;
  }
	
	/*
	 * Updates the h
	 */
  private void updateTravel() {
    double minAng;

    minAng = getDestAngle(destx, desty);
    /*
     * Use the BasicNavigator turnTo here because 
     * minAng is going to be very small so just complete
     * the turn.
     */
    turnTo(minAng,false);
    this.setSpeeds(FAST, FAST);
}
	
	   public void setSpeeds(int lSpd, int rSpd) {
	        leftMotor.setSpeed(lSpd);
	        rightMotor.setSpeed(rSpd);
	        if (lSpd < 0)
	            leftMotor.backward();
	        else
	            leftMotor.forward();
	        if (rSpd < 0)
	            rightMotor.backward();
	        else
	            rightMotor.forward();
	    }

	   double originalAngle = 0.0d;
	   
	   /**
	    * This method is where the logic for the odometer will run. Use the methods provided from the
	    * OdometerData class to implement the odometer.
	    */
	public void run() {
	  //pcontroller
	// clear the display
		ObstacleAvoidance avoidance = null;
		
		state = State.INIT;
		while (true) {
			switch (state) {
			case INIT:
				if (isNavigating) {
					state = State.TURNING;
				}
				break;
			case TURNING:
				/*
				 * Note: you could probably use the original turnTo()
				 * from BasicNavigator here without doing any damage.
				 * It's cheating the idea of "regular and periodic" a bit
				 * but if you're sure you never need to interrupt a turn there's
				 * no harm.
				 * 
				 * However, this implementation would be necessary if you would like
				 * to stop a turn in the middle (e.g. if you were travelling but also
				 * scanning with a sensor for something...)
				 * 
				 */
				double destAngle = getDestAngle(destx, desty);
				turnTo(destAngle);
				if(facingDest(destAngle)){
					setSpeeds(0,0);
					state = State.TRAVELLING;
				}
				break;
			case TRAVELLING:
				if (checkEmergency()) { // order matters!
					state = State.EMERGENCY;
					//this.up.start();
					//Lab3.usPoller.start();
					originalAngle = this.odometer.getAng();
					avoidance = new ObstacleAvoidance(this);
					avoidance.start();
					
				} else if (!checkIfDone(destx, desty)) {
					updateTravel();
				} else { // Arrived!
					setSpeeds(0, 0);
					isNavigating = false;
					state = State.INIT;
				}
				break;
			case EMERGENCY:
				if (avoidance.resolved()) {
				  //Lab3.usPoller.interrupt();;
					state = State.TURNING;
				}
				break;
			}
			//Log.log(Log.Sender.Navigator, "state: " + state);
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	private boolean checkEmergency() {
	  //return false;
	  //return pCont.readUSDistance() < 10;
		return Lab3.usPoller.getDistance() < 10;
	}


	private void turnTo(double angle) {
		double error;
		error = angle - this.odometer.getAng();

		if (error < -180.0) {
			this.setSpeeds(-SLOW, SLOW);
		} else if (error < 0.0) {
			this.setSpeeds(SLOW, -SLOW);
		} else if (error > 180.0) {
			this.setSpeeds(SLOW, -SLOW);
		} else {
			this.setSpeeds(-SLOW, SLOW);
		}

	}

	/**
	 * Go foward a set distance in cm with or without avoidance
	 */
	public void goForward(double distance, boolean avoid) {
		double x = this.odometer.getX()
				+ Math.cos(Math.toRadians(this.odometer.getAng())) * distance;
		double y = this.odometer.getY()
				+ Math.sin(Math.toRadians(this.odometer.getAng())) * distance;

		this.travelTo(x, y, avoid);

	}

	/**
	   * This method is where the logic for the odometer will run. Use the methods provided from the
	   * OdometerData class to implement the odometer.
	   */
	public boolean isTravelling() {
		return isNavigating;
	}

}