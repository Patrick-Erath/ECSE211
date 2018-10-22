package ca.mcgill.ecse211.lab3;

/**
 * This method is where the logic for the odometer will run. Use the methods provided from the
 * OdometerData class to implement the odometer.
 */
public class ObstacleAvoidance extends Thread{

	Navigation nav;
	boolean safe;
	
	/**
	   * This method is where the logic for the odometer will run. Use the methods provided from the
	   * OdometerData class to implement the odometer.
	   */
	public ObstacleAvoidance(Navigation nav){
		this.nav = nav;
		safe = false;
	}
	
	/**
	   * This method is where the logic for the odometer will run. Use the methods provided from the
	   * OdometerData class to implement the odometer.
	   */
	public void run(){
		
		/*
		 * The "avoidance" just stops and turns to heading 0
		 * to make sure that the threads are working properly.
		 * 
		 * If you want to call travelTo from this class you
		 * MUST call travelTo(x,y,false) to go around the
		 * state machine
		 * 
		 * This means that you can't detect a new obstacle
		 * while avoiding the first one. That's probably not something
		 * you were going to do anyway.
		 * 
		 * Otherwise things get complicated and a lot of 
		 * new states will be necessary.
		 * 
		 */
		
		//Log.log(Log.Sender.avoidance,"avoiding obstacle!");
		nav.setSpeeds(0, 0);
		nav.turnTo(0,true);
		nav.goForward(24, false); //using false means the Navigation method is used
		//Log.log(Log.Sender.avoidance,"obstacle avoided!");
		safe = true;
	}

	   /**
	   * This method is where the logic for the odometer will run. Use the methods provided from the
	   * OdometerData class to implement the odometer.
	   */
	public boolean resolved() {
		return safe;
	}
}
