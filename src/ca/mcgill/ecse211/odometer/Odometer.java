package ca.mcgill.ecse211.odometer;
 
/*
 * File: Odometer.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * Changed to Thread - Jonah Caplan
 * 2015
 * 
 * Class which controls the odometer for the robot
 * 
 * Odometer defines cooridinate system as such...
 * 
 *                  90Deg:pos y-axis
 *                          |
 *                          |
 *                          |
 *                          |
 * 180Deg:neg x-axis------------------0Deg:pos x-axis
 *                          |
 *                          |
 *                          |
 *                          |
 *                  270Deg:neg y-axis
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 */
 
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements odometry on our robot.
 * 
 * @author Caspar Cedro & Patrick Erath
 */
public class Odometer extends Thread {
 
    private EV3LargeRegulatedMotor leftMotor, rightMotor;
    private final int TIMEOUT_PERIOD = 75;
    private double leftRadius, rightRadius, width;
    private double x, y, theta;
    private double[] oldDH, dDH;
 
    /**
     * This is the default constructor of this class. It initiates all motors and variables once.It
     * cannot be accessed externally.
     * 
     * @param leftMotor
     * @param rightMotor
     * @throws OdometerExceptions
     */
    public Odometer(EV3LargeRegulatedMotor leftMotor,
            EV3LargeRegulatedMotor rightMotor) {
 
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
 
        // default values, modify for your robot
        this.rightRadius = 2.1;
        this.leftRadius = 2.1;
        this.width = 11.7;
 
        this.x = 0.0;
        this.y = 0.0;
        this.theta = 90.0;
        this.oldDH = new double[2];
        this.dDH = new double[2];
 
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    private void getDisplacementAndHeading(double[] data) {
        int leftTacho, rightTacho;
        leftTacho = leftMotor.getTachoCount();
        rightTacho = rightMotor.getTachoCount();
 
        data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI
                / 360.0;
        data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public void run() {
        while (true) {
            this.getDisplacementAndHeading(dDH);
            dDH[0] -= oldDH[0];
            dDH[1] -= oldDH[1];
 
            // update the position in a critical region
            synchronized (this) {
                theta += dDH[1];
                theta = fixDegAngle(theta);
 
                x += dDH[0] * Math.cos(Math.toRadians(theta));
                y += dDH[0] * Math.sin(Math.toRadians(theta));
            }
 
            oldDH[0] += dDH[0];
            oldDH[1] += dDH[1];
 
            try {
                Thread.sleep(TIMEOUT_PERIOD);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
 
            //Log.log(Log.Sender.odometer,String.format("x: %f, y: %f, a: %f",
                        //getX(), getY(), getAng()));
             
        }
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public double getX() {
        synchronized (this) {
            return x;
        }
    }
     
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public void setX(double val) {
        synchronized (this) {
            this.x = val;
        }
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public double getY() {
        synchronized (this) {
            return y;
        }
    }
     
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public void setY(double val) {
        synchronized (this) {
            this.y = val;
        }
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public double getAng() {
        synchronized (this) {
            return theta;
        }
    }
     
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public void setAng(double val) {
        synchronized (this) {
            this.theta = val;
        }
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public void setPosition(double[] position, boolean[] update) {
        synchronized (this) {
            if (update[0])
                x = position[0];
            if (update[1])
                y = position[1];
            if (update[2])
                theta = position[2];
        }
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public void getPosition(double[] position) {
        synchronized (this) {
            position[0] = x;
            position[1] = y;
            position[2] = theta;
        }
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public double[] getPosition() {
        synchronized (this) {
            return new double[] { x, y, theta };
        }
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public EV3LargeRegulatedMotor[] getMotors() {
        return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public EV3LargeRegulatedMotor getLeftMotor() {
        return this.leftMotor;
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public EV3LargeRegulatedMotor getRightMotor() {
        return this.rightMotor;
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public static double fixDegAngle(double angle) {
        if (angle < 0.0)
            angle = 360.0 + (angle % 360.0);
 
        return angle % 360.0;
    }
 
    /**
     * This method is where the logic for the odometer will run. Use the methods provided from the
     * OdometerData class to implement the odometer.
     */
    public static double minimumAngleFromTo(double a, double b) {
        double d = fixDegAngle(b - a);
 
        if (d < 180.0)
            return d;
        else
            return d - 360.0;
    }
}