/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Class that implements a Lidar sensor
 */
public class Lidar extends BaseSensor {
    public double LIDAR_TOLERANCE = 0; // tune
    private Counter counter;
    private static Lidar _instance = null;
    private static DigitalInput _source = null;
	
	private Lidar() {
		super();

        if (_source == null){
            
            System.out.println("Initializing Lidar");
            _source = new DigitalInput(Constants.LIDAR_DIO); // although this brings in depency, using setDevice this can be overwritten before calling getInstance
        }

		System.out.println("Source: " + _source);
		counter = new Counter(_source);

	    counter.setMaxPeriod(1.0);
	    // Configure for measuring rising to falling pulses
	    counter.setSemiPeriodMode(true);
	    counter.reset();
		System.out.println("Counter: " + counter);
    }
    
    // for dependency injection - use this to inject a different device
    public static void setDevice(DigitalInput source){
        
        if (_source == null)
            _source = source; // It should only be set once.
    }

    public static Lidar getInstance(){
        
        if (_instance == null){
            _instance = new Lidar();
        }

        return _instance;
    }

	/**
	 * Take a measurement and return the distance in cm
	 * 
	 * @return Distance in cm
	 */
	public double getDistance() {
		double cm;

		SmartDashboard.putNumber("Period", counter.getPeriod());

		// if (counter.get() < 1)
			// return 0;

		// while (counter.get() < 1) {
		// 	System.out.println("Lidar: waiting for distance measurement");
		// }
		/* getPeriod returns time in seconds. The hardware resolution is microseconds.
		 * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
		 */
		cm = (counter.getPeriod() * 1000000.0 / 10.0);
		return cm;
	}

	/**
	 * Converts getDistance() of cm to inches
	 * @return Distance in inches
	 */
	public double getInches() {
		return getDistance() / 2.54;
    }
    
    /**
     * Gets distance in Feet
     * @return Distance in Feet
     */
    public double getFeet(){
        return getInches() / 12;
    }
}
