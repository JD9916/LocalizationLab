package ca.mcgill.ecse211.localizationlab;

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */

public class UltrasonicPoller extends Thread {
  private SampleProvider us;               //Buffer for ultrasonic sensor values
  private float[] usData;				   //Array that stores samples from the sensor
  private static float[] localizationScan;
  private static int distance;			   //The distance registered by the sensor
  private int counter = 0;
  private static int output;
  private static final int FILTER_OUT = 10;
  private static int filterControl;
  private static int lastDistance = 50;

  public UltrasonicPoller(SampleProvider us, float[] usData) { //Constructor for this class
    this.us = us;						   
    this.usData = usData;
    this.localizationScan = localizationScan;
    this.filterControl = 0;
  }

  /*
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255]
   */
  
  public void run() {
    while (true) {
      us.fetchSample(usData, 0); 			//Acquire data
      
      
      distance = (int) (usData[0] * 100.0); //Extract from buffer, cast to int
      
      try {									//Timed Sampling
        Thread.sleep(50);
      } catch (Exception e) {
      
      } 
    }
  }
  
  public static int getDistance(){			//Getter for distance (used in navigation class)
	  
	  
	  if (distance > 255 && filterControl < FILTER_OUT) {
		// bad value, do not set the distance var, however do increment the
	    // filter value
		filterControl ++;
		output = lastDistance;
	  } else if (distance > 255){
		// We have repeated large values, so there must actually be nothing
	    // there: leave the distance alone
		output = distance; 
	  } else {
		// distance went below 255: reset filter and leave
	    // distance alone.
        filterControl = 0;
		output = distance;
		}
	  	lastDistance = output;
		return output;
  }

}
