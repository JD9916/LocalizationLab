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

  public UltrasonicPoller(SampleProvider us, float[] usData, float[] localizationScan) { //Constructor for this class
    this.us = us;						   
    this.usData = usData;
    this.localizationScan = localizationScan;
  }

  /*
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255]
   */
  
  public void run() {
    while (true) {
      us.fetchSample(usData, 0); 			//Acquire data
      
      /*
      if(counter > 49){
        counter = 0;
      }
      us.fetchSample(localizationScan, counter);
      localizationScan[counter] = localizationScan[counter] * 100;
      counter++;
      
      */
      distance = (int) (usData[0] * 100.0); //Extract from buffer, cast to int
      
      try {									//Timed Sampling
        Thread.sleep(50);
      } catch (Exception e) {
      
      } 
    }
  }
  
 public void getScan(){
    /*
    for (int i = 0; i < 50; i++){
      us.fetchSample(localizationScan, i);
      
      localizationScan [i] = localizationScan[i] * 100;
      try {                                 //Timed Sampling
        Thread.sleep(180);
      } catch (Exception e) {
      }   
    }
    */

  }
  
  public static int getDistance(){			//Getter for distance (used in navigation class)
	  return distance;
  }

}
