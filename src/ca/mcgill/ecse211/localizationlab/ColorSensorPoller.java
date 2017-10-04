package ca.mcgill.ecse211.localizationlab;

import lejos.robotics.SampleProvider;


/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class ColorSensorPoller extends Thread {
  private SampleProvider cs;
  private float[] csData;
  public int color;
  
  public ColorSensorPoller(SampleProvider cs, float[] csData) {
    this.cs = cs;
    this.csData = csData;
  }

  /*
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255] (non-Javadoc)
   * 
   * @see java.lang.Thread#run()
   */
  public void run() {
    
    while (true) {
      cs.fetchSample(csData, 0); // acquire data
      
      //System.out.println("Color Level: " + csData[0]);
      
      
      color = (int)(100*csData[0]); // extract from buffer, cast to int

      //cont.processUSData(distance); // now take action depending on value
      
      
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
  }
  
  
  public int getColor(){
	  return this.color;
  }

}
