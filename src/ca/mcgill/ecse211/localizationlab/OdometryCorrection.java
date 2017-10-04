package ca.mcgill.ecse211.localizationlab;
import lejos.hardware.Sound;

public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private ColorSensorPoller poller;
  private int colorValue;
  
  

  // constructor
  public OdometryCorrection(Odometer odometer, ColorSensorPoller ColorSensorPoller) {   //Passes in the ColorSensorPoller class
    this.odometer = odometer;
    this.poller = ColorSensorPoller; //initiates the poller variable of this class. This is done to pass in values from the color sensor 
    
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    
    while (true) {
      correctionStart = System.currentTimeMillis();
      
      colorValue = poller.getColor();   //Assigns values form the color sensor to a variable that is later used to detect a black line
            
      
      //TODO Place correction implementation here
      

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometry correction will be
          // interrupted by another thread
        }
      }
    }
  }
}
