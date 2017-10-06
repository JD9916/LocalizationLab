package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class UltrasonicLocalizer extends Thread {

  private int mode;
  private static final int ROTATE_SPEED = 100; 
  private static int fallingEdge = 30;
  private static int risingEdge = 42;
  private static final int FILTER_OUT = 10;
  private double avgHeading;
  private double dTheta;
  private static final int NOISE = 1;
  
  
  EV3LargeRegulatedMotor leftMotor;
  EV3LargeRegulatedMotor rightMotor;
    
  double leftRadius;                    //Holds the left wheel radius (cm)
  double rightRadius;                   //Holds the right wheel radius (cm)
  double width;                         //Holds the length of the wheel base (cm)
  private Odometer odometer;
  private UltrasonicPoller usPoller;
  private static float[] localizationScan;
  private double heading1;
  private double heading2;
  private int distance;
  
  public UltrasonicLocalizer(Odometer odo, UltrasonicPoller usPoller, float[] localizationScan, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width){
	for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
	      motor.stop();
	      motor.setAcceleration(3000);}
	this.odometer = odo;
    this.usPoller = usPoller;
    this.localizationScan = localizationScan;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.width = width;
  }
  
  public void run(){
    if(mode == 1){    //Falling Edge Mode
      this.fallingEdge();  
    }
    if(mode == 2){    //Rising Edge Mode 
      this.risingEdge();  
    } 
  }

  public void fallingEdge(){
	  
	try {
	  Thread.sleep(2000);
	} catch (InterruptedException e) {      } 
	  
	leftMotor.setAcceleration(250);
	rightMotor.setAcceleration(250);
    leftMotor.setSpeed(ROTATE_SPEED);    //Sets the motors to rotation speed  
    rightMotor.setSpeed(ROTATE_SPEED);
    
    
    while( UltrasonicPoller.getDistance() < fallingEdge + NOISE) {
    
    	leftMotor.forward();                 //Rotates clockwise till it sees nothing
        rightMotor.backward();  
    }
    Sound.beep();
    
    while( UltrasonicPoller.getDistance() >= fallingEdge - NOISE) {
        
    	leftMotor.forward();                 //Rotates clockwise till it sees the bottom wall
        rightMotor.backward();  
    }
    rightMotor.stop(true);
    leftMotor.stop(true);
    
    Sound.beep();
    heading1 = odometer.getTheta();
    System.out.println("        " + heading1);
    
    
    
    try {
        Thread.sleep(1500);
      } catch (InterruptedException e) {      } 
    
    
    while( UltrasonicPoller.getDistance() < fallingEdge + NOISE) {
    	
    	leftMotor.backward();                //Rotates counterclockwise till it sees nothing
    	rightMotor.forward();
    }
    Sound.beep();
    
    while( UltrasonicPoller.getDistance() >= fallingEdge - NOISE) {
    	leftMotor.backward();                 //Rotates counterclockwise till it sees the left wall
    	rightMotor.forward();
    }
    leftMotor.stop(true);
    rightMotor.stop(true);
    
    Sound.beep();
    heading2 = odometer.getTheta();
    System.out.println("        " + heading2);
    
    
    try {
        Thread.sleep(1500);
      } catch (InterruptedException e) {      } 
    
    if(heading1 > heading2) {
    	heading1 -= 360;
    }
    
    avgHeading = (heading1 + heading2)/2;
    dTheta = heading2 - avgHeading;
    
    /*if(heading1 < heading2) {
    	dTheta = 225 - avgHeading;
    }
    else {
    	dTheta = 45 - avgHeading;
    	dTheta += 360;
    }*/
    
    leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , dTheta), true);
    rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , dTheta), false);
    
    leftMotor.stop(true);
    rightMotor.stop(true);
    
    odometer.setTheta(0.0);
    Sound.twoBeeps();
    
      /*if (distance >= 255 && filterControl < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the
        // filter value
        filterControl++;
      } else if (distance >= 255) {
        // We have repeated large values, so there must actually be nothing
        // there: leave the distance alone
          this.distance = (int) distance;
      } else {
        // distance went below 255: reset filter and leave
        // distance alone.
        filterControl = 0;
        this.distance = (int) distance;
      }*/

      /*if(distance < fallingEdge){
        heading1 = odometer.getTheta();
        break;
      }
      
    }
    
    leftMotor.backward();
    rightMotor.forward();*/
    
    
    /*while(true){
      
      if (distance >= 255 && filterControl < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the
        // filter value
        filterControl++;
      } else if (distance >= 255) {
        // We have repeated large values, so there must actually be nothing
        // there: leave the distance alone
          this.distance = (int) distance;
      } else {
        // distance went below 255: reset filter and leave
        // distance alone.
        filterControl = 0;
        this.distance = (int) distance;
      }
      
      if(distance < fallingEdge){
        heading2 = odometer.getTheta();
        break;
      }
    }
    
    leftMotor.forward();
    rightMotor.backward();
    
    while(odometer.getTheta() < Math.abs(heading1-heading2) - 1 || odometer.getTheta() > Math.abs(heading1-heading2) + 1){
      //do nothing
    }
    
    leftMotor.stop();
    rightMotor.stop();
    
    System.out.print("Done");*/
      
  }
    
    
  
  
  
  public void risingEdge(){
    
  }
  
  public void setMode(int mode){
    this.mode = mode;
  }
  
  private static int convertAngle(double radius, double width, double angle) {
    //Given wheel radius (cm), wheel base (cm), and desired turn angle (degrees)
    //converts to the angle the wheels need to drive.
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  private static int convertDistance(double radius, double distance) {
    //Given the distance (cm) to travel and the wheel radius (cm), 
    //converts to amount of wheel rotation needed.
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
  
 

}
