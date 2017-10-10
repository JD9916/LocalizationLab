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
  private boolean isComplete = false;
  
  
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
    
    
    while( UltrasonicPoller.getDistance() > fallingEdge - NOISE) {
        
    	leftMotor.forward();                 //Rotates clockwise till it sees the bottom wall
        rightMotor.backward();  
    }
    rightMotor.stop(true);
    leftMotor.stop(true);
    
    Sound.beep();
    heading1 = odometer.getTheta();

    try {
        Thread.sleep(1500);
      } catch (InterruptedException e) {      } 
    
    
    while( UltrasonicPoller.getDistance() < fallingEdge + NOISE) {
    	
    	leftMotor.backward();                //Rotates counterclockwise till it sees nothing
    	rightMotor.forward();
    }
    
    while( UltrasonicPoller.getDistance() > fallingEdge - NOISE) {
    	leftMotor.backward();                 //Rotates counterclockwise till it sees the left wall
    	rightMotor.forward();
    }
    leftMotor.stop(true);
    rightMotor.stop(true);
    
    Sound.beep();
    heading2 = odometer.getTheta();
 
    
    try {
        Thread.sleep(1500);
      } catch (InterruptedException e) {      } 
    
    if(heading1 > heading2) {
    	heading1 -= 360;
    }
    
    avgHeading = (heading1 + heading2)/2;
    dTheta = heading2 - avgHeading -10;

    leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , dTheta), true);
    rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , dTheta), false);
    
    leftMotor.stop(true);
    rightMotor.stop(true);
    
    odometer.setTheta(0.0);
    Sound.twoBeeps();
    
    this.isComplete = true;
     
  }
    
    
  
  
  
  public void risingEdge(){
	  
	  try {
		  Thread.sleep(2000);
		} catch (InterruptedException e) {      } 
		  
		leftMotor.setAcceleration(250);
		rightMotor.setAcceleration(250);
	    leftMotor.setSpeed(ROTATE_SPEED);    //Sets the motors to rotation speed  
	    rightMotor.setSpeed(ROTATE_SPEED);
    
	    //If the robot has been placed outward, turn clockwise until the bottom wall is seen
	    while( UltrasonicPoller.getDistance() > risingEdge - NOISE) {   
	    	leftMotor.forward();             //Rotates clockwise till it sees the bottom wall
	        rightMotor.backward();  
	    }
	    
	    //Rotates clockwise till it sees nothing
	    while( UltrasonicPoller.getDistance() < risingEdge + NOISE) {
	    	leftMotor.forward();             
	        rightMotor.backward();  
	    }
	    
	    rightMotor.stop(true);
	    leftMotor.stop(true);
	    
	    Sound.beep();
	    heading1 = odometer.getTheta();
	    
	    try {
	        Thread.sleep(1500);
	      } catch (InterruptedException e) {      } 
	    
	    //Rotates counterclockwise till it sees the left wall
	    while( UltrasonicPoller.getDistance() > risingEdge + NOISE) {
	    	leftMotor.backward();                
	    	rightMotor.forward();
	    }
	    
	    //Rotates counterclockwise till it sees nothing
	    while( UltrasonicPoller.getDistance() < risingEdge - NOISE) {
	    	leftMotor.backward();                 
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
	    
	    
	    if(heading1 < heading2) {
	        heading2 -= 360;
	    }
	    
	    avgHeading = (heading1 + heading2)/2;
	    dTheta = heading1 - avgHeading;

	    leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , dTheta), true);
	    rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , dTheta), false);
	    
	    leftMotor.stop(true);
	    rightMotor.stop(true);
	    
	    odometer.setTheta(0.0);
	    Sound.twoBeeps();
	    
	    this.isComplete = true;
  
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
  
  public boolean isComplete(){
    return this.isComplete;
  }
  
 

}
