package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class LightLocalizer extends Thread {
  
  private static final int ROTATE_SPEED = 100; 
  private static double colorSensorDistance = 13.9; //How far the color sensor is from the center of rotation
  
  private double thetaY; //The angle that subtends the arc connecting the intersections of the light sensor's path with the y axis
  private double thetaY1;
  private double thetaY2;
  private double thetaX; //The angle that subtends the arc connecting the intersections of the light sensor's path with the y axis
  private double thetaX1;
  private double thetaX2;
  private double dTheta; //The angle needed to correct the odometer
  
  
  private Odometer odometer;
  private Navigation navigator;
  private UltrasonicPoller usPoller;
  private ColorSensorPoller csPoller;
  EV3LargeRegulatedMotor leftMotor;
  EV3LargeRegulatedMotor rightMotor;
  double leftRadius;                    //Holds the left wheel radius (cm)
  double rightRadius;                   //Holds the right wheel radius (cm)
  double width;  						//Holds the Track value
  
  
  
  //Constructor for the light localizer class
  public LightLocalizer(Odometer odo, Navigation navigator, UltrasonicPoller usPoller, ColorSensorPoller csPoller, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width){
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
          motor.stop();
          motor.setAcceleration(1000);}  //sets the motors to come to a stop and adjusts acceleration to 300 for reduced slipping
    this.odometer = odo;
    this.navigator = navigator;
    this.usPoller = usPoller;
    this.csPoller = csPoller;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.width = width;
  }
  
  public void run(){		   //thread that runs for the light localizer
    
    positionRobot();           //runs the method to realign the center of rotation with the diagonal line
    
    leftMotor.stop(true);	   //brings the motors to a stop
    rightMotor.stop(true);
    
    try {					   //waits for 1000 ms
        Thread.sleep(1000);
      } catch (InterruptedException e) {      } 
    
    scan();                    //runs the method that detects the black grid lines
    
    leftMotor.stop(true);      //stops the motors
    rightMotor.stop(true);

    calculate();			   //runs the method that calculates the x and y positions of the robot
    
    try {
        Thread.sleep(1000);	   //waits for 1000 ms
      } catch (InterruptedException e) {      } 
    
    navigator.travelTo(0,0);   //moves the robot to the (0,0) position
    leftMotor.stop(true);	   //stops the motors
    rightMotor.stop(true);
    
    
    try {
        Thread.sleep(1000);	   //waits for 1000 ms
      } catch (InterruptedException e) {      }

    navigator.turnTo(0 - odometer.getTheta()-20);  //turns the robot to face the 0 degree direction. The -20 is used to make up for errors that occur because of the right wheel moving faster than the left
    odometer.setTheta(0.0);						   //sets the angle shown on the robot to 0 degrees
    Sound.twoBeeps();
  }
  
  
  //used to realign the robot to the diagonal line
  private void positionRobot(){
    leftMotor.forward();				//moves the robot forward till it detects the black line
    rightMotor.forward();
    
    while(csPoller.getColor() > 50){
    //wait
    }
    //Sound.beep();
    leftMotor.stop(true);				//stops the robot
    rightMotor.stop(true);
    try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {      } 
    leftMotor.backward();				//moves the robot backwards for 5.5 seconds
    rightMotor.backward();
    try {
      Thread.sleep(5500);
    } catch (InterruptedException e) {      } 
    leftMotor.stop(true);
    rightMotor.stop(true);
    
    try {
      Thread.sleep(1500);
    } catch (InterruptedException e) {      } 
    
    //turns the robot clockwise by 90 degrees
    leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , 90), true);
    rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , 90), false);
    
    try {
      Thread.sleep(1500);
    } catch (InterruptedException e) {      } 
    
    rightMotor.forward();				//moves the robot forward till it detects the black line
    leftMotor.forward();
    
    while(csPoller.getColor() > 50){
      //wait
      }
    
    leftMotor.stop(true);
    rightMotor.stop(true);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {      } 
    leftMotor.backward();				//moves the robot backwards for 5.5 s
    rightMotor.backward();
    try {
      Thread.sleep(5500);
    } catch (InterruptedException e) {      } 
    
  }
  
  
  //scans 4 black grid lines and latches the angles they are detected at
  private void scan(){
    
    leftMotor.forward();				//rotates the robot clockwise
    rightMotor.backward();
    
    while(csPoller.getColor() > 50){
      leftMotor.forward();
      rightMotor.backward();
    }
    this.thetaY1 = odometer.getTheta();	//latches the angle at the first black line
    Sound.beep();
    while(csPoller.getColor() > 50){
      leftMotor.forward();
      rightMotor.backward();
    }
    this.thetaX1 = odometer.getTheta();	//latches the angle at the second black line
    Sound.beep();
    while(csPoller.getColor() > 50){
      leftMotor.forward();
      rightMotor.backward();
    }
    this.thetaY2 = odometer.getTheta();	//latches the angle at the third black line
    Sound.beep();
    while(csPoller.getColor() > 50){
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.stop(true);
    rightMotor.stop(true);
    this.thetaX2 = odometer.getTheta();	//latches the angle at the fourth black line
    Sound.twoBeeps();
    
    
  }
  
  
  //calculates the x and y coordinates of the robot
  private void calculate(){
    this.thetaY = Math.abs(this.thetaY2-this.thetaY1);  //calculates the angles used in determining the robot's position
    this.thetaX = Math.abs(this.thetaX2-this.thetaX1);
    
    //calculates and updates the x and y positions
    odometer.setX((-1)*(colorSensorDistance)*Math.cos((this.thetaY/360)*Math.PI));
    odometer.setY((-1)*(colorSensorDistance)*Math.cos((this.thetaX/360)*Math.PI));
 
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
